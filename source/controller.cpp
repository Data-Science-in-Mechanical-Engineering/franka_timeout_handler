#include "../include/franka_timeout_handler/controller.h"
#include "../include/franka_timeout_handler/robot_core.h"
#include "../include/franka_timeout_handler/constants.h"
#include <stdexcept>

void franka_timeout_handler::Controller::_state_to_input(const franka::RobotState &robot_state)
{
    _joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    _joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    _transform = Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data());
    _position = _transform.translation();
    _orientation = _transform.linear();
    _jacobian = Eigen::Matrix<double, 6, 7>::Map(_robot->_model->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    _velocity_rotation = _jacobian * _joint_velocities;
}

void franka_timeout_handler::Controller::_input_to_robot_input()
{
    _robot->_joint_positions = _joint_positions;
    _robot->_joint_velocities = _joint_velocities;
    _robot->_position = _position;
    _robot->_orientation = _orientation;
    _robot->_velocity = _velocity_rotation.head(3);
    _robot->_rotation = _velocity_rotation.tail(3);
    _robot->_call = _call;
}

void franka_timeout_handler::Controller::_calculate_temporary(const franka::RobotState &robot_state)
{
    _coriolis = Eigen::Matrix<double, 7, 1>::Map(_robot->_model->coriolis(robot_state).data());
}

void franka_timeout_handler::Controller::_robot_output_to_output()
{
    if (_robot->_update_timeout != Update::never) _timeout = _robot->_timeout;
    if (_robot->_update_joint_torques_limit != Update::never) _joint_torques_limit = _robot->_joint_torques_limit;
    if (_robot->_update_frequency_divider != Update::never) _frequency_divider = _robot->_frequency_divider;
}

void franka_timeout_handler::Controller::_result_to_robot_result()
{
    if (_robot->_update_joint_torques != Update::never) _robot->_joint_torques = _joint_torques;
}

void franka_timeout_handler::Controller::_robot_output_to_late_output()
{
    if (_robot->_update_timeout == Update::always) { _late_timeout = _robot->_timeout; _late_update_timeout = true; }
    if (_robot->_update_joint_torques_limit == Update::always) { _late_joint_torques_limit = _robot->_joint_torques_limit; _late_update_joint_torques_limit = true; }
    if (_robot->_update_frequency_divider == Update::always) { _late_frequency_divider = _robot->_frequency_divider; _late_update_frequency_divider = true; }
}

void franka_timeout_handler::Controller::_late_output_to_output()
{
    if (_late_update_timeout) { _timeout = _late_timeout; _late_update_timeout = false; }
    if (_late_update_joint_torques_limit) { _joint_torques_limit = _late_joint_torques_limit; _late_update_joint_torques_limit = false; }
    if (_late_update_frequency_divider) { _frequency_divider = _late_frequency_divider; _late_update_frequency_divider = false; }
}

void franka_timeout_handler::Controller::_result_to_late_result()
{
    _late_joint_torques = _joint_torques;
}

void franka_timeout_handler::Controller::_late_result_to_robot_result()
{
    if (_robot->_update_joint_torques == Update::always) _robot->_joint_torques = _joint_torques;
}

void franka_timeout_handler::Controller::_control(const franka::RobotState &robot_state)
{
    _call++;
    
    //Frequency divider
    if (++_frequency_divider_count < _frequency_divider)
    {
        _state_to_input(robot_state);
        _calculate_temporary(robot_state);
        _calculate_result();
        _joint_torques_finished = false;
        return;
    }
    _frequency_divider_count = 0;

    //Setting backend inputs
    _state_to_input(robot_state);

    //Receive, receive_and_send or ~Robot
    pthread_mutex_lock(&_mutex);
    ReceiveState receive_state = _receive_state;
    _late_output_to_output();
    if (_receive_state == ReceiveState::receive)
    {
        //Set input
        _send_state = SendState::pre_wait;
        _input_to_robot_input();
        pthread_cond_signal(&_receive_condition);
    }
    else if (_receive_state == ReceiveState::receive_and_send || _receive_state == ReceiveState::destructor)
    {
        //Full cycle here
        _send_state = SendState::other;
        _input_to_robot_input();
        _calculate_temporary(robot_state);
        _robot_output_to_output();
        _calculate_result();
        _result_to_robot_result();
        _robot->_late = false;
        pthread_cond_signal(&_receive_condition);
    }
    else
    {
        _send_state = SendState::other;
    }
    pthread_mutex_unlock(&_mutex);

    //Deciding between receive variants
    if (receive_state == ReceiveState::receive)
    {
        //Robot::receive() was called, waiting for send() and calculating torques
        _calculate_temporary(robot_state);

        pthread_mutex_lock(&_mutex);
        if (_send_state == SendState::wait)
        {
            //Scenario 1: Frontend arrived before backend started to wait
            _robot_output_to_output();
            _calculate_result();        
            _result_to_robot_result();
            _robot->_late = false;
            pthread_cond_signal(&_send_condition);
        }
        else
        {
            _send_state = SendState::wait;
            timespec time;
            time.tv_sec = 0;
            time.tv_nsec = 1000 * _timeout;
            pthread_cond_timedwait(&_send_condition, &_mutex, &time);
            if (_send_state == SendState::post_wait)
            {
                //Scenario 2: Frontend arrived when backend waited it
            }
            else
            {
                //Scenario 3: Frontend did not arrive
                _send_state = SendState::post_wait;
                _calculate_result();
                _result_to_late_result();
            }
        }
        pthread_mutex_unlock(&_mutex);
        _joint_torques_finished = false;
    }
    else if (receive_state == ReceiveState::receive_and_send)
    {
        //Robot::receive_and_send() was called, torques are already calculated
        _joint_torques_finished = false;
    }
    else if (receive_state == ReceiveState::destructor)
    {
        //Robot::~Robot() was called, torques are already calculated
        _joint_torques_finished = true;
    }
    else
    {
        //No receive was called, calculating torques
        _calculate_temporary(robot_state);
        _calculate_result();
        _joint_torques_finished = false;
    }
}

franka_timeout_handler::Controller::Controller(RobotCore *robot_core)
{
    _robot = robot_core;
    
    //Init call count
    _call = 0;

    //Init outputs
    _robot_output_to_output();

    //Technical
    if (pthread_cond_init(&_receive_condition, nullptr) != 0) throw std::runtime_error("franka_timeout_handler: pthread_cond_init failed");
    if (pthread_cond_init(&_send_condition, nullptr) != 0) throw std::runtime_error("franka_timeout_handler: pthread_cond_init failed");
    if (pthread_mutex_init(&_mutex, nullptr) != 0) throw std::runtime_error("franka_timeout_handler: pthread_mutex_init failed");
    pthread_attr_t pthread_attributes;
    if (pthread_attr_init(&pthread_attributes) != 0) throw std::runtime_error("franka_timeout_handler: pthread_attr_init failed");;
    if (pthread_attr_setschedpolicy(&pthread_attributes, SCHED_FIFO) != 0) throw std::runtime_error("franka_timeout_handler: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 90;
    if (pthread_attr_setschedparam(&pthread_attributes, &scheduling_parameters) != 0) throw std::runtime_error("franka_timeout_handler: pthread_attr_setschedparam failed");;;
    if (pthread_attr_setinheritsched(&pthread_attributes, PTHREAD_EXPLICIT_SCHED) != 0) throw std::runtime_error("franka_timeout_handler: pthread_attr_setinheritsched failed");;;
    if (pthread_create(&_backend_thread, &pthread_attributes, [](void* controller) -> void*
    {
        ((Controller*)controller)->_robot->_robot.control([controller](const franka::RobotState &robot_state, franka::Duration duration) -> franka::Torques
        {
            //Calculate
            ((Controller*)controller)->_control(robot_state);
            
            //Apply security
            Eigen::Array<double, 7, 1> limits = (((Controller*)controller)->_joint_torques_limit * max_joint_torque.array() / ((Controller*)controller)->_joint_torques.array()).abs();
            if (limits.minCoeff() < 1.0) ((Controller*)controller)->_joint_torques *= limits.minCoeff();  
            
            //Return
            franka::Torques joint_torques{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            Eigen::Matrix<double, 7, 1>::Map(&joint_torques.tau_J[0]) = ((Controller*)controller)->_joint_torques;
            joint_torques.motion_finished = ((Controller*)controller)->_joint_torques_finished;
            return joint_torques;
        });
        return nullptr;
    }, this) != 0) throw std::runtime_error("franka_timeout_handler: pthread_create failed");
}

void franka_timeout_handler::Controller::receive()
{
    pthread_mutex_lock(&_mutex);
    _receive_state = ReceiveState::receive;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _receive_state = ReceiveState::post_receive;
    pthread_mutex_unlock(&_mutex);
}
		
void franka_timeout_handler::Controller::send()
{
    pthread_mutex_lock(&_mutex);
    bool error = _receive_state != ReceiveState::post_receive;
    if (!error)
    {
        _receive_state = ReceiveState::other;
        if (_send_state == SendState::pre_wait)
        {
            //Scenario 1: Frontend arrived before backend started to wait
            _send_state = SendState::wait;
            pthread_cond_wait(&_send_condition, &_mutex);
        }
        else if (_send_state == SendState::wait)
        {
            //Scenario 2: Frontend arrived when backend waited it
            _send_state = SendState::post_wait;
            _robot_output_to_output();
            _calculate_result();
            _result_to_robot_result();
            _robot->_late = false;
            pthread_cond_signal(&_send_condition);
        }
        else
        {
            //Scenario 3: Frontend did not arrive on time
            _robot_output_to_late_output();
            _late_result_to_robot_result();
            _robot->_late = true;
        }
    }
    pthread_mutex_unlock(&_mutex);

    if (error) throw std::runtime_error("franka_timeout_handler: Controller did not call receive()");
}

void franka_timeout_handler::Controller::receive_and_send()
{
    pthread_mutex_lock(&_mutex);
    _receive_state = ReceiveState::receive_and_send;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _receive_state = ReceiveState::other;
    pthread_mutex_unlock(&_mutex);
}

void franka_timeout_handler::Controller::send_and_receive()
{
    send();
    receive();
}

franka_timeout_handler::Controller::~Controller()
{
    pthread_mutex_lock(&_mutex);
    _receive_state = ReceiveState::destructor;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _receive_state = ReceiveState::other;
    pthread_mutex_unlock(&_mutex);

    pthread_join(_backend_thread, nullptr);
    pthread_cond_destroy(&_receive_condition);
    pthread_cond_destroy(&_send_condition);
    pthread_mutex_destroy(&_mutex);
}