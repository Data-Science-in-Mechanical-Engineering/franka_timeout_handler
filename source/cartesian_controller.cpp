#include "../include/franka_real_time/cartesian_controller.h"
#include "../include/franka_real_time/robot.h"
#include <stdexcept>

void franka_real_time::CartesianController::_state_to_input(const franka::RobotState &robot_state)
{
    _joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    _joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    _transform = Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data());
    _position = _transform.translation();
    _orientation = _transform.linear();
    _jacobian = Eigen::Matrix<double, 6, 7>::Map(_robot->_model->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    _velocity_rotation = _jacobian * _joint_velocities;
}

void franka_real_time::CartesianController::_input_to_robot_input()
{
    _robot->_joint_positions = _joint_positions;
    _robot->_joint_velocities = _joint_velocities;
    _robot->_position = _position;
    _robot->_orientation = _orientation;
    _robot->_velocity = _velocity_rotation.head(3);
    _robot->_rotation = _velocity_rotation.tail(3);
    _robot->_call = _call;
}

void franka_real_time::CartesianController::_calculate_temporary(const franka::RobotState &robot_state)
{
    _coriolis = Eigen::Matrix<double, 7, 1>::Map(_robot->_model->coriolis(robot_state).data());
}

void franka_real_time::CartesianController::_robot_output_to_output()
{
    if (_robot->_update_timeout != Update::no) _timeout = _robot->_timeout;
    if (_robot->_update_target_position != Update::no) _target_position = _robot->_target_position;
    if (_robot->_update_target_orientation != Update::no) _target_orientation = _robot->_target_orientation;
    if (_robot->_update_translation_stiffness != Update::no) _translation_stiffness = _robot->_translation_stiffness;
    if (_robot->_update_rotation_stiffness != Update::no) _rotation_stiffness = _robot->_rotation_stiffness;
    if (_robot->_update_translation_damping != Update::no) _translation_damping = _robot->_translation_damping;
    if (_robot->_update_rotation_damping != Update::no) _rotation_damping = _robot->_rotation_damping;
    if (_robot->_update_control_rotation != Update::no) _control_rotation = _robot->_control_rotation;
    if (_robot->_update_joint_torques_limit != Update::no) _joint_torques_limit = _robot->_joint_torques_limit;
    if (_robot->_update_frequency_divider != Update::no) _frequency_divider = _robot->_frequency_divider;
    if(_robot->_update_use_joint_controller_flag != Update::no) _use_joint_controller=_robot->_use_joint_controller;
    if(_robot->_update_target_joint_position != Update::no) _target_joint_position = _robot->_target_joint_position;
}

void franka_real_time::CartesianController::_calculate_result()
{   if(_use_joint_controller){
        _joint_torques.setZero();
        //
        const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
        // Damping
        const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

        for (size_t i = 0; i < 7; i++) {
        _joint_torques(i) =
            0.1*k_gains[i] * (_target_joint_position(i) - _joint_positions(i)) - 0.32*d_gains[i] * _joint_velocities(i) + _coriolis(i);
      }
    // std::cout<<_joint_torques<<std::endl;
    //_joint_torques.setZero();
    }
    else{
    Eigen::Matrix<double, 6, 6> stiffness;
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << _translation_stiffness;
    stiffness.bottomRightCorner(3, 3) << _rotation_stiffness;
    Eigen::Matrix<double, 6, 6> damping;
    damping.setZero();
    damping.topLeftCorner(3, 3) << _translation_damping;
    damping.bottomRightCorner(3, 3) << _rotation_damping;
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << _position - _target_position;
    if (_target_orientation.coeffs().dot(_orientation.coeffs()) < 0.0) _orientation.coeffs() << -_orientation.coeffs();
    Eigen::Quaterniond error_quaternion(_orientation.inverse() * _target_orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << -_transform.linear() * error.tail(3);
    //std::cout<<-_joint_positions(0)<<std::endl;
    _joint_torques << _rotation_correction *(_jacobian.transpose() * (-stiffness * error - damping * _velocity_rotation) + _coriolis);
    //_joint_torques(0)=0;
    //if(_joint_torques(0)!=0) std::cout<<_joint_torques(0)<<std::endl;
    //std::cout<<-_joint_torques(0)<<std::endl;
    
    }
    //Security
    //Eigen::Array<double, 7, 1> limits = (_joint_torques_limit * _joint_torques_maximum.array() / _joint_torques.array()).abs();
    //if (limits.minCoeff() < 1.0) _joint_torques *= limits.minCoeff();
}

void franka_real_time::CartesianController::_result_to_robot_result()
{
    if (_robot->_update_joint_torques != Update::no) _robot->_joint_torques = _joint_torques;
}

void franka_real_time::CartesianController::_robot_output_to_late_output()
{
    if (_robot->_update_timeout == Update::yes) { _late_timeout = _robot->_timeout; _late_update_timeout = true; }
    if (_robot->_update_target_position == Update::yes) { _late_target_position = _robot->_target_position; _late_update_target_position = true; }
    if(_robot->_update_target_joint_position==Update::yes){_late_target_joint_position = _robot->_target_joint_position; _late_update_target_joint_position = true;}
    if (_robot->_update_target_orientation == Update::yes) { _late_target_orientation = _robot->_target_orientation; _late_update_target_orientation = true; }
    if (_robot->_update_translation_stiffness == Update::yes) { _late_translation_stiffness = _robot->_translation_stiffness; _late_update_translation_stiffness = true; }
    if (_robot->_update_rotation_stiffness == Update::yes) { _late_rotation_stiffness = _robot->_rotation_stiffness; _late_update_rotation_stiffness = true; }
    if (_robot->_update_translation_damping == Update::yes) { _late_translation_damping = _robot->_translation_damping; _late_update_translation_damping = true; }
    if (_robot->_update_rotation_damping == Update::yes) { _late_rotation_damping = _robot->_rotation_damping; _late_update_rotation_damping = true; }
    if (_robot->_update_control_rotation == Update::yes) { _late_control_rotation = _robot->_control_rotation; _late_update_control_rotation = true; }
    if (_robot->_update_joint_torques_limit == Update::yes) { _late_joint_torques_limit = _robot->_joint_torques_limit; _late_update_joint_torques_limit = true; }
    if (_robot->_update_frequency_divider == Update::yes) { _late_frequency_divider = _robot->_frequency_divider; _late_update_frequency_divider = true; }
    if (_robot->_update_use_joint_controller_flag == Update::yes){_late_use_joint_controller=_robot->_use_joint_controller;_late_update_use_joint_controller =true;}
    
}

void franka_real_time::CartesianController::_late_output_to_output()
{
    if (_late_update_timeout) { _timeout = _late_timeout; _late_update_timeout = false; }
    if (_late_update_target_position) { _target_position = _late_target_position; _late_update_target_position = false; }
    if(_late_update_target_joint_position){_target_joint_position = _late_target_joint_position; _late_update_target_joint_position = false;}
    if (_late_update_target_orientation) { _target_orientation = _late_target_orientation; _late_update_target_orientation = false; }
    if (_late_update_translation_stiffness) { _translation_stiffness = _late_translation_stiffness; _late_update_translation_stiffness = false; }
    if (_late_update_rotation_stiffness) { _rotation_stiffness = _late_rotation_stiffness; _late_update_rotation_stiffness = false; }
    if (_late_update_translation_damping) { _translation_damping = _late_translation_damping; _late_update_translation_damping = false; }
    if (_late_update_rotation_damping) { _rotation_damping = _late_rotation_damping; _late_update_rotation_damping = false; }
    if (_late_update_control_rotation) { _control_rotation = _late_control_rotation; _late_update_control_rotation = false; }
    if (_late_update_joint_torques_limit) { _joint_torques_limit = _late_joint_torques_limit; _late_update_joint_torques_limit = false; }
    if (_late_update_frequency_divider) { _frequency_divider = _late_frequency_divider; _late_update_frequency_divider = false; }
    if(_late_update_use_joint_controller ) {_use_joint_controller = _late_use_joint_controller; _late_update_use_joint_controller = false;}
}

void franka_real_time::CartesianController::_result_to_late_result()
{
    _late_joint_torques = _joint_torques;
}

void franka_real_time::CartesianController::_late_result_to_robot_result()
{
    if (_robot->_update_joint_torques == Update::yes) _robot->_joint_torques = _joint_torques;
}

void franka_real_time::CartesianController::_control(const franka::RobotState &robot_state)
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
            timespec timeout;
            clock_gettime(CLOCK_REALTIME, &timeout);
            timeout.tv_nsec += 1000 * _timeout;
            if (timeout.tv_nsec > 1000*1000*1000) { timeout.tv_nsec -= 1000*1000*1000; timeout.tv_sec++; } 
            pthread_cond_timedwait(&_send_condition, &_mutex, &timeout);
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

franka_real_time::CartesianController::CartesianController(Robot *robot)
{
    _robot = robot;
    
    //Init call count
    _call = 0;
    //Init constants
    _rotation_correction.setZero();
    _rotation_correction.diagonal() << 0.5, 1.0, 1.0, 0.5, 0.05, 1.0, 0.05;
    _joint_torques_maximum << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;

    //Init outputs
    _robot_output_to_output();
    //Technical
    if (pthread_cond_init(&_receive_condition, nullptr) != 0) throw std::runtime_error("franka_real_time: pthread_cond_init failed");
    if (pthread_cond_init(&_send_condition, nullptr) != 0) throw std::runtime_error("franka_real_time: pthread_cond_init failed");
    if (pthread_mutex_init(&_mutex, nullptr) != 0) throw std::runtime_error("franka_real_time: pthread_mutex_init failed");
    pthread_attr_t pthread_attributes;
    if (pthread_attr_init(&pthread_attributes) != 0) throw std::runtime_error("franka_real_time: pthread_attr_init failed");;
    if (pthread_attr_setschedpolicy(&pthread_attributes, SCHED_FIFO) != 0) throw std::runtime_error("franka_real_time: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 90;
    if (pthread_attr_setschedparam(&pthread_attributes, &scheduling_parameters) != 0) throw std::runtime_error("franka_real_time: pthread_attr_setschedparam failed");;;
    if (pthread_attr_setinheritsched(&pthread_attributes, PTHREAD_EXPLICIT_SCHED) != 0) throw std::runtime_error("franka_real_time: pthread_attr_setinheritsched failed");;;
    if (pthread_create(&_backend_thread, &pthread_attributes, [](void* controller) -> void*
    {
        ((CartesianController*)controller)->_robot->_robot.control([controller](const franka::RobotState &robot_state, franka::Duration duration) -> franka::Torques
        {
            ((CartesianController*)controller)->_control(robot_state);
            franka::Torques joint_torques{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            Eigen::Matrix<double, 7, 1>::Map(&joint_torques.tau_J[0]) = ((CartesianController*)controller)->_joint_torques;
            joint_torques.motion_finished = ((CartesianController*)controller)->_joint_torques_finished;
            return joint_torques;
        });
        return nullptr;
    }, this) != 0) throw std::runtime_error("franka_real_time: pthread_create failed");
}

void franka_real_time::CartesianController::receive()
{
    pthread_mutex_lock(&_mutex);
    _receive_state = ReceiveState::receive;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _receive_state = ReceiveState::post_receive;
    pthread_mutex_unlock(&_mutex);
}
		
void franka_real_time::CartesianController::send()
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

    if (error) throw std::runtime_error("franka_real_time: CartesianController did not call receive()");
}

void franka_real_time::CartesianController::receive_and_send()
{
    pthread_mutex_lock(&_mutex);
    _receive_state = ReceiveState::receive_and_send;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _receive_state = ReceiveState::other;
    pthread_mutex_unlock(&_mutex);
}

void franka_real_time::CartesianController::send_and_receive()
{
    send();
    receive();
}

franka_real_time::CartesianController::~CartesianController()
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