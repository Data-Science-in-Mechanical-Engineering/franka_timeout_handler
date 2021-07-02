#include "../include/franka_real_time/cartesian_controller.h"
#include "../include/franka_real_time/robot.h"
#include <stdexcept>

void franka_real_time::CartesianController::_calculate_joint_torques()
{
    Eigen::Matrix<double, 6, 1> cartesian_reaction;
    cartesian_reaction.setZero();

    //Position
    Eigen::Matrix<double, 3, 1> position_error = _position - _target_position;
    cartesian_reaction.head(3) = -_translation_stiffness * position_error -_translation_damping * _velocity_rotation.head(3);

    //Orientation
    if (_control_rotation)
    {
        if (_target_orientation.coeffs().dot(_orientation.coeffs()) < 0.0) _orientation.coeffs() = -_orientation.coeffs();
        Eigen::Quaterniond orientation_error_quaternion = _orientation.inverse() * _target_orientation;
        Eigen::Matrix<double, 3, 1> orientation_error(orientation_error_quaternion.x(), orientation_error_quaternion.y(), orientation_error_quaternion.z());
        orientation_error = -_transform.linear() * orientation_error;
        cartesian_reaction.tail(3) = -_rotation_stiffness * orientation_error -_rotation_damping * _velocity_rotation.tail(3);
        _joint_torques = _jacobian.transpose() * cartesian_reaction + _coriolis;  
    }
    else
    {
        _joint_torques = _rotation_correction * (_jacobian.transpose() * cartesian_reaction + _coriolis);
    }
}

void franka_real_time::CartesianController::_control(const franka::RobotState &robot_state, franka::Torques *joint_torques_native)
{
    //Setting backend inputs
    _joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    _joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    _transform = Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data());
    _position = _transform.translation();
    _orientation = _transform.linear();
    _jacobian = Eigen::Matrix<double, 6, 7>::Map(_robot->_model->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    _velocity_rotation = _jacobian * _joint_velocities;

    //Receive
    bool front_received = false;
    bool front_resended = false;
    pthread_mutex_lock(&_mutex);
    {
        //Copying late outputs to backend outputs
        if (_late_update_timeout) { _timeout = _late_timeout; _late_update_timeout = false; }
        if (_late_update_target_position) { _target_position = _late_target_position; _late_update_target_position = false; }
        if (_late_update_target_orientation) { _target_orientation = _late_target_orientation; _late_update_target_orientation = false; }
        if (_late_update_translation_stiffness) { _translation_stiffness = _late_translation_stiffness; _late_update_translation_stiffness = false; }
        if (_late_update_rotation_stiffness) { _rotation_stiffness = _late_rotation_stiffness; _late_update_rotation_stiffness = false; }
        if (_late_update_translation_damping) { _translation_damping = _late_translation_damping; _late_update_translation_damping = false; }
        if (_late_update_rotation_damping) { _rotation_damping = _late_rotation_damping; _late_update_rotation_damping = false; }
        if (_late_update_control_rotation) { _control_rotation = _late_control_rotation; _late_update_control_rotation = false; }

        //Copying finish (it must be in receive, but receive is not done in destructor)
        joint_torques_native->motion_finished = _finish;

        if (_front_receiving)
        {
            //Copying backend inputs to frontend inputs and signaling
            front_received = true;
            _front_received = true;
            _back_receiving = false;
            _back_received = false;
            _back_timeout = false;
            _robot->_joint_positions = _joint_positions;
            _robot->_joint_velocities = _joint_velocities;
            _robot->_position = _position;
            _robot->_orientation = _orientation;
            _robot->_velocity = _velocity_rotation.head(3);
            _robot->_rotation = _velocity_rotation.tail(3);
            pthread_cond_signal(&_receive_condition);
        }
        else if (_front_resending)
        {
            //Copying backend inputs to frontend inputs, copying frontend outputs to backend outputs, copying result to frontend result, and signaling
            front_resended = true;
            _robot->_joint_positions = _joint_positions;
            _robot->_joint_velocities = _joint_velocities;
            _robot->_position = _position;
            _robot->_orientation = _orientation;
            _robot->_velocity = _velocity_rotation.head(3);
            _robot->_rotation = _velocity_rotation.tail(3);

            if (_robot->_update_timeout != Update::no) _timeout = _robot->_timeout;
            if (_robot->_update_target_position != Update::no) _target_position = _robot->_target_position;
            if (_robot->_update_target_orientation != Update::no) _target_orientation = _robot->_target_orientation;
            if (_robot->_update_translation_stiffness != Update::no) _translation_stiffness = _robot->_translation_stiffness;
            if (_robot->_update_rotation_stiffness != Update::no) _rotation_stiffness = _robot->_rotation_stiffness;
            if (_robot->_update_translation_damping != Update::no) _translation_damping = _robot->_translation_damping;
            if (_robot->_update_rotation_damping != Update::no) _rotation_damping = _robot->_rotation_damping;
            if (_robot->_update_control_rotation != Update::no) _control_rotation = _robot->_control_rotation;

            _coriolis = Eigen::Matrix<double, 7, 1>::Map(_robot->_model->coriolis(robot_state).data());
            _calculate_joint_torques();
            if (_robot->_update_joint_torques != Update::no) _robot->_joint_torques = _joint_torques;
            _robot->_late = false;

            pthread_cond_signal(&_receive_condition);
        }
    }
    pthread_mutex_unlock(&_mutex);
    
    if (!front_resended)
    {
        //Calculating everything possible without outputs
        //Only one lonely coriolis here...
        _coriolis = Eigen::Matrix<double, 7, 1>::Map(_robot->_model->coriolis(robot_state).data());

        if (!front_received)
        {
            //No send
            //Calculating
            _calculate_joint_torques();
        }
        else
        {
            //Send
            pthread_mutex_lock(&_mutex);
            {
                if (_back_received)
                {
                    //Copying frontend outputs to backend outputs
                    if (_robot->_update_timeout != Update::no) _timeout = _robot->_timeout;
                    if (_robot->_update_target_position != Update::no) _target_position = _robot->_target_position;
                    if (_robot->_update_target_orientation != Update::no) _target_orientation = _robot->_target_orientation;
                    if (_robot->_update_translation_stiffness != Update::no) _translation_stiffness = _robot->_translation_stiffness;
                    if (_robot->_update_rotation_stiffness != Update::no) _rotation_stiffness = _robot->_rotation_stiffness;
                    if (_robot->_update_translation_damping != Update::no) _translation_damping = _robot->_translation_damping;
                    if (_robot->_update_rotation_damping != Update::no) _rotation_damping = _robot->_rotation_damping;
                    if (_robot->_update_control_rotation != Update::no) _control_rotation = _robot->_control_rotation;

                    //Calculating the rest
                    _calculate_joint_torques();

                    //Setting results
                   if (_robot->_update_joint_torques != Update::no) _robot->_joint_torques = _joint_torques;
                    _robot->_late = false;

                    //Signaling
                    pthread_cond_signal(&_send_condition);
                }
                else
                {
                    _back_receiving = true;
                    timespec time;
                    time.tv_sec = 0;
                    time.tv_nsec = 1000 * _timeout;
                    pthread_cond_timedwait(&_send_condition, &_mutex, &time);

                    if (_back_received)
                    {
                        //Everything already done by frontend
                    }
                    else
                    {
                        _back_timeout = true;

                        //Calculating the rest
                        _calculate_joint_torques();

                        //Setting late results
                        _late_joint_torques = _joint_torques;
                    }
                }
            }
            pthread_mutex_unlock(&_mutex);
        }
    }
    Eigen::Matrix<double, 7, 1>::Map(&joint_torques_native->tau_J[0]) = _joint_torques;
}

Eigen::Matrix<double, 7, 7> franka_real_time::CartesianController::_pseudoinverse(const Eigen::Matrix<double, 7, 7> &a, double epsilon)
{
    Eigen::BDCSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon * std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}

franka_real_time::CartesianController::CartesianController(Robot *robot)
{
    _robot = robot;

    //Init rotation correction
    Eigen::Matrix<double, 7, 7> T;
    T.setZero();
    T.diagonal() << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0;
    _rotation_correction = Eigen::Matrix<double, 7, 7>::Identity() - T;
    //Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - _pseudoinverse(T, 1e-4) * T;
    //_rotation_correction = N * _pseudoinverse(N, 1e-4);

    //Init outputs
    const double translational_stiffness_constant = 150.0;
    const double rotational_stiffness_constant = 10.0;
    _timeout = 200;
    _target_position = _robot->_position;
    _target_orientation = _robot->_orientation;
    _translation_stiffness = translational_stiffness_constant * Eigen::Matrix<double, 3, 3>::Identity();
    _rotation_stiffness = rotational_stiffness_constant * Eigen::Matrix<double, 3, 3>::Identity();
    _translation_damping = 2.0 * sqrt(translational_stiffness_constant) * Eigen::Matrix<double, 3, 3>::Identity();
    _rotation_damping = 2.0 * sqrt(rotational_stiffness_constant) * Eigen::Matrix<double, 3, 3>::Identity();
    _control_rotation = true;
    _robot->_timeout = _timeout;
    _robot->_target_position = _target_position;
    _robot->_target_orientation = _target_orientation;
    _robot->_translation_stiffness = _translation_stiffness;
    _robot->_rotation_stiffness = _rotation_stiffness;
    _robot->_translation_damping = _translation_damping;
    _robot->_rotation_damping = _rotation_damping;
    _robot->_control_rotation = _control_rotation;

    //Technical
    if (pthread_cond_init(&_receive_condition, nullptr) != 0) throw std::runtime_error("franka_real_time: pthread_cond_init failed");
    if (pthread_cond_init(&_send_condition, nullptr) != 0) throw std::runtime_error("franka_real_time: pthread_cond_init failed");
    if (pthread_mutex_init(&_mutex, nullptr) != 0) throw std::runtime_error("franka_real_time: pthread_mutex_init failed");
    pthread_attr_t pthread_attributes;
    if (pthread_attr_init(&pthread_attributes) != 0) throw std::runtime_error("franka_real_time: pthread_attr_init failed");;
    if (pthread_attr_setschedpolicy(&pthread_attributes, SCHED_FIFO) != 0) throw std::runtime_error("franka_real_time: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 98;
    if (pthread_attr_setschedparam(&pthread_attributes, &scheduling_parameters) != 0) throw std::runtime_error("franka_real_time: pthread_attr_setschedparam failed");;;
    if (pthread_attr_setinheritsched(&pthread_attributes, PTHREAD_EXPLICIT_SCHED) != 0) throw std::runtime_error("franka_real_time: pthread_attr_setinheritsched failed");;;
    if (pthread_create(&_backend_thread, &pthread_attributes, [](void* controller) -> void*
    {
        ((CartesianController*)controller)->_robot->_robot.control([controller](const franka::RobotState &robot_state, franka::Duration duration) -> franka::Torques
        {
            franka::Torques joint_torques_native{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            ((CartesianController*)controller)->_control(robot_state, &joint_torques_native);
            return joint_torques_native;
        });
        return nullptr;
    }, this) != 0) throw std::runtime_error("franka_real_time: pthread_create failed");
}

void franka_real_time::CartesianController::receive()
{
    pthread_mutex_lock(&_mutex);
    _front_receiving = true;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _front_receiving = false;
    pthread_mutex_unlock(&_mutex);
    _front_received = true;
}

		
void franka_real_time::CartesianController::send()
{
    if (!_front_received) throw std::runtime_error("franka_real_time: CartesianControlle did not call receive()");
    _front_received = false;

    pthread_mutex_lock(&_mutex);
    {
        if (!_back_receiving)
        {
            _back_received = true;
            pthread_cond_wait(&_send_condition, &_mutex);
        }
        else if (!_back_timeout)
        {
            _back_received = true;

            //Copying frontend outputs to backend outputs
            if (_robot->_update_timeout != Update::no) _timeout = _robot->_timeout;
            if (_robot->_update_target_position != Update::no) _target_position = _robot->_target_position;
            if (_robot->_update_target_orientation != Update::no) _target_orientation = _robot->_target_orientation;
            if (_robot->_update_translation_stiffness != Update::no) _translation_stiffness = _robot->_translation_stiffness;
            if (_robot->_update_rotation_stiffness != Update::no) _rotation_stiffness = _robot->_rotation_stiffness;
            if (_robot->_update_translation_damping != Update::no) _translation_damping = _robot->_translation_damping;
            if (_robot->_update_rotation_damping != Update::no) _rotation_damping = _robot->_rotation_damping;
            if (_robot->_update_control_rotation != Update::no) _control_rotation = _robot->_control_rotation;

            //Calculating results
            _calculate_joint_torques();

            //Setting results
            if (_robot->_update_joint_torques != Update::no) _robot->_joint_torques = _joint_torques;
            _robot->_late = false;

            //Signaling
            pthread_cond_signal(&_receive_condition);
        }
        else
        {
            //Copying frontend outputs to late backend outputs
            if (_robot->_update_timeout == Update::yes) { _late_timeout = _robot->_timeout; _late_update_timeout = true; }
            if (_robot->_update_target_position == Update::yes) { _late_target_position = _robot->_target_position; _late_update_target_position = true; }
            if (_robot->_update_target_orientation == Update::yes) { _late_target_orientation = _robot->_target_orientation; _late_update_target_orientation = true; }
            if (_robot->_update_translation_stiffness == Update::yes) { _late_translation_stiffness = _robot->_translation_stiffness; _late_update_translation_stiffness = true; }
            if (_robot->_update_rotation_stiffness == Update::yes) { _late_rotation_stiffness = _robot->_rotation_stiffness; _late_update_rotation_stiffness = true; }
            if (_robot->_update_translation_damping == Update::yes) { _late_translation_damping = _robot->_translation_damping; _late_update_translation_damping = true; }
            if (_robot->_update_rotation_damping == Update::yes) { _late_rotation_damping = _robot->_rotation_damping; _late_update_rotation_damping = true; }
            if (_robot->_update_control_rotation == Update::yes) { _late_control_rotation = _robot->_control_rotation; _late_update_control_rotation = true; }

            //Copying late results
            if (_robot->_update_joint_torques == Update::yes) _robot->_joint_torques = _late_joint_torques;
            _robot->_late = true;
        }
    }
    pthread_mutex_unlock(&_mutex);
}

void franka_real_time::CartesianController::receive_and_send()
{
    pthread_mutex_lock(&_mutex);
    _front_resending = true;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _front_resending = false;
    pthread_mutex_unlock(&_mutex);
    _front_received = false;
}

void franka_real_time::CartesianController::send_and_receive()
{
    send();
    receive();
}

franka_real_time::CartesianController::~CartesianController()
{
    _finish = true;
    pthread_join(_backend_thread, nullptr);
    pthread_cond_destroy(&_receive_condition);
    pthread_cond_destroy(&_send_condition);
    pthread_mutex_destroy(&_mutex);
}