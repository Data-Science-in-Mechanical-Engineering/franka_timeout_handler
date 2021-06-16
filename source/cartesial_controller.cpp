#include "../include/franka_real_time/cartesial_controller.h"
#include <stdexcept>

void franka_real_time::CartesialController::_calculate_joint_torques()
{
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << _position - _target_position;
    if (_target_orientation.coeffs().dot(_orientation.coeffs()) < 0.0) _orientation.coeffs() << -_orientation.coeffs();
    Eigen::Quaterniond orientation_error;
    orientation_error = _orientation.inverse() * _target_orientation;
    error.tail(3) << orientation_error.x(), orientation_error.y(), orientation_error.z();
    error.tail(3) << -_transform.linear() * error.tail(3);
    _joint_torques << _jacobian.transpose() * (-_stiffness * error - _damping * _velocity) + _coriolis;  
}

void franka_real_time::CartesialController::_control(const franka::RobotState &robot_state, franka::Torques *joint_torques_native)
{
    //Setting backend inputs
    _joint_positions << Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    _joint_velocities << Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    _transform = Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data());
    _position << _transform.translation();
    _orientation = _transform.linear();

    //Receive
    bool front_received = false;
    pthread_mutex_lock(&_mutex);
    {
        //Copying late outputs to backend outputs
        if (_late_timeout_update) { _timeout = _late_timeout; _late_timeout_update = false; }
        if (_late_target_position_update) { _target_position = _late_target_position; _late_target_position_update = false; }
        if (_late_target_orientation_update) { _target_orientation = _late_target_orientation; _late_target_orientation_update = false; }
        if (_late_stiffness_update) { _stiffness = _late_stiffness; _late_stiffness_update = false; }
        if (_late_damping_update) { _damping = _late_damping; _late_damping_update = false; }

        //Copying finish (it must be in receive, but receive is not done in destructor)
        joint_torques_native->motion_finished = _finish;

        //Copying backend inputs to frontend inputs and signaling
        if (_front_receiving)
        {
            front_received = true;
            _back_receiving = false;
            _back_received = false;
            _back_timeout = false;
            position = _position;
            orientation = _orientation;
            joint_positions = _joint_positions;
            joint_velocities = _joint_velocities;
            pthread_cond_signal(&_receive_condition);
        }
    }
    pthread_mutex_unlock(&_mutex);
        
    //Calculating everything possible without outputs
    _coriolis << Eigen::Matrix<double, 7, 1>::Map(_get_model()->coriolis(robot_state).data());
    _jacobian << Eigen::Matrix<double, 6, 7>::Map(_get_model()->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    _velocity << _jacobian * _joint_velocities;

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
                if (timeout_update != Update::no) _timeout = timeout;
                if (target_position_update != Update::no) _target_position = target_position;
                if (target_orientation_update != Update::no) _target_orientation = target_orientation;
                if (stiffness_update != Update::no) _stiffness = stiffness;
                if (damping_update != Update::no) _damping = damping;

                //Calculating the rest
                _calculate_joint_torques();

                //Setting results
                if (joint_torques_update != Update::no) joint_torques = _joint_torques;
                late = false;

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
    Eigen::Matrix<double, 7, 1>::Map(&joint_torques_native->tau_J[0]) = _joint_torques;
}

void franka_real_time::CartesialController::_control_thread_function(CartesialController *controller)
{
    controller->_get_robot()->control([controller](const franka::RobotState &robot_state, franka::Duration duration) -> franka::Torques
    {
        franka::Torques joint_torques_native{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        controller->_control(robot_state, &joint_torques_native);
        return joint_torques_native;
    });
}

franka_real_time::CartesialController::CartesialController(Robot &robot)
{
    _robot = &robot;
    _set_controller(&robot, this);

    //Init inputs
    franka::RobotState robot_state = _get_robot()->readOnce();
    joint_positions << Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    joint_velocities << Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data()));
    position << transform.translation();
    orientation = transform.linear();

    //Init outputs
    timeout = 100;
    _timeout = timeout;
    target_position << position;
    _target_position << position;
    target_orientation = orientation;
    _target_orientation = orientation;
    const double translational_stiffness{150.0};
    const double rotational_stiffness{10.0};
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::Matrix<double, 3, 3>::Identity();
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::Matrix<double, 3, 3>::Identity();
    _stiffness << stiffness;
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix<double, 3, 3>::Identity();
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix<double, 3, 3>::Identity();
    _damping << damping;

    //Technical
    if (pthread_cond_init(&_receive_condition, nullptr) < 0) throw std::runtime_error("franka_real_time: pthread_cond_init failed");
    if (pthread_cond_init(&_send_condition, nullptr) < 0) throw std::runtime_error("franka_real_time: pthread_cond_init failed");
    if (pthread_mutex_init(&_mutex, nullptr) < 0) throw std::runtime_error("franka_real_time: pthread_mutex_init failed");
    _backend_thread = std::thread(_control_thread_function, this);
}

void franka_real_time::CartesialController::update(Update upd)
{
    timeout_update = upd;
    target_position_update = upd;
    stiffness_update = upd;
    damping_update = upd;
    joint_torques_update = upd;
}

void franka_real_time::CartesialController::receive()
{
    pthread_mutex_lock(&_mutex);
    _front_receiving = true;
    pthread_cond_wait(&_receive_condition, &_mutex);
    _front_receiving = false;
    pthread_mutex_unlock(&_mutex);
    _front_received = true;
}

		
void franka_real_time::CartesialController::send()
{
    if (!_front_received) throw std::runtime_error("franka_real_time: CartesialControlle did not call receive()");
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
            if (timeout_update != Update::no) _timeout = timeout;
            if (target_position_update != Update::no) _target_position = target_position;
            if (target_orientation_update != Update::no) _target_orientation = target_orientation;
            if (stiffness_update != Update::no) _stiffness = stiffness;
            if (damping_update != Update::no) _damping = damping;

            //Calculating results
            _calculate_joint_torques();

            //Setting results
            if (joint_torques_update != Update::no) joint_torques = _joint_torques;
            late = false;

            //Signaling
            pthread_cond_signal(&_receive_condition);
        }
        else
        {
            //Copying frontend outputs to late backend outputs
            if (timeout_update == Update::yes) { _late_timeout = timeout; _late_timeout_update = true; }
            if (target_position_update == Update::yes) { _late_target_position = target_position; _late_target_position_update = true; }
            if (target_orientation_update == Update::yes) { _late_target_orientation = target_orientation; _late_target_orientation_update = true; }
            if (stiffness_update == Update::yes) { _late_stiffness = stiffness; _late_stiffness_update = true; }
            if (damping_update == Update::yes) { _late_damping = damping; _late_damping_update = true; }

            //Copying late results
            if (joint_torques_update == Update::yes) joint_torques = _late_joint_torques;
            late = true;
        }
    }
    pthread_mutex_unlock(&_mutex);
}

franka_real_time::CartesialController::~CartesialController()
{
    _finish = true;
    _backend_thread.join();
    pthread_cond_destroy(&_receive_condition);
    pthread_cond_destroy(&_send_condition);
    pthread_mutex_destroy(&_mutex);
    _set_controller(_robot, nullptr);
}