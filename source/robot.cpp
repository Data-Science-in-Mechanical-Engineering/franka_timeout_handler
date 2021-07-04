#include "../include/franka_real_time/robot.h"
#include <stdexcept>

franka_real_time::Robot::Robot(std::string ip) : _robot(ip)
{
    _robot.automaticErrorRecovery();
    _model = new franka::Model(_robot.loadModel());
    
    //Initializing input
    receive();

    //Initializing output
    set_default();

    //Initializing result
    _joint_torques.setZero();
    _late = false;
}

void franka_real_time::Robot::start()
{
    if (_controller != nullptr) throw std::runtime_error("franka_real_time: controller was already started");
    _controller = new CartesianController(this);
}

void franka_real_time::Robot::stop()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: controller was not started");
    delete _controller;
    _controller = nullptr;
}

void franka_real_time::Robot::receive()
{
    if (_controller == nullptr)
    {
        franka::RobotState robot_state = _robot.readOnce();
        Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(robot_state.O_T_EE.data()));
        Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Map(_model->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
        _joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
        _joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
        _position = transform.translation();
        _orientation = transform.linear();
        Eigen::Matrix<double, 6, 1> velocity_rotation = jacobian * _joint_velocities;
        _velocity = velocity_rotation.head(3);
        _rotation = velocity_rotation.tail(3);
    }
    else _controller->receive();
}

void franka_real_time::Robot::send()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: controller was not started");
    _controller->send();
}

void franka_real_time::Robot::receive_and_send()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: controller was not started");
    _controller->receive_and_send();
}

void franka_real_time::Robot::send_and_receive()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: controller was not started");
    _controller->send_and_receive();
}

Eigen::Matrix<double, 7, 1> franka_real_time::Robot::get_joint_positions() const
{
    return _joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_real_time::Robot::get_joint_velocities() const
{
    return _joint_velocities;
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_position() const
{
    return _position;
}

Eigen::Quaterniond franka_real_time::Robot::get_orientation() const
{
    return _orientation;
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_orientation_euler() const
{
    return _orientation.toRotationMatrix().eulerAngles(0, 1, 2);
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_velocity() const
{
    return _velocity;
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_rotation() const
{
    return _rotation;
}

void franka_real_time::Robot::set_timeout(unsigned int timeout)
{
    _timeout = timeout;
}

void franka_real_time::Robot::set_target_position(Eigen::Matrix<double, 3, 1> position)
{
    _position = position;
}

void franka_real_time::Robot::set_target_orientation(Eigen::Quaterniond orientation)
{
    _orientation = orientation;
}

void franka_real_time::Robot::set_target_orientation_euler(Eigen::Matrix<double, 3, 1> euler)
{
    _orientation = Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX());
}

void franka_real_time::Robot::set_translation_stiffness(Eigen::Matrix<double, 3, 3> stiffness)
{
    _translation_stiffness = stiffness;
}

void franka_real_time::Robot::set_rotation_stiffness(Eigen::Matrix<double, 3, 3> stiffness)
{
    _rotation_stiffness = stiffness;
}

void franka_real_time::Robot::set_translation_damping(Eigen::Matrix<double, 3, 3> damping)
{
    _translation_damping = damping;
}

void franka_real_time::Robot::set_rotation_damping(Eigen::Matrix<double, 3, 3> damping)
{
    _rotation_damping = damping;
}

void franka_real_time::Robot::set_control_rotation(bool control)
{
    _control_rotation = control;
}

void franka_real_time::Robot::set_joint_torques_limit(double limit)
{
    _joint_torques_limit = limit;
}

void franka_real_time::Robot::set_frequency_divider(unsigned int divider)
{
    _frequency_divider = divider;
}

unsigned int franka_real_time::Robot::get_timeout() const
{
    return _timeout;
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_target_position() const
{
    return _target_position;
}

Eigen::Quaterniond franka_real_time::Robot::get_target_orientation() const
{
    return _target_orientation;
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_target_orientation_euler() const
{
    return _target_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
}

Eigen::Matrix<double, 3, 3> franka_real_time::Robot::get_translation_stiffness() const
{
    return _translation_stiffness;
}

Eigen::Matrix<double, 3, 3> franka_real_time::Robot::get_rotation_stiffness() const
{
    return _translation_stiffness;
}

Eigen::Matrix<double, 3, 3> franka_real_time::Robot::get_translation_damping() const
{
    return _translation_damping;
}

Eigen::Matrix<double, 3, 3> franka_real_time::Robot::get_rotation_damping() const
{
    return _rotation_damping;
}

bool franka_real_time::Robot::get_control_rotation() const
{
    return _control_rotation;
}

double franka_real_time::Robot::get_joint_torques_limit() const
{
    return _joint_torques_limit;
}

unsigned int franka_real_time::Robot::get_frequency_divider() const
{
    return _frequency_divider;
}

void franka_real_time::Robot::set_timeout_update(Update update)
{
    _update_timeout = update;
}

void franka_real_time::Robot::set_target_position_update(Update update)
{
    _update_target_position = update;
}

void franka_real_time::Robot::set_target_orientation_update(Update update)
{
    _update_target_orientation = update;
}

void franka_real_time::Robot::set_translation_stiffness_update(Update update)
{
    _update_translation_stiffness = update;
}

void franka_real_time::Robot::set_rotation_stiffness_update(Update update)
{
    _update_rotation_stiffness = update;
}

void franka_real_time::Robot::set_translation_damping_update(Update update)
{
    _update_translation_damping = update;
}

void franka_real_time::Robot::set_rotation_damping_update(Update update)
{
    _update_rotation_damping = update;
}

void franka_real_time::Robot::set_control_rotation_update(Update update)
{
    _update_control_rotation = update;
}

void franka_real_time::Robot::set_joint_torques_limit_update(Update update)
{
    _update_joint_torques_limit = update;
}

void franka_real_time::Robot::set_frequency_divider_update(Update update)
{
    _update_frequency_divider = update;
}

franka_real_time::Update franka_real_time::Robot::get_timeout_update() const
{
    return _update_timeout;
}

franka_real_time::Update franka_real_time::Robot::get_target_position_update() const
{
    return _update_target_position;
}

franka_real_time::Update franka_real_time::Robot::get_target_orientation_update() const
{
    return _update_target_orientation;
}

franka_real_time::Update franka_real_time::Robot::get_translation_stiffness_update() const
{
    return _update_translation_stiffness;
}

franka_real_time::Update franka_real_time::Robot::get_rotation_stiffness_update() const
{
    return _update_rotation_stiffness;
}

franka_real_time::Update franka_real_time::Robot::get_translation_damping_update() const
{
    return _update_translation_damping;
}

franka_real_time::Update franka_real_time::Robot::get_rotation_damping_update() const
{
    return _update_rotation_damping;
}

franka_real_time::Update franka_real_time::Robot::get_control_rotation_update() const
{
    return _update_control_rotation;
}

franka_real_time::Update franka_real_time::Robot::get_joint_torques_limit_update() const
{
    return _update_joint_torques_limit;
}

franka_real_time::Update franka_real_time::Robot::get_frequency_divider_update() const
{
    return _update_frequency_divider;
}

Eigen::Matrix<double, 7, 1> franka_real_time::Robot::get_joint_torques() const
{
    return _joint_torques;
}

bool franka_real_time::Robot::get_late() const
{
    return _late;
}

void franka_real_time::Robot::set_joint_torques_update(Update update)
{
    _update_joint_torques = update;
}

franka_real_time::Update franka_real_time::Robot::get_joint_torques_update() const
{
    return _update_joint_torques;
}

void franka_real_time::Robot::set_update(Update update)
{
    _update_timeout = update;
    _update_target_position = update;
    _update_translation_stiffness = update;
    _update_rotation_stiffness = update;
    _update_translation_damping = update;
    _update_rotation_damping = update;
    _update_control_rotation = update;
    _update_joint_torques_limit = update;
    _update_frequency_divider = update;
    _update_joint_torques = update;
}

void franka_real_time::Robot::set_default()
{
    receive(); //For position and orientation
    const double translational_stiffness_constant = 100.0;
    const double rotational_stiffness_constant = 10.0;
    _timeout = 200;
    _target_position = _position;
    _target_orientation = _orientation;
    _translation_stiffness = translational_stiffness_constant * Eigen::Matrix<double, 3, 3>::Identity();
    _rotation_stiffness = rotational_stiffness_constant * Eigen::Matrix<double, 3, 3>::Identity();
    _translation_damping = 2.0 * sqrt(translational_stiffness_constant) * Eigen::Matrix<double, 3, 3>::Identity();
    _rotation_damping = 2.0 * sqrt(rotational_stiffness_constant) * Eigen::Matrix<double, 3, 3>::Identity();
    _control_rotation = false;
    _joint_torques_limit = 1.0;
    _frequency_divider = 1;
}

double franka_real_time::Robot::distance() const
{
    Eigen::Matrix<double, 6, 1> error;
    error.setZero();

    //Position
    error.head(3) = _position - _target_position;

    //Orientation
    if (_control_rotation)
    {
        Eigen::Quaterniond orient = _orientation;
        if (_target_orientation.coeffs().dot(orient.coeffs()) < 0.0) orient.coeffs() = -orient.coeffs();
        Eigen::Quaterniond orientation_error = orient.inverse() * _target_orientation;
        error.tail(3) << orientation_error.x(), orientation_error.y(), orientation_error.z();
    }

    return error.norm();
}

void franka_real_time::Robot::loop(double tolerance, unsigned int iterations)
{
    receive_and_send();
    for (unsigned int i = 0; i < iterations; i++)
    {
        if (distance() <= tolerance) break;
        receive();
    }
}

franka_real_time::Robot::~Robot()
{
    if (_controller != nullptr) delete _controller;
}