#include "../include/franka_timeout_handler/robot_core.h"
#include "../include/franka_timeout_handler/cartesian_controller.h"
#include "../include/franka_timeout_handler/joint_controller.h"
#include <stdexcept>

franka_timeout_handler::RobotCore::RobotCore(std::string ip) : _robot(ip)
{
    _robot.automaticErrorRecovery();
    _model = new franka::Model(_robot.loadModel());
    receive();
    set_target_position(get_position());
    set_target_orientation(get_orientation());
    set_target_joint_positions(get_joint_positions());
}

void franka_timeout_handler::RobotCore::start(ControllerType typ)
{
    stop();
    if (typ == ControllerType::cartesian) _controller = new CartesianController;
    else _controller = new JointController;
    _controller->start(this);
}

void franka_timeout_handler::RobotCore::stop()
{
    if (_controller != nullptr)
    {
        _controller->stop();
        delete _controller;
        _controller = nullptr;
    }
}

bool franka_timeout_handler::RobotCore::started() const
{
    return _controller != nullptr;
}

franka_timeout_handler::ControllerType franka_timeout_handler::RobotCore::typ() const
{
    if (!started()) throw std::runtime_error("franka_timeout_handler: controller was not started");
    return _controller->typ();
}

void franka_timeout_handler::RobotCore::receive()
{
    if (!started())
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
        _call = 0;
    }
    else _controller->receive();
}

void franka_timeout_handler::RobotCore::send()
{
    if (!started()) throw std::runtime_error("franka_timeout_handler: controller was not started");
    _controller->send();
}

void franka_timeout_handler::RobotCore::receive_and_send()
{
    if (!started()) throw std::runtime_error("franka_timeout_handler: controller was not started");
    _controller->receive_and_send();
}

void franka_timeout_handler::RobotCore::send_and_receive()
{
    if (!started()) throw std::runtime_error("franka_timeout_handler: controller was not started");
    _controller->send_and_receive();
}

Eigen::Matrix<double, 7, 1> franka_timeout_handler::RobotCore::get_joint_positions() const
{
    return _joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_timeout_handler::RobotCore::get_joint_velocities() const
{
    return _joint_velocities;
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_position() const
{
    return _position;
}

Eigen::Quaterniond franka_timeout_handler::RobotCore::get_orientation() const
{
    return _orientation;
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_orientation_euler() const
{
    return _orientation.toRotationMatrix().eulerAngles(2, 1, 0);
}

Eigen::Matrix<double, 4, 1> franka_timeout_handler::RobotCore::get_orientation_wxyz() const
{
    return Eigen::Matrix<double, 4, 1> (_orientation.w(), _orientation.x(), _orientation.y(), _orientation.z());
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_velocity() const
{
    return _velocity;
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_rotation() const
{
    return _rotation;
}


std::uint64_t franka_timeout_handler::RobotCore::get_call() const
{
    return _call;
}

void franka_timeout_handler::RobotCore::set_timeout(unsigned int timeout)
{
    _timeout = timeout;
}

void franka_timeout_handler::RobotCore::set_joint_torques_limit(double limit)
{
    _joint_torques_limit = limit;
}

void franka_timeout_handler::RobotCore::set_frequency_divider(unsigned int divider)
{
    _frequency_divider = divider;
}

void franka_timeout_handler::RobotCore::set_target_position(const Eigen::Matrix<double, 3, 1> &position)
{
    _target_position = position;
}

void franka_timeout_handler::RobotCore::set_target_orientation(const Eigen::Quaterniond &orientation)
{
    _target_orientation = orientation;
}

void franka_timeout_handler::RobotCore::set_target_orientation_euler(const Eigen::Matrix<double, 3, 1> &euler)
{
    _target_orientation = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
}

void franka_timeout_handler::RobotCore::set_target_orientation_wxyz(const Eigen::Matrix<double, 4, 1> &wxyz)
{
    _target_orientation = Eigen::Quaterniond(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
}

void franka_timeout_handler::RobotCore::set_target_velocity(const Eigen::Matrix<double, 3, 1> &velocity)
{
    _target_velocity = velocity;
}

void franka_timeout_handler::RobotCore::set_target_rotation(const Eigen::Matrix<double, 3, 1> &rotation)
{
    _target_rotation = rotation;
}

void franka_timeout_handler::RobotCore::set_translation_stiffness(const Eigen::Matrix<double, 3, 3> &stiffness)
{
    _translation_stiffness = stiffness;
}

void franka_timeout_handler::RobotCore::set_rotation_stiffness(const Eigen::Matrix<double, 3, 3> &stiffness)
{
    _rotation_stiffness = stiffness;
}

void franka_timeout_handler::RobotCore::set_translation_damping(const Eigen::Matrix<double, 3, 3> &damping)
{
    _translation_damping = damping;
}

void franka_timeout_handler::RobotCore::set_rotation_damping(const Eigen::Matrix<double, 3, 3> &damping)
{
    _rotation_damping = damping;
}

void franka_timeout_handler::RobotCore::set_control_rotation(bool control)
{
    _control_rotation = control;
}

void franka_timeout_handler::RobotCore::set_target_joint_positions(const Eigen::Matrix<double, 7, 1> &positions)
{
    _target_joint_positions = positions;
}

void franka_timeout_handler::RobotCore::set_target_joint_velocities(const Eigen::Matrix<double, 7, 1> &velocities)
{
    _target_joint_velocities = velocities;
}

void franka_timeout_handler::RobotCore::set_joint_stiffness(const Eigen::Matrix<double, 7, 7> &stiffness)
{
    _joint_stiffness = stiffness;
}

void franka_timeout_handler::RobotCore::set_joint_damping(const Eigen::Matrix<double, 7, 7> &damping)
{
    _joint_damping = damping;
}

unsigned int franka_timeout_handler::RobotCore::get_timeout() const
{
    return _timeout;
}

double franka_timeout_handler::RobotCore::get_joint_torques_limit() const
{
    return _joint_torques_limit;
}

unsigned int franka_timeout_handler::RobotCore::get_frequency_divider() const
{
    return _frequency_divider;
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_target_position() const
{
    return _target_position;
}

Eigen::Quaterniond franka_timeout_handler::RobotCore::get_target_orientation() const
{
    return _target_orientation;
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_target_orientation_euler() const
{
    return _target_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
}

Eigen::Matrix<double, 4, 1> franka_timeout_handler::RobotCore::get_target_orientation_wxyz() const
{
    return Eigen::Matrix<double, 4, 1> (_target_orientation.w(), _target_orientation.x(), _target_orientation.y(), _target_orientation.z());
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_target_velocity() const
{
    return _target_velocity;
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::RobotCore::get_target_rotation() const
{
    return _target_rotation;
}

Eigen::Matrix<double, 3, 3> franka_timeout_handler::RobotCore::get_translation_stiffness() const
{
    return _translation_stiffness;
}

Eigen::Matrix<double, 3, 3> franka_timeout_handler::RobotCore::get_rotation_stiffness() const
{
    return _translation_stiffness;
}

Eigen::Matrix<double, 3, 3> franka_timeout_handler::RobotCore::get_translation_damping() const
{
    return _translation_damping;
}

Eigen::Matrix<double, 3, 3> franka_timeout_handler::RobotCore::get_rotation_damping() const
{
    return _rotation_damping;
}

bool franka_timeout_handler::RobotCore::get_control_rotation() const
{
    return _control_rotation;
}

Eigen::Matrix<double, 7, 1> franka_timeout_handler::RobotCore::get_target_joint_positions() const
{
    return _target_joint_positions;
}

Eigen::Matrix<double, 7, 1> franka_timeout_handler::RobotCore::get_target_joint_velocities() const
{
    return _target_joint_velocities;
}

Eigen::Matrix<double, 7, 7> franka_timeout_handler::RobotCore::get_joint_stiffness() const
{
    return _joint_stiffness;
}

Eigen::Matrix<double, 7, 7> franka_timeout_handler::RobotCore::get_joint_damping() const
{
    return _joint_damping;
}

void franka_timeout_handler::RobotCore::set_timeout_update(Update update)
{
    _update_timeout = update;
}

void franka_timeout_handler::RobotCore::set_joint_torques_limit_update(Update update)
{
    _update_joint_torques_limit = update;
}

void franka_timeout_handler::RobotCore::set_frequency_divider_update(Update update)
{
    _update_frequency_divider = update;
}

void franka_timeout_handler::RobotCore::set_target_position_update(Update update)
{
    _update_target_position = update;
}

void franka_timeout_handler::RobotCore::set_target_orientation_update(Update update)
{
    _update_target_orientation = update;
}

void franka_timeout_handler::RobotCore::set_target_velocity_update(Update update)
{
    _update_target_velocity = update;
}

void franka_timeout_handler::RobotCore::set_target_rotation_update(Update update)
{
    _update_target_rotation = update;
}

void franka_timeout_handler::RobotCore::set_translation_stiffness_update(Update update)
{
    _update_translation_stiffness = update;
}

void franka_timeout_handler::RobotCore::set_rotation_stiffness_update(Update update)
{
    _update_rotation_stiffness = update;
}

void franka_timeout_handler::RobotCore::set_translation_damping_update(Update update)
{
    _update_translation_damping = update;
}

void franka_timeout_handler::RobotCore::set_rotation_damping_update(Update update)
{
    _update_rotation_damping = update;
}

void franka_timeout_handler::RobotCore::set_control_rotation_update(Update update)
{
    _update_control_rotation = update;
}

void franka_timeout_handler::RobotCore::set_target_joint_positions_update(Update update)
{
    _update_target_joint_positions = update;
}

void franka_timeout_handler::RobotCore::set_target_joint_velocities_update(Update update)
{
    _update_target_joint_velocities = update;
}

void franka_timeout_handler::RobotCore::set_joint_stiffness_update(Update update)
{
    _update_joint_stiffness = update;
}

void franka_timeout_handler::RobotCore::set_joint_damping_update(Update update)
{
    _update_joint_damping = update;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_timeout_update() const
{
    return _update_timeout;
}

bool franka_timeout_handler::RobotCore::get_late() const
{
    return _late;
}

void franka_timeout_handler::RobotCore::set_joint_torques_update(Update update)
{
    _update_joint_torques = update;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_target_position_update() const
{
    return _update_target_position;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_target_orientation_update() const
{
    return _update_target_orientation;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_target_velocity_update() const
{
    return _update_target_velocity;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_target_rotation_update() const
{
    return _update_target_rotation;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_translation_stiffness_update() const
{
    return _update_translation_stiffness;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_rotation_stiffness_update() const
{
    return _update_rotation_stiffness;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_translation_damping_update() const
{
    return _update_translation_damping;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_rotation_damping_update() const
{
    return _update_rotation_damping;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_control_rotation_update() const
{
    return _update_control_rotation;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_joint_torques_limit_update() const
{
    return _update_joint_torques_limit;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_frequency_divider_update() const
{
    return _update_frequency_divider;
}

Eigen::Matrix<double, 7, 1> franka_timeout_handler::RobotCore::get_joint_torques() const
{
    return _joint_torques;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_target_joint_positions_update() const
{
    return _update_target_joint_positions;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_target_joint_velocities_update() const
{
    return _update_target_joint_velocities;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_joint_stiffness_update() const
{
    return _update_joint_stiffness;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_joint_damping_update() const
{
    return _update_joint_damping;
}

franka_timeout_handler::Update franka_timeout_handler::RobotCore::get_joint_torques_update() const
{
    return _update_joint_torques;
}

franka_timeout_handler::RobotCore::~RobotCore()
{
    stop();
}