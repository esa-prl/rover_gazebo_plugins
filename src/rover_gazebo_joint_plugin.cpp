#include <rover_gazebo_plugins/rover_gazebo_joint_plugin.hpp>

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rover_msgs/msg/joint_command.hpp>
#include <rover_msgs/msg/joint_command_array.hpp>

#include <ignition/math/Rand.hh>

namespace gazebo_plugins
{
class RoverGazeboJointPluginPrivate
{
public:
    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// Callback when a joint command array is received
    /// \param[in] _msg Array of joint commands.
    void OnJointCmdArray(rover_msgs::msg::JointCommandArray::SharedPtr _msg);

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Subscriber to joint command arrays
    rclcpp::Subscription<rover_msgs::msg::JointCommandArray>::SharedPtr joint_cmd_array_sub_;

    /// Subscriber to joint commands
    rclcpp::Subscription<rover_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    /// Pointer to model
    gazebo::physics::ModelPtr model_;

    /// Update period in seconds.
    double update_period_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// Protect variables accessed on callbacks.
    std::mutex lock_;

    /// Joint controller that is managing controllers of all model joints
    gazebo::physics::JointControllerPtr joint_controller_;
};

RoverGazeboJointPlugin::RoverGazeboJointPlugin()
    : impl_(std::make_unique<RoverGazeboJointPluginPrivate>())
{
}

RoverGazeboJointPlugin::~RoverGazeboJointPlugin()
{
}

void RoverGazeboJointPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Get the model
    impl_->model_ = _model;

    // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get the joint controller of the model
    impl_->joint_controller_ = impl_->model_->GetJointController();

    // Reset all the joints to a zero position
    // TODO:
    // - DO NOT RELY ON THE NAME

    ignition::math::Vector3d pid_deploy;
    if (_sdf->HasElement("pid_deploy"))
    {
        pid_deploy = _sdf->GetElement("pid_deploy")->Get<ignition::math::Vector3d>();
    }
    else
    {
        pid_deploy = ignition::math::Vector3d(50.0, 0.1, 0.0);
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "rover_gazebo_joint_plugin missing parameter pid_deploy. Defaults to P: %f I: %f D: %f", pid_deploy.X(), pid_deploy.Y(), pid_deploy.Z());
    }

    ignition::math::Vector3d pid_drive;
    if (_sdf->HasElement("pid_drive"))
    {
        pid_drive = _sdf->GetElement("pid_drive")->Get<ignition::math::Vector3d>();
    }
    else
    {
        pid_drive = ignition::math::Vector3d(5.0, 0.1, 0.0);
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "rover_gazebo_joint_plugin missing parameter pid_drive. Defaults to P: %f I: %f D: %f", pid_drive.X(), pid_drive.Y(), pid_drive.Z());
    }

    ignition::math::Vector3d pid_steer;
    if (_sdf->HasElement("pid_steer"))
    {
        pid_steer = _sdf->GetElement("pid_steer")->Get<ignition::math::Vector3d>();
    }
    else
    {
        pid_steer = ignition::math::Vector3d(5.0, 0.1, 0.0);
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "rover_gazebo_joint_plugin missing parameter pid_steer. Defaults to P: %f I: %f D: %f", pid_steer.X(), pid_steer.Y(), pid_steer.Z());
    }

    for (auto const &pair : impl_->joint_controller_->GetJoints())
    {
        auto name = pair.first;
        auto joint = pair.second;

        if (name.find("DEP") != std::string::npos)
        {
            impl_->joint_controller_->SetPositionPID(name, gazebo::common::PID(pid_deploy.X(), pid_deploy.Y(), pid_deploy.Z()));
            impl_->joint_controller_->SetPositionTarget(name, 0.0);
            RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "PID DEP set on joint: %s to P: %f I: %f D: %f", name.c_str(), pid_deploy.X(), pid_deploy.Y(), pid_deploy.Z());
        }
        else if (name.find("DRV") != std::string::npos)
        {
            impl_->joint_controller_->SetVelocityPID(name, gazebo::common::PID(pid_drive.X(), pid_drive.Y(), pid_drive.Z()));
            impl_->joint_controller_->SetVelocityTarget(name, 0.0);
            RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "PID DRV set on joint: %s", name.c_str(), pid_drive.X(), pid_drive.Y(), pid_drive.Z());
        }
        else if (name.find("STR") != std::string::npos)
        {
            impl_->joint_controller_->SetPositionPID(name, gazebo::common::PID(pid_steer.X(), pid_steer.Y(), pid_steer.Z()));
            impl_->joint_controller_->SetPositionTarget(name, 0.0);
            RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "PID STR set on joint: %s", name.c_str(), pid_steer.X(), pid_steer.Y(), pid_steer.Z());
        }
    }

    // Update rate
    auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
    if (update_rate > 0.0)
    {
        impl_->update_period_ = 1.0 / update_rate;
    }
    else
    {
        impl_->update_period_ = 0.0;
    }
    impl_->last_update_time_ = _model->GetWorld()->SimTime();

    // Subscribe to joint command array topic
    impl_->joint_cmd_array_sub_ = impl_->ros_node_->create_subscription<rover_msgs::msg::JointCommandArray>(
        "joint_cmds", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&RoverGazeboJointPluginPrivate::OnJointCmdArray, impl_.get(), std::placeholders::_1));

    // Listen to the update event (broadcast every simulation iteration)
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&RoverGazeboJointPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void RoverGazeboJointPlugin::Reset()
{
    impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
    impl_->joint_controller_->Reset();
}

void RoverGazeboJointPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    std::lock_guard<std::mutex> lock(lock_);

    // Check for update constraints
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    if (seconds_since_last_update < update_period_)
    {
        return;
    }

    // Update all joint controllers
    joint_controller_->Update();
}

void RoverGazeboJointPluginPrivate::OnJointCmdArray(rover_msgs::msg::JointCommandArray::SharedPtr _msg)
{
    std::lock_guard<std::mutex> scoped_lock(lock_);

    // Iterate over received joint command array and set controller targets
    for (auto const cmd : _msg->joint_command_array)
    {
        if (cmd.mode == "POSITION")
        {
            if (!joint_controller_->SetPositionTarget(model_->GetJoint(cmd.name)->GetScopedName(), cmd.value))
            {
                RCLCPP_WARN(ros_node_->get_logger(), "Joint %s from recieved command was npt found in model.", cmd.name.c_str());
            }
        }
        else if (cmd.mode == "VELOCITY")
        {
            if (!joint_controller_->SetVelocityTarget(model_->GetJoint(cmd.name)->GetScopedName(), cmd.value))
            {
                RCLCPP_WARN(ros_node_->get_logger(), "Joint %s from received command was not found in model.", cmd.name.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(ros_node_->get_logger(), "Undefined mode in joint command received: %s", cmd.mode.c_str());
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(RoverGazeboJointPlugin)
} // namespace gazebo_plugins
