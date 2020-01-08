#include <rover_gazebo_plugins/rover_gazebo_joint_plugin.hpp>

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_ros/node.hpp>

#include <rover_msgs/msg/joint_command.hpp>
#include <rover_msgs/msg/joint_command_array.hpp>

namespace gazebo_plugins
{
class RoverGazeboJointPluginPrivate
{
public:
    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// Callback when a joint command is received
    /// \param[in] _msg Array of joint commands.
    void OnJointCmd(rover_msgs::msg::JointCommandArray::SharedPtr _msg);

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Subscriber to joint commands
    rclcpp::Subscription<rover_msgs::msg::JointCommandArray>::SharedPtr cmd_joint_sub_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    /// Pointers to joints.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Pointer to model
    gazebo::physics::ModelPtr model_;

    /// Update period in seconds.
    double update_period_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// PID control for right steering control
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
    impl_->model_ = _model;

    auto world = impl_->model_->GetWorld();
    auto physicsEngine = world->Physics();
    physicsEngine->SetParam("friction_model", std::string("cone_model"));

    // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get all the joints in the model
    impl_->joints_ = impl_->model_->GetJoints();

    for (auto joint : impl_->joints_)
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), joint->GetName());
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

    // Listen to the update event (broadcast every simulation iteration)
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&RoverGazeboJointPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void RoverGazeboJointPlugin::Reset()
{
}

void RoverGazeboJointPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    // joints_[RoverGazeboJointPluginPrivate::DRIVE_LEFT_FRONT]->SetPosition(0, 30);
    // joints_[RoverGazeboJointPluginPrivate::DRIVE_LEFT_FRONT]->SetVelocity(0, 100);
    // joints_[RoverGazeboJointPluginPrivate::DRIVE_RIGHT_FRONT]->SetVelocity(0, 100);

    // joints_[RoverGazeboJointPluginPrivate::DEPLOY_LEFT_FRONT]->SetPosition(0, 0);
    // joints_[RoverGazeboJointPluginPrivate::DEPLOY_RIGHT_FRONT]->SetPosition(0, 0);
}

void RoverGazeboJointPluginPrivate::OnJointCmd(rover_msgs::msg::JointCommandArray::SharedPtr _msg)
{
    for (auto cmd : _msg->joint_command_array)
    {
        // Find joint in list of joints
        auto joint = std::find_if(joints_.begin(), joints_.end(),
                                  [cmd](const gazebo::physics::JointPtr &m) -> bool { return m->GetName() == cmd.name; });
    }
}

GZ_REGISTER_MODEL_PLUGIN(RoverGazeboJointPlugin)
} // namespace gazebo_plugins
