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
    /// Indicates which joint
    enum
    {
        DRIVE_LEFT_FRONT,
        DRIVE_RIGHT_FRONT,
        DRIVE_LEFT_MIDDLE,
        DRIVE_RIGHT_MIDDLE,
        DRIVE_LEFT_REAR,
        DRIVE_RIGHT_REAR,
        STEER_LEFT_FRONT,
        STEER_RIGHT_FRONT,
        STEER_LEFT_MIDDLE,
        STEER_RIGHT_MIDDLE,
        STEER_LEFT_REAR,
        STEER_RIGHT_REAR,
        DEPLOY_LEFT_FRONT,
        DEPLOY_RIGHT_FRONT,
        DEPLOY_LEFT_MIDDLE,
        DEPLOY_RIGHT_MIDDLE,
        DEPLOY_LEFT_REAR,
        DEPLOY_RIGHT_REAR,

    };

    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo &_info);

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

    impl_->joints_.resize(18);

    // Read the joints
    // Right front
    auto drive_right_front_joint = _sdf->Get<std::string>("drive_right_front_joint", "drive_right_front_joint").first;
    impl_->joints_[RoverGazeboJointPluginPrivate::DRIVE_RIGHT_FRONT] = _model->GetJoint(drive_right_front_joint);
    if (!impl_->joints_[RoverGazeboJointPluginPrivate::DRIVE_RIGHT_FRONT])
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Drive right front joint [%s] not found, plugin will not work.",
                     drive_right_front_joint.c_str());
        impl_->ros_node_.reset();
        return;
    }

    auto deploy_right_front_joint = _sdf->Get<std::string>("deploy_right_front_joint", "deploy_right_front_joint").first;
    impl_->joints_[RoverGazeboJointPluginPrivate::DEPLOY_RIGHT_FRONT] = _model->GetJoint(deploy_right_front_joint);
    if (!impl_->joints_[RoverGazeboJointPluginPrivate::DEPLOY_RIGHT_FRONT])
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Deploy right front joint [%s] not found, plugin will not work.",
                     deploy_right_front_joint.c_str());
        impl_->ros_node_.reset();
        return;
    }

    auto steer_right_front_joint = _sdf->Get<std::string>("steer_right_front_joint", "steer_right_front_joint").first;
    impl_->joints_[RoverGazeboJointPluginPrivate::STEER_RIGHT_FRONT] = _model->GetJoint(steer_right_front_joint);
    if (!impl_->joints_[RoverGazeboJointPluginPrivate::STEER_RIGHT_FRONT])
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Steer right front joint [%s] not found, plugin will not work.",
                     steer_right_front_joint.c_str());
        impl_->ros_node_.reset();
        return;
    }

    // Left front
    auto drive_left_front_joint = _sdf->Get<std::string>("drive_left_front_joint", "drive_left_front_joint").first;
    impl_->joints_[RoverGazeboJointPluginPrivate::DRIVE_LEFT_FRONT] = _model->GetJoint(drive_left_front_joint);
    if (!impl_->joints_[RoverGazeboJointPluginPrivate::DRIVE_LEFT_FRONT])
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Drive left front joint [%s] not found, plugin will not work.",
                     drive_left_front_joint.c_str());
        impl_->ros_node_.reset();
        return;
    }

    auto deploy_left_front_joint = _sdf->Get<std::string>("deploy_left_front_joint", "deploy_left_front_joint").first;
    impl_->joints_[RoverGazeboJointPluginPrivate::DEPLOY_LEFT_FRONT] = _model->GetJoint(deploy_left_front_joint);
    if (!impl_->joints_[RoverGazeboJointPluginPrivate::DEPLOY_LEFT_FRONT])
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Deploy left front joint [%s] not found, plugin will not work.",
                     deploy_left_front_joint.c_str());
        impl_->ros_node_.reset();
        return;
    }

    auto steer_left_front_joint = _sdf->Get<std::string>("steer_left_front_joint", "steer_left_front_joint").first;
    impl_->joints_[RoverGazeboJointPluginPrivate::STEER_LEFT_FRONT] = _model->GetJoint(steer_left_front_joint);
    if (!impl_->joints_[RoverGazeboJointPluginPrivate::STEER_LEFT_FRONT])
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                     "Steer left front joint [%s] not found, plugin will not work.",
                     steer_left_front_joint.c_str());
        impl_->ros_node_.reset();
        return;
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
    joints_[RoverGazeboJointPluginPrivate::DRIVE_LEFT_FRONT]->SetVelocity(0, 100);
    joints_[RoverGazeboJointPluginPrivate::DRIVE_RIGHT_FRONT]->SetVelocity(0, 100);
    joints_[RoverGazeboJointPluginPrivate::DEPLOY_LEFT_FRONT]->SetPosition(0, 0);
}

GZ_REGISTER_MODEL_PLUGIN(RoverGazeboJointPlugin)
} // namespace gazebo_plugins
