#ifndef GAZEBO_PLUGINS_ROVER_GAZEBO_JOINT_PLUGIN_HPP_
#define GAZEBO_PLUGINS_ROVER_GAZEBO_JOINT_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>

namespace gazebo_plugins
{
class RoverGazeboJointPluginPrivate;

class RoverGazeboJointPlugin : public gazebo::ModelPlugin
{
public:
    /// Constructor
    RoverGazeboJointPlugin();
    /// Destructor
    ~RoverGazeboJointPlugin();

protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;

private:
    /// Private data pointer
    std::unique_ptr<RoverGazeboJointPluginPrivate> impl_;
};

} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS_ROVER_GAZEBO_JOINT_PLUGIN_HPP_