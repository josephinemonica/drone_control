#ifndef _REVOLUTE_CONTROL_PLUGIN_HH_
#define _REVOLUTE_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a revolute joint
  class revolute_control_plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: revolute_control_plugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe revolute_control_plugin is attach to model[" <<
        _model->GetName() << "]\n";
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(revolute_control_plugin)
}
#endif
