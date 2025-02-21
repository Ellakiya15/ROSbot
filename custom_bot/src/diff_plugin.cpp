#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
// #include <gz/sim/components/Twist.hh>
#include <gz/transport/Node.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/twist.pb.h>

namespace custom_diff_drive
{
  class DiffDrive : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
  {
  private:
    gz::sim::Model model{gz::sim::kNullEntity};
    gz::transport::Node node;
    gz::math::PID left_pid, right_pid;
    gz::sim::Entity left_wheel_joint, right_wheel_joint;
    double wheel_radius = 0.033;
    double wheel_separation = 0.287;
    double left_wheel_speed = 0.0;
    double right_wheel_speed = 0.0;

    void CmdVelCallback(const gz::msgs::Twist &msg)
    {
      double linear_vel = msg.linear().x();
      double angular_vel = msg.angular().z();

      left_wheel_speed = (linear_vel - angular_vel * wheel_separation / 2.0) / wheel_radius;
      right_wheel_speed = (linear_vel + angular_vel * wheel_separation / 2.0) / wheel_radius;
    }

  public:
    DiffDrive() = default;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
      model = gz::sim::Model(_entity);
      left_wheel_joint = model.JointByName(_ecm, "wheel_left_joint");
      right_wheel_joint = model.JointByName(_ecm, "wheel_right_joint");

      node.Subscribe("/cmd_vel", &DiffDrive::CmdVelCallback, this);
    }

    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &_ecm) override
    {
      if (left_wheel_joint != gz::sim::kNullEntity && right_wheel_joint != gz::sim::kNullEntity)
      {
        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
            left_wheel_joint, {left_wheel_speed});
        _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
            right_wheel_joint, {right_wheel_speed});
      }
    }
  };
}

GZ_ADD_PLUGIN(custom_diff_drive::DiffDrive,
              gz::sim::System,
              custom_diff_drive::DiffDrive::ISystemConfigure,
              custom_diff_drive::DiffDrive::ISystemPreUpdate)
