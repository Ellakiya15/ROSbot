#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double.pb.h> // For receiving RPM messages

namespace custom_diff_drive
{
  class DiffDrive : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
  {
  private:
    gz::sim::Model model{gz::sim::kNullEntity};
    gz::transport::Node node;
    gz::sim::Entity left_wheel_joint, right_wheel_joint;
    double wheel_radius = 0.033;
    double left_wheel_speed = 0.0;
    double right_wheel_speed = 0.0;

    void LeftWheelRPMCallback(const gz::msgs::Double &msg)
    {
      left_wheel_speed = (msg.data() * 2.0 * GZ_PI) / 60.0; // Convert RPM to rad/s
    }

    void RightWheelRPMCallback(const gz::msgs::Double &msg)
    {
      right_wheel_speed = (msg.data() * 2.0 * GZ_PI) / 60.0; // Convert RPM to rad/s
    }

  public:
    DiffDrive() = default;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
      model = gz::sim::Model(_entity);
      left_wheel_joint = model.JointByName(_ecm, "left_wheel_joint");
      right_wheel_joint = model.JointByName(_ecm, "right_wheel_joint");

      node.Subscribe("/left_wheel_rpm", &DiffDrive::LeftWheelRPMCallback, this);
      node.Subscribe("/right_wheel_rpm", &DiffDrive::RightWheelRPMCallback, this);
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
