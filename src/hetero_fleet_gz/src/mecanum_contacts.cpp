// Equivalent free-roller contact constraints without extra passive joints.
#include <unordered_map>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/physics/Events.hh>

namespace hetero_fleet
{
class MecanumContacts final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  using Params = gz::physics::SetContactPropertiesCallbackFeature::
      ContactSurfaceParams<gz::sim::events::Policy>;
  gz::sim::Entity model{gz::sim::kNullEntity};
  std::unordered_map<gz::sim::Entity, gz::math::Vector3d> directions;
  gz::math::Quaterniond rotation;
  gz::common::ConnectionPtr connection;

  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_events) override
  {
    model = _entity;
    gz::sim::Model robot(model);
    // Free directions for standard X arrangement: FL/RR +45, FR/RL -45.
    for (const auto &[name, sign] : std::unordered_map<std::string, double>{
        {"front_left", 1}, {"front_right", -1},
        {"rear_left", -1}, {"rear_right", 1}})
    {
      const auto link = robot.LinkByName(_ecm, name + "_wheel_link");
      const auto collision = _ecm.EntityByComponents(
          gz::sim::components::ParentEntity(link), gz::sim::components::Collision());
      if (collision == gz::sim::kNullEntity)
        throw std::runtime_error("Mecanum wheel collision missing: " + name);
      directions.emplace(collision, gz::math::Vector3d(1, sign, 0).Normalized());
      _ecm.CreateComponent(collision,
          gz::sim::components::EnableContactSurfaceCustomization(true));
    }
    connection = _events.Connect<gz::sim::events::CollectContactSurfaceProperties>(
      [this](const auto &a, const auto &b, const auto &, const auto,
             const auto normal, const auto, const auto, Params &params)
      {
        auto wheel = directions.find(a);
        if (wheel == directions.end()) wheel = directions.find(b);
        if (wheel == directions.end()) return;
        auto direction = rotation.RotateVector(wheel->second);
        if (normal)
          direction -= direction.Dot(*normal) * (*normal);
        if (direction.Length() < 1e-6) return;
        direction.Normalize();
        params.firstFrictionalDirection = Eigen::Vector3d(
            direction.X(), direction.Y(), direction.Z());
        params.frictionCoeff = 0.01;       // rollers spin freely along this tangent
        params.secondaryFrictionCoeff = 1.2;
        params.restitutionCoeff = 0.0;
      });
  }
  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    rotation = gz::sim::worldPose(model, _ecm).Rot();
  }
};
}
GZ_ADD_PLUGIN(hetero_fleet::MecanumContacts, gz::sim::System,
    hetero_fleet::MecanumContacts::ISystemConfigure,
    hetero_fleet::MecanumContacts::ISystemPreUpdate)
