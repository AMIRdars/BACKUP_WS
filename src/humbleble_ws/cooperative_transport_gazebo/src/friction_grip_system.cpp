#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

namespace cooperative_transport
{
class FrictionGripSystem final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->linearStiffness = _sdf->Get<double>("linear_stiffness", 12500.0).first;
    this->linearDamping = _sdf->Get<double>("linear_damping", 250.0).first;
    this->maximumForce = _sdf->Get<double>("maximum_force", 250.0).first;
    this->maximumForceRate =
      _sdf->Get<double>("maximum_force_rate", 600.0).first;
    this->verticalStiffness =
      _sdf->Get<double>("vertical_stiffness", 8000.0).first;
    this->verticalDamping =
      _sdf->Get<double>("vertical_damping", 120.0).first;
    this->maximumVerticalForce =
      _sdf->Get<double>("maximum_vertical_force", 40.0).first;
    this->maximumVerticalForceRate =
      _sdf->Get<double>("maximum_vertical_force_rate", 250.0).first;
    this->yawStiffness = _sdf->Get<double>("yaw_stiffness", 20.0).first;
    this->yawDamping = _sdf->Get<double>("yaw_damping", 3.0).first;
    this->maximumTorque = _sdf->Get<double>("maximum_torque", 8.0).first;
    this->activationHeight = _sdf->Get<double>("activation_height", 0.472).first;
    std::cerr << "[FrictionGripSystem] configured; bounded force model ready\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->FindLinks(_ecm)) {
      return;
    }

    const gz::math::Pose3d payloadPose =
      gz::sim::worldPose(this->payloadModel.Entity(), _ecm);
    const gz::math::Pose3d firstPose =
      gz::sim::worldPose(this->firstModel.Entity(), _ecm);
    const gz::math::Pose3d secondPose =
      gz::sim::worldPose(this->secondModel.Entity(), _ecm);

    const gz::math::Vector3d center =
      0.5 * (firstPose.Pos() + secondPose.Pos());
    const double formationYaw = std::atan2(
      secondPose.Pos().Y() - firstPose.Pos().Y(),
      secondPose.Pos().X() - firstPose.Pos().X());

    if (!this->active) {
      if (payloadPose.Pos().Z() < this->activationHeight) {
        return;
      }
      const gz::math::Vector3d delta = payloadPose.Pos() - center;
      this->offsetX =
        std::cos(formationYaw) * delta.X() +
        std::sin(formationYaw) * delta.Y();
      this->offsetY =
        -std::sin(formationYaw) * delta.X() +
        std::cos(formationYaw) * delta.Y();
      this->desiredPayloadZ = payloadPose.Pos().Z();
      this->offsetYaw = Normalize(payloadPose.Rot().Yaw() - formationYaw);
      this->lastForce = gz::math::Vector3d::Zero;
      this->lastVerticalForce = 0.0;
      this->lastCenter = center;
      this->havePreviousCenter = true;
      this->active = true;
      std::cerr << "[FrictionGripSystem] activated at payload z="
                << payloadPose.Pos().Z() << "\n";
    }

    const gz::math::Vector3d desired(
      center.X() + std::cos(formationYaw) * this->offsetX -
        std::sin(formationYaw) * this->offsetY,
      center.Y() + std::sin(formationYaw) * this->offsetX +
        std::cos(formationYaw) * this->offsetY,
      this->desiredPayloadZ);
    gz::math::Vector3d error = payloadPose.Pos() - desired;
    const double dt = std::max(
      0.0, std::chrono::duration<double>(_info.dt).count());

    gz::math::Vector3d formationVelocity = gz::math::Vector3d::Zero;
    if (this->havePreviousCenter && dt > 0.0) {
      formationVelocity = (center - this->lastCenter) / dt;
      if (formationVelocity.Length() > 0.20) {
        formationVelocity = formationVelocity.Normalized() * 0.20;
      }
    }
    this->lastCenter = center;
    this->havePreviousCenter = true;

    gz::math::Vector3d relativeVelocity = gz::math::Vector3d::Zero;
    auto payloadVelocity = this->payload.WorldLinearVelocity(_ecm);
    if (payloadVelocity) {
      relativeVelocity = *payloadVelocity - formationVelocity;
    }

    double verticalForce = std::clamp(
      -this->verticalStiffness * error.Z() -
      this->verticalDamping * relativeVelocity.Z(),
      -this->maximumVerticalForce, this->maximumVerticalForce);
    const double maximumVerticalStep = this->maximumVerticalForceRate * dt;
    verticalForce = std::clamp(
      verticalForce,
      this->lastVerticalForce - maximumVerticalStep,
      this->lastVerticalForce + maximumVerticalStep);
    this->lastVerticalForce = verticalForce;

    // Planar and vertical bristle forces are limited separately. This keeps
    // gravity support from consuming the available horizontal grip force.
    error.Z(0.0);
    relativeVelocity.Z(0.0);

    gz::math::Vector3d force =
      -this->linearStiffness * error - this->linearDamping * relativeVelocity;
    if (force.Length() > this->maximumForce) {
      force = force.Normalized() * this->maximumForce;
    }
    const double maximumForceStep = this->maximumForceRate * dt;
    const gz::math::Vector3d forceStep = force - this->lastForce;
    if (forceStep.Length() > maximumForceStep && maximumForceStep > 0.0) {
      force = this->lastForce + forceStep.Normalized() * maximumForceStep;
    }
    this->lastForce = force;
    force.Z(verticalForce);

    double relativeYawRate = 0.0;
    auto payloadAngular = this->payload.WorldAngularVelocity(_ecm);
    auto firstAngular = this->firstBase.WorldAngularVelocity(_ecm);
    auto secondAngular = this->secondBase.WorldAngularVelocity(_ecm);
    if (payloadAngular && firstAngular && secondAngular) {
      relativeYawRate = payloadAngular->Z() -
        0.5 * (firstAngular->Z() + secondAngular->Z());
    }
    const double yawError = Normalize(
      payloadPose.Rot().Yaw() - formationYaw - this->offsetYaw);
    const double torqueZ = std::clamp(
      -this->yawStiffness * yawError - this->yawDamping * relativeYawRate,
      -this->maximumTorque, this->maximumTorque);

    this->payload.AddWorldWrench(
      _ecm, force, gz::math::Vector3d(0.0, 0.0, torqueZ));
  }

private:
  static double Normalize(double _angle)
  {
    return std::atan2(std::sin(_angle), std::cos(_angle));
  }

  bool FindLinks(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->payload.Valid(_ecm) && this->firstBase.Valid(_ecm) &&
      this->secondBase.Valid(_ecm))
    {
      return true;
    }
    const auto payloadModelEntity = _ecm.EntityByComponents(
      gz::sim::components::Name("cooperative_payload"),
      gz::sim::components::Model());
    const auto firstModelEntity = _ecm.EntityByComponents(
      gz::sim::components::Name("amir1"), gz::sim::components::Model());
    const auto secondModelEntity = _ecm.EntityByComponents(
      gz::sim::components::Name("amir2"), gz::sim::components::Model());
    if (payloadModelEntity == gz::sim::kNullEntity ||
      firstModelEntity == gz::sim::kNullEntity ||
      secondModelEntity == gz::sim::kNullEntity)
    {
      return false;
    }
    this->payloadModel = gz::sim::Model(payloadModelEntity);
    this->firstModel = gz::sim::Model(firstModelEntity);
    this->secondModel = gz::sim::Model(secondModelEntity);
    this->payload = gz::sim::Link(
      this->payloadModel.CanonicalLink(_ecm));
    this->firstBase = gz::sim::Link(
      this->firstModel.CanonicalLink(_ecm));
    this->secondBase = gz::sim::Link(
      this->secondModel.CanonicalLink(_ecm));
    this->payload.EnableVelocityChecks(_ecm);
    this->firstBase.EnableVelocityChecks(_ecm);
    this->secondBase.EnableVelocityChecks(_ecm);
    const bool valid = this->payload.Valid(_ecm) && this->firstBase.Valid(_ecm) &&
      this->secondBase.Valid(_ecm);
    if (valid && !this->linksReported) {
      std::cerr << "[FrictionGripSystem] payload and robot links discovered\n";
      this->linksReported = true;
    }
    return valid;
  }

  gz::sim::Link payload{gz::sim::kNullEntity};
  gz::sim::Link firstBase{gz::sim::kNullEntity};
  gz::sim::Link secondBase{gz::sim::kNullEntity};
  gz::sim::Model payloadModel{gz::sim::kNullEntity};
  gz::sim::Model firstModel{gz::sim::kNullEntity};
  gz::sim::Model secondModel{gz::sim::kNullEntity};
  bool active{false};
  bool linksReported{false};
  double offsetX{0.0};
  double offsetY{0.0};
  double offsetYaw{0.0};
  double linearStiffness{12500.0};
  double linearDamping{250.0};
  double maximumForce{250.0};
  double maximumForceRate{600.0};
  gz::math::Vector3d lastForce{gz::math::Vector3d::Zero};
  double verticalStiffness{8000.0};
  double verticalDamping{120.0};
  double maximumVerticalForce{40.0};
  double maximumVerticalForceRate{250.0};
  double lastVerticalForce{0.0};
  double desiredPayloadZ{0.0};
  gz::math::Vector3d lastCenter{gz::math::Vector3d::Zero};
  bool havePreviousCenter{false};
  double yawStiffness{20.0};
  double yawDamping{3.0};
  double maximumTorque{8.0};
  double activationHeight{0.472};
};
}  // namespace cooperative_transport

IGNITION_ADD_PLUGIN(
  cooperative_transport::FrictionGripSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  cooperative_transport::FrictionGripSystem,
  "cooperative_transport::FrictionGripSystem")
