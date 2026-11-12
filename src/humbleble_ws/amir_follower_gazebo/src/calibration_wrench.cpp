// Evaluation-only external force. Never writes model poses or joint positions.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

namespace amir_follower
{
class CalibrationWrench final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    world_ = entity;
    model_ = sdf->Get<std::string>("model");
    link_ = sdf->Get<std::string>("link");
    base_ = sdf->Get<std::string>("base");
    offset_ = sdf->Get<gz::math::Vector3d>("offset");
    const auto topic = sdf->Get<std::string>("topic");
    if (!node_.Subscribe(topic, &CalibrationWrench::Receive, this)) {
      throw std::runtime_error("Cannot subscribe calibration wrench topic");
    }
  }

  void Receive(const gz::msgs::Wrench &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    force_ = {msg.force().x(),msg.force().y(),msg.force().z()};
    torque_ = {msg.torque().x(),msg.torque().y(),msg.torque().z()};
    for (int i=0;i<3;++i) {
      if (!std::isfinite(force_[i]) || !std::isfinite(torque_[i])) {
        force_.Set(0,0,0); torque_.Set(0,0,0); return;
      }
      force_[i] = std::clamp(force_[i],-1.0,1.0);
      torque_[i] = std::clamp(torque_[i],-0.1,0.1);
    }
    received_ = std::chrono::steady_clock::now();
    fresh_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) {return;}
    gz::math::Vector3d force,torque;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!fresh_ || std::chrono::duration<double>(
        std::chrono::steady_clock::now()-received_).count()>.2) {return;}
      force=force_; torque=torque_;
    }
    auto model = gz::sim::World(world_).ModelByName(ecm,model_);
    if (model==gz::sim::kNullEntity) {return;}
    auto link = gz::sim::Model(model).LinkByName(ecm,link_);
    auto base = gz::sim::Model(model).LinkByName(ecm,base_);
    if (link==gz::sim::kNullEntity || base==gz::sim::kNullEntity) {return;}
    auto orientation = gz::sim::worldPose(base,ecm).Rot();
    // Input is the audited base_footprint axes. Offset is in link axes.
    gz::sim::Link target(link);
    const auto force_world = orientation.RotateVector(force);
    const auto arm_world = gz::sim::worldPose(link,ecm).Rot().RotateVector(offset_);
    // Use one origin wrench. Fortress AddWorldForce requires a WorldPose
    // component that a noncanonical link may not have; worldPose() resolves
    // the hierarchy without that component. The lever arm is from link origin.
    target.AddWorldWrench(ecm,force_world,
      orientation.RotateVector(torque)+arm_world.Cross(force_world));
  }
private:
  gz::sim::Entity world_{};
  std::string model_,link_,base_;
  gz::math::Vector3d offset_,force_,torque_;
  gz::transport::Node node_;
  std::mutex mutex_;
  bool fresh_{false};
  std::chrono::steady_clock::time_point received_;
};
}
IGNITION_ADD_PLUGIN(amir_follower::CalibrationWrench,gz::sim::System,
  gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
