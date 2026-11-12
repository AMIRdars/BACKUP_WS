// Report actual local constraint components, including to late subscribers.
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <atomic>
#include <functional>
#include <iostream>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/transport/Node.hh>

namespace amir_follower
{
class GraspHeartbeat final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &,gz::sim::EventManager &) override
  {
    world_=entity;
    payload_=sdf->Get<std::string>("payload");
    manage_=sdf->Get<bool>("manage_joints",false).first;
    state_period_=sdf->Get<double>("state_publish_period_s",.1).first;
    if (!std::isfinite(state_period_) || state_period_<=0.) {
      throw std::invalid_argument("positive finite grasp state_publish_period_s required");
    }
    for (size_t i=0;i<2;++i) {
      const auto suffix=std::to_string(i+1);
      robots_[i]=sdf->Get<std::string>("robot"+suffix);
      parents_[i]=sdf->Get<std::string>("parent_link"+suffix);
      child_=sdf->Get<std::string>("child_link");
      publishers_[i]=node_.Advertise<gz::msgs::StringMsg>("/"+robots_[i]+"/grasp/state");
      if (manage_) {
        std::function<void(const gz::msgs::Empty &)> attach=[this,i](const auto &) {desired_[i]=true;};
        std::function<void(const gz::msgs::Empty &)> detach=[this,i](const auto &) {desired_[i]=false;};
        node_.Subscribe("/"+robots_[i]+"/grasp/attach",attach);
        node_.Subscribe("/"+robots_[i]+"/grasp/detach",detach);
      }
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) {return;}
    const auto now=std::chrono::duration<double>(info.simTime).count();
    const bool publish=now<last_ || now-last_>=state_period_;
    if (publish) {last_=now;}
    const auto payload=gz::sim::World(world_).ModelByName(ecm,payload_);
    // A missing payload is not a confirmed detached payload. Startup waits
    // for its first real state before approaching it.
    if (payload==gz::sim::kNullEntity) {return;}
    for (size_t i=0;i<2;++i) {
      bool attached=false;
      const auto robot=gz::sim::World(world_).ModelByName(ecm,robots_[i]);
      if (payload!=gz::sim::kNullEntity && robot!=gz::sim::kNullEntity) {
        const auto parent=gz::sim::Model(payload).LinkByName(ecm,parents_[i]);
        const auto child=gz::sim::Model(robot).LinkByName(ecm,child_);
        if (manage_ && parent!=gz::sim::kNullEntity && child!=gz::sim::kNullEntity) {
          if (!desired_[i] && joints_[i]!=gz::sim::kNullEntity) {
            ecm.RequestRemoveEntity(joints_[i]);
            joints_[i]=gz::sim::kNullEntity;
          } else if (desired_[i] && joints_[i]==gz::sim::kNullEntity) {
            joints_[i]=ecm.CreateEntity();
            ecm.CreateComponent(joints_[i],gz::sim::components::DetachableJoint({parent,child,"fixed"}));
            std::clog << "[ControlledGrasp] " << robots_[i] << " joint=" << joints_[i]
                      << " parent=" << parent << " child=" << child << "\n";
          }
        }
        ecm.Each<gz::sim::components::DetachableJoint>(
          [&](const gz::sim::Entity &,const gz::sim::components::DetachableJoint *joint) {
            const auto &data=joint->Data();
            attached=attached || (parent!=gz::sim::kNullEntity && child!=gz::sim::kNullEntity &&
              data.parentLink==parent && data.childLink==child);
            return true;
          });
      }
      gz::msgs::StringMsg state;
      state.set_data(attached?"attached":"detached");
      if (publish) {publishers_[i].Publish(state);}
    }
  }
private:
  gz::sim::Entity world_{};
  std::string payload_,child_;
  std::array<std::string,2> robots_,parents_;
  gz::transport::Node node_;
  std::array<gz::transport::Node::Publisher,2> publishers_;
  double last_{-1.};
  double state_period_{.1};
  bool manage_{false};
  std::array<std::atomic_bool,2> desired_{{false,false}};
  std::array<gz::sim::Entity,2> joints_{{gz::sim::kNullEntity,gz::sim::kNullEntity}};
};
}
IGNITION_ADD_PLUGIN(amir_follower::GraspHeartbeat,gz::sim::System,
  gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
