// Gazebo-only test brake. Initially free for native grasp preparation.
// The controller never consumes these experiment topics. No payload attachment.
#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <ignition/math/Pose3.hh>
#include <gz/transport/Node.hh>

namespace amir_follower {
class BaseHoldFixture final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
    world_=entity;
    robot_=sdf->Get<std::string>("robot");
    base_=sdf->Get<std::string>("base_link");
    const std::string prefix="/experiment/"+robot_+"/base_fixture/";
    pub_=node_.Advertise<gz::msgs::StringMsg>(prefix+"state");
    std::function<void(const gz::msgs::Empty &)> lock=[this](const auto &){desired_=true;};
    node_.Subscribe(prefix+"lock",lock);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    auto world=gz::sim::World(world_);
    const auto robot=world.ModelByName(ecm,robot_);
    if (robot==gz::sim::kNullEntity) return;
    const auto child=gz::sim::Model(robot).LinkByName(ecm,base_);
    if (child==gz::sim::kNullEntity) return;
    if (desired_ && !locked_) {hold_=gz::sim::worldPose(child,ecm);locked_=true;}
    if (locked_) {
      // Kinematic BENCH fixture: keep the original articulated skeleton and
      // all sensor/controller handles. Constrain root velocities only; do not
      // teleport the articulated model or reset any arm joint configuration.
      // Cancel accumulated integration drift through root velocity commands.
      // Ideal kinematic bench restraint; no articulated model pose resets.
      const double dt=std::chrono::duration<double>(info.dt).count();
      if (dt<=0) return;
      const auto pose=gz::sim::worldPose(child,ecm);
      const auto linear=pose.Rot().RotateVectorReverse(hold_.Pos()-pose.Pos())/dt;
      const auto delta=pose.Rot().Inverse()*hold_.Rot();
      ignition::math::Vector3d axis;double angle;
      delta.ToAxis(axis,angle);
      gz::sim::Link(child).SetLinearVelocity(ecm,linear);
      gz::sim::Link(child).SetAngularVelocity(ecm,axis*(angle/dt));
    }
    const double now=std::chrono::duration<double>(info.simTime).count();
    if (now-last_>=.1 || now<last_) {
      last_=now;gz::msgs::StringMsg msg;msg.set_data(locked_?"locked":"unlocked");pub_.Publish(msg);
    }
  }
private:
  gz::sim::Entity world_{};
  bool locked_{false};
  ignition::math::Pose3d hold_;
  std::string robot_,base_;
  std::atomic_bool desired_{false};
  double last_{-1.};
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};
}
IGNITION_ADD_PLUGIN(amir_follower::BaseHoldFixture,gz::sim::System,
  gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
