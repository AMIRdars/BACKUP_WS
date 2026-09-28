#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>
// Added after all existing systems. Between the final PreUpdate and final
// Update, Gazebo executes its Update pass (Physics and Sensors in this world).
// This measures that entire pass plus the framework transition, not Physics
// alone. This system does not read or modify any simulation entity.
class PhaseMeter final : public ignition::gazebo::System,
 public ignition::gazebo::ISystemPreUpdate,
 public ignition::gazebo::ISystemUpdate {
 double c0=0,w0=0,cpu=0,wall=0,maxwall=0; long second=0;unsigned long count=0;
 std::ofstream file;
 static double now(clockid_t id){timespec t;clock_gettime(id,&t);return t.tv_sec+t.tv_nsec*1e-9;}
 public:
 void PreUpdate(const ignition::gazebo::UpdateInfo &info, ignition::gazebo::EntityComponentManager &) override {
  if(info.paused)return;c0=now(CLOCK_THREAD_CPUTIME_ID);w0=now(CLOCK_MONOTONIC);
 }
 void Update(const ignition::gazebo::UpdateInfo &info, ignition::gazebo::EntityComponentManager &) override {
  if(info.paused||!w0)return;double c=now(CLOCK_THREAD_CPUTIME_ID)-c0,w=now(CLOCK_MONOTONIC)-w0,t=now(CLOCK_MONOTONIC);
  cpu+=c;wall+=w;count++;if(w>maxwall)maxwall=w;
  if((long)t!=second){
   if(!file.is_open()){const char*dir=getenv("NATIVE_PROFILE_DIR");if(dir)file.open(std::string(dir)+"/update_pass.csv");file<<"monotonic,sim,count,cpu,wall,maxwall\n";}
   file.precision(12);file<<t<<","<<std::chrono::duration<double>(info.simTime).count()<<","<<count<<","<<cpu<<","<<wall<<","<<maxwall<<std::endl;
   second=(long)t;count=0;cpu=wall=maxwall=0;
  }
 }
};
IGNITION_ADD_PLUGIN(PhaseMeter, ignition::gazebo::System, PhaseMeter::ISystemPreUpdate,PhaseMeter::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(PhaseMeter,"performance::PhaseMeter")
