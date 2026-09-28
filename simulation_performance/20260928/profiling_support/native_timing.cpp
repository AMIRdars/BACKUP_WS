
#define _GNU_SOURCE
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <atomic>
#include <mutex>
struct Meter { std::mutex mutex; unsigned long count=0; double cpu=0,wall=0,maxwall=0; long sec=0; };
static double seconds(clockid_t id) { struct timespec t; clock_gettime(id,&t); return t.tv_sec+t.tv_nsec*1e-9; }
static int logfile() { static int fd=[](){ char path[2048]; const char *dir=getenv("NATIVE_PROFILE_DIR"); if(!dir)return -1; snprintf(path,sizeof(path),"%s/native-%d.jsonl",dir,getpid());return open(path,O_CREAT|O_WRONLY|O_APPEND,0644); }(); return fd; }
static void record(Meter &m,const char *name,double cpu,double wall,double now) {
 std::lock_guard<std::mutex> lock(m.mutex);
 m.count++;m.cpu+=cpu;m.wall+=wall;if(wall>m.maxwall)m.maxwall=wall;
 if((long)now!=m.sec) { char row[1024];int n=snprintf(row,sizeof(row),"{\"monotonic\":%.6f,\"stage\":\"%s\",\"count\":%lu,\"cpu\":%.9f,\"wall\":%.9f,\"maxwall\":%.9f}\n",now,name,m.count,m.cpu,m.wall,m.maxwall); int fd=logfile();if(fd>=0)write(fd,row,n);m.sec=(long)now;m.count=0;m.cpu=m.wall=m.maxwall=0; }
}
extern "C" void hook_0(void*,const void*,void*) asm("_ZThn16_N8ignition6gazebo2v67systems11ForceTorque10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
extern "C" void hook_0(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-forcetorque-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn16_N8ignition6gazebo2v67systems11ForceTorque10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::ForceTorque::PostUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::ForceTorque::PostUpdate",ce-c,we-w,we);
 }
extern "C" void hook_1(void*,const void*,void*) asm("_ZThn8_N8ignition6gazebo2v67systems11ForceTorque9PreUpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
extern "C" void hook_1(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-forcetorque-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn8_N8ignition6gazebo2v67systems11ForceTorque9PreUpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::ForceTorque::PreUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::ForceTorque::PreUpdate",ce-c,we-w,we);
 }
extern "C" void hook_2(void*,const void*,void*) asm("_ZThn16_N8ignition6gazebo2v67systems7Physics6UpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
extern "C" void hook_2(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-physics-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn16_N8ignition6gazebo2v67systems7Physics6UpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::Physics::Update\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::Physics::Update",ce-c,we-w,we);
 }
extern "C" void hook_3(void*,const void*,void*) asm("_ZThn16_N8ignition6gazebo2v67systems7Contact10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
extern "C" void hook_3(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-contact-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn16_N8ignition6gazebo2v67systems7Contact10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::Contact::PostUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::Contact::PostUpdate",ce-c,we-w,we);
 }
extern "C" void hook_4(void*,const void*,void*) asm("_ZThn8_N8ignition6gazebo2v67systems7Contact9PreUpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
extern "C" void hook_4(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-contact-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn8_N8ignition6gazebo2v67systems7Contact9PreUpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::Contact::PreUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::Contact::PreUpdate",ce-c,we-w,we);
 }
extern "C" void hook_5(void*,const void*,void*) asm("_ZThn16_N8ignition6gazebo2v67systems16SceneBroadcaster10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
extern "C" void hook_5(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-scene-broadcaster-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn16_N8ignition6gazebo2v67systems16SceneBroadcaster10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::SceneBroadcaster::PostUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::SceneBroadcaster::PostUpdate",ce-c,we-w,we);
 }
extern "C" void hook_6(void*,const void*,void*) asm("_ZThn16_N8ignition6gazebo2v67systems7Sensors6UpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
extern "C" void hook_6(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-sensors-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn16_N8ignition6gazebo2v67systems7Sensors6UpdateERKNS1_10UpdateInfoERNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::Sensors::Update\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::Sensors::Update",ce-c,we-w,we);
 }
extern "C" void hook_7(void*,const void*,void*) asm("_ZThn24_N8ignition6gazebo2v67systems7Sensors10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
extern "C" void hook_7(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-sensors-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn24_N8ignition6gazebo2v67systems7Sensors10PostUpdateERKNS1_10UpdateInfoERKNS1_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original ignition::gazebo::v6::systems::Sensors::PostUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"ignition::gazebo::v6::systems::Sensors::PostUpdate",ce-c,we-w,we);
 }
extern "C" void hook_8(void*,const void*,void*) asm("_ZThn16_N15gz_ros2_control26GazeboSimROS2ControlPlugin9PreUpdateERKN8ignition6gazebo2v610UpdateInfoERNS3_22EntityComponentManagerE");
extern "C" void hook_8(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/opt/ros/humble/lib/libgz_ros2_control-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn16_N15gz_ros2_control26GazeboSimROS2ControlPlugin9PreUpdateERKN8ignition6gazebo2v610UpdateInfoERNS3_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original gz_ros2_control::GazeboSimROS2ControlPlugin::PreUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"gz_ros2_control::GazeboSimROS2ControlPlugin::PreUpdate",ce-c,we-w,we);
 }
extern "C" void hook_9(void*,const void*,void*) asm("_ZThn24_N15gz_ros2_control26GazeboSimROS2ControlPlugin10PostUpdateERKN8ignition6gazebo2v610UpdateInfoERKNS3_22EntityComponentManagerE");
extern "C" void hook_9(void* self,const void* info,void* ecm) {
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("/opt/ros/humble/lib/libgz_ros2_control-system.so",RTLD_LAZY|RTLD_NOLOAD),"_ZThn24_N15gz_ros2_control26GazeboSimROS2ControlPlugin10PostUpdateERKN8ignition6gazebo2v610UpdateInfoERKNS3_22EntityComponentManagerE");
 if(!fn){fprintf(stderr,"Cannot resolve profiler original gz_ros2_control::GazeboSimROS2ControlPlugin::PostUpdate\n");abort();}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"gz_ros2_control::GazeboSimROS2ControlPlugin::PostUpdate",ce-c,we-w,we);
 }
