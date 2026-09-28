#define _GNU_SOURCE
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <signal.h>
#include <time.h>
#include <ucontext.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string>
// Main simulation thread only. One interrupted instruction pointer per 10 ms
// of thread CPU time. Handler uses async-signal-safe write only.
static int sample_fd=-1;
static void capture(int,siginfo_t*,void*ctx){auto *u=(ucontext_t*)ctx;uint64_t ip=u->uc_mcontext.gregs[REG_RIP];if(sample_fd>=0){ssize_t n=write(sample_fd,&ip,sizeof(ip));(void)n;}}
class PcSampler final : public ignition::gazebo::System,public ignition::gazebo::ISystemPreUpdate {
 bool started=false;timer_t timer{}; bool valid=false;
 public:~PcSampler(){if(valid)timer_delete(timer);if(sample_fd>=0)close(sample_fd);}
 void PreUpdate(const ignition::gazebo::UpdateInfo&,ignition::gazebo::EntityComponentManager&)override{
  if(started)return;started=true;const char*dir=getenv("NATIVE_PROFILE_DIR");if(!dir)return;
  int sig=SIGRTMIN+12;struct sigaction previous{};sigaction(sig,nullptr,&previous);if(previous.sa_handler!=SIG_DFL)return;
  sample_fd=open((std::string(dir)+"/main_cpu_pc.bin").c_str(),O_WRONLY|O_CREAT|O_APPEND,0644);
  struct sigaction action{};action.sa_sigaction=capture;action.sa_flags=SA_SIGINFO|SA_RESTART;sigemptyset(&action.sa_mask);sigaction(sig,&action,nullptr);
  struct sigevent event{};event.sigev_notify=SIGEV_THREAD_ID;event.sigev_signo=sig;event._sigev_un._tid=syscall(SYS_gettid);
  if(timer_create(CLOCK_THREAD_CPUTIME_ID,&event,&timer)==0){struct itimerspec rate{};rate.it_value.tv_nsec=10000000;rate.it_interval.tv_nsec=10000000;valid=timer_settime(timer,0,&rate,nullptr)==0;}
 }
};
IGNITION_ADD_PLUGIN(PcSampler,ignition::gazebo::System,PcSampler::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(PcSampler,"performance::PcSampler")
