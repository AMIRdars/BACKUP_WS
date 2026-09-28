from pathlib import Path
import subprocess
base=Path(__file__).resolve().parent
files=list(Path('/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins').glob('libignition-gazebo-*-system.so'))+[Path('/opt/ros/humble/lib/libgz_ros2_control-system.so')]
selected=[]
for file in files:
 if not any(x in file.name for x in ('physics-system','contact-system','forcetorque-system','scene-broadcaster-system','sensors-system','ros2_control-system')):continue
 for row in subprocess.check_output(['nm','-D','--defined-only',str(file)],text=True).splitlines():
  symbol=row.split()[-1]
  demangled=subprocess.check_output(['c++filt',symbol],text=True).strip()
  if 'non-virtual thunk to' in demangled and any(f'::{x}(' in demangled for x in ('PreUpdate','PostUpdate','Update')):
   selected.append((str(file),symbol,demangled))
source=r'''
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
'''
for i,(file,symbol,name) in enumerate(selected):
 stage=name.split(' to ')[1].split('(')[0]
 source+=f'extern "C" void hook_{i}(void*,const void*,void*) asm("{symbol}");\n'
 source+=f'''extern "C" void hook_{i}(void* self,const void* info,void* ecm) {{
 using Fn=void(*)(void*,const void*,void*);
 static Fn fn=(Fn)dlsym(dlopen("{file}",RTLD_LAZY|RTLD_NOLOAD),"{symbol}");
 if(!fn){{fprintf(stderr,"Cannot resolve profiler original {stage}\\n");abort();}}
 static Meter meter; double w=seconds(CLOCK_MONOTONIC),c=seconds(CLOCK_THREAD_CPUTIME_ID);
 fn(self,info,ecm);double ce=seconds(CLOCK_THREAD_CPUTIME_ID),we=seconds(CLOCK_MONOTONIC);
 record(meter,"{stage}",ce-c,we-w,we);
 }}\n'''
(base/'native_timing.cpp').write_text(source)
subprocess.run(['g++','-std=c++17','-shared','-fPIC','-O2',str(base/'native_timing.cpp'),'-ldl','-pthread','-o',str(base/'native_timing.so')],check=True)
print('Native stage timing wrappers:',len(selected))
