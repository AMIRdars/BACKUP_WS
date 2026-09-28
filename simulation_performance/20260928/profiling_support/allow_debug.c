/* Only diagnostic Ruby/Gazebo processes opt in to same-user debugger attachment.
 * This does not modify system ptrace settings. Baseline runs do not load it. */
#define _GNU_SOURCE
#include <sys/prctl.h>
#include <unistd.h>
#include <string.h>
__attribute__((constructor)) static void allow_debug(void) {
  char path[1024]; ssize_t n = readlink("/proc/self/exe", path, sizeof(path)-1);
  if(n > 0) { path[n] = 0; if(strstr(path,"ruby")) prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0); }
}
