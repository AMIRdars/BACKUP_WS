"""Optional process-local SMT-core preference, based on observed Linux topology."""
import os
import shutil
from pathlib import Path


def cpu_list(value):
    result=set()
    for part in value.strip().split(','):
        ends=part.split('-');a=int(ends[0]);b=int(ends[-1])
        result.update(range(a,b+1))
    return result


def select_smt(allowed, root=Path('/sys/devices/system/cpu')):
    topology={};selected=set()
    for cpu in sorted(allowed):
        folder=root/f'cpu{cpu}'
        try:
            siblings=cpu_list((folder/'topology/thread_siblings_list').read_text())
            frequency=folder/'cpufreq/cpuinfo_max_freq'
            topology[str(cpu)]={'siblings':sorted(siblings),
                'max_frequency_khz':int(frequency.read_text()) if frequency.exists() else None}
            if len(siblings)>1:selected.add(cpu)
        except (OSError,ValueError):continue
    return selected or set(allowed),topology


def prefer_smt_cores():
    before=os.sched_getaffinity(0);selected,topology=select_smt(before)
    os.sched_setaffinity(0,selected)
    return {'scope':'this runner and subsequently created threads/children only; no other process or watchdog change',
        'policy':'prefer observed SMT cores, otherwise inherit','before':sorted(before),
        'after':sorted(os.sched_getaffinity(0)),'observed_topology':topology}


def gui_command(argv,policy):
    """Only the new GUI child uses spare permitted CPUs; critical tasks keep theirs."""
    spare=set(policy.get('before',[]))-set(policy.get('after',[])) if policy else set()
    taskset=shutil.which('taskset')
    if spare and taskset:
        return [taskset,'--cpu-list',','.join(map(str,sorted(spare))),*argv],{
            'scope':'new owned GUI child and its threads only','gui_cpus':sorted(spare),
            'control_cpus':policy['after'],'watchdogs_changed':False}
    return argv,{'scope':'GUI inherits existing permitted CPUs; no spare CPU set available',
                 'watchdogs_changed':False}
