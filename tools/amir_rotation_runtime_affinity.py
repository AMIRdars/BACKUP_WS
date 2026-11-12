"""Topology-derived affinity for owned simulation processes, no control change."""
from pathlib import Path


def select_groups(rows,allowed,count=4):
    groups={}
    for cpu,package,core,frequency in rows:
        if cpu in allowed:
            value=groups.setdefault((package,core),dict(cpus=[],frequency=frequency))
            value['cpus'].append(cpu);value['frequency']=max(value['frequency'],frequency)
    ordered=sorted(groups.values(),key=lambda v:(-v['frequency'],min(v['cpus'])))
    if len(ordered)<count:raise ValueError('four distinct allowed physical CPU cores required')
    return [sorted(v['cpus']) for v in ordered[:count]]


def topology(sysfs=Path('/sys/devices/system/cpu')):
    rows=[]
    for path in sysfs.glob('cpu[0-9]*'):
        number=path.name[3:]
        if not number.isdigit():continue
        try:
            rows.append((int(number),int((path/'topology/physical_package_id').read_text()),
                int((path/'topology/core_id').read_text()),int((path/'cpufreq/cpuinfo_max_freq').read_text())))
        except OSError:continue
    return rows


def role(argv,comm):
    basenames={Path(v).name for v in argv}
    for name,value in [('follower_rotation_integrated','controller'),('rotation_compensation_probe','controller'),
                       ('wrench_preprocessor','preprocessor'),('rotation_observer','observer')]:
        if name in basenames:return value
    if (comm in ('ign gazebo','gz sim','gz-sim-server','ignition-gazebo')
            or any(Path(a).name=='ign' and b=='gazebo' for a,b in zip(argv,argv[1:]))):
        return 'physics'
    return None


def owned(env,partition,domain):
    return (partition.startswith('amir_cp9_cp8r3_') and bool(domain)
        and env.get(b'IGN_PARTITION')==partition.encode() and env.get(b'ROS_DOMAIN_ID')==domain.encode())
