"""Reap only processes in this exact, explicitly isolated experiment partition."""
import os
from pathlib import Path
import signal
import time


def reap_partition(partition, domain):
    if not partition.startswith(('amir_cp8r3_', 'amir_cp9_cp8r3_')) or not domain:
        raise ValueError('exact owned simulation partition and ROS domain required')
    def matching():
        result = []
        for item in Path('/proc').iterdir():
            if not item.name.isdigit() or int(item.name) == os.getpid():
                continue
            try:
                env = dict(v.split(b'=', 1) for v in (item/'environ').read_bytes().split(b'\0') if b'=' in v)
                if env.get(b'IGN_PARTITION') == partition.encode() and env.get(b'ROS_DOMAIN_ID') == domain.encode():
                    result.append(int(item.name))
            except OSError:
                pass
        return result
    found = matching()
    for pid in found:
        try: os.kill(pid, signal.SIGINT)
        except ProcessLookupError: pass
    deadline = time.monotonic()+2.
    while matching() and time.monotonic() < deadline:
        time.sleep(.05)
    killed = matching()
    for pid in killed:
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
    return {'partition': partition, 'domain': domain, 'reaped_pids': found, 'kill_required_pids': killed}
