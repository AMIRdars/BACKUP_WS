#!/usr/bin/env python3
"""Record only the Gazebo window owned by one isolated trial, without desktop input."""
import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import time
import cv2
import numpy as np


def window(partition):
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    for wid in re.findall(r'(0x[0-9a-fA-F]+) "Gazebo":', tree):
        value = subprocess.check_output(['xprop', '-id', wid, '_NET_WM_PID'], text=True)
        match = re.search(r'= (\d+)', value)
        if not match:
            continue
        try:
            if ('IGN_PARTITION='+partition).encode() in Path('/proc/'+match[1]+'/environ').read_bytes().split(b'\0'):
                return wid, int(match[1])
        except OSError:
            continue
    return None


def capture(wid):
    data = subprocess.check_output(['xwd', '-silent', '-id', wid])
    header = struct.unpack('>25I', data[:100])
    if header[7] != 0 or header[11] not in (24, 32):
        raise ValueError('unsupported owned window XWD format')
    offset = header[0]+header[19]*12
    pixels = np.frombuffer(data[offset:], dtype=np.uint8).reshape(header[5], header[12])
    return pixels[:, :header[4]*(header[11]//8)].reshape(header[5], header[4], header[11]//8)[:, :, :3].copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=300.)
    parser.add_argument('--fps', type=float, default=2.)
    parser.add_argument('--include-preparation',action='store_true')
    parser.add_argument('--title',default='ARM_ONLY: base assist OFF')
    args = parser.parse_args()
    if not 0 < args.duration_s <= 1800 or not 0 < args.fps <= 10:
        parser.error('duration must be (0,1800] s and frame rate (0,10] Hz')
    if args.output.exists():
        parser.error('video output already exists')
    partition = json.loads((args.case/'command.json').read_text())['env']['IGN_PARTITION']
    deadline = time.monotonic()+args.duration_s
    owned = None
    while time.monotonic() < deadline:
        owned = window(partition)
        if owned and (args.include_preparation or (args.case/'graph.json').exists()):
            break
        time.sleep(.5)
    else:
        raise RuntimeError('no audited owned Gazebo window found')
    wid, pid = owned
    frame = capture(wid)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*'MJPG'), args.fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError('OpenCV MJPG video writer unavailable')
    records = []; paused_since = None; started = time.monotonic()
    try:
        cv2.imwrite(str(args.output.with_suffix('.first.png')), frame)
        while time.monotonic() < deadline:
            tick = time.monotonic()
            if not Path('/proc/'+str(pid)).exists():
                break
            frame = capture(wid)
            sample = {'frame': len(records), 'wall_elapsed_s': tick-started}
            source = args.case/'control_samples.jsonl'
            if source.exists() and source.stat().st_size:
                with source.open('rb') as stream:
                    stream.seek(max(0, source.stat().st_size-16000))
                    value = json.loads(stream.read().splitlines()[-1])['value']
                sample.update(sim_time=value['sim_time'], mode=value.get('phase_mode'))
            text = args.title+' | wall +%.2fs | sim %s | %s' % (
                sample['wall_elapsed_s'], sample.get('sim_time','initializing'), sample.get('mode','INIT'))
            cv2.putText(frame, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 2)
            writer.write(frame); records.append(sample)
            if (args.case/'paused_review_proof.json').exists():
                paused_since = paused_since or tick
                if tick-paused_since >= 3.:
                    break
            time.sleep(max(0., 1/args.fps-(time.monotonic()-tick)))
        cv2.imwrite(str(args.output.with_suffix('.last.png')), frame)
    finally:
        writer.release()
        args.output.with_suffix('.json').write_text(json.dumps({
            'partition':partition, 'window':wid, 'pid':pid, 'fps':args.fps, 'frames':records,
            'source':'owned window XGetImage only; no desktop capture or input events',
            'time_axis':'wall time; exact wall/simulation frame samples provided, no realtime-physics claim'}, indent=2))
    print(args.output)


if __name__ == '__main__':
    main()
