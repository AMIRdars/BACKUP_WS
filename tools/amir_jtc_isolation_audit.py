#!/usr/bin/env python3
"""Offline evaluator: detect JTC references outside recent local command knots.

The 120 ms window exceeds the native 84 ms trajectory and includes 20 ms
source-clock skew. Align inputs by their absolute first-knot time, since the
recorder may dequeue them later than the controller. This is a diagnostic
envelope, not an additional controller.
Exact matches to the peer command identify the observed cross-instance race.
"""
import argparse
import bisect
import json
from pathlib import Path


def audit(case):
    local=[];peer=[]
    for line in (case/'events.jsonl').open():
        row=json.loads(line)
        if row['phase']!='leader_translation':continue
        if row['kind']=='arm_actuator_cmd' and row['robot']=='follower':
            value=row['value'];points=value['points']
            if value['stamp_s']>0 and len(points)==2:
                local.append((value['stamp_s']+points[0]['time_from_start_s'],[p['positions'] for p in points]))
        if row['kind']=='arm_cmd' and row['robot']=='leader':
            peer.append((row['sim_time'],row['value']))
    local.sort();peer.sort();local_t=[x[0] for x in local];peer_t=[x[0] for x in peer]
    checked=0;outside=0;cross=0;max_escape=0.;first=None
    for line in (case/'jtc_state_samples.jsonl').open():
        row=json.loads(line)
        if row['robot']!='follower' or row['phase']!='leader_translation':continue
        t=row['stamp_s'];q=row['reference_positions']
        if not local_t or t<local_t[0]+.12:continue
        nearby=local[bisect.bisect_left(local_t,t-.12):bisect.bisect_right(local_t,t+.02)]
        if not nearby:continue
        knots=[point for _,points in nearby for point in points]
        escape=max(max(min(p[j] for p in knots)-q[j],q[j]-max(p[j] for p in knots),0.) for j in range(5))
        checked+=1;max_escape=max(max_escape,escape)
        if escape<=1e-7:continue
        outside+=1
        candidates=peer[bisect.bisect_left(peer_t,t-.02):bisect.bisect_right(peer_t,t+.02)]
        matched=any(max(abs(a-b) for a,b in zip(q,p))<1e-8 for _,p in candidates)
        cross+=int(matched)
        if first is None:first={'stamp_s':t,'escape_rad':escape,'reference_rad':q,'matches_peer_command':matched}
    return {'checked_follower_reference_samples':checked,'outside_local_knot_envelope':outside,
            'exact_peer_command_matches_outside_local_envelope':cross,'max_escape_rad':max_escape,
            'first_outside_sample':first,'window_s':[-.12,.02],'tolerance_rad':1e-7,
            'alignment':'local input absolute first-knot time versus JTC source stamp',
            'passed':bool(checked) and outside==0,
            'scope':'offline reference-envelope diagnostic; not a full concurrent scheduling proof'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--case',type=Path,required=True)
    args=parser.parse_args();result=audit(args.case)
    (args.case/'jtc_isolation_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
