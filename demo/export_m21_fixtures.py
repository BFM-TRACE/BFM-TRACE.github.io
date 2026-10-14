#!/usr/bin/env python3
"""Source-side sampling/orientation fixtures, independent of browser JS."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--root',type=Path,required=True)
ap.add_argument('--bundle',type=Path,required=True)
ap.add_argument('--output',type=Path,required=True)
a=ap.parse_args()
sys.path.insert(0,str(a.root/'src'))
from interface.motion_replay.motion import Motion
manifest=json.loads((a.bundle/'references/manifest.json').read_text())
base=Rotation.from_euler('xyz',[.07,-.05,.3])
base_quat=base.as_quat()[[3,0,1,2]].tolist()
fixtures=[]
for entry in manifest['motions']:
    path=a.root/entry['source']
    if (path/'replay.json').exists():
        path=path/json.loads((path/'replay.json').read_text())['motion']['file']
    motion=Motion.load(path)
    root0=Rotation.from_quat(motion.root[0][[1,2,3,0]])
    alignment=Rotation.from_euler('z',base.as_euler('xyz')[2]-root0.as_euler('xyz')[2])
    for elapsed in [0,.14,motion.duration]:
        sample=motion.sample(elapsed)
        roots=Rotation.from_quat(np.asarray(sample['root_quat'])[:,[1,2,3,0]])
        relative=base.inv()*alignment*roots
        ori=relative.as_matrix()[:,:,:2].reshape(-1)
        full=np.zeros(1751,dtype=np.float32)
        selector={'h2':0,'g1':3,'smpl':2}[motion.reference_type]
        full[0]=selector
        if selector<3: full[1+selector]=1
        if selector==2:
            joints=np.asarray(sample['joints'])
            transformed=np.stack([roots[i].inv().apply(joints[i]) for i in range(10)])
            full[911:1631]=transformed.reshape(-1)
            full[1631:1691]=ori
            full[1691:1751]=np.asarray(sample['wrist_joints']).reshape(-1)
        else:
            full[4:294]=np.asarray(sample['joint_pos']).reshape(-1)
            full[294:584]=np.asarray(sample['joint_vel']).reshape(-1)
            full[590:650]=ori
        fixtures.append(dict(id=entry['id'],space=motion.reference_type,elapsed=elapsed,
            baseQuat=base_quat,fullObservation=full.tolist()))
a.output.write_text(json.dumps(dict(source='SONIC_unitree_h2 Motion.sample + released observations.hpp layout',fixtures=fixtures),separators=(',',':'))+'\n')
print('Exported',len(fixtures),'source sampling fixtures')
