#!/usr/bin/env python3
"""Offline SONIC planner targets projected to H2; never controls a simulator.

Only the generated reference targets are exported. Browser motion must still
come from the native encoder/decoder and MuJoCo physics loop.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ort-site',type=Path)
    args=p.parse_args()
    if args.ort_site:sys.path.insert(0,str(args.ort_site))
    import numpy as np
    import onnxruntime as ort
    from scipy.spatial.transform import Rotation, Slerp
    header=(args.root/'gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp').read_text()
    def array(name):
        body=re.search(r'\b'+name+r'\s*=\s*\{(.*?)\};',header,re.S).group(1)
        body=re.sub(r'//[^\n]*','',body)
        return np.asarray([float(x.strip().rstrip('Ff')) for x in body.split(',') if x.strip()])
    g1_order=array('mujoco_to_isaaclab').astype(int)
    h2_order=array('h2lora_body_to_h2_mujoco').astype(int)
    native_default=array('h2_native_default_angles_policy')
    # Exact Phi/legacy comparison maps in policy_parameters.hpp.
    to_g1=np.array([[[.8660245537757874,0],[-.08700508624315262,.9847439527511597]],
                    [[.8660245537757874,0],[ .08700508624315262,.9847439527511597]]])
    to_h2=np.array([[[.8660245537757874,-.08700508624315262],[0,.9847439527511597]],
                    [[.8660245537757874, .08700508624315262],[0,.9847439527511597]]])
    semantic=native_default.copy()
    for side,pair in enumerate(((0,3),(1,4))):semantic[list(pair)]=to_g1[side]@semantic[list(pair)]
    qpos=np.zeros(36,np.float32);qpos[2]=.788740;qpos[3]=1
    qpos[7+g1_order]=semantic
    context=np.repeat(qpos[None,None,:],4,axis=1)
    model=args.root/'gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx'
    opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
    start=time.monotonic()
    session=ort.InferenceSession(str(model),opts,providers=['CPUExecutionProvider'])
    print(f'Loaded planner in {time.monotonic()-start:.2f}s',flush=True)
    pieces=[];calls=[]
    for i in range(7):
        inputs=dict(context_mujoco_qpos=context.astype(np.float32),
            target_vel=np.array([.4],np.float32),mode=np.array([1],np.int64),
            movement_direction=np.array([[1,0,0]],np.float32),
            facing_direction=np.array([[1,0,0]],np.float32),
            random_seed=np.array([9450+i],np.int64),has_specific_target=np.zeros((1,1),np.int64),
            specific_target_positions=np.zeros((1,4,3),np.float32),
            specific_target_headings=np.zeros((1,4),np.float32),
            allowed_pred_num_tokens=np.array([[0,0,0,1,1,1,0,0,0,0,0]],np.int64),
            height=np.array([-1],np.float32))
        t=time.monotonic();outputs=session.run(None,inputs);dt=time.monotonic()-t
        count=int(outputs[1].ravel()[0]);pred=outputs[0][0,:count].copy()
        if not np.isfinite(pred).all():raise RuntimeError('Planner emitted nonfinite values')
        overlap=float(np.max(np.abs(pred[:4]-context[0])))
        calls.append(dict(seed=9450+i,frames30Hz=count,seconds=dt,contextMaxDifference=overlap))
        print(json.dumps(calls[-1]),flush=True)
        # The learned decoder reconstructs context approximately, rather than
        # copying it exactly. Keep that discrepancy in provenance. The final
        # browser reference uses one prediction window, never a stitched seam.
        if overlap>1:raise RuntimeError('Planner context reconstruction diverged')
        pieces.append(pred if i==0 else pred[4:])
        context=pred[None,-4:,:]
    rollout=np.concatenate(pieces)
    raw=pred
    t30=np.arange(len(raw))/30
    t50=np.arange(int((len(raw)-1)/30*50)+1)/50
    root=np.stack([np.interp(t50,t30,raw[:,j]) for j in range(3)],axis=1)
    raw[:,3:7]/=np.linalg.norm(raw[:,3:7],axis=1,keepdims=True)
    rootquat=Slerp(t30,Rotation.from_quat(raw[:,[4,5,6,3]]))(t50).as_quat()[:,[3,0,1,2]]
    g1sem=np.stack([np.interp(t50,t30,raw[:,7+j]) for j in g1_order],axis=1)
    for side,pair in enumerate(((0,3),(1,4))):g1sem[:,pair]=g1sem[:,pair]@to_h2[side].T
    q31=np.zeros((len(t50),31));q31[:,h2_order]=g1sem
    qd31=np.diff(q31,axis=0,append=q31[-1:])*50;qd31[-1]=qd31[-2]
    linvel=np.diff(root,axis=0,append=root[-1:])*50;linvel[-1]=linvel[-2]
    rots=Rotation.from_quat(rootquat[:,[1,2,3,0]])
    angvel=np.zeros((len(t50),3));angvel[:-1]=(rots[:-1].inv()*rots[1:]).as_rotvec()*50;angvel[-1]=angvel[-2]
    # Recommend a same-phase interval without changing any generated frame.
    best=None
    for a in range(0,len(q31)-35):
        for length in range(35,min(100,len(q31)-a)):
            b=a+length
            speed=(root[b,0]-root[a,0])/(length/50)
            if not .2<speed<.8:continue
            score=float(np.mean((q31[a]-q31[b])**2)+.01*np.mean((qd31[a]-qd31[b])**2))
            if best is None or score<best['score']:
                best=dict(startFrame=a,endFrame=b,frames=length,score=score,meanForwardSpeed=speed)
    document=dict(id='planner_walk_04',fps=50,frames=len(t50),rootPosition=root.tolist(),
        rootQuat=rootquat.tolist(),q31=q31.tolist(),qd31=qd31.tolist(),
        rootLinearVelocity=linvel.tolist(),rootAngularVelocity=angvel.tolist(),recommendedLoop=best,
        provenance=dict(kind='Offline generated learned-planner reference; not a simulator rollout',
            plannerSource=str(model.relative_to(args.root)),plannerSHA256=hashlib.sha256(model.read_bytes()).hexdigest(),
            plannerTargetSpeed=.4,plannerMode=1,generatorFPS=30,resampledFPS=50,
            window='Final single prediction after seven autoregressive warmup calls; no stitched seams',
            projection='G1PlannerReferenceToH2NativePolicy (legacy explicit comparison map)',
            calls=calls,onnxRuntime=ort.__version__,executionProvider='CPUExecutionProvider'))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'planner_walk_04.json').write_text(json.dumps(document,indent=2)+'\n')
    manifest_path=args.output/'manifest.json'
    if manifest_path.exists():
        manifest=json.loads(manifest_path.read_text())
        manifest['motions']=[entry for entry in manifest['motions'] if entry['id']!='planner_walk_04']
        manifest['motions'].append(dict(id='planner_walk_04',file='planner_walk_04.json',
            frames=len(t50),fps=50,source=document['provenance'],recommendedLoop=best,
            label='Slow forward walk · generated reference'))
        manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    np.save(args.output/'planner_walk_04_raw_g1.npy',raw)
    np.save(args.output/'planner_walk_04_warmup_g1.npy',rollout)
    print(json.dumps(dict(exported=str(args.output/'planner_walk_04.json'),frames=len(t50),loop=best)),flush=True)


if __name__=='__main__':main()
