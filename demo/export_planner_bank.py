#!/usr/bin/env python3
"""Export offline SONIC planner candidates; never activate a browser motion bank.

Runs the original planner with CPU ONNX Runtime. Moving modes 1/2/3 use the
deployment's 9-11 token mask, fixed seed, current-cursor context and blend timing,
assuming zero inference latency. Exports are genuine reference targets, not H2
physical trajectories. Every selected candidate still requires H2 closed-loop
validation, including movement, posture, direction, transitions and stopping.
No simulation, DDS, control socket, robot connection or active manifest is used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time
import platform
from datetime import datetime, timezone


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True, help='Original SONIC-H2 checkout containing planner weights and deployment headers')
    p.add_argument('--output', type=Path, required=True, help='Isolated candidate directory; an active bank manifest is never updated')
    p.add_argument('--ort-site', type=Path, help='Optional Python site directory containing onnxruntime')
    p.add_argument('--mode', type=int, choices=[1,2,3], default=3, help='Moving style: 1 slow walk, 2 walk, 3 run')
    p.add_argument('--direction', choices=['forward','back','left','right','forward_left','forward_right'], default='forward')
    p.add_argument('--speeds', nargs='+', type=float, default=[2.5], help='Positive planner target speeds in m/s, not guaranteed H2 execution speeds')
    p.add_argument('--seconds', type=float, default=12, help='Continuous reference duration; first 4 seconds are warmup')
    p.add_argument('--seed', type=int, default=1234, help='Fixed random seed reused on every replan')
    p.add_argument('--compare-tail', action='store_true', help='Explicit historical A/B diagnostic: compare old tail context with 16 versus 9-11 tokens; never export its output as a candidate')
    p.add_argument('--rescore', action='store_true', help='Reuse saved continuous model output; do not run inference')
    p.add_argument('--posture-mode', choices=['upright','stable-lean'], default='upright')
    p.add_argument('--peak-prominence', type=float, default=.12, help='Left-knee peak prominence in radians for source stride extraction; not a physical acceptance threshold')
    p.add_argument('--min-directional-speed', type=float, default=.1, help='Source reference speed screening in m/s; does not establish actual H2 movement')
    args = p.parse_args()
    if not 6 <= args.seconds < float('inf') or any(not (0 < speed < float('inf')) for speed in args.speeds):
        p.error('--seconds must be >= 6 and --speeds must be finite and positive')
    if not 0 < args.peak_prominence < float('inf') or not 0 <= args.min_directional_speed < float('inf'):
        p.error('Source screening values must be finite; peak prominence must be positive and minimum speed nonnegative')
    if args.compare_tail and args.rescore:
        p.error('--compare-tail runs inference and cannot be combined with --rescore')
    if (args.output / 'manifest.json').exists():
        p.error('Choose an isolated candidate directory, not a directory containing an active manifest.json')
    if args.ort_site:
        sys.path.insert(0, str(args.ort_site))
    import numpy as np
    import onnxruntime as ort
    from scipy.spatial.transform import Rotation, Slerp
    from scipy.signal import find_peaks
    import scipy

    args.output.mkdir(parents=True, exist_ok=True)
    model = args.root / 'gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx'
    header_path = args.root / 'gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp'
    header = header_path.read_text()
    def array(name):
        body = re.search(r'\b'+name+r'\s*=\s*\{(.*?)\};', header, re.S).group(1)
        body = re.sub(r'//[^\n]*', '', body)
        return np.array([float(x.strip().rstrip('Ff')) for x in body.split(',') if x.strip()])
    g1_order = array('mujoco_to_isaaclab').astype(int)
    h2_order = array('h2lora_body_to_h2_mujoco').astype(int)
    initial_semantic = array('h2_native_default_angles_policy')
    to_g1 = np.array([[[.8660245537757874,0],[-.08700508624315262,.9847439527511597]],[[.8660245537757874,0],[.08700508624315262,.9847439527511597]]])
    to_h2 = np.array([[[.8660245537757874,-.08700508624315262],[0,.9847439527511597]],[[.8660245537757874,.08700508624315262],[0,.9847439527511597]]])
    for side, pair in enumerate(((0,3),(1,4))): initial_semantic[list(pair)] = to_g1[side] @ initial_semantic[list(pair)]
    initial = np.zeros(36, np.float32); initial[2] = .788740; initial[3] = 1
    initial[7+g1_order] = initial_semantic
    opts = ort.SessionOptions(); opts.intra_op_num_threads = 2; opts.inter_op_num_threads = 1
    session = None if args.rescore else ort.InferenceSession(str(model), opts, providers=['CPUExecutionProvider'])
    model_hash = hashlib.sha256(model.read_bytes()).hexdigest()
    script_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    header_hash = hashlib.sha256(header_path.read_bytes()).hexdigest()
    normal_mask = np.array([[0,0,0,1,1,1,0,0,0,0,0]], np.int64)
    long_mask = np.array([[0,0,0,0,0,0,0,0,0,0,1]], np.int64)
    movement=np.array({'forward':[1,0,0],'back':[-1,0,0],'left':[0,1,0],'right':[0,-1,0],
                       'forward_left':[2**-.5,2**-.5,0],'forward_right':[2**-.5,-2**-.5,0]}[args.direction])

    def infer(context, mode, speed, mask, seed):
        inputs = dict(context_mujoco_qpos=context[None].astype(np.float32),
                      target_vel=np.array([speed],np.float32),mode=np.array([mode],np.int64),
                      movement_direction=np.array([movement if mode else [0,0,0]],np.float32),
                      facing_direction=np.array([[1,0,0]],np.float32),random_seed=np.array([seed],np.int64),
                      has_specific_target=np.zeros((1,1),np.int64),specific_target_positions=np.zeros((1,4,3),np.float32),
                      specific_target_headings=np.zeros((1,4),np.float32),allowed_pred_num_tokens=mask,height=np.array([-1],np.float32))
        result = session.run(None, inputs); count = int(result[1][0]); raw = result[0][0,:count].copy()
        assert count == len(raw) and np.isfinite(raw).all()
        return raw

    def sample(raw, indices):
        # Official linear position/joint interpolation and quaternion SLERP.
        n = len(raw); at = np.clip(indices,0,n-1)
        q = raw[:,3:7] / np.linalg.norm(raw[:,3:7],axis=1,keepdims=True)
        result = np.stack([np.interp(at,np.arange(n),raw[:,j]) for j in range(36)],axis=1)
        result[:,3:7] = Slerp(np.arange(n),Rotation.from_quat(q[:,[1,2,3,0]]))(at).as_quat()[:,[3,0,1,2]]
        return result

    def to50(raw):
        return sample(raw,np.arange(int(np.floor(len(raw)*50/30)))*30/50)

    def blend(old, new, cursor, offset=2):
        # CurrentFrameAdvancement, zero inference latency: old cursor is the
        # mode-specific number of control frames between periodic replans;
        # gen_frame=cursor+2; cross-fade begins after 2 frames and lasts 8.
        count = offset+len(new)
        a = old[np.minimum(np.arange(count)+cursor,len(old)-1)]
        b = new[np.clip(np.arange(count)-offset,0,len(new)-1)]
        weight = np.clip((np.arange(count)-offset)/8,0,1)
        result = (1-weight[:,None])*a+weight[:,None]*b
        # Batched shortest-path SLERP, equivalent to the deployment operation.
        qa=a[:,3:7].copy(); qb=b[:,3:7].copy()
        qa/=np.linalg.norm(qa,axis=1,keepdims=True);qb/=np.linalg.norm(qb,axis=1,keepdims=True)
        dot=np.sum(qa*qb,axis=1); qb[dot<0]*=-1; dot=np.clip(np.abs(dot),0,1)
        theta=np.arccos(dot); sine=np.sin(theta); close=dot>.9995
        factors=np.ones_like(dot); factors[~close]=1/sine[~close]
        out=np.sin((1-weight)*theta)[:,None]*factors[:,None]*qa+np.sin(weight*theta)[:,None]*factors[:,None]*qb
        out[close]=(1-weight[close,None])*qa[close]+weight[close,None]*qb[close]
        result[:,3:7]=out/np.linalg.norm(out,axis=1,keepdims=True)
        return result

    def angle_series(data):
        q=data[:,3:7]/np.linalg.norm(data[:,3:7],axis=1,keepdims=True)
        return Rotation.from_quat(q[:,[1,2,3,0]]).as_euler('xyz')

    def statistics(data, fps):
        angles=angle_series(data); joints=data[:,7+g1_order]; waist=joints[:,8]
        stats={'frames':len(data),'seconds':(len(data)-1)/fps,
               'meanVelocity':((data[-1,:3]-data[0,:3])/((len(data)-1)/fps)).tolist(),
               'rootRPYMinDegrees':np.rad2deg(angles.min(axis=0)).tolist(),
               'rootRPYMaxDegrees':np.rad2deg(angles.max(axis=0)).tolist(),
               'waistPitchDegrees':[float(np.rad2deg(waist.min())),float(np.rad2deg(waist.max()))]}
        peaks,_=find_peaks(joints[:,9],distance=max(1,int(.3*fps)),prominence=.1)
        stats['leftKneePeakTimes']=(peaks/fps).tolist()
        stats['medianLeftKneePeriodSeconds']=float(np.median(np.diff(peaks))/fps) if len(peaks)>1 else None
        return stats

    def select_cycle(data, fps=50):
        q=data[:,7+g1_order]; dq=np.diff(q,axis=0,append=q[-1:])*fps; dq[-1]=dq[-2]
        angles=angle_series(data); best=None
        min_len,max_len={3:(16,43),2:(30,85),1:(35,105)}[args.mode]
        peaks,_=find_peaks(q[:,9],distance=min_len,prominence=args.peak_prominence)
        # Start after the 4 second warm-up and use one full left-leg stride.
        for a,b in zip(peaks,peaks[1:]):
            if a<4*fps: continue
            for b in [b]:
                if b-a<min_len or b-a>max_len: continue
                region=angles[a:b+1]; waist=q[a:b+1,8]
                if args.posture_mode == 'upright':
                    if np.max(np.abs(region[:,1]))>np.deg2rad(15): continue
                    if np.max(np.abs(waist))>np.deg2rad(22): continue
                else:
                    # Permit honest steady running lean, never clamp source angles.
                    if np.max(np.abs(region[:,1]))>np.deg2rad(45): continue
                    if np.ptp(region[:,1])>np.deg2rad(15): continue
                    if np.max(np.abs(waist))>np.deg2rad(35): continue
                    if np.ptp(waist)>np.deg2rad(15): continue
                if np.max(np.abs(region[:,0]))>np.deg2rad(15): continue
                if np.max(np.abs(region[:,2]))>np.deg2rad(12): continue
                velocity=(data[b,:3]-data[a,:3])/((b-a)/fps)
                parallel=float(velocity@movement)
                lateral=float(np.linalg.norm(velocity[:2]-parallel*movement[:2]))
                if parallel<args.min_directional_speed or lateral>.3*parallel: continue
                qerror=float(np.mean((q[a]-q[b])**2)); dqerror=float(np.mean((dq[a]-dq[b])**2))
                score=qerror+.01*dqerror
                if best is None or score<best['score']:
                    best=dict(startFrame=int(a),endFrame=int(b),frames=int(b-a),score=score,
                              meanVelocity=velocity.tolist(),qMeanSquaredError=qerror,dqMeanSquaredError=dqerror,
                              seamVelocityErrorRMS=float(np.sqrt(qerror)*fps),
                              rootRPYMinDegrees=np.rad2deg(region.min(axis=0)).tolist(),
                              rootRPYMaxDegrees=np.rad2deg(region.max(axis=0)).tolist(),
                              waistPitchDegrees=[float(np.rad2deg(waist.min())),float(np.rad2deg(waist.max()))],
                              rootRPYPeakToPeakDegrees=np.rad2deg(np.ptp(region,axis=0)).tolist(),
                              waistPitchPeakToPeakDegrees=float(np.rad2deg(np.ptp(waist))),
                              postureScreen=args.posture_mode,
                              singleCycle=True,strideCount=1,meanDirectionalSpeed=parallel,
                              phaseAnchor='Consecutive left-knee swing peaks; one complete same-leg stride')
        return best

    settings={'mode':args.mode,'direction':args.direction,'speeds':args.speeds,'seed':args.seed,
              'durationSeconds':args.seconds,'warmupSeconds':4,'postureScreen':args.posture_mode,
              'peakProminenceRadians':args.peak_prominence,'minimumReferenceSpeed':args.min_directional_speed,
              'rescoreOnly':args.rescore,'legacyTailAblation':args.compare_tail,
              'plannerFPS':30,'controlFPS':50,'lookaheadFrames':2,'blendFrames':8,
              'allowedTokens':[9,10,11],'inferenceLatencyAssumptionSeconds':0}
    report={'generatedAt':datetime.now(timezone.utc).isoformat(),'modelSHA256':model_hash,
            'source':'gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx','provider':'CPUExecutionProvider',
            'sourceScriptSHA256':script_hash,'policyParametersSHA256':header_hash,
            'versions':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'onnxruntime':ort.__version__},
            'settings':settings,'physicalValidation':'not_run','activation':'not_performed','experiments':[]}
    report_path=args.output/'planner-candidate-report.json'
    for speed in args.speeds:
        if args.compare_tail:
            for label,mask in [('tail_16tokens',long_mask),('tail_default_tokens',normal_mask)]:
                context=np.repeat(initial[None],4,axis=0)
                for i in range(16):
                    raw=infer(context,args.mode,speed,mask,args.seed+i)
                    context=raw[-4:]
                result={'strategy':label,'mode':args.mode,'speedCommand':speed,**statistics(raw,30)}
                report['experiments'].append(result);print(json.dumps(result),flush=True)
        for label,mask in [('deployment_default_tokens',normal_mask)]:
            start=time.monotonic();counts=[];interval=.1 if args.mode==3 else 1.
            name=f'planner_mode{args.mode}_{args.direction}_speed{speed:g}'.replace('.','p')
            continuous_path=args.output/(name+'_continuous_g1.npy')
            generation_path=args.output/(name+'_continuous-provenance.json')
            if args.rescore:
                if not generation_path.exists():
                    p.error('--rescore requires the matching continuous-provenance.json emitted by this exporter; old audit outputs need their original extractor')
                generation=json.loads(generation_path.read_text())
                expected={'mode':args.mode,'direction':args.direction,'targetSpeed':speed,'fixedSeed':args.seed,
                          'modelSHA256':model_hash,'policyParametersSHA256':header_hash}
                if any(generation.get(key)!=value for key,value in expected.items()):
                    p.error(f'Generation settings differ from the recorded source: {generation_path}')
                if hashlib.sha256(continuous_path.read_bytes()).hexdigest()!=generation['continuousSHA256']:
                    p.error(f'Continuous source hash mismatch: {continuous_path}')
                continuous=np.load(continuous_path)
                counts=generation['predictionFrameCounts']
                steady=continuous[int(4*50):]
            else:
                start=time.monotonic();raw=infer(np.repeat(initial[None],4,axis=0),0,-1,mask,args.seed)
                buffer=to50(raw);played=[];counts=[]
                # C++ periodic replan intervals: RUN 0.1 s, ordinary walk 1 s.
                interval=.1 if args.mode==3 else 1.
                cursor=round(interval*50)
                for i in range(int(args.seconds/interval)):
                    played.append(sample(buffer,np.arange(cursor)))
                    context=sample(buffer,cursor+2+np.arange(4)*50/30)
                    raw=infer(context,args.mode,speed,mask,args.seed);counts.append(len(raw))
                    buffer=blend(buffer,to50(raw),cursor)
                continuous=np.concatenate(played); steady=continuous[int(4*50):]
                np.save(continuous_path,continuous)
                generation={'generatedAt':datetime.now(timezone.utc).isoformat(),'modelSHA256':model_hash,
                            'policyParametersSHA256':header_hash,'generatorScriptSHA256':script_hash,
                            'continuousFile':continuous_path.name,'continuousSHA256':hashlib.sha256(continuous_path.read_bytes()).hexdigest(),
                            'mode':args.mode,'direction':args.direction,'targetSpeed':speed,'fixedSeed':args.seed,
                            'settings':settings,'predictionFrameCounts':sorted(set(counts)),'versions':report['versions']}
                generation_path.write_text(json.dumps(generation,indent=2)+'\n')
            cycle=select_cycle(continuous)
            result={'strategy':label,'mode':args.mode,'direction':args.direction,'speedCommand':speed,'secondsToGenerate':time.monotonic()-start,
                    'fixedSeed':args.seed,'rescoreOnly':args.rescore,'postureScreen':args.posture_mode,'predictionFrameCounts':sorted(set(counts)),
                    'steady':statistics(steady,50),'cycle':cycle}
            result['continuousFile']=continuous_path.name
            result['continuousSHA256']=generation['continuousSHA256']
            result['generationProvenanceFile']=generation_path.name
            result['generationProvenanceSHA256']=hashlib.sha256(generation_path.read_bytes()).hexdigest()
            result['sourceScreen']='candidate_requires_physical_validation' if cycle else 'no_qualifying_source_stride'
            if cycle:
                q=continuous[:,7+g1_order];dq=np.diff(q,axis=0,append=q[-1:])*50;dq[-1]=dq[-2]
                h2=q.copy()
                for side,pair in enumerate(((0,3),(1,4))):h2[:,pair]=h2[:,pair]@to_h2[side].T
                q31=np.zeros((len(q),31));q31[:,h2_order]=h2
                dq31=np.diff(q31,axis=0,append=q31[-1:])*50;dq31[-1]=dq31[-2]
                document=dict(id=name,fps=50,frames=len(q),mode=args.mode,direction=args.direction,movement=movement.tolist(),facing=[1,0,0],targetSpeed=speed,
                              rootPosition=continuous[:,:3].tolist(),rootQuat=continuous[:,3:7].tolist(),q29G1=q.tolist(),qd29G1=dq.tolist(),q31=q31.tolist(),qd31=dq31.tolist(),recommendedLoop=cycle,
                              provenance={'kind':'Offline continuous learned-planner reference, not physical state','modelSHA256':model_hash,
                                          'policyParametersSHA256':header_hash,'generation':generation,
                                          'strategy':'Fixed seed, official 9-11 token mask, 2-frame look-ahead, 4x30Hz context, 8x50Hz crossfade, zero inference latency assumption',
                                          'plannerCallsHz':1/interval,'fixedSeed':args.seed,'scriptSHA256':report['sourceScriptSHA256'],
                                          'extractionOnly':args.rescore,'continuousFile':result['continuousFile'],'continuousSHA256':result['continuousSHA256'],
                                          'extractionSettings':settings,'physicalValidation':'not_run','activation':'not_performed',
                                          'warning':'H2 physical closed-loop behavior is not validated by this export'})
                candidate_path=args.output/(name+'.json')
                candidate_path.write_text(json.dumps(document,separators=(',',':'))+'\n')
                result['candidateFile']=candidate_path.name
                result['candidateSHA256']=hashlib.sha256(candidate_path.read_bytes()).hexdigest()
            elif (args.output/(name+'.json')).exists():
                # A rejected re-extraction must not leave an old candidate under
                # the expected filename. Preserve it for audit, never activate it.
                old_path=args.output/(name+'.json')
                archived=args.output/(name+'.previous-'+hashlib.sha256(old_path.read_bytes()).hexdigest()[:12]+'.json')
                old_path.replace(archived)
                result['previousCandidateArchivedAs']=archived.name
            report['experiments'].append(result);print(json.dumps(result),flush=True)
            report_path.write_text(json.dumps(report,indent=2)+'\n')
    report_path.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
