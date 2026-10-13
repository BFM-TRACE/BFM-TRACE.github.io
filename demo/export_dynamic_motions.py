#!/usr/bin/env python3
"""Export isolated genuine native H2 targets and FK-only previews. No dynamics/DDS.
CSV conversion follows build_h2lora_seed_motion_lib.py: named 31 joints, cm/degrees,
120 to 50 Hz time-domain linear interpolation and root SLERP; no G1 projection.
"""
import argparse,ast,hashlib,json
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
import mujoco
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation,Slerp

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,separators=(',',':'))+'\n')
def velocity(q):return np.concatenate((np.diff(q,axis=0)*50,((q[-1]-q[-2])*50)[None]),axis=0).astype(np.float32)
def angular(q):
 r=Rotation.from_quat(q[:,[1,2,3,0]]);v=np.empty((len(q),3));v[0]=(r[1]*r[0].inv()).as_rotvec()*50;v[-1]=(r[-1]*r[-2].inv()).as_rotvec()*50;v[1:-1]=(r[2:]*r[:-2].inv()).as_rotvec()*25;return v.astype(np.float32)

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--root',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--spec',type=Path,required=True);a=ap.parse_args()
 specs=json.loads(a.spec.read_text());r=a.root;o=a.output;o.mkdir(parents=True,exist_ok=True)
 src=r/'scripts/export_h2lora_deploy_reference.py';names=None
 for node in ast.parse(src.read_text()).body:
  if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='H2_MJCF_JOINT_NAMES' for t in node.targets):names=ast.literal_eval(node.value)
 assert len(names)==31
 xml=r/'gear_sonic/data/assets/robot_description/unitree_ros/h2_description/H2_loop.xml';m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
 addresses=[int(m.joint(n).qposadr[0]) for n in names];body_names=[m.body(i).name for i in range(m.nbody)]
 refs={'motions':[],'quaternionOrder':'wxyz','localNames':names,'kind':'Isolated native H2 candidates; physical validation not run','encoder':'native H2 selector 0'}
 preview={'schema':'trace-motion-references-v1','bodyCount':m.nbody,'source':'Native H2 retargeted source','representation':'kinematic reference; not policy rollout','motions':[]}
 for s in specs:
  source=Path(s['source']);name=s.get('id',source.stem if source.is_file() else source.name);fps=50
  if source.is_dir():
   mapping={'rootPosition':'body_pos.csv','rootQuat':'body_quat.csv','q31':'h2_joint_pos.csv','qd31':'h2_joint_vel.csv','rootLinearVelocity':'body_lin_vel.csv','rootAngularVelocity':'body_ang_vel.csv'}
   arrays={key:np.loadtxt(source/file,delimiter=',',skiprows=1).astype(np.float32) for key,file in mapping.items()}
   meta=json.loads((source/'h2_lora_reference.json').read_text());assert meta['source_joint_space']=='h2_physical_31d_mujoco' and meta['fps']==50
   provenance={'source':str(source),'sourceType':'native H2 deployment bundle','sourceFilesSHA256':{f:sha(source/f) for f in [*mapping.values(),'h2_lora_reference.json']},'originalMetadata':meta,'conversion':'CSV values preserved as float32, no coordinate or time changes'}
  else:
   with source.open() as f:header=f.readline().strip().split(',');raw=np.loadtxt(f,delimiter=',',ndmin=2)
   assert np.isfinite(raw).all() and np.all(np.diff(raw[:,0])==1)
   q=np.deg2rad(raw[:,[header.index(n+'_dof') for n in names]]).astype(np.float32);pos=raw[:,1:4].astype(np.float32)/100
   quat=Rotation.from_euler('xyz',raw[:,4:7],degrees=True).as_quat().astype(np.float32)
   st=np.arange(len(raw))/120;tt=np.arange(0,st[-1]+1e-9,1/50);lerp=lambda x:interp1d(st,x,axis=0,kind='linear',bounds_error=False,fill_value=(x[0],x[-1]),assume_sorted=True)(tt).astype(np.float32)
   q=lerp(q);pos=lerp(pos);quat=Slerp(st,Rotation.from_quat(quat))(tt).as_quat().astype(np.float32);quat/=np.linalg.norm(quat,axis=1,keepdims=True);quat=quat[:,[3,0,1,2]]
   arrays={'rootPosition':pos,'rootQuat':quat,'q31':q,'qd31':velocity(q),'rootLinearVelocity':np.gradient(pos.astype(np.float64),1/50,axis=0).astype(np.float32),'rootAngularVelocity':angular(quat)}
   provenance={'source':str(source),'sourceType':'BONES native H2 retargeted CSV','sourceSHA256':sha(source),'sourceFPS':120,'sourceFrames':len(raw),'sourceJointSpace':'h2_physical_31d_mujoco','conversion':'Native 31 named H2 joints; cm to m; degrees to rad; 120->50Hz linear q/position interpolation and quaternion SLERP; no G1 mapping or posture edits','converterReference':'gear_sonic/data_process/build_h2lora_seed_motion_lib.py','converterReferenceSHA256':sha(r/'gear_sonic/data_process/build_h2lora_seed_motion_lib.py')}
  n=len(arrays['q31']);assert all(len(x)==n and np.isfinite(x).all() for x in arrays.values());assert arrays['q31'].shape==(n,31);assert np.max(np.abs(np.linalg.norm(arrays['rootQuat'],axis=1)-1))<1e-5
  provenance.update({'exporterSHA256':sha(__file__),'jointOrderSourceSHA256':sha(src),'generatedAt':datetime.now(timezone.utc).isoformat(),'physicalValidation':'not_run','initialization':'Use unmodified production controller default qpos/qvel; no candidate initialQpos supplied','rootQuaternionOrder':'wxyz','policyReference':'H2 native selector 0; policy maps local31 through controller.policyToLocal'})
  entry={k:v for k,v in s.items() if k not in ('source',)};entry.update({'id':name,'file':name+'.json','frames':n,'fps':50,'duration':n/50,'source':str(source),'kind':s.get('kind','dynamic')})
  dest=o/'references'/entry['file'];save(dest,dict(id=name,fps=fps,frames=n,**{k:v.tolist() for k,v in arrays.items()},provenance=provenance));entry['sha256']=sha(dest);refs['motions'].append(entry)
  poses=[]
  for i in range(0,n,2):
   d.qpos[:]=m.qpos0;d.qpos[:3]=arrays['rootPosition'][i];d.qpos[:2]-=arrays['rootPosition'][0,:2];d.qpos[3:7]=arrays['rootQuat'][i];d.qpos[addresses]=arrays['q31'][i];mujoco.mj_forward(m,d);poses.append(np.concatenate([d.xpos,d.xquat],axis=1))
  binary=np.asarray(poses,dtype='<f4');file=o/'preview'/(name+'.bin');file.parent.mkdir(exist_ok=True);file.write_bytes(binary.tobytes())
  preview['motions'].append({'id':name,'label':s['label'],'file':file.name,'frames':len(binary),'fps':25,'duration':n/50,'sourceFrames':n,'sourceFPS':50,'sourceFile':str(source),'sourceSHA256':entry['sha256'],'previewSHA256':sha(file),'modelSHA256':sha(xml),'note':'FK only; root XY translated to initial origin; auxiliary joints at model defaults, matching existing reference viewer'})
  save(o/'references/manifest.json',refs);save(o/'preview/motions.json',preview)
  print(json.dumps({'id':name,'frames':n,'duration':n/50,'rootHeightRange':[float(arrays['rootPosition'][:,2].min()),float(arrays['rootPosition'][:,2].max())],'rootHorizontalDisplacement':(arrays['rootPosition'][-1,:2]-arrays['rootPosition'][0,:2]).tolist(),'reference':str(dest)}),flush=True)
 save(o/'preview/model-compatibility.json',{'bodyNames':body_names,'bodyCount':m.nbody,'modelSHA256':sha(xml),'mujocoVersion':mujoco.__version__,'compatibleWith':'homepage/demo/assets/h2-model.json; reuse existing visual mesh assets'})
if __name__=='__main__':main()
