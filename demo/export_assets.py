#!/usr/bin/env python3
"""Export H2 mesh and real motion references; run inside the SONIC-H2 checkout.

No simulation process, network interface, or robot connection is started.
Reference playback is kinematic and is intentionally labelled as such in the UI.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root, out = args.root, args.output
    out.mkdir(parents=True, exist_ok=True)
    xml = root / 'gear_sonic/data/assets/robot_description/unitree_ros/h2_description/H2_loop.xml'
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    meshes, blobs, byte_offset = [], [], 0
    for i in range(model.nmesh):
        a, n = model.mesh_vertadr[i], model.mesh_vertnum[i]
        fa, fn = model.mesh_faceadr[i], model.mesh_facenum[i]
        vertices = model.mesh_vert[a:a+n].astype('<f4')
        faces = model.mesh_face[fa:fa+fn].astype('<u4')
        meshes.append(dict(name=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, i),
                           vertexOffset=byte_offset, vertexCount=int(n),
                           indexOffset=byte_offset+vertices.nbytes, indexCount=int(fn*3)))
        blobs.extend((vertices.tobytes(), faces.tobytes()))
        byte_offset += vertices.nbytes + faces.nbytes
    (out/'h2-meshes.bin').write_bytes(b''.join(blobs))
    geoms = []
    seen = set()
    for i in range(model.ngeom):
        if model.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        body, mesh = int(model.geom_bodyid[i]), int(model.geom_dataid[i])
        if (body, mesh) in seen:
            continue
        seen.add((body, mesh))
        mesh_name = meshes[mesh]['name']
        # Only mesh visuals are exported; nonmesh mechanism/debug geoms are
        # excluded. The official visible shell is preserved without scaling.
        dark = any(s in mesh_name for s in ('hand','wrist_yaw','ankle_pitch','hip_pitch','Link-pitch'))
        geoms.append(dict(body=body, mesh=mesh, position=model.geom_pos[i].tolist(),
                          quaternion=model.geom_quat[i].tolist(), dark=dark))
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]
    model_meta = dict(schema='trace-h2-mesh-v1', bodies=body_names, meshes=meshes, geoms=geoms,
                      model='H2_loop.xml', sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
                      quaternionOrder='wxyz', upAxis='z')
    (out/'h2-model.json').write_text(json.dumps(model_meta, separators=(',',':')))
    simple = mujoco.MjModel.from_xml_path(str(xml.with_name('h2_mujoco.xml')))
    names = [mujoco.mj_id2name(simple, mujoco.mjtObj.mjOBJ_JOINT,i) for i in range(1,simple.njnt)]
    addresses = [int(model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)]) for n in names]
    motions = [('squat_001__A359','Squat'),('forward_lunge_R_001__A359_M','Lunge'),
               ('dance_in_da_party_001__A464','Dance'),('macarena_001__A545','Macarena'),
               ('walking_quip_360_R_002__A428','Walk & turn')]
    manifest = dict(schema='trace-motion-references-v1', bodyCount=int(model.nbody),
                    source='SONIC-H2 native H2 retargeted references',
                    representation='kinematic reference; not policy rollout', motions=[])
    for name,label in motions:
        folder = root/'gear_sonic_deploy/reference/example_h2/deploy'/name
        positions = np.loadtxt(folder/'body_pos.csv', delimiter=',', skiprows=1)
        quats = np.loadtxt(folder/'body_quat.csv', delimiter=',', skiprows=1)
        joints = np.loadtxt(folder/'h2_joint_pos.csv', delimiter=',', skiprows=1)
        poses=[]
        for j in range(0,len(joints),2):
            data.qpos[:] = model.qpos0
            data.qpos[:3] = positions[j]
            data.qpos[:2] -= positions[0,:2]
            data.qpos[3:7] = quats[j]
            data.qpos[addresses] = joints[j]
            mujoco.mj_forward(model,data)
            poses.append(np.concatenate((data.xpos,data.xquat),axis=1))
        poses=np.asarray(poses,dtype='<f4')
        filename=name+'.bin'
        (out/filename).write_bytes(poses.tobytes())
        provenance=json.loads((folder/'h2_lora_reference.json').read_text())
        manifest['motions'].append(dict(id=name,label=label,file=filename,frames=len(poses),
            fps=25,duration=len(joints)/50,sourceFrames=len(joints),sourceFPS=50,
            sourceFile=str(folder.relative_to(root)),sourceSHA256=provenance.get('source_csv_sha256')))
    (out/'motions.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(dict(meshBytes=byte_offset,motions=len(motions),bodies=int(model.nbody))))


if __name__ == '__main__':
    main()
