#!/usr/bin/env python3
"""Export exact simulation assets for local policy inference, without DDS.

Run in SONIC-H2 with its .venv_sim Python. This creates no network connection,
starts no policy process, and never imports a hardware-control entry point.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import mmap
import re
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET


def fields(buf):
    def vint(i):
        value, shift = 0, 0
        while True:
            byte = buf[i]
            i += 1
            value |= (byte & 127) << shift
            if byte < 128:
                return value, i
            shift += 7
    i = 0
    while i < len(buf):
        tag, i = vint(i)
        number, wire = tag >> 3, tag & 7
        if wire == 0:
            value, i = vint(i)
        elif wire in (1, 5):
            n = 8 if wire == 1 else 4
            value, i = buf[i:i+n], i+n
        elif wire == 2:
            n, i = vint(i)
            value, i = buf[i:i+n], i+n
        else:
            raise ValueError(f'Unsupported protobuf wire type {wire}')
        yield number, value


def model_info(path):
    """Read ONNX protobuf metadata only; large tensor payloads stay memory-mapped."""
    f = path.open('rb')
    mapped = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    model = dict(fields(memoryview(mapped)))
    text = lambda b: bytes(b).decode()
    def value_info(buf):
        value = dict(fields(buf))
        tensor = dict(fields(dict(fields(value[2]))[1]))
        shape = []
        for number, dim in fields(tensor[2]):
            d = dict(fields(dim))
            shape.append(d.get(1, text(d[2]) if 2 in d else None))
        return dict(name=text(value[1]), shape=shape, dtype=tensor[1])
    inputs, outputs, ops, dtypes = [], [], Counter(), Counter()
    external = False
    for number, buf in fields(model[7]):
        if number == 11:
            inputs.append(value_info(buf))
        elif number == 12:
            outputs.append(value_info(buf))
        elif number == 1:
            node = dict(fields(buf))
            ops[text(node.get(7, b'')) + ':' + text(node[4])] += 1
        elif number == 5:
            tensor = dict(fields(buf))
            dtypes[str(tensor[2])] += 1
            external |= tensor.get(14, 0) == 1 or 13 in tensor
    opsets, metadata = [], {}
    for number, buf in fields(memoryview(mapped)):
        if number == 8:
            item = dict(fields(buf))
            opsets.append(dict(domain=text(item.get(1, b'')), version=item[2]))
        elif number == 14:
            item = dict(fields(buf))
            metadata[text(item[1])] = text(item[2])
    return dict(bytes=path.stat().st_size, sha256=hashlib.file_digest(f, 'sha256').hexdigest()
                if hasattr(hashlib, 'file_digest') else hashlib.sha256(mapped).hexdigest(),
                ir_version=model[1], opsets=opsets, inputs=inputs, outputs=outputs,
                operators=dict(ops), initializer_dtypes=dict(dtypes), external_data=external,
                metadata=metadata)


def serial(value):
    import numpy as np
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {str(k):serial(v) for k,v in value.items()}
    if isinstance(value, (list, tuple, set)): return [serial(v) for v in value]
    if isinstance(value, Path): return str(value)
    return value


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(serial(document), indent=2) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--metadata-only', action='store_true')
    args = p.parse_args()
    root, out = args.root.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    model_root = root / 'sonic_h2_native_release/experiments/gpu3_20260907_stage2_step009450'
    model_paths = {'encoder_native': model_root / 'model_encoder.onnx',
                   'decoder': model_root / 'model_decoder.onnx',
                   'encoder_full': model_root / 'model_encoder_full.onnx',
                   'planner': root / 'gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx'}
    models = {name:dict(model_info(path), source=str(path.relative_to(root)),
                        file=name+'.onnx', bundled=name in ('encoder_native', 'decoder'))
              for name,path in model_paths.items()}
    write_json(out / 'models/manifest.json', dict(models=models))
    print(json.dumps({k:dict(bytes=v['bytes'], inputs=v['inputs'], outputs=v['outputs'],
                            opsets=v['opsets']) for k,v in models.items()}), flush=True)
    if args.metadata_only:
        return
    sys.path.insert(0, str(root))
    import mujoco
    import numpy as np
    from gear_sonic.utils.mujoco_sim.base_sim import DefaultEnv
    from gear_sonic.utils.mujoco_sim.configs import BaseConfig
    config = BaseConfig(wbc_version='h2_native_unitree_ros_loop', interface='lo',
                        enable_onscreen=False).load_wbc_yaml()
    config.update(PRINT_SCENE_INFORMATION=False, USE_JOYSTICK=False)
    # DefaultEnv is deliberately used directly, never BaseSimulator: no DDS.
    env = DefaultEnv(config, onscreen=False)
    model, data = env.mj_model, env.mj_data
    physics = out / 'physics'
    physics.mkdir(exist_ok=True)
    mujoco.mj_saveLastXML(str(physics / 'model.xml'), model)
    xml = ET.parse(physics / 'model.xml')
    compiler = xml.getroot().find('compiler')
    if compiler is not None:
        compiler.attrib.pop('meshdir', None)
    original = root / 'gear_sonic/data/assets/robot_description/unitree_ros/h2_description'
    assets = []
    for mesh in xml.findall('.//asset/mesh'):
        filename = Path(mesh.get('file'))
        source = filename if filename.is_absolute() else original / filename
        relative = 'meshes/' + filename.name
        (physics / 'meshes').mkdir(exist_ok=True)
        shutil.copy2(source, physics / relative)
        mesh.set('file', relative)
        assets.append(relative)
    xml.write(physics / 'model.xml', encoding='unicode')
    groups = [{k:serial(v) for k,v in group.items()} for group in env.closed_loop_transmissions]
    physics_config = dict(mujocoVersion=mujoco.__version__, nq=model.nq, nv=model.nv,
        nu=model.nu, nbody=model.nbody, timestep=model.opt.timestep,
        initialQpos=data.qpos, initialQvel=data.qvel,
        bodyNames=[model.body(i).name for i in range(model.nbody)],
        jointNames=[model.joint(i).name for i in range(model.njnt)],
        localNames=env.body_joint_logical_names, physicalLocalNames=env.body_joint_names,
        localQposAddresses=env.body_qpos_index, localDofAddresses=env.body_dof_index,
        bodyActuatorIndices=env.body_actuator_index,
        actuatorJointIndices=env.body_actuator_joint_index,
        dofDamping=model.dof_damping, dofArmature=model.dof_armature,
        dofFrictionloss=model.dof_frictionloss, actuatorForceRange=model.actuator_forcerange,
        torqueLimits=env.torque_limit, transmissions=groups,
        local_to_sim=list(range(len(env.body_joint_names))),
        body_qpos_index=env.body_qpos_index, body_dof_index=env.body_dof_index,
        body_actuator_index=env.body_actuator_index, torque_limit=env.torque_limit,
        closed_loop_transmissions=groups,
        motor_response_enabled=env.motor_response is not None,
        ankle_response_enabled=env.ankle_response is not None,
        modelConfig=config,
        robotConfig={k:serial(v) for k,v in vars(env.robot).items()
                     if isinstance(v,(str,int,float,bool,list,tuple,dict,np.ndarray))})
    write_json(out / 'physics/physics-config.json', physics_config)
    initial_qpos = data.qpos.copy()
    # Validate the exact portable XML bytes, including save/reload rounding.
    env.mj_model = mujoco.MjModel.from_xml_path(str(physics / 'model.xml'))
    env.mj_data = mujoco.MjData(env.mj_model)
    model, data = env.mj_model, env.mj_data
    data.qpos[:] = initial_qpos
    mujoco.mj_forward(model,data)
    regression = []
    for label in ('initial', 'ankle_perturbed'):
        data.qpos[:] = initial_qpos
        if label == 'ankle_perturbed':
            data.qpos[env.body_qpos_index[5]] += .03
        mujoco.mj_forward(model, data)
        env._closed_loop_jacobian_cache_time = None
        torques = np.sin(np.arange(31, dtype=float)) * 5
        regression.append(dict(label=label,qpos=data.qpos.copy(),bodyTorques=torques,
            jacobians=[dict(name=g['name'],J=env._closed_loop_output_to_motor_jacobian(g))
                       for g in env.closed_loop_transmissions],
            actuatorTorques=env._map_body_torques_to_actuators(torques)))
    data.qpos[:] = initial_qpos
    mujoco.mj_forward(model,data)
    write_json(physics / 'transmission-regression.json', regression)
    write_json(physics / 'manifest.json', dict(entry='model.xml',mujocoVersion=mujoco.__version__,
        files=[dict(path=n, url=n) for n in ['model.xml', *assets]], config='physics-config.json'))
    reference_root = root / 'gear_sonic_deploy/reference/example_h2/deploy'
    ref_manifest = json.loads((reference_root / 'h2_lora_reference_manifest.json').read_text())
    motions = []
    sources = [(reference_root, item) for item in ref_manifest['motions']]
    stand_root = root / 'gear_sonic_deploy/reference/h2_default_stand_only'
    stand_manifest = json.loads((stand_root / 'h2_lora_reference_manifest.json').read_text())
    sources.extend((stand_root, item) for item in stand_manifest['motions'])
    for motion_root, item in sources:
        name = item['motion_name']
        folder = motion_root / name
        read = lambda file: np.loadtxt(folder / file, delimiter=',', skiprows=1).astype('<f4')
        arrays = dict(rootPosition=read('body_pos.csv'),rootQuat=read('body_quat.csv'),
                      q31=read('h2_joint_pos.csv'),qd31=read('h2_joint_vel.csv'),
                      rootLinearVelocity=read('body_lin_vel.csv'),rootAngularVelocity=read('body_ang_vel.csv'))
        frames = len(arrays['q31'])
        filename = name + '.json'
        write_json(out / 'references' / filename, dict(id=name,fps=50,frames=frames,**arrays))
        motions.append(dict(id=name,file=filename,frames=frames,fps=50,
                            source=str(folder.relative_to(root))))
    write_json(out / 'references/manifest.json',dict(motions=motions,quaternionOrder='wxyz',
        localNames=env.body_joint_logical_names,kind='native H2 reference targets, not simulated state'))
    header = (root / 'gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp').read_text()
    def cpp_array(name):
        body = re.search(r'\b'+name+r'\s*=\s*\{(.*?)\};',header,re.S).group(1)
        body = re.sub(r'//[^\n]*','',body)
        return [float(s.strip().rstrip('Ff')) for s in body.split(',') if s.strip()]
    policy_to_local = [int(v) for v in cpp_array('h2lora_body_to_h2_mujoco')]
    write_json(out/'controller.json',dict(
        schema='trace-h2-native-controller-v1',policyHz=50,physicsHz=1000,
        sourceCheckpoint='gpu3_20260907_stage2_step009450',
        policyToLocal=policy_to_local,
        defaultAnglesPolicy=cpp_array('h2_native_default_angles_policy'),
        actionScalePolicy=cpp_array('h2_native_action_scale_policy'),
        defaultAnglesLocal=env.robot.DEFAULT_DOF_ANGLES,
        localNames=env.body_joint_logical_names,
        localQposAddresses=env.body_qpos_index,localDofAddresses=env.body_dof_index,
        kp=cpp_array('h2_native_kps'),kd=cpp_array('h2_native_kds'),
        initialQpos=initial_qpos,initialQvel=data.qvel,
        encoder=dict(input='obs_dict',inputSize=640,output='encoded_tokens',outputSize=64,
                     futureFrames=10,futureFrameStep=5,referenceFPS=50,
                     orientationContract='sonic_full_root_relative_v0'),
        decoder=dict(input='obs_dict',inputSize=994,output='action',outputSize=29,
                     historyFrames=10,historyOrder='oldest-first',
                     features=['token64','omega30','q_minus_default290','qd290','action290','gravity30']),
        motionsManifest='references/manifest.json',physicsManifest='physics/manifest.json'))
    shutil.copy2(model_root/'local_model_validation.json', out/'models/source-validation.json')
    shutil.copy2(model_root/'deployment_manifest.json', out/'models/source-deployment.json')
    print(json.dumps(dict(exported=str(out),nq=model.nq,nbody=model.nbody,meshes=len(assets),motions=len(motions))),flush=True)


if __name__ == '__main__':
    main()
