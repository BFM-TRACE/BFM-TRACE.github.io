#!/usr/bin/env python3
"""Export SONIC_unitree_h2 into an isolated web bundle. No DDS or robot I/O.

Run with the source repository's .venv_sim/bin/python, --root and --output.
Preserves released ONNX bytes and all 38 source references; exports H2 FK previews.
"""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import mujoco
from export_local_assets import model_info, serial


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(serial(value), ensure_ascii=False, separators=(',', ':')) + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    root, out = a.root.resolve(), a.output.resolve()
    if out.exists():
        raise ValueError('Use a new isolated output directory')
    out.mkdir(parents=True)
    sys.path.insert(0, str(root / 'src'))
    from sim.config import SimLoopConfig
    from sim.environment import DefaultEnv
    from interface.motion_replay.motion import Motion, _csv
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD']).decode().strip()
    release = root / 'models/sonic_h2_released'
    deployment = json.loads((release / 'deployment_manifest.json').read_text())
    models = {}
    for key, source_name in [('encoder_full', 'model_encoder_full'), ('encoder_native', 'model_encoder'), ('decoder', 'model_decoder')]:
        p = release / (source_name + '.onnx')
        assert sha(p) == deployment['model_sha256'][source_name]
        info = model_info(p)
        info.update(file=key + '.onnx', bundled=key != 'encoder_native', source=str(p.relative_to(root)), sourceRevision=revision)
        models[key] = info
        shutil.copy2(p, out / info['file'])
    save(out / 'models/manifest.json', {'models': models, 'sourceRepository': 'QinnyuLan/SONIC_unitree_h2', 'sourceRevision': revision})
    save(out / 'models/source-deployment.json', deployment)
    config = SimLoopConfig(interface='lo', enable_onscreen=False).load_wbc_yaml()
    config.update(PRINT_SCENE_INFORMATION=False, USE_JOYSTICK=False)
    env = DefaultEnv(config, onscreen=False)
    assert env.unitree_bridge is None
    m, d = env.mj_model, env.mj_data
    initial = d.qpos.copy()
    physics = out / 'physics'
    physics.mkdir()
    mujoco.mj_saveLastXML(str(physics / 'model.xml'), m)
    tree = ET.parse(physics / 'model.xml')
    compiler = tree.getroot().find('compiler')
    if compiler is not None:
        compiler.attrib.pop('meshdir', None)
    assets = []
    for mesh in tree.findall('.//asset/mesh'):
        p = Path(mesh.get('file'))
        source = p if p.is_absolute() else root / 'assets/robots/h2' / p
        relative = 'meshes/' + p.name
        (physics / 'meshes').mkdir(exist_ok=True)
        shutil.copy2(source, physics / relative)
        mesh.set('file', relative)
        assets.append(relative)
    tree.write(physics / 'model.xml', encoding='unicode')
    groups = [{k: serial(v) for k, v in g.items()} for g in env.closed_loop_transmissions]
    pc = dict(mujocoVersion=mujoco.__version__, sourceRevision=revision, nq=m.nq, nv=m.nv, nu=m.nu, nbody=m.nbody,
        initialQpos=initial, initialQvel=d.qvel.copy(), qposAddresses=env.body_qpos_index,
        dofAddresses=env.body_dof_index, actuatorAddresses=env.body_actuator_index,
        localNames=env.body_joint_logical_names, dofDamping=m.dof_damping,
        dofArmature=m.dof_armature, dofFrictionloss=m.dof_frictionloss,
        actuatorForceRange=m.actuator_forcerange, torqueLimits=env.torque_limit,
        transmissions=groups, local_to_sim=list(range(31)), body_qpos_index=env.body_qpos_index,
        body_dof_index=env.body_dof_index, body_actuator_index=env.body_actuator_index,
        torque_limit=env.torque_limit, closed_loop_transmissions=groups,
        motor_response_enabled=env.motor_response is not None, ankle_response_enabled=env.ankle_response is not None,
        modelConfig=config, robotConfig={k: serial(v) for k, v in vars(env.robot).items()
            if isinstance(v, (str, int, float, bool, list, tuple, dict, np.ndarray))})
    save(physics / 'physics-config.json', pc)
    save(physics / 'manifest.json', dict(entry='model.xml', mujocoVersion=mujoco.__version__, files=[dict(path=p, url=p) for p in ['model.xml', *assets]], config='physics-config.json'))
    # Reload the exact portable XML, including MuJoCo's serialization rounding.
    env.mj_model = m = mujoco.MjModel.from_xml_path(str(physics / 'model.xml'))
    env.mj_data = d = mujoco.MjData(m)
    d.qpos[:] = initial
    mujoco.mj_forward(m, d)
    regression = []
    for label in ['initial', 'ankle_perturbed']:
        d.qpos[:] = initial
        if label == 'ankle_perturbed':
            d.qpos[env.body_qpos_index[5]] += .03
        mujoco.mj_forward(m, d)
        env._closed_loop_jacobian_cache_time = None
        torques = np.sin(np.arange(31)) * 5
        regression.append(dict(label=label, qpos=d.qpos.copy(), bodyTorques=torques,
            jacobians=[dict(name=g['name'], J=env._closed_loop_output_to_motor_jacobian(g)) for g in env.closed_loop_transmissions],
            actuatorTorques=env._map_body_torques_to_actuators(torques)))
    save(physics / 'transmission-regression.json', regression)
    header = (root / 'src/runtime/include/policy_parameters.hpp').read_text()
    def array(name):
        body = re.search(r'\b' + name + r'\s*=\s*\{(.*?)\};', header, re.S).group(1)
        body = re.sub(r'//[^\n]*', '', body)
        return [float(s.strip().rstrip('Ff')) for s in body.split(',') if s.strip()]
    controller = dict(schema='trace-h2-native-controller-v1', policyHz=50, physicsHz=1000,
        sourceCheckpoint='SONIC_unitree_h2/models/sonic_h2_released', sourceRevision=revision,
        policyToLocal=[int(v) for v in array('h2lora_body_to_h2_mujoco')],
        defaultAnglesPolicy=array('h2_native_default_angles_policy'), actionScalePolicy=array('h2_native_action_scale_policy'),
        defaultAnglesLocal=env.robot.DEFAULT_DOF_ANGLES, localNames=env.body_joint_logical_names,
        localQposAddresses=env.body_qpos_index, localDofAddresses=env.body_dof_index,
        kp=array('h2_native_kps'), kd=array('h2_native_kds'), initialQpos=initial,
        initialQvel=np.zeros(m.nv), encoder=dict(inputSize=1751, outputSize=64, futureFrames=10,
        orientationContract='sonic_full_root_relative_v0'), decoder=dict(inputSize=994, outputSize=29),
        motionsManifest='references/manifest.json', physicsManifest='physics/manifest.json')
    save(out / 'controller.json', controller)
    entries, previews, inventory = [], [], []
    labels = {'squat': 'Squat', 'walking_quip': 'Walk and turn', 'macarena': 'Macarena', 'dance_in_da_party': 'Dance',
        'forward_lunge': 'Forward lunge', 'tired_forward_lunge': 'Low lunge', 'neutral_kick': 'High kick',
        'tired_one_leg_jumping': 'Single-leg hops', 'prayer': 'Single-knee kneel', 'dejection': 'Double-knee kneel',
        'jump_and_land_light': 'Light jump', 'broad_jump': 'Broad jump', 'high_jump': 'High jump',
        'jump_twice': 'Double jump', 'show_bicep': 'Crawl and rotate'}
    collections = [('example_h2_replay', 'h2', ''), ('example_g1_replay', 'g1', 'g1_'),
        ('example_g1_explore_replay', 'g1', 'g1_explore_'), ('example_smpl_replay', 'smpl', 'smpl_'),
        ('example_text2motion_replay', 'g1', 'text2motion_')]
    for collection, kind, prefix in collections:
        folder = root / 'motions' / collection
        manifest_path = folder / 'manifest.json'
        source_entries = json.loads(manifest_path.read_text())['motions'] if manifest_path.exists() else [dict(motion_name=p.name) for p in sorted(folder.iterdir()) if p.is_dir()]
        for spec in source_entries:
            name = spec['motion_name']
            source = folder / name
            hashes = spec.get('sha256', {})
            for file, expected in hashes.items():
                assert sha(source / file) == expected, str(source / file)
            bundle = json.loads((source / 'replay.json').read_text()) if (source / 'replay.json').exists() else None
            if bundle:
                assert sha(source / bundle['motion']['file']) == bundle['motion']['sha256']
            motion = Motion.load(source / bundle['motion']['file'] if bundle else source)
            assert motion.reference_type == kind and motion.fps == 50
            n = len(motion.times)
            id = prefix + name
            # Release validation: this hopping clip needs one second of live
            # frame-zero preparation; 0.5 s falls at 7.86 s with these weights.
            hold = 1.0 if id == 'g1_tired_one_leg_jumping_R_001__A359' else .5
            label = next((v for k, v in labels.items() if name.startswith(k)), name)
            if name.endswith('_M'):
                label += ' · mirrored'
            label = ('G1 explore' if 'explore' in collection else kind.upper()) + ' · ' + label
            if bundle:
                label = 'Speech gesture · ' + ('Chinese' if name.endswith('zh') else 'English')
            provenance = dict(repository='QinnyuLan/SONIC_unitree_h2', revision=revision,
                source=str(source.relative_to(root)), sourceFileSHA256=hashes or {'motion.npz': bundle['motion']['sha256']},
                conversion='Source native values; original 50 Hz timeline; no retargeting or posture edits')
            ref = dict(id=id, referenceSpace=kind, encoderMode={'h2': 0, 'g1': 3, 'smpl': 2}[kind], fps=50,
                frames=n, rootQuat=motion.root, startupHoldSeconds=hold, provenance=provenance)
            if kind == 'smpl':
                ref.update(smplJoints=motion.joints.reshape(n, 72), wristJoints=motion.wrists)
            elif kind == 'g1':
                ref.update(q29G1=motion.q, qd29G1=motion.dq, jointOrder=[env.body_joint_logical_names[i] for i in controller['policyToLocal']])
            else:
                ref.update(q31=_csv(source / 'h2_joint_pos.csv'), qd31=_csv(source / 'h2_joint_vel.csv'), rootPosition=_csv(source / 'body_pos.csv')[:, :3])
            path = out / 'references' / (id + '.json')
            save(path, ref)
            entry = dict(id=id, label=label, file=path.name, sha256=sha(path), frames=n, fps=50,
                duration=(n - 1) / 50, referenceSpace=kind, referenceType=kind, encoderMode=ref['encoderMode'],
                startupHoldSeconds=hold, source=provenance['source'], collection=collection)
            entries.append(entry)
            preview = dict(id=id, label=label, file=None, previewAvailable=False, frames=n, fps=50,
                duration=(n - 1) / 50, sourceFrames=n, sourceFPS=50, referenceSpace=kind,
                referenceSHA256=entry['sha256'], collection=collection)
            if kind == 'h2':
                poses = []
                for frame in range(0, n, 2):
                    d.qpos[:] = m.qpos0
                    d.qpos[:3] = ref['rootPosition'][frame]
                    d.qpos[:2] -= ref['rootPosition'][0, :2]
                    d.qpos[3:7] = ref['rootQuat'][frame]
                    d.qpos[env.body_qpos_index] = ref['q31'][frame]
                    mujoco.mj_forward(m, d)
                    poses.append(np.concatenate([d.xpos, d.xquat], axis=1))
                p = out / 'preview' / (id + '.bin')
                p.parent.mkdir(exist_ok=True)
                p.write_bytes(np.asarray(poses, dtype='<f4').tobytes())
                preview.update(file=p.name, previewAvailable=True, frames=len(poses), fps=25, previewSHA256=sha(p))
            previews.append(preview)
            inventory.append(dict(**provenance, id=id, frames=n, referenceType=kind, exportedSHA256=entry['sha256']))
    assert len(entries) == 38
    save(out / 'references/manifest.json', dict(motions=entries, localNames=env.body_joint_logical_names,
        quaternionOrder='wxyz', sourceRepository='QinnyuLan/SONIC_unitree_h2', sourceRevision=revision))
    save(out / 'preview/motions.json', dict(schema='trace-motion-references-v1', bodyCount=m.nbody,
        source='SONIC_unitree_h2 released references', representation='typed policy targets; H2 previews are kinematic only', motions=previews))
    save(out / 'source-inventory.json', dict(sourceRevision=revision, motions=inventory,
        modelHashes=deployment['model_sha256'], bodyNames=[m.body(i).name for i in range(m.nbody)],
        plannerSHA256=sha(root / 'models/motion_planner_g1/planner_sonic.onnx'),
        scope='Exact source ONNX bytes and verified source motion files; no runtime or robot connection was started'))
    print(json.dumps(dict(output=str(out), motions=len(entries), modelHashes=deployment['model_sha256'])))


if __name__ == '__main__':
    main()
