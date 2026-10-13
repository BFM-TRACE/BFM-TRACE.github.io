#!/usr/bin/env python3
"""Export unchanged typed G1 candidates from the original m21 reference/example.

Offline candidate extraction only. No H2 retarget, cropping, resampling,
initial-state override or active homepage installation is performed.
"""
import argparse
import ast
import csv
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def table(path, width):
    with path.open(newline='') as file:
        reader = csv.reader(file)
        assert len(next(reader)) == width, path
        rows = [[float(x) for x in row] for row in reader]
    assert rows and all(len(row) == width and all(math.isfinite(x) for x in row) for row in rows), path
    return rows


def save(path, data, pretty=False):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2 if pretty else None,
                              separators=None if pretty else (',', ':')) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    root = args.root
    bundle = root / 'source-bundle'
    refs = root / 'references'
    measurements = root / 'source-measurements'
    refs.mkdir(exist_ok=True)
    measurements.mkdir(exist_ok=True)
    source_manifest = json.loads((bundle / 'h2_lora_reference_manifest.json').read_text())
    assert source_manifest['reference_type'] == 'g1' and source_manifest['fps'] == 50
    fingerprints = json.loads((root / 'evidence/remote-source-fingerprints.json').read_text())
    for name, expected in fingerprints.items():
        assert sha(bundle / name) == expected['sha256'], ('Source copy mismatch', name)
    joint_names = None
    for node in ast.parse((root / 'evidence/joint_utils.py').read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'G1_ISAACLab_ORDER' for t in node.targets):
            joint_names = ast.literal_eval(node.value)
    assert joint_names is not None and len(joint_names) == 29
    specs = [
        ('neutral_kick_R_001__A543', 'kick', 'right', 'Right high kick'),
        ('neutral_kick_R_001__A543_M', 'kick', 'left', 'Left high kick'),
        ('tired_one_leg_jumping_R_001__A359', 'single_leg_hop', 'right', 'Right-foot hop'),
        ('tired_one_leg_jumping_R_001__A359_M', 'single_leg_hop', 'left', 'Left-foot hop'),
    ]
    body_names = ['pelvis', 'left_hip_roll_link', 'left_knee_link', 'left_ankle_roll_link',
                  'right_hip_roll_link', 'right_knee_link', 'right_ankle_roll_link', 'torso_link',
                  'left_shoulder_roll_link', 'left_elbow_link', 'left_wrist_yaw_link',
                  'right_shoulder_roll_link', 'right_elbow_link', 'right_wrist_yaw_link']
    entries = []
    validation = []
    for source_id, kind, side, label in specs:
        directory = bundle / source_id
        q = table(directory / 'joint_pos.csv', 29)
        dq = table(directory / 'joint_vel.csv', 29)
        pos = table(directory / 'body_pos.csv', 42)
        quat = table(directory / 'body_quat.csv', 56)
        lin = table(directory / 'body_lin_vel.csv', 42)
        ang = table(directory / 'body_ang_vel.csv', 42)
        frames = len(q)
        assert frames == (165 if kind == 'kick' else 500)
        assert all(len(values) == frames for values in [dq, pos, quat, lin, ang])
        quaternion_error = max(abs(sum(v*v for v in row[:4])-1) for row in quat)
        assert quaternion_error < 4e-6, (source_id, quaternion_error)
        id = 'g1_standard_' + source_id
        source_hashes = {path.name: sha(path) for path in sorted(directory.iterdir()) if path.is_file()}
        source_index = [m['motion_name'] for m in source_manifest['motions']].index(source_id)
        provenance = {
            'kind': 'Unmodified original SONIC G1 deployment reference; not an H2 retarget or simulator rollout',
            'serverHostname': 'ai-precog-machine21',
            'userRequestedCollection': 'reference/example_g1',
            'actualCollection': 'gear_sonic_deploy/reference/example',
            'sourceDirectory': '/home/qingyul/workspace/SONIC-H2/gear_sonic_deploy/reference/example/' + source_id,
            'sourceBundleSHA256': source_hashes,
            'sourceManifestSHA256': sha(bundle / 'h2_lora_reference_manifest.json'),
            'sourceJointSchemaSHA256': sha(root / 'evidence/joint_utils.py'),
            'candidateExporterSHA256': sha(Path(__file__)),
            'sourceRuntimeIndex0Based': source_index,
            'sourceManifestPosition1Based': source_index + 1,
            'sourceFPS': 50, 'sourceFrames': frames,
            'sourceFPSBasis': 'Typed collection manifest fps=50 and C++ MotionSequence default fps=50; legacy per-motion metadata has no FPS field.',
            'upstreamRawMotionSource': 'Not declared in legacy bundle metadata; no upstream path or hash inferred from a matching name.',
            'candidateTransformation': 'CSV-to-JSON only; all original 50 Hz q/qdot, world pelvis values and frames retained.',
            'jointProjection': 'none', 'initialQposOverride': False,
            'mirror': 'Original source library _M folder' if source_id.endswith('_M') else 'Original non-mirrored source folder',
            'physicalValidation': 'not_run',
        }
        ref = {
            'id': id, 'sourceMotionId': source_id, 'label': label, 'kind': kind,
            'referenceType': 'g1', 'referenceSpace': 'g1', 'encoderMode': 3,
            'jointOrder': joint_names, 'quaternionOrder': 'wxyz',
            'fps': 50, 'frames': frames, 'duration': (frames-1)/50,
            'q29G1': q, 'qd29G1': dq, 'rootQuat': [row[:4] for row in quat],
            'rootPosition': [row[:3] for row in pos],
            'rootLinearVelocity': [row[:3] for row in lin],
            'rootAngularVelocity': [row[:3] for row in ang], 'provenance': provenance,
        }
        save(refs / (id + '.json'), ref)
        feet = {'left': [row[9:12] for row in pos], 'right': [row[18:21] for row in pos]}
        stats = {foot: {'minimumBodyHeight': min(row[2] for row in values),
                        'maximumBodyHeight': max(row[2] for row in values),
                        'meanBodyHeight': sum(row[2] for row in values)/frames}
                 for foot, values in feet.items()}
        expected_high = side if kind == 'kick' else ('left' if side == 'right' else 'right')
        assert stats[expected_high]['meanBodyHeight'] > stats['left' if expected_high == 'right' else 'right']['meanBodyHeight']
        measure = {
            'id': id, 'fps': 50, 'frames': frames, 'footPositionG1': feet,
            'rootPosition': ref['rootPosition'], 'bodyNames': body_names,
            'footBodyIndices': {'left': 3, 'right': 6},
            'footBodyIndicesMeaning': 'Zero-based storage position in the 14-body body_pos.csv. Anatomical body indexes are left 18, right 19.',
            'footBodyNames': {'left': 'left_ankle_roll_link', 'right': 'right_ankle_roll_link'},
            'sourceBodyPosSHA256': source_hashes['body_pos.csv'], 'sourceHeightStats': stats,
            'sideInference': 'Source kick side is the high moving foot. Source hop side is the lower supporting foot while the opposite foot is raised. This is source kinematics, not measured H2 contact.',
            'kickSide': side if kind == 'kick' else None,
            'hopSide': side if kind == 'single_leg_hop' else None,
            'liftedSide': expected_high if kind == 'single_leg_hop' else None,
            'measurementBoundary': 'Native G1 ankle body positions, not sole clearance or H2 FK; use normalized timing/waveform comparisons rather than raw cross-robot height equality.',
        }
        save(measurements / (id + '.json'), measure)
        entry = {key: ref[key] for key in ['id', 'sourceMotionId', 'label', 'kind', 'referenceType', 'referenceSpace', 'encoderMode', 'frames', 'fps', 'duration']}
        entry.update({'file': id+'.json', 'sha256': sha(refs/(id+'.json')), 'startupHoldSeconds': .5,
                      'source': provenance['sourceDirectory'], 'sourceRuntimeIndex0Based': source_index,
                      'sourceManifestPosition1Based': source_index+1,
                      'sourceMeasurementFile': '../source-measurements/'+id+'.json',
                      'sourceMeasurementSHA256': sha(measurements/(id+'.json')),
                      'referenceOnlyPreview': None})
        entry.update({key: measure[key] for key in ['kickSide', 'hopSide', 'liftedSide']})
        entries.append(entry)
        validation.append({'id': id, 'finite': True, 'allFramesRetained': True, 'maxRootQuatNormSquaredError': quaternion_error,
                           'sourceJointPositionSHA256': source_hashes['joint_pos.csv'], 'sourceHeightStats': stats})
    save(refs/'manifest.json', {'schema':'trace_g1_tracking_candidates_v1', 'motions': entries,
                              'sourceBoundary':'Requested example_g1 is absent on m21; these are the actual typed G1 reference/example clips, approved by task parent.',
                              'startupHoldSecondsMeaning':'0.5 seconds of real controller first-frame hold before playback; separate policy initialization, not source padding.',
                              'physicalValidation':'pending'}, True)
    save(root/'validation.json', {'sourceFingerprintsVerified': len(fingerprints), 'motions':validation}, True)
    print(json.dumps({'exported':len(entries),'references':str(refs),'measurements':str(measurements),'frames':[e['frames'] for e in entries]}))


if __name__ == '__main__':
    main()
