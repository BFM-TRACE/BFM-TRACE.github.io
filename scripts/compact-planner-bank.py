#!/usr/bin/env python3
"""Copy a validated candidate bank, retaining only sampled loop frames.

This does not edit joint values, orientations, timing, models, or body states.
Source candidate files and full generation trajectories remain audit evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', required=True, type=Path)
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
if args.source.resolve() == args.output.resolve():
    parser.error('Use a separate output directory to preserve original candidates.')
bank = json.loads((args.source / 'manifest.json').read_text())
args.output.mkdir(parents=True, exist_ok=True)
proof = []
for style in bank['styles'].values():
    for primary in style['clips'].values():
        for entry in [primary, *primary.get('variants', [])]:
            path = args.source / entry['file']
            raw = path.read_bytes()
            ref = json.loads(raw)
            loop = entry.get('recommendedLoop', ref['recommendedLoop'])
            start, end = loop['startFrame'], loop['endFrame']
            original_frames = ref['frames']
            fields = [key for key, value in ref.items()
                      if isinstance(value, list) and len(value) == original_frames
                      and value and isinstance(value[0], list)]
            assert {'q29G1', 'rootQuat'} <= set(fields)
            for key in fields:
                ref[key] = ref[key][start:end + 1]
            ref['frames'] = end - start + 1
            entry['recommendedLoop'] = {**loop, 'startFrame': 0, 'endFrame': end - start}
            ref['recommendedLoop'] = entry['recommendedLoop']
            ref['compaction'] = {'sourceSHA256': hashlib.sha256(raw).hexdigest(),
                                 'originalFrames': original_frames, 'sourceStartFrame': start,
                                 'sourceEndFrameInclusive': end,
                                 'operation': 'Exact array slice; sampled q/quaternion values and timing unchanged'}
            data = (json.dumps(ref, separators=(',', ':')) + '\n').encode()
            (args.output / entry['file']).write_bytes(data)
            entry['sha256'] = hashlib.sha256(data).hexdigest()
            proof.append({'file': entry['file'], **ref['compaction'],
                          'outputSHA256': entry['sha256'], 'beforeBytes': len(raw), 'afterBytes': len(data)})
bank['compaction'] = {'script': 'scripts/compact-planner-bank.py',
                       'sourceManifestSHA256': hashlib.sha256((args.source / 'manifest.json').read_bytes()).hexdigest(),
                       'beforeBytes': sum(e['beforeBytes'] for e in proof),
                       'afterBytes': sum(e['afterBytes'] for e in proof)}
(args.output / 'manifest.json').write_text(json.dumps(bank, indent=2) + '\n')
(args.output / 'compaction.json').write_text(json.dumps(proof, indent=2) + '\n')
print(json.dumps(bank['compaction']))
