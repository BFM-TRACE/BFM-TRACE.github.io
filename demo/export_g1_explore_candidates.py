#!/usr/bin/env python3
"""Extract unchanged native G1 semantics from a cached typed deployment bundle.

This only creates isolated candidates. It neither retargets them to H2 nor
initializes the physical robot from source poses. Closed-loop QA is required.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path, frames, width):
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = [[float(v) for v in row] for row in reader]
    assert len(header) == width, (path, len(header), width)
    assert len(rows) == frames, (path, len(rows), frames)
    assert all(len(row) == width and all(math.isfinite(v) for v in row) for row in rows), path
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path(__file__).parent / "source-bundle")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "references")
    args = parser.parse_args()
    assert args.bundle.resolve() != args.output.resolve(), "Use an isolated output directory"
    manifest_path = args.bundle / "h2_lora_reference_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    assert manifest["reference_type"] == "g1" and len(manifest["joint_names"]) == 29
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    kinds = ["single_kneel", "double_kneel", "crawl", "jump", "long_jump", "jump"]
    names = ["Single-knee prayer", "Double-knee kneel and rise", "Backward crawl and rise", "Light jump and land", "Standing broad jump", "Double forward jump"]
    entries = []
    for index, (source, kind, label) in enumerate(zip(manifest["motions"], kinds, names)):
        source_id = source["motion_name"]
        directory = args.bundle / source_id
        assert source["reference_type"] == "g1" and source["encoder_mode"] == 3
        hashes = {name: sha(directory / name) for name in source["output_sha256"]}
        assert hashes == source["output_sha256"], (source_id, "source bundle checksum mismatch")
        frames, fps = source["frames"], source["fps"]
        assert fps == 50
        q = read_csv(directory / "joint_pos.csv", frames, 29)
        dq = read_csv(directory / "joint_vel.csv", frames, 29)
        pos = read_csv(directory / "body_pos.csv", frames, 42)
        quat = read_csv(directory / "body_quat.csv", frames, 56)
        lin = read_csv(directory / "body_lin_vel.csv", frames, 42)
        ang = read_csv(directory / "body_ang_vel.csv", frames, 42)
        assert max(abs(sum(v*v for v in row[:4]) - 1) for row in quat) < 1e-7
        id = "g1_explore_" + source_id
        provenance = {
            "kind": "Original native G1 deployment reference; not an H2 retarget or simulator rollout",
            "serverHostname": "ai-precog-machine21",
            "sourceDirectory": "/home/qingyul/workspace/SONIC-H2/gear_sonic_deploy/reference/example_g1_explore/" + source_id,
            "sourceCSV": source["source_csv"],
            "sourceCSVSHA256": source["source_sha256"],
            "sourceFPS": source["source_fps"], "sourceFrames": source["source_frames"],
            "sourceManifestSHA256": sha(manifest_path),
            "sourceBundleSHA256": hashes,
            "exporterSHA256": sha(Path(__file__)),
            "deploymentResampling": manifest["resampling"],
            "deploymentVelocityMethod": manifest["velocity_method"],
            "candidateTransformation": "CSV-to-JSON only; original 50 Hz joint values/velocities and world pelvis pose retained, all frames retained",
            "jointProjection": "none", "initialQposOverride": False,
            "physicalValidation": "not_run",
            "sourceMenuNumber1Based": index + 1,
            "sourceRuntimeIndex0Based": index,
        }
        ref = {
            "id": id, "sourceMotionId": source_id, "label": label, "kind": kind,
            "referenceType": "g1", "referenceSpace": "g1", "encoderMode": 3,
            "fps": fps, "frames": frames, "duration": (frames-1)/fps,
            "jointOrder": manifest["joint_names"], "quaternionOrder": "wxyz",
            "q29G1": q, "qd29G1": dq,
            "rootPosition": [row[:3] for row in pos], "rootQuat": [row[:4] for row in quat],
            "rootLinearVelocity": [row[:3] for row in lin], "rootAngularVelocity": [row[:3] for row in ang],
            "provenance": provenance,
        }
        path = output / (id + ".json")
        path.write_text(json.dumps(ref, ensure_ascii=False, separators=(",", ":")) + "\n")
        entries.append({
            "id": id, "file": path.name, "sha256": sha(path), "sourceMotionId": source_id,
            "label": label, "description": source["description"], "kind": kind,
            "referenceType": "g1", "referenceSpace": "g1", "encoderMode": 3,
            "sourceMenuNumber1Based": index+1, "sourceRuntimeIndex0Based": index,
            "frames": frames, "fps": fps, "duration": (frames-1)/fps,
            "source": provenance["sourceDirectory"], "sourceCSVSHA256": source["source_sha256"],
            "requestedByReadmeNumber": index+1 in [1,2,4,5],
            "requestedByRuntimeIndex": index in [1,2,4,5],
            "referenceOnlyPreview": None,
        })
    (output / "manifest.json").write_text(json.dumps({
        "schema": "trace_g1_tracking_candidates_v1", "motions": entries,
        "numbering": "README/motion_summary are 1-based; C++ Playing motion/Reset motion log is 0-based. The union is retained pending user clarification.",
        "encoderRequirement": "G1 selector 3 from the same full encoder release as the H2 decoder. Never send these q29 values to native-H2 selector 0.",
        "preview": "No H2 reference-only preview: projecting G1 bodies onto H2 would misrepresent the source.",
    }, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"exported": len(entries), "output": str(output), "hashesVerified": True, "ids": [e["id"] for e in entries]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
