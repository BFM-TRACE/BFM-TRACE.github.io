#!/usr/bin/env python3
"""Extract the exact G1 selector-3 branch of the shipped full encoder ONNX.

Reads existing ONNX nodes and initializers without retraining, quantizing, or
modifying the source. Validates against full selector 3 using random tensors and
real planner references. Requires numpy, scipy, onnx and onnxruntime.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import onnxruntime as ort
from scipy.spatial.transform import Rotation


SOURCE_SHA256 = "a92f1703be9949b44d305997c32b622084a11580dd57e73e827506fee49d0710"
BOUNDARY = "/Reshape_10_output_0"
BRANCH_OUTPUT = "/quantizer_3/Cast_output_0"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(source, output):
    if digest(source) != SOURCE_SHA256:
        raise ValueError("This extractor is audited for a different full-encoder SHA-256")
    full = onnx.load(source)
    metadata = {item.key: item.value for item in full.metadata_props}
    assert metadata["selector_encoder_names"] == '["h2", "teleop", "smpl", "g1"]'
    assert metadata["reference_orientation_contract"] == "sonic_full_root_relative_v0"
    producers = {value: node for node in full.graph.node for value in node.output}
    initializers = {item.name: item for item in full.graph.initializer}
    wanted_nodes, wanted_initializers = set(), set()

    def visit(value):
        if value == BOUNDARY or not value:
            return
        if value in initializers:
            wanted_initializers.add(value)
            return
        node = producers[value]
        if node.name in wanted_nodes:
            return
        wanted_nodes.add(node.name)
        for parent in node.input:
            visit(parent)

    visit(BRANCH_OUTPUT)
    # Compact layout remains [q290, qd290, orientation60]. The source full
    # encoder first reshapes the 580D block to 10x58, appends each 6D orientation,
    # then flattens. Keep this exact preprocessing, rather than feeding raw640
    # directly to the G1 MLP (which would silently change feature order).
    constants = {
        "compact_axis": [1], "compact_start_q": [0], "compact_end_q": [580],
        "compact_start_ori": [580], "compact_end_ori": [640],
        "compact_q_shape": [1, 1, 10, 58], "compact_ori_shape": [1, 1, 10, 6],
        "compact_window_shape": [-1, 10, 64], "compact_output_shape": [1, 64],
    }
    extra = [numpy_helper.from_array(np.array(value, dtype=np.int64), name)
             for name, value in constants.items()]
    nodes = [
        helper.make_node("Slice", ["obs_dict", "compact_start_q", "compact_end_q", "compact_axis"], ["compact_q"], name="compact_q_slice"),
        helper.make_node("Slice", ["obs_dict", "compact_start_ori", "compact_end_ori", "compact_axis"], ["compact_ori"], name="compact_ori_slice"),
        helper.make_node("Reshape", ["compact_q", "compact_q_shape"], ["compact_q_window"], name="compact_q_reshape"),
        helper.make_node("Reshape", ["compact_ori", "compact_ori_shape"], ["compact_ori_window"], name="compact_ori_reshape"),
        helper.make_node("Concat", ["compact_q_window", "compact_ori_window"], ["compact_features"], axis=-1, name="compact_feature_concat"),
        helper.make_node("Reshape", ["compact_features", "compact_window_shape"], [BOUNDARY], name="compact_window_reshape"),
    ]
    nodes.extend(copy.deepcopy(node) for node in full.graph.node if node.name in wanted_nodes)
    nodes.append(helper.make_node("Reshape", [BRANCH_OUTPUT, "compact_output_shape"], ["encoded_tokens"], name="compact_output_reshape"))
    weights = [copy.deepcopy(item) for item in full.graph.initializer if item.name in wanted_initializers]
    assert len(weights) == 10
    assert all(item.name.startswith("module.encoders.g1.") for item in weights)
    assert all(item.SerializeToString() == initializers[item.name].SerializeToString() for item in weights)
    graph = helper.make_graph(nodes, "trace_g1_encoder_selector3",
        [helper.make_tensor_value_info("obs_dict", TensorProto.FLOAT, [1, 640])],
        [helper.make_tensor_value_info("encoded_tokens", TensorProto.FLOAT, [1, 64])],
        initializer=weights + extra)
    compact = helper.make_model(graph, producer_name="TRACE selector-3 graph extraction", opset_imports=list(full.opset_import))
    compact.ir_version = full.ir_version
    helper.set_model_props(compact, {
        "model_role": "encoder", "encoder_name": "g1", "policy_interface": "g1_reference_h2_decoder",
        "observation_contract": metadata["observation_contract"],
        "reference_orientation_contract": metadata["reference_orientation_contract"],
        "global_step": metadata["global_step"], "checkpoint_sha256": metadata["checkpoint_sha256"],
        "source_full_sha256": SOURCE_SHA256, "source_full_selector": "3",
        "input_features": '["command_multi_future_nonflat", "motion_anchor_ori_b_mf_nonflat"]',
        "compact_layout": "q290,qd290,orientation60; raw G1 semantic joint order",
        "extraction": "Original G1 nodes, weights and token quantizer; no retraining or weight conversion",
    })
    onnx.checker.check_model(compact, full_check=True)
    onnx.save(compact, output)
    return full, compact, weights


def real_cases(directory, rng):
    records = []
    for path in sorted(directory.glob("*.json")):
        item = json.loads(path.read_text())
        if not isinstance(item, dict) or "q29G1" not in item:
            continue
        q = np.asarray(item["q29G1"], dtype=np.float32)
        qd = np.asarray(item["qd29G1"], dtype=np.float32)
        quat = np.asarray(item["rootQuat"], dtype=np.float64)
        assert q.shape == qd.shape and q.shape[1] == 29
        assert len(quat) == len(q)
        records.append({"file": path.name, "sha256": digest(path), "frames": len(q)})
        for start in np.unique(np.linspace(0, len(q) - 1, 12).astype(int)):
            indices = np.minimum(start + np.arange(10) * 5, len(q) - 1)
            future_rot = Rotation.from_quat(quat[indices][:, [1, 2, 3, 0]])
            base = Rotation.from_quat(quat[start, [1, 2, 3, 0]])
            for kind, perturb in [("aligned", Rotation.identity()),
                                  ("perturbed", Rotation.from_euler("xyz", rng.normal(0, .15, 3)))]:
                relative = (base * perturb).inv() * future_rot
                ori6 = relative.as_matrix()[:, :, :2].reshape(-1)
                value = np.concatenate([q[indices].reshape(-1), qd[indices].reshape(-1), ori6]).astype(np.float32)[None]
                yield f"{path.stem}:{start}:{kind}", value, records[-1]


def validate(source, compact, references, trials, fixture_path):
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    sessions = [ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
                for path in (source, compact)]
    rng = np.random.default_rng(9450)
    random = [("zero", np.zeros((1, 640), np.float32), None)]
    for scale in (.001, .1, 1., 3., 10.):
        random.extend((f"random:{scale}:{i}", rng.normal(0, scale, (1, 640)).astype(np.float32), None)
                      for i in range(trials))
    cases = random + list(real_cases(references, rng))
    if len(cases) == len(random):
        raise ValueError("No real q29G1 references were found; refusing a random-only proof")
    worst, unequal, real_max, references_used, fixtures, by_category = 0., 0, 0., {}, [], {}
    t = time.monotonic()
    for label, value, reference in cases:
        # Fill unused fields too, proving that selector 3 ignores those slices.
        full_input = rng.normal(0, .2, (1, 1751)).astype(np.float32)
        full_input[:, :4] = 0
        full_input[:, 0] = 3
        full_input[:, 4:584] = value[:, :580]
        full_input[:, 590:650] = value[:, 580:]
        expected = sessions[0].run(None, {"obs_dict": full_input})[0]
        actual = sessions[1].run(None, {"obs_dict": value})[0]
        assert expected.shape == actual.shape == (1, 64)
        assert np.isfinite(expected).all() and np.isfinite(actual).all()
        np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=1e-6, err_msg=label)
        error = float(np.max(np.abs(expected - actual)))
        worst = max(worst, error)
        unequal += not np.array_equal(expected.view(np.uint32), actual.view(np.uint32))
        category = "real" if reference else "random"
        by_category[category] = by_category.get(category, 0) + 1
        if reference:
            real_max = max(real_max, error)
            first_for_reference = reference["file"] not in references_used
            references_used[reference["file"]] = reference
            if first_for_reference:
                fixtures.append({"label": label, "input": value[0].tolist(), "expectedTokens": expected[0].tolist()})
        elif label == "zero":
            fixtures.append({"label": label, "input": value[0].tolist(), "expectedTokens": expected[0].tolist()})
    fixture_path.write_text(json.dumps({"sourceFullSha256": SOURCE_SHA256, "selector": 3,
        "layout": "q290,qd290,orientation60", "fixtures": fixtures}, indent=2) + "\n")
    return {"status": "passed", "onnxRuntime": ort.__version__, "provider": "CPUExecutionProvider",
        "onnx": onnx.__version__, "threads": 1, "cases": len(cases), "casesByCategory": by_category,
        "maxAbsError": worst, "realMaxAbsError": real_max, "nonBitExactCases": unequal,
        "atol": 1e-6, "rtol": 1e-6, "seconds": time.monotonic() - t,
        "referenceFiles": list(references_used.values()), "fixtureFile": fixture_path.name,
        "scope": "Finite sampled numerical equivalence to full selector 3; not a closed-loop stability or cross-platform guarantee"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--references", type=Path, required=True)
    parser.add_argument("--trials-per-scale", type=int, default=32)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    full, model, weights = extract(args.source, args.output)
    fixtures = args.output.with_name("encoder-g1-fixtures.json")
    validation = validate(args.source, args.output, args.references, args.trials_per_scale, fixtures)
    report = {"file": args.output.name, "bytes": args.output.stat().st_size, "sha256": digest(args.output),
        "source": "sonic_h2_native_release/experiments/gpu3_20260907_stage2_step009450/model_encoder_full.onnx",
        "sourceSha256": SOURCE_SHA256, "sourceSelector": 3, "globalStep": 9450,
        "input": {"name": "obs_dict", "type": "float32", "shape": [1, 640]},
        "output": {"name": "encoded_tokens", "type": "float32", "shape": [1, 64]},
        "inputLayout": {"jointPositions": [0, 290], "jointVelocities": [290, 580], "orientation6D": [580, 640],
            "jointOrder": "G1 semantic (mujoco_to_isaaclab), without G1-to-H2 hip projection",
            "fullMapping": {"selector": 3, "qAndQd": [4, 584], "orientation": [590, 650]}},
        "irVersion": model.ir_version, "opsets": [{"domain": o.domain, "version": o.version} for o in model.opset_import],
        "extraction": {"kind": "Exact graph branch extraction", "nodes": len(model.graph.node),
            "retainedWeightTensors": len(weights), "allWeightsByteIdentical": True,
            "weightHashes": {w.name: hashlib.sha256(w.SerializeToString()).hexdigest() for w in weights},
            "tokenQuantizerRetained": True, "checkpointReexport": False,
            "reason": "The historical training checkpoint directory is not present; the validated deployed full ONNX is the source"},
        "validation": validation, "script": Path(__file__).name, "scriptSha256": digest(Path(__file__))}
    args.output.with_name("encoder-g1-validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"model": args.output.name, "bytes": report["bytes"], "sha256": report["sha256"],
        "cases": validation["cases"], "maxAbsError": validation["maxAbsError"], "nonBitExactCases": validation["nonBitExactCases"]}))


if __name__ == "__main__":
    main()
