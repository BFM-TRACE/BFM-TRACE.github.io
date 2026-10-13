// Preserve the source robot's joint space. G1 targets are encoded by the G1
// branch; only decoded H2 actions enter the H2 controller and physics.
export function prepareTrackingReference(ref, entry = {}) {
  const name = ref.id || entry.id || 'tracking reference';
  const fail = message => { throw new Error(`${name}: ${message}`); };
  const declared = [ref.referenceSpace, ref.referenceType, entry.referenceSpace, entry.referenceType].filter(value => value !== undefined);
  const referenceSpace = declared[0] ?? 'h2';
  if (!['h2', 'g1'].includes(referenceSpace) || declared.some(value => value !== referenceSpace)) fail('invalid or conflicting reference space');
  const encoderMode = referenceSpace === 'g1' ? 3 : 0;
  for (const value of [ref.encoderMode, entry.encoderMode]) if (value !== undefined && value !== encoderMode) fail('encoder mode does not match reference space');
  if (entry.id !== undefined && ref.id !== undefined && entry.id !== ref.id) fail('manifest id does not match reference id');
  const fps = ref.fps ?? entry.fps ?? 50;
  if (!Number.isFinite(fps) || fps <= 0) fail('fps must be a positive finite number');
  if (ref.fps !== undefined && entry.fps !== undefined && ref.fps !== entry.fps) fail('manifest fps does not match reference fps');
  const startupHoldSeconds = ref.startupHoldSeconds ?? entry.startupHoldSeconds ?? 0;
  if (!Number.isFinite(startupHoldSeconds) || startupHoldSeconds < 0) fail('startup hold must be a nonnegative finite number of seconds');
  if (ref.startupHoldSeconds !== undefined && entry.startupHoldSeconds !== undefined && ref.startupHoldSeconds !== entry.startupHoldSeconds) fail('manifest startup hold does not match reference startup hold');
  const q = referenceSpace === 'g1' ? ref.q29G1 : ref.q31;
  const dq = referenceSpace === 'g1' ? ref.qd29G1 : ref.qd31;
  const width = referenceSpace === 'g1' ? 29 : 31;
  if (referenceSpace === 'g1' && (ref.q31 !== undefined || ref.qd31 !== undefined || ref.initialQpos !== undefined)) fail('G1 targets must not contain H2 joint data or a physics initial pose');
  const count = ref.rootQuat?.length;
  if (!Array.isArray(ref.rootQuat) || !count || !Array.isArray(q) || !Array.isArray(dq) || q.length !== count || dq.length !== count) fail('joint and root quaternion frame counts must match');
  for (const frames of [ref.frames, entry.frames]) if (frames !== undefined && (!Number.isInteger(frames) || frames !== count)) fail('declared frame count does not match reference arrays');
  const validRow = (row, size) => Array.isArray(row) && row.length === size && row.every(Number.isFinite);
  const frames = ref.rootQuat.map((quat, i) => {
    if (!validRow(quat, 4) || Math.hypot(...quat) < 1e-8 || !validRow(q[i], width) || !validRow(dq[i], width)) fail(`invalid numeric target at frame ${i}`);
    return referenceSpace === 'g1' ? {quat, policyQ:q[i], policyDq:dq[i]} : {quat, q:q[i], dq:dq[i]};
  });
  return {...ref, id:ref.id ?? entry.id, fps, referenceSpace, encoderMode, startupHoldSeconds, frames};
}
