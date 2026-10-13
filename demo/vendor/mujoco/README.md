# MuJoCo browser runtime

Unmodified single-threaded `mujoco.js`, `mujoco.wasm`, and declarations from
[`@mujoco/mujoco@3.10.0`](https://www.npmjs.com/package/@mujoco/mujoco/v/3.10.0),
the canonical Google DeepMind WebAssembly bindings. This version matches the
server physics export. All browser runtime files are served locally.

- [Official bindings and API guide](https://github.com/google-deepmind/mujoco/blob/3.10.0/wasm/README.md)
- [Source and license](https://github.com/google-deepmind/mujoco/tree/3.10.0)
- Package archive: `https://registry.npmjs.org/@mujoco/mujoco/-/mujoco-3.10.0.tgz`

The single-threaded build runs inside the controller's module worker and does
not require cross-origin isolation. Serve the WASM with `application/wasm`.
The bundled Apache 2.0 `LICENSE` applies to MuJoCo.
