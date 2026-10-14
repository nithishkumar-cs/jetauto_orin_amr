# Runtime Modes

Hardware, replay, debug, profile, and production mode definitions.

Primary file: `runtime_modes.yaml`.

Use `scripts/dev_helpers/build_mode.sh debug|profile|production` to select
compiler build type separately from runtime instrumentation. `export_metrics`
makes every perception node publish per-cycle metrics on `/diagnostics` (see
`src/perception/metrics/README.md`); `enable_cuda_timing` adds detector CUDA
event timing. Profile mode enables both; debug and production disable them. Assertions follow the compiler
build type and have no runtime switch.
