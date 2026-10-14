# Runtime Modes

Hardware, replay, debug, profile, and production mode definitions.

Primary file: `runtime_modes.yaml`.

Use `scripts/dev_helpers/build_mode.sh debug|profile|production` to select
compiler build type separately from runtime instrumentation. `enable_cuda_timing`
and `export_metrics` are detector instrumentation switches; profile mode enables
them, while debug and production disable them. Assertions follow the compiler
build type and have no runtime switch.
