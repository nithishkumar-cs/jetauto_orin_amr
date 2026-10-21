# Detector publish-to-result latency, Release build (2026-10-21)

Setup: `perception_detector` built `-DCMAKE_BUILD_TYPE=Release`, deployed config
(`configs/perception/detector.yaml`, YOLO26n FP16 engine), Orin, clocks as found.
Tool: `tests/performance_regression_tests/bench_detector_latency.py` publishes a
generated 810x1080 rgb8 noise image (5 Hz, 5 warmup, 30 measured) and matches the
result by header stamp. Content-independent except postprocessing (native
end-to-end output, no NMS), so noise is a fair latency proxy; it does not test accuracy.

| preprocessing | runs | mean ms | min-max ms | dropped (best-effort) |
| --- | --- | ---: | --- | --- |
| cuda | 2 x 30 | 18.62 | 18.1-19.3 | 0 and 4 |
| cpu | 2 x 30 | 63.8 | 49.7-66.0 | 2 and 0 |

Versus the 2026-08-24 README figures (19.00 / 113.84 ms): CUDA is unchanged within
noise (the GPU path dominates); CPU preprocessing is about 44% faster, consistent
with an optimized build (earlier build type was not recorded). CUDA stays the
default; CPU remains a baseline only. Drops are large best-effort messages lost
in DDS transport, not detector failures; the node answered every frame it received.
Gotcha: killing `ros2 run` leaves `detector_node` running and answers both runs;
launch the binary directly.
