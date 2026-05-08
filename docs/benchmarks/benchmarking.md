# Benchmarking

Benchmarks should answer two questions:

1. Is the robot perception graph real-time enough for safe indoor operation?
2. What does RF-DETR cost versus a baseline detector on Orin Nano?

Use `benchmarks/latency_probe_node` during live runs and bag replays. Record:

- image FPS
- detection FPS
- obstacle FPS
- detection header-to-now latency
- obstacle header-to-now latency
- CUDA preprocessing kernel latency
- detector backend status
- system health summary

Suggested report table:

| Detector | Precision | Input | Preproc ms | Inference ms | Post ms | E2E ms | FPS | Notes |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| YOLO baseline | FP16 | 640 | TBD | TBD | TBD | TBD | TBD | First hardware baseline |
| RF-DETR | FP16 | TBD | TBD | TBD | TBD | TBD | TBD | Primary engineering target |

Keep raw CSV files out of git unless they are small curated benchmark summaries.

