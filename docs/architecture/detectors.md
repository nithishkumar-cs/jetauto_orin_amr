# Detector Roadmap

## Phase 1: Debug Backend

The debug backend publishes a deterministic center detection. It exists only to validate graph timing, message contracts, visualization, and downstream safety plumbing before a model engine is available.

## Phase 2: YOLO TensorRT

YOLO is the first real detector path. It should be used to prove:

- camera timing
- CUDA preprocessing
- TensorRT engine loading
- postprocessing format
- detection rate and latency reporting
- benchmark harness behavior

Config: `src/perception/inference_core/config/yolo_detector.yaml`.

## Phase 3: RF-DETR TensorRT

RF-DETR should plug into the same `DetectorBackend` API and publish the same `Detection2DArray` output. The RF-DETR work should focus on:

- ONNX export shape stability
- TensorRT parser/plugin blockers
- FP16 and INT8 feasibility
- memory footprint on Orin Nano
- latency versus YOLO/baseline detectors
- quality/latency tradeoff reporting

Config: `src/perception/inference_core/config/rf_detr_detector.yaml`.

## Required Report

Each detector run should record:

- engine path and model version
- precision mode
- input resolution
- preproc latency
- inference latency
- postprocess latency
- end-to-end detection latency
- GPU memory use
- observed failures or dropped frames
