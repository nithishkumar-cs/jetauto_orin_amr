# Perception detector

`detector_node` converts rectified RGB images into standard
`vision_msgs/Detection2DArray` messages:

```text
/camera/rgb/image_rect_color  (sensor_msgs/Image: rgb8 or bgr8)
                    ↓
       TensorRT detector
                    ↓
/perception/detections_2d     (vision_msgs/Detection2DArray)
```

The ROS node depends only on the `DetectorBackend` contract. The first production
backend is YOLO26/TensorRT; a later architecture such as RF-DETR can implement the
same interface without changing the topics, depth projection, tracking, or fusion.
There is no Python, PyTorch, or ONNX Runtime dependency in the robot workspace.

The node-scoped `backend` parameter selects the registered production backend.
`DetectorNode` does not include concrete model/runtime configuration; the backend
factory currently maps `yolo_tensorrt` to `YoloTensorRtBackend` and owns its ROS
parameter parsing.

TensorRT runtime mechanics are isolated in a private `runtime/tensorrt/Session`.
The session owns engine deserialization, the execution context, tensor discovery
and shape resolution, bound CUDA buffers, the CUDA stream, enqueueing, and output
downloads. Model backends reuse that runtime and retain only their own tensor
contract, preprocessing, and postprocessing. This keeps the YOLO implementation
small and avoids repeating TensorRT plumbing when another architecture is added.

## TensorRT engine contract

The current YOLO backend accepts an engine with exactly one float32/float16 NCHW
image input and one float32/float16 output. Batch size is one. Fixed input sizes
come from the engine; `dynamic_input_width` and `dynamic_input_height` select the
shape only when the engine input is dynamic.

Two output layouts are supported:

- `raw_xywh`: `[1, 4 + class_count, prediction_count]` or its transpose. Boxes
  are `center_x, center_y, width, height`; remaining values are class scores.
- `end_to_end_xyxy`: YOLO26's native `[1, prediction_count, 6]` output. Rows are
  `left, top, right, bottom, score, class_index`. The exported graph already
  performs suppression, so the backend filters and limits these rows without
  applying NMS a second time.

Coordinates are expected in the letterboxed network-input grid. The backend
maps them back into the original RGB grid before publishing. A labels file has
one class name per line in the exact output-class order.

## Deployment

The validated deployment uses the official Ultralytics YOLO26n COCO checkpoint.
Export it to ONNX, place the ONNX model on the Jetson, and build the serialized
engine on that Jetson:

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=/opt/jetauto_orin_amr/models/detector/yolo26n.onnx \
  --saveEngine=/opt/jetauto_orin_amr/models/detector/yolo26n_fp16.engine \
  --fp16 \
  --builderOptimizationLevel=3
```

The `.onnx` and `.engine` artifacts are ignored by Git. TensorRT engine files are
deployment artifacts tied to the target runtime/GPU and should be rebuilt on the
Jetson rather than committed.

The package builds its core and ROS tests without TensorRT so a development
laptop can still compile the workspace. Instantiating the production backend in
such a build fails explicitly; it never falls back to fake detections.

On CUDA/TensorRT builds, both preprocessing implementations satisfy one internal
destination contract: write a normalized CHW tensor into TensorRT's bound input
buffer on its CUDA stream and return the letterbox transform. Select one with
`preprocessing_backend`:

- `cuda` (default) uploads the packed source image and runs one fused kernel for
  RGB/BGR conversion, bilinear resize, letterboxing, normalization, HWC-to-CHW
  conversion, and direct FP32/FP16 TensorRT-input writes;
- `cpu` runs the portable reference algorithm and then uploads its completed
  FP32/FP16 tensor. It is retained as a correctness and performance baseline,
  not as the deployed default.

Inference and postprocessing are identical for both selections, which makes the
parameter useful for parity tests and end-to-end benchmarking.

## Tests

The package separates portable unit tests, fake-backend ROS component tests,
CUDA preprocessing tests, and a real-engine node integration test. The
integration test publishes a generated RGB image over the configured ROS topic
and requires the production `DetectorNode` to load the engine, preprocess on
CUDA, execute TensorRT, postprocess, and publish a structurally valid
`Detection2DArray` with the original header.

The integration test is built only when CUDA and TensorRT development files are
available. At runtime it skips when no CUDA device or engine artifacts exist. It
uses the deployment paths below by default; alternate artifacts can be selected
with:

```bash
export PERCEPTION_DETECTOR_TEST_ENGINE_PATH=/path/to/model.engine
export PERCEPTION_DETECTOR_TEST_LABELS_PATH=/path/to/labels.txt
```

Run the full package suite with:

```bash
colcon test --merge-install --packages-select perception_detector
colcon test-result --test-result-base build/perception_detector/test_results --verbose
```

## Validated artifact record

Validated on 2026-08-03 using Ultralytics 8.4.55, TensorRT 10.3.0, and an Orin
GPU (compute capability 8.7):

- checkpoint: official `yolo26n.pt`, SHA-256
  `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`;
- ONNX: fixed `images` input `[1,3,640,640]`, `output0` output `[1,300,6]`,
  SHA-256 `8ed735745540f721a925e871e3482ed10ebd2b464b02563881434e18224fc8a3`;
- FP16 engine: 8.09 MiB, SHA-256
  `cbb440ab4d5bfdbfa8f0c015bd34d059adf1214a130a8272046d945c52e6b92b`;
- labels: 80 COCO classes, SHA-256
  `bd17f1ee35d5f3c862a4894605855abbb9dda4b0621fdb0ac4c2c8c7bb7e730a`.

`trtexec` measured 4.49 ms mean GPU compute and 4.80 ms mean host latency
with random input and unlocked clocks. A ROS smoke test with Ultralytics'
810×1080 bus image produced one bus and four person detections. After DDS
discovery, full publish-to-result latency was 88.5–90.2 ms for four consecutive
steady-state frames. One initial best-effort publication was dropped during
startup; the following five delivered frames all produced results.

On 2026-08-24, the selectable implementations were exercised through the same
production ROS node and FP16 engine. CUDA FP32 output matched the CPU reference
within `1e-5`, direct FP16 output passed a separate device test, and both paths
produced the same five detections for every measured bus-image frame. Ten
sequential steady-state publish-to-result measurements produced:

- `cuda`: 18.87–19.20 ms, 19.00 ms mean;
- `cpu`: 112.64–115.55 ms, 113.84 ms mean.

No measured attempts were dropped in either run.
