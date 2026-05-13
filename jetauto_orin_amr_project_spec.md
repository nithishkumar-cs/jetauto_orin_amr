# JetAuto + Jetson Orin Nano Production-Style Indoor AMR Project Spec

## 1. Project Summary

This project is a **production-style indoor AMR perception, mapping, sensor-fusion, navigation, and safety stack** built on a **Hiwonder JetAuto Standard Kit / Without Controller** and a separately purchased **NVIDIA Jetson Orin Nano Developer Kit**.

The project is designed to look and feel like a real robotics perception subsystem, not a toy demo. The main focus is to build a high-performance, modular, measurable robot software stack using **ROS2, modern C++, CUDA, TensorRT, and Jetson deployment**.

The project starts with system design. Before implementation, the architecture should define package responsibilities, ROS2 topic/message contracts, runtime modes, TF tree, detector backend interface, safety contract, testing strategy, profiling modes, and development order.

---

## 2. Final Project Definition

### One-line definition

A **Jetson-powered indoor warehouse-style AMR** that can:

- localize itself,
- build and update maps,
- detect and track static and dynamic obstacles,
- fuse detections with depth, LiDAR, odometry, and robot pose,
- generate safe navigation outputs,
- support live hardware operation and rosbag replay input modes,
- and run in real time with an efficient C++/CUDA software stack.

### What this project should demonstrate

- production-style ROS2 system design
- strong modern C++ engineering
- extensive CUDA use on Jetson
- TensorRT-based model deployment
- plug-and-play detector backend design
- RF-DETR deployment and optimization on embedded hardware
- YOLO / RT-DETR / other model backend support
- calibration and transform handling
- depth, point clouds, and 3D geometry
- sensor fusion
- localization and mapping
- safety logic and degraded-mode behavior
- rosbag replay support
- unit tests, integration tests, system tests, benchmarking, and profiling
- real hardware debugging and deployment

### What this project is not

- not a toy line-following robot
- not a Python-first research prototype
- not a 2D object-detection demo
- not a simulation-only project
- not a full autonomous vehicle stack
- not a manipulation / robot arm project
- not a product with fleet management

---

## 3. Project Philosophy

This project should be built like a **real robotics software subsystem**, not like a collection of demos.

### Core rules

- **C++ is the primary implementation language** for runtime-critical components.
- **CUDA is used aggressively** for image/depth preprocessing, point cloud preparation, filtering, tensor preparation, and other dense hot paths.
- **TensorRT is used for optimized inference** where possible.
- **Python is allowed for tooling**, evaluation, plotting, dataset inspection, and automation scripts, but not for critical runtime loops unless there is a clear reason.
- **No unnecessary CPU-GPU copies** in hot paths.
- **No per-frame dynamic allocations** in hot paths when persistent buffers can be used.
- **No hardcoded detector dependency** in the downstream stack.
- **Hardware and replay modes should use the same topic contracts.** External simulation, if used later, should live outside this Jetson robot repository and target those contracts through ROS topics or recorded bags.
- **Every critical subsystem should be benchmarked.**
- **Every important design decision should be documented.**

### Desired engineering style

- modular domain-based repository layout
- clean package interfaces
- explicit ownership of memory and resources
- reusable C++/CUDA components
- deterministic safety behavior
- strong logging and diagnostics
- replayable evaluation
- package-local unit tests
- top-level integration/system tests
- runtime modes for debug, profiling, and production

---

## 4. Exact Hardware Configuration

### 4.1 Selected robot platform

- **Hiwonder JetAuto Standard Kit / Without Controller**

This is the selected base platform for the project. Since it is the **without controller** version, the main onboard computer is provided separately.

### 4.2 Selected onboard computer

- **NVIDIA Jetson Orin Nano Developer Kit**

This is the main onboard compute platform. It will run:

- Ubuntu / JetPack
- ROS2
- CUDA
- TensorRT
- the robot runtime stack
- evaluation and benchmarking tools

### 4.3 JetAuto hardware assumed for this build

For this project, the selected JetAuto Standard Kit / Without Controller configuration is assumed to include:

- mecanum-wheel mobile robot chassis
- wheel motors with encoder feedback
- low-level robot-side controller/electronics
- LiDAR
- 3D depth camera
- **7-inch LCD display**
- base power hardware
- robot mounting structure and wiring

This selected configuration assumes:

- **no microphone array**

### 4.4 Additional hardware you provide

- NVIDIA Jetson Orin Nano Developer Kit
- NVMe SSD for Jetson
- compatible power wiring for the Jetson dev kit
- mounting plate / standoffs / mechanical hardware for the Jetson
- keyboard / mouse / monitor for initial bring-up
- Ethernet cable or stable Wi-Fi setup
- optional gamepad/controller for teleoperation

### 4.5 Important hardware integration note

This setup should be treated as a **custom integration**, not a guaranteed plug-and-play supported configuration.

You may need to handle:

- mechanical mounting for the official Orin Nano dev kit
- safe power routing into the Jetson
- USB routing to robot peripherals
- bandwidth management for camera/LiDAR/controller traffic
- cable management and strain relief
- thermal management and airflow

This integration work is part of making the project realistic and production-oriented. It should be documented clearly.

---

## 5. Robot Role and Use Case

The robot should be framed as a:

### Mini indoor warehouse AMR prototype

Primary use cases:

- drive through corridor / aisle-like indoor spaces
- build and save maps
- localize against existing maps
- detect and avoid static obstacles
- detect dynamic obstacles such as people, carts, boxes, and chairs
- fuse perception outputs into robot/world-frame obstacle states
- generate safe navigation decisions such as slow-down or stop
- expose clear status, diagnostics, and benchmark metrics

This gives the project a strong warehouse/mobile robotics identity while staying realistic in scope.

---

## 6. System Design Comes First

Before writing major components, the system design should define:

- runtime graph
- package responsibilities
- ROS2 topic/message contracts
- TF tree
- detector plugin interface
- hardware/replay input abstraction
- safety input/output contract
- profiling and logging strategy
- unit and system test structure
- failure modes
- performance budgets
- component-by-component development order

The project should not start as random coding. It should start as a system architecture that can be implemented component by component.

---

## 7. Main Architectural Split

The system has two major planes.

### 7.1 Data plane

The high-rate runtime path:

- camera frames
- depth frames
- LiDAR scans
- IMU data if available
- wheel odometry
- preprocessed tensors
- detections
- 3D obstacle candidates
- fused obstacles
- tracks
- robot pose
- safety state

This is where performance matters most.

### 7.2 Control plane

The lower-rate orchestration path:

- startup
- configuration
- calibration loading
- runtime mode selection
- model backend selection
- task assignment
- health checks
- diagnostics
- profiling controls
- replay controls

The control plane should not pollute the high-rate data plane.

---

## 8. Runtime Modes

The main robot project supports one runtime mode axis for bring-up:

### 8.1 Instrumentation mode

The runtime instrumentation level can be:

#### `debug`

Used during development.

Includes:

- verbose logs
- extra assertions
- sanity checks
- detailed topic validation
- transform validation
- slower but more transparent behavior

#### `profile`

Used during performance measurement.

Includes:

- CUDA event timing
- per-stage latency
- end-to-end latency
- FPS counters
- GPU/CPU/memory usage
- detector comparison metrics
- rosbag replay benchmarking

#### `production`

Used for normal robot operation.

Includes:

- minimal logs
- lightweight counters
- health status
- stale-topic detection
- dropped-frame counters
- critical latency summaries
- error reporting

Production mode should still have observability, but not heavy profiling overhead by default.

Bring-up should use a single launch file. The user selects `debug`, `profile`,
or `production` through launch arguments, and `system_modes.yaml` defines the
default subsystem set for each mode. The launch system should not branch on
whether data is coming from live hardware or rosbag playback; nodes should
launch and listen on the standard topic contracts either way.

### 8.2 Replay and simulation boundary

Simulation is not a Jetson runtime package group for this repository.

The main robot stack defines clean topic contracts for live hardware and rosbag
replay. If an external simulator is used later, it should live in a separate
workstation project and publish compatible ROS2 topics or generate compatible
bags. The Jetson repository should not contain simulator assets, simulator
packages, or Jetson-side simulation launch paths.

---

## 9. External Simulation Boundary

Simulation is a supporting workflow outside this repository, not a core package
group in this repository.

The main AMR project should not contain heavy Isaac Sim worlds, simulator robot assets, or simulator-specific runtime packages. Instead, the main project should expose stable ROS2 contracts so a separate simulation workflow can connect cleanly.

Any separate simulation project should define:

- laptop/workstation Isaac Sim setup
- simulated robot model
- simulated sensors
- ROS2 bridge configuration
- topic remapping into the Jetson stack
- scenario definitions
- optional bag generation from simulation

Design rule:

**The robot stack should not know or care whether compatible topics came from live hardware, rosbag replay, or an external simulator.**

---

## 10. ROS2 Workspace and Repository Layout

This project should be structured as a proper ROS2/colcon workspace. Domain folders should live mostly under `src/`, because ROS2 packages are built from there.

### 10.1 Top-level layout

```text
jetauto_orin_amr/
  README.md

  docker/
    Dockerfile.jetson
    Dockerfile.dev
    compose.yaml

  configs/
    robot/
    sensors/
    models/
    calibration/
    runtime_modes/

  docs/
    architecture/
    calibration/
    deployment/
    benchmarks/
    failure_analysis/

  scripts/
    setup/
    flashing/
    networking/
    dev_helpers/

  bags/
    README.md

  datasets/
    README.md

  src/
    platform/
    perception/
    localization/
    navigation/
    safety/
    tools/

  tests/
    system_tests/
    integration_tests/
    replay_tests/
    performance_regression_tests/
```

### 10.2 Why `src/` exists

`src/` contains ROS2 packages that are built by:

```bash
colcon build
```

Project-level assets such as docs, configs, Dockerfiles, datasets, and bags should not be treated as ROS2 packages and should stay outside `src/`.

### 10.3 Domain-based package layout under `src/`

```text
src/
  robot_bringup/

  platform/
    base_interface/
    sensor_drivers/
    tf_and_calibration/

  perception/
    cuda_common/
    preproc/
    inference_core/
    models/
      rf_detr_backend/
      yolo_backend/
      rt_detr_backend/
    geometry/
    sensor_fusion/
    tracking/

  localization/
    localization_mapping/
    odometry_fusion/

  navigation/
    navigation_tasks/
    teleop_tools/

  safety/
    safety_layer/

  tools/
    diagnostics/
    benchmarks/
    evaluation_tools/
    bag_tools/
    calibration_tools/

```

---

## 11. Per-Package Structure

Every serious ROS2 package should have a consistent internal structure.

### 11.1 C++ package template

```text
src/<domain>/<package_name>/
  CMakeLists.txt
  package.xml
  README.md

  include/<package_name>/
    public_headers.hpp

  src/
    implementation.cpp

  test/
    test_<component>.cpp
    test_<math_or_logic>.cpp

  launch/
    package_specific.launch.py

  config/
    package_params.yaml
```

Not every package needs every folder on day one, but runtime packages should eventually follow this pattern.

### 11.2 CUDA package template

```text
src/perception/<cuda_package>/
  CMakeLists.txt
  package.xml
  README.md

  include/<cuda_package>/
    public_api.hpp
    cuda_buffer.hpp

  src/
    node.cpp
    runtime.cpp

  cuda/
    kernels.cu
    preprocessing_kernels.cu

  test/
    test_cpu_reference.cpp
    test_cuda_correctness.cpp
    test_buffer_lifecycle.cpp

  launch/

  config/
```

CUDA packages should include CPU reference checks where possible, so optimized kernels can be tested for correctness.

### 11.3 Python/tooling package template

```text
src/tools/<tool_package>/
  package.xml
  setup.py
  README.md

  <tool_package>/
    __init__.py
    main.py

  test/
    test_tool_logic.py

  config/
```

Python is acceptable for tools, reports, plotting, dataset inspection, and evaluation support.

---

## 12. Testing Structure

Testing should be split into package-local tests and top-level cross-package tests.

### 12.1 Package-local unit tests

Each package should own its own `test/` directory.

Examples:

```text
src/perception/preproc/test/
src/perception/geometry/test/
src/perception/inference_core/test/
src/perception/models/yolo_backend/test/
src/perception/models/rf_detr_backend/test/
src/safety/safety_layer/test/
src/platform/tf_and_calibration/test/
src/localization/odometry_fusion/test/
```

Package-local tests should cover:

- math correctness
- transform handling
- preprocessing correctness
- CUDA kernel correctness against CPU reference
- safety rule logic
- detector output normalization
- buffer ownership/lifecycle
- small mocked inputs

### 12.2 Top-level system tests

Cross-package tests live in the top-level `tests/` directory.

```text
tests/
  system_tests/
  integration_tests/
  replay_tests/
  performance_regression_tests/
```

These include:

- launch/system graph tests
- rosbag replay tests
- hardware smoke tests
- full perception pipeline tests
- detector backend comparison tests
- safety failure-mode tests
- performance regression tests

### 12.3 Rule of thumb

- if it tests one package's internal behavior, keep it inside that package
- if it tests multiple packages working together, put it under top-level `tests/`

---

## 13. Core Components

### 13.1 `robot_bringup`

Purpose: system entry point for launching the robot stack.

Responsibilities:

- launch the robot graph from one entrypoint
- load configuration
- load calibration files
- select detector backend
- select runtime mode
- apply mode-default component toggles from config
- start diagnostics
- validate required topics and parameters

This package should fail early if required resources are missing.

### 13.2 `platform/base_interface`

Purpose: bridge between the Jetson and the JetAuto low-level robot controller.

Responsibilities:

- send velocity commands to the base
- read wheel odometry
- publish chassis state
- publish controller health
- detect stale or dropped controller communication

IMU note: if IMU data physically arrives through the base controller, the transport can live near `base_interface`, but the logical output should still be exposed as a standard IMU sensor topic.

### 13.3 `platform/sensor_drivers`

Purpose: provide controlled access to robot sensors.

Responsibilities:

- camera/depth bring-up
- LiDAR bring-up
- IMU topic exposure if available
- normalized topic names/types
- timestamp validation
- bandwidth and frame-drop monitoring

### 13.4 `platform/tf_and_calibration`

Purpose: define and validate the robot's geometric frame system.

Responsibilities:

- maintain TF tree
- load camera intrinsics
- load camera-to-base extrinsics
- load LiDAR-to-base extrinsics
- validate frame consistency
- validate timestamp alignment
- provide calibration sanity tools

### 13.5 `perception/cuda_common`

Purpose: reusable CUDA infrastructure for the whole perception stack.

Responsibilities:

- CUDA stream wrappers
- device buffer wrappers
- pinned host buffer wrappers
- CUDA error handling
- CUDA event timing
- common GPU utilities
- shared memory/resource patterns

This should be one of the clean reusable engineering components of the project.

### 13.6 `perception/preproc`

Purpose: high-performance preprocessing for image/depth/model inputs.

Responsibilities:

- image resize/crop
- color conversion
- normalization
- layout conversion
- tensor preparation
- depth filtering
- depth masking
- point-wise filtering where needed

Implementation style:

- C++ runtime API
- CUDA kernels for hot paths
- persistent buffers
- minimal host-device copies
- CPU reference tests for correctness

### 13.7 `perception/inference_core`

Purpose: model-agnostic inference runtime interface.

Responsibilities:

- define detector plugin interface
- load selected model backend
- manage common detector configuration
- normalize all model outputs into a shared detection schema
- expose timing/health information

The rest of the robot should not care whether the active model is RF-DETR, YOLO, RT-DETR, or another detector later.

### 13.8 `perception/models/rf_detr_backend`

Purpose: RF-DETR detector backend.

RF-DETR is the main engineering challenge and flagship detector target.

Responsibilities:

- deploy RF-DETR on Jetson where possible
- integrate with `inference_core`
- optimize preprocessing/postprocessing path
- measure latency, throughput, memory, and stability
- compare against YOLO/RT-DETR baselines

### 13.9 `perception/models/yolo_backend`

Purpose: YOLO detector backend.

Role:

- high-speed baseline
- fallback detector option
- detector plugin architecture validation
- RF-DETR comparison path

### 13.10 `perception/models/rt_detr_backend`

Purpose: RT-DETR or similar DETR-family backend.

Role:

- DETR-style comparison backend
- useful architectural comparison against RF-DETR
- validates support for multiple detector families

### 13.11 `perception/geometry`

Purpose: convert image-space and depth-space perception into robot/world-space obstacle information.

Responsibilities:

- lift 2D detections into 3D using depth
- convert depth to point cloud when needed
- project obstacles into `base_link` or map frame
- perform geometric filtering
- extract free-space / occupied-space primitives
- reject invalid or low-confidence geometry

### 13.12 `perception/sensor_fusion`

Purpose: fuse perception, geometry, and robot-state signals into a consistent scene representation.

Responsibilities:

- fuse detections with depth
- fuse obstacle positions with robot pose
- fuse LiDAR/depth obstacle evidence where appropriate
- combine confidence signals
- produce authoritative fused obstacle observations

### 13.13 `perception/tracking`

Purpose: maintain temporal continuity for obstacles.

Responsibilities:

- associate obstacles across frames
- maintain track IDs
- smooth obstacle state
- remove stale tracks
- track confidence over time
- estimate simple motion if useful

Start simple: nearest-neighbor association, gating, track timeouts, and basic smoothing.

### 13.14 `localization/localization_mapping`

Purpose: maintain robot localization and map representation.

Responsibilities:

- integrate odometry
- run or integrate mapping/localization system
- produce map/costmap outputs
- monitor localization confidence
- expose localization failure states

### 13.15 `localization/odometry_fusion`

Purpose: combine motion-estimation signals where needed.

Responsibilities:

- fuse wheel odometry with IMU if available
- expose cleaner odometry estimate
- handle stale/missing motion signals
- provide uncertainty/confidence metadata if possible

### 13.16 `navigation/navigation_tasks`

Purpose: high-level robot tasking and mission control.

Responsibilities:

- named navigation goals
- waypoint missions
- patrol routines
- go-home behavior
- task state machine
- task progress reporting

### 13.17 `navigation/teleop_tools`

Purpose: manual operation for bring-up, testing, and emergency control.

Responsibilities:

- keyboard teleop
- gamepad teleop
- safe stop
- manual override

### 13.18 `safety/safety_layer`

Purpose: convert perception/localization/system-health outputs into safe robot behavior.

Responsibilities:

- stop-zone logic
- slow-zone logic
- degraded-mode behavior
- emergency stop conditions
- sensor timeout response
- localization failure response
- detector slowdown response

The safety layer should be deterministic, explainable, and heavily tested.

### 13.19 `tools/diagnostics`

Purpose: make the robot debuggable during real operation.

Responsibilities:

- node health status
- topic freshness monitoring
- dropped-frame monitoring
- hardware status
- sensor heartbeat checks
- startup checks
- error summaries

### 13.20 `tools/benchmarks`

Purpose: measure performance like a real engineering team.

Responsibilities:

- FPS measurement
- per-stage latency
- end-to-end latency
- CPU usage
- GPU usage
- memory usage
- detector backend comparisons
- replay-based benchmarks
- performance regression reports

### 13.21 `tools/evaluation_tools`

Purpose: offline evaluation and failure analysis.

Responsibilities:

- rosbag evaluation runner
- metrics export
- detection comparison reports
- dataset inspection
- calibration validation reports
- failure-case summaries
- plots and tables

### 13.22 `tools/bag_tools`

Purpose: manage recording and replay workflows.

Responsibilities:

- standardize bag recording profiles
- replay bags with consistent remaps
- tag bags by scenario
- record failure cases
- record benchmark datasets

### 13.23 `tools/calibration_tools`

Purpose: support calibration validation and debugging.

Responsibilities:

- verify camera intrinsics loaded correctly
- verify camera/depth alignment
- verify LiDAR-to-base transform
- verify camera-to-base transform
- visualize projected detections/points
- detect obvious frame mistakes

---

## 14. Detector Plugin Architecture

The detector system should be model-agnostic.

### 14.1 Detector interface

All detector backends should implement the same conceptual interface:

```text
initialize(model_config)
prepare(runtime_config)
run(input_tensor, timestamp)
postprocess(raw_outputs)
get_detections()
get_metrics()
shutdown()
```

### 14.2 Common detection output schema

All detectors should output a normalized detection format.

Suggested fields:

```text
Detection2D
  header
  model_name
  backend_name
  class_id
  class_name
  confidence
  bbox_xyxy
  bbox_cxcywh
  image_width
  image_height
  inference_timestamp
  optional_mask_id
  optional_debug_info
```

Downstream packages should not consume RF-DETR-specific outputs directly.

### 14.3 Backend goals

#### RF-DETR

- main challenge backend
- optimize and profile seriously
- document deployment and bottlenecks

#### YOLO

- fast baseline backend
- useful for real-time comparison
- fallback detector option

#### RT-DETR / other DETR backend

- DETR-style comparison backend
- useful for architecture comparison

---

## 15. Main Runtime Data Flow

### 15.1 Live hardware path

```text
JetAuto sensors/base
  -> platform/sensor_drivers
  -> platform/tf_and_calibration
  -> perception/preproc
  -> perception/inference_core + selected model backend
  -> perception/geometry
  -> perception/sensor_fusion
  -> perception/tracking
  -> localization/localization_mapping
  -> safety/safety_layer
  -> navigation/navigation_tasks
  -> platform/base_interface
```

### 15.2 Replay path

```text
external rosbag playback
  -> same topic contracts
  -> same perception/fusion/localization/safety stack
  -> tools/benchmarks + tools/evaluation_tools
```

Rosbag playback should be started outside `robot_bringup`. Bring-up
launches the robot graph and listens on topic contracts; it should not branch
on where compatible messages originated.

### 15.3 Hot path

The most performance-critical path is:

```text
image/depth
  -> CUDA preprocessing
  -> detector inference
  -> postprocessing
  -> depth projection / geometry
  -> fusion
  -> tracking
  -> safety decision
```

This path should be mostly C++ and CUDA/TensorRT where applicable.

---

## 16. Profiling and Observability Strategy

Profiling is not only for development. A production-style robot still needs observability.

### 16.1 Debug mode

Used for development.

Includes:

- verbose logs
- assertions
- extra validation
- detailed topic checks
- transform checks
- slower but more explainable behavior

### 16.2 Profile mode

Used for measurement.

Includes:

- CUDA event timing
- per-node latency
- per-stage latency
- end-to-end latency
- FPS
- memory usage
- GPU utilization
- detector comparison metrics
- replay-based benchmark reports

### 16.3 Production mode

Used for normal robot operation.

Includes lightweight observability:

- health status
- stale topic detection
- dropped frame counters
- lightweight latency summaries
- error counters
- safety decision reason codes

Production mode should avoid heavy profiling overhead by default, but should still expose enough data to debug failures.

---

## 17. Safety Strategy

The safety layer should be rule-based, explainable, and testable.

### 17.1 Basic safety states

- `CLEAR`
- `SLOW`
- `STOP`
- `LOCALIZATION_LOST`
- `SENSOR_DEGRADED`
- `SYSTEM_FAULT`

### 17.2 Example safety rules

- if obstacle enters stop zone, publish `STOP`
- if obstacle enters slow zone, publish `SLOW`
- if localization is lost, stop or limit speed
- if camera/depth stream is stale, degrade behavior
- if detector exceeds latency budget repeatedly, degrade behavior
- if base controller communication is stale, stop

---

## 18. Language and Technology Policy

### 18.1 Primary languages

- **C++17 or newer** for runtime-critical components
- **CUDA** for dense image/depth/point/tensor operations
- **Python** for tooling and offline workflows

### 18.2 Where C++ is mandatory

- platform/base interface
- runtime perception pipeline
- detector interface
- geometry processing
- sensor fusion
- tracking if runtime-critical
- localization integration wrappers
- safety layer
- navigation task state machine if runtime-critical

### 18.3 Where CUDA should be used aggressively

- image preprocessing
- depth preprocessing
- tensor layout conversion
- point-wise filtering
- dense geometry operations
- dense postprocessing where beneficial
- benchmarked acceleration paths

### 18.4 Where Python is acceptable

- rosbag analysis
- plotting
- report generation
- dataset inspection
- calibration reports
- benchmark summaries
- experiment orchestration

---

## 19. Development Flow

The project should be built component by component in this order.

### Phase 0 — System design

Deliverables:

- full architecture document
- runtime graph
- topic contracts
- TF tree plan
- detector plugin interface
- hardware/replay mode design
- profiling mode design
- initial safety contract

### Phase 1 — Repository skeleton and development environment

Deliverables:

- workspace structure with `src/`
- domain folders
- package templates with `include/`, `src/`, `test/`, `launch/`, and `config/`
- Docker/dev container plan
- formatting/linting setup
- basic CI or local test runner

### Phase 2 — Platform bring-up

Build first:

1. `robot_bringup`
2. `platform/base_interface`
3. `navigation/teleop_tools`
4. `platform/sensor_drivers`

Deliverables:

- Jetson boots on robot
- base communication works
- teleop works
- camera/depth stream works
- LiDAR stream works
- display path verified
- basic diagnostics available

### Phase 3 — TF and calibration foundation

Build:

1. `platform/tf_and_calibration`
2. `tools/calibration_tools`

Deliverables:

- TF tree defined
- camera intrinsics loaded
- camera-to-base transform configured
- LiDAR-to-base transform configured
- calibration validation tools started
- unit tests for frame/transform assumptions

### Phase 4 — Replay support

Build:

1. `tools/bag_tools`
2. replay launch configs
3. first top-level replay tests

Deliverables:

- record standard bags
- replay standard bags
- downstream nodes receive same topic contracts
- first regression bags created


### Phase 5 — CUDA foundation

Build:

1. `perception/cuda_common`
2. unit tests for buffer ownership and error handling
3. CUDA timing helpers

Deliverables:

- reusable device buffer wrappers
- CUDA stream wrappers
- pinned host buffer wrappers
- CUDA event timers
- package-local unit tests

### Phase 6 — CUDA preprocessing

Build:

1. `perception/preproc`
2. CPU reference implementation
3. CUDA implementation
4. correctness tests
5. latency benchmark

Deliverables:

- image resize/color/normalize path
- depth filtering path
- persistent buffers
- no per-frame allocation in hot path
- CPU-vs-CUDA correctness checks
- preprocessing benchmark report

### Phase 7 — Inference core and YOLO baseline

Build:

1. `perception/inference_core`
2. `perception/models/yolo_backend`

Deliverables:

- detector plugin interface
- normalized detection schema
- backend selection from config
- YOLO baseline running
- detector unit tests
- detector benchmark harness

### Phase 8 — RF-DETR backend

Build:

1. `perception/models/rf_detr_backend`
2. RF-DETR runtime integration
3. RF-DETR benchmark path

Deliverables:

- RF-DETR running on Jetson
- integrated with same detector interface
- latency/memory/FPS measured
- bottlenecks documented
- comparison against YOLO baseline

### Phase 9 — Geometry projection

Build:

1. `perception/geometry`

Deliverables:

- 2D detections lifted to 3D using depth
- robot-frame obstacle positions
- invalid-depth rejection
- unit tests for projection math
- replay-based geometry validation

### Phase 10 — Sensor fusion and tracking

Build:

1. `perception/sensor_fusion`
2. `perception/tracking`

Deliverables:

- detection + depth fusion
- LiDAR/depth evidence integration where useful
- fused obstacle observations
- tracked obstacles with IDs
- timeout/confidence behavior
- unit tests for association and stale-track handling

### Phase 11 — Localization and mapping

Build:

1. `localization/odometry_fusion`
2. `localization/localization_mapping`

Deliverables:

- odometry integration
- localization/mapping pipeline
- map save/load
- localization status exposed
- failure states defined

### Phase 12 — Safety layer

Build:

1. `safety/safety_layer`

Deliverables:

- stop/slow logic
- degraded-mode behavior
- safety output states
- decision reason logs
- unit tests for safety rules
- system tests for obstacle/sensor/localization failures

### Phase 13 — Navigation tasks

Build:

1. `navigation/navigation_tasks`

Deliverables:

- named goals
- waypoint missions
- patrol behavior
- task status
- safety-aware task behavior

### Phase 14 — Diagnostics, benchmarks, and evaluation

Build:

1. `tools/diagnostics`
2. `tools/benchmarks`
3. `tools/evaluation_tools`

Deliverables:

- health dashboard/logging
- latency/FPS reports
- detector comparison reports
- replay benchmark reports
- failure-case summaries
- performance regression tests

### Phase 15 — System tests and portfolio polish

Build:

1. top-level `tests/system_tests`
2. top-level `tests/integration_tests`
3. top-level `tests/replay_tests`
4. top-level `tests/performance_regression_tests`
5. final docs and videos

Deliverables:

- full demo flow
- reproducible setup guide
- architecture diagrams
- benchmark results
- failure analysis
- demo videos
- resume-ready project summary

---

## 20. What You Should Build Yourself

You should build yourself:

- system architecture
- domain-based repository structure
- ROS2 package interfaces
- hardware/replay input abstraction
- C++/CUDA preprocessing path
- detector plugin system
- RF-DETR backend integration and optimization
- YOLO/RT-DETR baseline backend support
- geometry projection
- sensor fusion
- tracking
- safety layer
- benchmarking/evaluation pipeline
- tests and docs

You should not spend project time reinventing:

- low-level motor firmware
- vendor chassis electronics
- raw wheel-driver implementation
- every SLAM algorithm from scratch

The value of the project is the **upper-stack robotics engineering and performance engineering**, not bare-metal motor control.

---

## 21. What Makes This a Strong Hiring Project

This project can cover most of what hiring managers want to see in a transition into robotics perception:

- real hardware integration
- ROS2
- embedded deployment on Jetson
- strong C++ engineering
- practical CUDA/TensorRT usage
- RF-DETR deployment on embedded hardware
- plug-and-play detector architecture
- calibration and transforms
- depth / point clouds / geometry in practice
- localization and mapping
- sensor fusion
- rosbag replay
- testing and benchmarking
- systems debugging
- performance profiling and optimization
- safety-oriented design
- design tradeoffs and failure handling

---

## 22. What This Project Still Does Not Fully Cover

Even if the project is done well, you still need separate study for:

- interview-style C++ / DSA prep
- deeper estimation and geometry theory
- interview fluency around SLAM, calibration, uncertainty, and transforms

Those should run in parallel with the project.

---

## 23. Minimum Viable Final Demo

A good final demo should show:

- robot starts on Jetson
- one launch entrypoint supports debug, profile, and production
- robot can be teleoperated safely
- robot builds or loads a map
- robot localizes on the map
- robot navigates to a goal or waypoint sequence
- detector backend can be selected from config
- RF-DETR runs and is benchmarked
- YOLO or another baseline also runs
- detections are fused with depth/pose
- tracked obstacles are published
- robot slows/stops appropriately
- CUDA preprocessing is used in the hot path
- metrics are recorded and presented

---

## 24. Recommended Resume / Portfolio Positioning

Suggested summary:

> Built a production-style Jetson-powered indoor AMR perception, mapping, sensor-fusion, and safety stack on a real mobile robot platform using ROS2, modern C++, CUDA, and TensorRT. Designed a modular detector backend system supporting RF-DETR, YOLO, and other models; integrated GPU-accelerated preprocessing, geometry projection, depth/pose fusion, localization, mapping, obstacle tracking, safety-zone logic, external rosbag replay workflows, and real-time benchmarking on embedded hardware.

---

## 25. Approval Checklist

This document assumes the project will be:

- based on **Hiwonder JetAuto Standard Kit / Without Controller**
- powered by **your own NVIDIA Jetson Orin Nano Developer Kit**
- built with the **7-inch LCD display included**
- built with **no microphone array in the selected configuration**
- focused on **indoor AMR perception, mapping, fusion, navigation, and safety**
- built as a **production-style robotics software project**
- implemented as a **C++-first, CUDA-heavy system**
- designed around **one launch entrypoint plus debug/profile/production modes**, with external simulation kept outside the Jetson robot repository
- designed around **debug, profile, and production instrumentation modes**
- built with **RF-DETR as a serious engineering target**
- built with **plug-and-play detector backend support**, including YOLO or other baselines
- organized as a proper ROS2 workspace with packages under `src/`
- tested with **unit tests inside each package** and **system/integration tests outside packages**

If this direction is approved, the next step is to define the detailed interfaces for:

1. sensor topics,
2. TF tree,
3. detector plugin API,
4. detection output schema,
5. fused obstacle schema,
6. safety input/output contract,
7. runtime mode configuration,
8. benchmark metric format,
9. and package-level test requirements.
