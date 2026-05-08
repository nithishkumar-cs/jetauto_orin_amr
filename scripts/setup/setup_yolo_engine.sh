#!/usr/bin/env bash
set -euo pipefail

MODEL_DIR="${MODEL_DIR:-/opt/jetauto_orin_amr/models/yolo}"
MODEL_NAME="${MODEL_NAME:-yolov8n}"
IMG_SIZE="${IMG_SIZE:-640}"
YOLO_URL="${YOLO_URL:-https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.pt}"

PT_PATH="${MODEL_DIR}/${MODEL_NAME}.pt"
ONNX_PATH="${MODEL_DIR}/${MODEL_NAME}.onnx"
ENGINE_PATH="${MODEL_DIR}/${MODEL_NAME}.engine"
LABELS_PATH="${MODEL_DIR}/coco.labels"

mkdir -p "${MODEL_DIR}"

if [ ! -w "${MODEL_DIR}" ]; then
  sudo mkdir -p "${MODEL_DIR}"
  sudo chown -R "${USER}:${USER}" "${MODEL_DIR}"
fi

if [ ! -f "${PT_PATH}" ]; then
  echo "Downloading ${YOLO_URL}"
  curl -L "${YOLO_URL}" -o "${PT_PATH}"
else
  echo "Using existing ${PT_PATH}"
fi

cat > "${LABELS_PATH}" <<'LABELS'
person
bicycle
car
motorcycle
airplane
bus
train
truck
boat
traffic light
fire hydrant
stop sign
parking meter
bench
bird
cat
dog
horse
sheep
cow
elephant
bear
zebra
giraffe
backpack
umbrella
handbag
tie
suitcase
frisbee
skis
snowboard
sports ball
kite
baseball bat
baseball glove
skateboard
surfboard
tennis racket
bottle
wine glass
cup
fork
knife
spoon
bowl
banana
apple
sandwich
orange
broccoli
carrot
hot dog
pizza
donut
cake
chair
couch
potted plant
bed
dining table
toilet
tv
laptop
mouse
remote
keyboard
cell phone
microwave
oven
toaster
sink
refrigerator
book
clock
vase
scissors
teddy bear
hair drier
toothbrush
LABELS

if [ ! -f "${ONNX_PATH}" ]; then
  if python3 - <<'PY'
import importlib.util
raise SystemExit(0 if importlib.util.find_spec("ultralytics") and importlib.util.find_spec("torch") else 1)
PY
  then
    python3 - <<PY
from pathlib import Path
from ultralytics import YOLO

model = YOLO("${PT_PATH}")
exported = model.export(format="onnx", imgsz=${IMG_SIZE}, opset=12, simplify=False, dynamic=False)
exported = Path(exported)
target = Path("${ONNX_PATH}")
if exported.resolve() != target.resolve():
    target.write_bytes(exported.read_bytes())
print(f"Exported {target}")
PY
  else
    cat <<MSG
${PT_PATH} was downloaded, but ONNX export needs Python torch + ultralytics.

Install an NVIDIA Jetson-compatible PyTorch build first, then run:

  python3 -m pip install --user ultralytics onnx
  ${0}

MSG
    exit 2
  fi
else
  echo "Using existing ${ONNX_PATH}"
fi

TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
if [ ! -x "${TRTEXEC}" ]; then
  echo "TensorRT trtexec not found at ${TRTEXEC}" >&2
  exit 3
fi

"${TRTEXEC}" \
  --onnx="${ONNX_PATH}" \
  --saveEngine="${ENGINE_PATH}" \
  --fp16

echo "YOLO TensorRT engine ready: ${ENGINE_PATH}"
echo "Labels ready: ${LABELS_PATH}"
