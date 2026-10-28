#!/bin/bash
# Prints the LD_LIBRARY_PATH needed by onnxruntime-gpu (pip-installed nvidia cu13/cudnn libs inside tools/yolo/venv).
# Usage (from tools/yolo): LD_LIBRARY_PATH=$(./ldpath.sh) venv/bin/python detect_replay.py --rec ../../recordings/real01
SP=$(cd "$(dirname "$0")" && pwd)/venv/lib/python3.13/site-packages/nvidia
echo "$SP/cu13/lib:$SP/cudnn/lib:"
