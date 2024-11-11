import torch
from ultralytics import YOLO

# Load the Ultralytics YOLOv8 model
model = YOLO("/home/roboticslab/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/models/yolov8s.pt").model

# Define dummy input based on your input size
dummy_input = torch.randn(1, 3, 480, 640)  # Match imgsz dimensions

# Export the model to ONNX format
torch.onnx.export(
    model,
    dummy_input,
    "yolov8s.onnx",           # Output ONNX file path
    export_params=True,        # Store trained parameter weights
    opset_version=11,          # Specify ONNX version
    do_constant_folding=True,  # Optimize constant expressions
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
)
print("Model successfully exported to ONNX format.")
