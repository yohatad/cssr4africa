import onnxruntime as ort

# Load the ONNX model
model = ort.InferenceSession('model.onnx')