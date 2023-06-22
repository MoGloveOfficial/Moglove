import tensorflow as tf
import numpy as np

# Path to the TFLite model
model_path = 'C:\\Users\\PC-kun\\Desktop\\MoGlove ML\\MoGloveModel.tflite'

# Load the TFLite model
interpreter = tf.lite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Create input tensor
input_data = np.array([[1.0, 2.0, 3.0, 4.0, 5.0]], dtype=np.float32)

# Set input tensor
interpreter.set_tensor(input_details[0]['index'], input_data)

# Run the inference
interpreter.invoke()

# Get the output tensor
output_data = interpreter.get_tensor(output_details[0]['index'])

# Print the output
print('Output:', output_data)

