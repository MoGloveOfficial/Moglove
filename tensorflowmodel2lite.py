import tensorflow as tf

# Path to the SavedModel directory
saved_model_dir = 'C:\\Users\\PC-kun\\Desktop\\MoGlove ML\\pattern_recognition_model'

# Path for the TensorFlow Lite model
tflite_model_path = 'C:\\Users\\PC-kun\\Desktop\\MoGlove ML\\MoGloveModel.tflite'

# Load the SavedModel
loaded_model = tf.saved_model.load(saved_model_dir)

# Convert the model to TensorFlow Lite
converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_dir)
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # Apply optimization
tflite_model = converter.convert()

# Save the TensorFlow Lite model to disk
with tf.io.gfile.GFile(tflite_model_path, 'wb') as f:
    f.write(tflite_model)
