import tensorflow as tf
import numpy as np

# Load the saved model
model = tf.keras.models.load_model('pattern_recognition_model')

# Preprocess your input
input_data = np.array([[0.74 ,0.92 ,0.87 ,0.88 ,0.90]])  # Replace with your own input values

#input_data = (input_data - np.mean(input_data, axis=0)) / np.std(input_data, axis=1)

# Make predictions on the input data
predictions = model.predict(input_data)

# Round the predictions to the nearest integer
estimated_patterns = np.round(predictions).astype(int).flatten()

# Calculate the confidence values
confidence = np.abs(predictions - np.round(predictions)).flatten()



# Print the estimated patterns and confidence for each prediction
for i in range(len(predictions)):
    print("Estimated Pattern: {}, Confidence: {:.2f}".format(estimated_patterns[i], confidence[i]))
