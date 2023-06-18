import tensorflow as tf
import numpy as np
import serial

# Load the saved model
model = tf.keras.models.load_model('pattern_recognition_model')

# Configure the serial connection
ser = serial.Serial('COM4', baudrate=115200)  # Replace 'COM1' with the appropriate serial port

# Continuously read and process serial input
while True:
    try:
        # Read serial input
        line = ser.readline().decode().strip()

        # Split the input into individual values
        values = line.split(',')

        # Convert values to floats
        features = np.array(values, dtype=np.float32)

        # Preprocess the features
        features = (features - np.mean(features)) / np.std(features)

        # Reshape the features to match the model input shape
        features = np.reshape(features, (1, -1))

        # Make predictions
        predictions = model.predict(features)

        # Calculate the estimated pattern and confidence
        estimated_pattern = int(round(predictions[0][0]))
        confidence = np.abs(predictions - np.round(predictions))[0][0]

        # Print the estimated pattern and confidence
        print("Estimated Pattern: {}, Confidence: {:.2f}".format(estimated_pattern, confidence))

    except KeyboardInterrupt:
        # Break the loop if Ctrl+C is pressed
        break

# Close the serial connection
ser.close()
