import tensorflow as tf
import numpy as np
import serial

model = tf.keras.models.load_model('pattern_recognition_model')

arduino = serial.Serial('COM4', 115200, timeout=10)
while True:
    while True:
        data = arduino.readline().decode('utf-8').rstrip().split(',')
        if len(data) ==5:
            break
    input_data = np.array([data])
    predictions = model.predict(input_data)
    estimated_pattern = np.round(predictions).astype(int)
    confidence = np.abs(predictions - np.round(predictions)).flatten()[0]
    print("Estimated Pattern: {}, Confidence: {:.2f}".format(estimated_pattern[0], confidence))
