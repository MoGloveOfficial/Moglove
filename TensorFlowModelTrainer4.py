import tensorflow as tf
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split

# Step 1: Load the CSV file into a pandas DataFrame
data = pd.read_csv('data_norm.csv')

# Step 2: Preprocess the data
features = data.iloc[:, :5].values
labels = data.iloc[:, -1].values
#Dont normalize the data
#features = (features - np.mean(features, axis=0)) / np.std(features, axis=0)

# Step 3: Split the data into training and testing sets
train_features, test_features, train_labels, test_labels = train_test_split(features, labels, test_size=0.2, random_state=42)

# Step 4: Define and train the TensorFlow model
model = tf.keras.Sequential([
    tf.keras.layers.Dense(5, activation=None, input_shape=(5,)),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(1)
])

model.compile(optimizer='adam', loss='mse')
model.fit(train_features, train_labels, epochs=1000, batch_size=16)

# Save the trained model
model.save('pattern_recognition_model')

# Input for testing
input_data = np.array([[0.4, 0.184, 0.56, 0.44, 0.184]]) # = 17

# Make predictions
predictions = model.predict(input_data)
# Calculate the estimated pattern integer and confidence
estimated_pattern = np.round(predictions).astype(int)
confidence = np.abs(predictions - np.round(predictions)).flatten()[0]

# Print the estimated pattern and confidence
print("Estimated Pattern: {}, Confidence: {:.2f}".format(estimated_pattern[0], confidence))
