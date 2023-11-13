import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import csv
import numpy as np
from tensorflow.keras.optimizers import Adam

# Data processing
filename_input = './data/planar-gripper-dynamic-cage-dataset/data_points_O.csv'
filename_label = './data/planar-gripper-dynamic-cage-dataset/ao_rrt.csv'

input = [] # 3542
with open(filename_input, mode='r', newline='') as file:
    reader = csv.reader(file)
    headers = next(reader)
    for row in reader:
        input.append([float(r) for r in row])

label = []
with open(filename_label, mode='r', newline='') as file:
    reader = csv.reader(file)
    headers = next(reader)
    for row in reader:
        label.append([float(r) for r in row])

noninf_idx = []
for i,l in enumerate(label):
    if l[4] != float('inf'): noninf_idx.append(i)

inputs = [input[i][1:] for i in noninf_idx] # 2802*10
labels = [label[i][-1] for i in noninf_idx] # 2802
inputs = np.asarray(inputs)
labels = np.asarray(labels)

# Assuming X is your input data and y is your output data
X_train, X_test, y_train, y_test = train_test_split(inputs, labels, test_size=0.3, random_state=42)

# Normalize the data
scaler = StandardScaler()
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)

# Build the model
hidden_layer_neurals = 32
model = tf.keras.models.Sequential([
    tf.keras.layers.Dense(hidden_layer_neurals, activation='relu', input_shape=(10,)),
    tf.keras.layers.Dense(hidden_layer_neurals, activation='relu'),
    tf.keras.layers.Dense(hidden_layer_neurals, activation='relu'),
    tf.keras.layers.Dense(hidden_layer_neurals, activation='relu'),
    tf.keras.layers.Dense(1, activation='linear')
])

# Compile the model
# Define your desired learning rate
learning_rate = 5e-4
adam_optimizer = Adam(learning_rate=learning_rate)
model.compile(optimizer=adam_optimizer, loss='mean_squared_error')

# # Train the model
model.fit(X_train, y_train, validation_split=0.15, epochs=200, batch_size=32, verbose=2)

# Evaluate the model
test_loss = model.evaluate(X_test, y_test)

# Generate predictions
y_pred = model.predict(X_test)

# Flatten y_pred to ensure it's in the same shape as y_test
y_pred = y_pred.flatten()

# Print true and predicted values
for true, pred in zip(y_test, y_pred):
    print(f"True: {true}, Predicted: {pred}")