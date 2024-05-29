import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
import numpy as np

# Завантаження даних
train_data = pd.read_csv('train_data(1).csv')
test_data = pd.read_csv('test_data(1).csv')

# Перетворення міток у числові значення
train_data['Label'] = train_data['Label'].map({'pothole': 0, 'smooth': 1})
test_data['Label'] = test_data['Label'].map({'pothole': 0, 'smooth': 1})

# Розділення даних на ознаки і мітки
X_train = train_data[['X', 'Y', 'Z']]
y_train = train_data['Label']
X_test = test_data[['X', 'Y', 'Z']]
y_test = test_data['Label']

# Створення моделі
model = tf.keras.models.Sequential([
    tf.keras.layers.Input(shape=(3,)),
    tf.keras.layers.Dense(16, activation='relu'),
    tf.keras.layers.Dense(16, activation='relu'),
    tf.keras.layers.Dense(1, activation='sigmoid')
])

# Компіляція моделі
model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# Тренування моделі
model.fit(X_train, y_train, epochs=100, batch_size=32, validation_split=0.2)

# Оцінка моделі
loss, accuracy = model.evaluate(X_test, y_test)
print(f'Accuracy: {accuracy * 100:.2f}%')

# Збереження моделі
model.save('pothole_detection_model.keras')
