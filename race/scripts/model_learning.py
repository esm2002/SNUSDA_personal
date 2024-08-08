import os
import numpy as np
import cv2
from sklearn.model_selection import train_test_split
from tensorflow.keras.utils import to_categorical
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout

# 데이터 로드 함수
def load_data(image_dir):
    images = []
    steer_values = []
    for filename in os.listdir(image_dir):
        if filename.endswith('.jpg'):
            img_path = os.path.join(image_dir, filename)
            img = cv2.imread(img_path)
            steer_path = filename.replace('.jpg', '.txt')
            with open(os.path.join(image_dir, steer_path), 'r') as f:
                steer_value = int(f.read())
            
            images.append(img)
            steer_values.append(steer_value)
    
    return np.array(images), np.array(steer_values)

# 데이터 불러오기
image_dir = '/path/to/save/images'  # 여기에 수집한 데이터가 저장된 경로를 입력하세요.
X, y = load_data(image_dir)

# 데이터 분할 (훈련 세트와 검증 세트)
X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)

# 데이터 전처리 (이미지 정규화)
X_train = X_train / 255.0
X_val = X_val / 255.0

# 조향값을 원-핫 인코딩
y_train = to_categorical(y_train, num_classes=3)
y_val = to_categorical(y_val, num_classes=3)

# 모델 설계
model = Sequential([
    Conv2D(32, (3, 3), activation='relu', input_shape=(480, 640, 3)),
    MaxPooling2D((2, 2)),
    Conv2D(64, (3, 3), activation='relu'),
    MaxPooling2D((2, 2)),
    Conv2D(128, (3, 3), activation='relu'),
    MaxPooling2D((2, 2)),
    Flatten(),
    Dense(512, activation='relu'),
    Dropout(0.5),
    Dense(3, activation='softmax')  # 좌회전, 직진, 우회전
])

# 모델 컴파일
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

# 모델 훈련
history = model.fit(
    X_train, y_train,
    epochs=10,
    validation_data=(X_val, y_val)
)

# 모델 평가
loss, accuracy = model.evaluate(X_val, y_val)
print(f'Validation Loss: {loss}')
print(f'Validation Accuracy: {accuracy}')

# 모델 저장
model.save('new_best_model.h5')
