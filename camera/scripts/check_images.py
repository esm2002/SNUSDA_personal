import pickle
import cv2
import numpy as np

# Pickle 파일 경로
pickle_file_path = '/home/rily/다운로드/traf.pickle'

# Pickle 파일을 읽기 모드로 엽니다
with open(pickle_file_path, 'rb') as file:
    data = pickle.load(file)

# image_data가 리스트 형태인지 확인합니다
if isinstance(data, list):
    # 모든 이미지를 순회하면서 시각화합니다
    for idx, image in enumerate(data):

        if isinstance(image, np.ndarray):
            # OpenCV를 사용하여 이미지를 표시합니다
            cv2.imshow(f'Image {idx + 1}', image)
            cv2.waitKey(100)  # 0.1초 동안 이미지를 표시합니다
            cv2.destroyAllWindows()  # 모든 창을 닫습니다
        else:
            print(f"Index {idx}의 데이터는 이미지 배열이 아닙니다.")
else:
    print("image_data가 리스트가 아닙니다.")

