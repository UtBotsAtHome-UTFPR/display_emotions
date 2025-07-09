import cv2
import numpy as np
import os

def resize_and_pad_lr(image):
    target_width = 1024
    target_height = 600

    resized_width = 800
    resized = cv2.resize(image, (resized_width, target_height), interpolation=cv2.INTER_AREA)

    left_pad = (target_width - resized_width) // 2  # e.g. 112
    right_pad = target_width - resized_width - left_pad  # e.g. 112

    output = np.zeros((target_height, target_width, 3), dtype=resized.dtype)

    # Place resized image in the center
    output[:, left_pad:left_pad+resized_width] = resized

    # Fill left padding with leftmost column repeated
    left_column = resized[:, 0:1, :]
    for i in range(left_pad):
        output[:, i:i+1, :] = left_column

    # Fill right padding with rightmost column repeated
    right_column = resized[:, -1:, :]
    for i in range(right_pad):
        output[:, left_pad + resized_width + i:left_pad + resized_width + i + 1] = right_column

    return output

def process_folder(input_folder, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in os.listdir(input_folder):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
            input_path = os.path.join(input_folder, filename)
            img = cv2.imread(input_path)
            if img is None:
                print(f"Failed to load {input_path}")
                continue

            out = resize_and_pad_lr(img)
            output_path = os.path.join(output_folder, filename)
            cv2.imwrite(output_path, out)
            print(f"Processed and saved: {output_path}")

if __name__ == "__main__":
    input_folder = "src/cropped/4_3"
    output_folder = "src/cropped/1024_600"
    process_folder(input_folder, output_folder)
