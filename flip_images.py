import os
from PIL import Image

# Set the input and output folder paths
input_folder = "./src/cropped/1024_600"
output_folder = "./src/cropped/1024_600_flipped"  # Set to input_folder to overwrite

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Supported image file extensions
image_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff', '.webp')

# Process each file in the folder
for filename in os.listdir(input_folder):
    if filename.lower().endswith(image_extensions):
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename)

        try:
            with Image.open(input_path) as img:
                # Rotate the image by 180 degrees
                rotated_img = img.rotate(180)

                # Save the rotated image
                rotated_img.save(output_path)
                print(f"Rotated and saved: {output_path}")
        except Exception as e:
            print(f"Failed to process {filename}: {e}")
