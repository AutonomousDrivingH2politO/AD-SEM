import cv2
import os
import glob
import re

def natural_sort_key(s):
    """
    Helper function to sort files numerically instead of alphabetically.
    This prevents photo10.jpg from being placed before photo2.jpg.
    """
    return [int(text) if text.isdigit() else text.lower() for text in re.split(r'(\d+)', s)]

def images_to_video(image_folder, output_video, fps=5):
    """
    Converts a folder of images into a video.
    """
    # 1. Grab all image files from the folder
    search_path = os.path.join(image_folder, "*.png")
    images = glob.glob(search_path)

    # 2. Sort the images using the natural sorting key
    images.sort(key=natural_sort_key)

    if not images:
        print("Error: No images found in the specified directory.")
        return

    # 3. Read the first image to determine the video dimensions
    first_frame = cv2.imread(images[0])
    if first_frame is None:
        print("Error: Could not read the first image. Check file permissions or corruption.")
        return

    height, width, layers = first_frame.shape
    frame_size = (width, height)

    # 4. Initialize the Video Writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_video, fourcc, fps, frame_size)

    print(f"Found {len(images)} images. Generating video at {fps} frames per second...")

    # 5. Loop through the images and write them to the video file
    for image_path in images:
        frame = cv2.imread(image_path)

        # Skip unreadable frames to prevent crashes
        if frame is None:
            continue

        # Resize image if it doesn't match the first frame's dimensions
        if frame.shape[:2] != (height, width):
            frame = cv2.resize(frame, frame_size)
 
        video_writer.write(frame)

    # 6. Save and close the file
    video_writer.release()
    print(f"Success! Video saved as: {output_video}")

# ==========================================
# Configuration and Execution
# ==========================================

image_directory = '/home/bylogix/Desktop/obs_Av_wbLACO'

# 2. Name your output file
output_filename = 'obstacle_avoidance_wBlaco3.mp4'

# 3. Set your Frames Per Second (FPS)
# Set to 3 for a smooth, slower progression  
frames_per_second = 10

images_to_video(image_directory, output_filename, frames_per_second)