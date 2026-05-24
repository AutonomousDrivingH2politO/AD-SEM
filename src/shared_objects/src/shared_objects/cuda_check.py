import cv2
import numpy as np
print(cv2.getBuildInformation())
# Check if GPU support is available
if not cv2.cuda.getCudaEnabledDeviceCount():
    print("CUDA-enabled GPU not found. Exiting...")
    exit()

# Load an image using OpenCV
image = cv2.imread('image.jpg', cv2.IMREAD_COLOR)

# Check if image is loaded
if image is None:
    print("Error loading image. Exiting...")
    exit()

# Upload the image to the GPU
gpu_image = cv2.cuda_GpuMat()
gpu_image.upload(image)

# Perform a Gaussian blur on the image using the GPU
gpu_blurred_image = cv2.cuda_GaussianBlur(gpu_image, (15, 15), 0)

# Download the result back to the CPU
blurred_image = gpu_blurred_image.download()

# Display the original and blurred images
cv2.imshow('Original Image', image)
cv2.imshow('Blurred Image', blurred_image)

cv2.waitKey(0)
cv2.destroyAllWindows()
