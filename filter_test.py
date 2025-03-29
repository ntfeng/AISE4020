import cv2
import numpy as np
import matplotlib.pyplot as plt

image_path = "maps/scan1_livingroom.png"
# image_path = "maps/room1.jpg"
original_gray = cv2.imread(image_path, 0)
blurred_gray = cv2.GaussianBlur(original_gray, (3, 3), 0)
blurred_gray = cv2.adaptiveThreshold(blurred_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

struct_elem = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
# struct_elem = np.ones((3,3))
# struct_elem = np.array([[1, 0, 1], [0, 1, 0], [1, 0, 1]], dtype=np.uint8)

temp = blurred_gray.copy()
skeleton = np.zeros(blurred_gray.shape, np.uint8)
iterations_results = [] 
num_of_iters = 3

for i in range(num_of_iters):
    eroded_img = cv2.erode(temp, struct_elem)
    dilated_img = cv2.dilate(eroded_img, struct_elem)
    cmp_img = cv2.subtract(temp, dilated_img)
    skeleton = cv2.bitwise_or(skeleton, cmp_img)

    # Store the result of each step for plotting
    iterations_results.append((temp.copy(), eroded_img, dilated_img, skeleton.copy()))
    temp = eroded_img.copy()

fig, axes = plt.subplots(num_of_iters, 4, figsize=(16, 4 * num_of_iters))
fig.suptitle(f"Opening With + Filter with Iterations = {num_of_iters}", fontsize=12)
fig.subplots_adjust(top=0.8)
font_s = 8
# Iterate over the results and plot
for i in range(num_of_iters):
    ax1, ax2, ax3, ax4 = axes[i]  # Each row has 4 subplots
    ax1.imshow(iterations_results[i][0], cmap='gray')
    ax1.set_title(f"Iteration {i+1} - temp", fontsize=font_s)
    ax1.axis("off")

    ax2.imshow(iterations_results[i][1], cmap='gray')
    ax2.set_title(f"Iteration {i+1} - Erosion", fontsize=font_s)
    ax2.axis("off")

    ax3.imshow(iterations_results[i][2], cmap='gray')
    ax3.set_title(f"Iteration {i+1} - Dilation", fontsize=font_s)
    ax3.axis("off")

    ax4.imshow(iterations_results[i][3], cmap='gray')
    ax4.set_title(f"Iteration {i+1} - Skeleton", fontsize=font_s)
    ax4.axis("off")

plt.tight_layout(pad=3)
plt.show()
