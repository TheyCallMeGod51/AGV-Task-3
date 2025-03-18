import cv2
import numpy as np

# Load images
img1 = cv2.imread("image1.png")
img2 = cv2.imread("image2.png")

# Convert images to grayscale
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# Initialize ORB detector
orb = cv2.ORB_create()

# Find keypoints and descriptors with ORB
kp1, des1 = orb.detectAndCompute(gray1, None)
kp2, des2 = orb.detectAndCompute(gray2, None)

# Use BFMatcher to find matches between descriptors
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)

# Sort matches by distance (best matches first)
matches = sorted(matches, key=lambda x: x.distance)

# Draw top 10 matches
img_matches = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# Extract location of good matches
pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

# Find the homography matrix using RANSAC
H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)
print("Homography matrix:")
print(H)

# Use the homography matrix to warp img1 to match img2
height, width, _ = img2.shape
img1_aligned = cv2.warpPerspective(img1, H, (width, height))

# Estimate affine transformation
affine_matrix, inliers = cv2.estimateAffine2D(pts1, pts2)
print("Affine transformation matrix:")
print(affine_matrix)

# Use the affine transformation to create a new aligned image
img1_aligned_affine = cv2.warpAffine(img1, affine_matrix, (width, height))
panorama = np.zeros((img2.shape[0], img2.shape[1] + img1.shape[1], 3), dtype=np.uint8)
panorama[:, :img2.shape[1]] = img2
panorama[:img1_aligned_affine.shape[0], img2.shape[1]:img2.shape[1]+img1_aligned_affine.shape[1]] = img1_aligned_affine
def blend_images(img1, img2):
    mask = np.where(img1 != 0, 1, 0).astype(np.float32)
    blended = img1 * mask + img2 * (1 - mask)
    return blended.astype(np.uint8)
merged_map = cv2.warpAffine(img1, affine_matrix, (width, height))
merged_map = cv2.addWeighted(img2, 0.5, merged_map, 0.5, 0)

stitched_image = blend_images(panorama[:, :img2.shape[1]], panorama[:, img2.shape[1]:])
# Save the output images
cv2.imwrite('aligned_image.jpg', img1_aligned)
cv2.imwrite('aligned_newimage.jpg', img1_aligned_affine)
cv2.imwrite('matches.jpg', img_matches)
cv2.imwrite('panoram.jpg',merged_map)