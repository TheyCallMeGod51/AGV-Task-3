import pygame
import cv2
import localization

import numpy as np


def occupancy_grid_to_grayscale(grid):
    # Normalize values
    normalized = np.interp(grid, [0, 1], [255, 0]).astype(np.uint8)
    
    # Create grayscale image
    grayscale = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)

    return grayscale



class MapMerger:
    def merge_maps(self, map1, map2):
        pygame.init()
        
        # Convert to OpenCV-compatible images

        img1=occupancy_grid_to_grayscale(map1)
        img2=occupancy_grid_to_grayscale(map2)


        cv2.imwrite('image1.png',img1)
        cv2.imwrite('image2.png',img2)

        img1 = cv2.imread("image1.png")
        img2 = cv2.imread("image2.png")

        # Initialize ORB detector
        orb = cv2.ORB_create()

        # Find keypoints and descriptors with ORB
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        # Use BFMatcher to find matches between descriptors
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)

        # Sort matches by distance (best matches first)
        matches = sorted(matches, key=lambda x: x.distance)

        # Extract location of good matches
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Find the homography matrix using RANSAC
        H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)
        print(H)
        affine_matrix, inliers = cv2.estimateAffine2D(pts1, pts2)
        print("Affine transformation matrix:")
        print(affine_matrix)
        # Use the homography matrix to warp img1 to match img2
        height, width, _ = img2.shape
        img1_aligned = cv2.warpPerspective(map1, H, (width, height))
        merged_map = cv2.warpAffine(img1, affine_matrix, (width, height))
        merged_map = cv2.addWeighted(img2, 0.5, merged_map, 0.5, 0)
        cv2.imwrite('newaligned_image.jpg',img1_aligned)
        
       
        return (merged_map,affine_matrix)

