from ROAR.utilities_module.utilities import dist_to_line_2d
from abc import abstractmethod
from collections import deque
import logging
from typing import Any
from ROAR.agent_module.agent import Agent
from ROAR.perception_module.detector import Detector

import cv2
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from PIL import Image
import os

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)
            
    return img

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

def weighted_img(img, initial_img, α=0.8, β=1., γ=0.):
    return cv2.addWeighted(initial_img, α, img, β, γ)

def read_img(img):
    return mpimg.imread(img)

def to_hls(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

def to_hsv(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

def isolate_color_mask(img, low_thresh, high_thresh):
    assert(low_thresh.all() >=0  and low_thresh.all() <=255)
    assert(high_thresh.all() >=0 and high_thresh.all() <=255)
    return cv2.inRange(img, low_thresh, high_thresh)

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
 
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def save_imgs(img_list, labels, prefix="Test", op_folder="test_imgs_output"):
    if not os.path.exists(op_folder):
        os.mkdir(op_folder)
    for img, label in zip(img_list, labels):
        PATH = op_folder + "/" + prefix + "_" + label
        Image.fromarray(img).save(PATH)

def display_imgs(img_list, labels=[],cols=2, fig_size=(15,15)):
    if len(labels) > 0:
        assert(len(img_list) == len(labels))
    assert(len(img_list) > 0)
    cmap = None
    tot = len(img_list)
    rows = tot / cols
    plt.figure(figsize=fig_size)
    for i in range(tot):
        plt.subplot(rows, cols, i+1)
        if len(img_list[i].shape) == 2:
            cmap = 'gray'
        if len(labels) > 0:
            plt.title(labels[i])
        plt.imshow(img_list[i], cmap=cmap)
        
    plt.tight_layout()
    plt.show()

def get_aoi(img):
    rows, cols = img.shape[:2]
    mask = np.zeros_like(img)
    
    left_bottom = [cols * -0.1, rows]
    right_bottom = [cols * 1.1, rows]
    left_top = [cols * 0.4, rows * 0.6]
    right_top = [cols * 0.6, rows * 0.6]
    
    vertices = np.array([[left_bottom, left_top, right_top, right_bottom]], dtype=np.int32)
    
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255, ) * mask.shape[2])
    return cv2.bitwise_and(img, mask)

def get_hough_lines(img, rho=1, theta=np.pi/180, threshold=20, min_line_len=20, max_line_gap=300):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), 
                            minLineLength=min_line_len, maxLineGap=max_line_gap)
    return lines

def get_line_length(line):
    for x1, y1, x2, y2 in line:
        return np.sqrt((y2-y1)**2 + (x2-x1)**2)

def get_line_slope_intercept(line):
    for x1, y1, x2, y2 in line:
        if x2-x1 == 0:
            return math.inf, 0
    slope = (y2-y1)/(x2-x1)
    intercept = y1 - slope * x1
    return slope, intercept
        
def get_lines_slope_intecept(lines, slope_threshold = 0.3):
    left_lines = []
    right_lines = []
    left_lengths = []
    right_lengths = []
    for line in lines:
        slope, intercept = get_line_slope_intercept(line)
        if slope == math.inf:
            continue
        line_len = get_line_length(line)
        if slope < - slope_threshold:
            left_lines.append((slope, intercept))
            left_lengths.append(line_len)
        elif slope > slope_threshold :
            right_lines.append((slope, intercept))
            right_lengths.append(line_len)
            
    # average
    left_avg = np.dot(left_lengths, left_lines)/np.sum(left_lengths) if len(left_lengths) > 0 else None
    right_avg = np.dot(right_lengths, right_lines)/np.sum(right_lengths) if len(right_lengths) > 0 else None
    
    return left_avg, right_avg

def convert_slope_intercept_to_line(y1, y2 , line, xmax):
    if line is None:
        return None
    
    slope, intercept = line
    x1 = int((y1- intercept)/slope)
    if x1 < 0:
        x1 = 0
        y1 = int(intercept)
    elif x1 > xmax:
        x1 = xmax
        y1 = int(x1 * slope + intercept)
    else:
        y1 = int(y1)
    x2 = int((y2- intercept)/slope)
    if x2 < 0:
        x2 = 0
        y2 = int(intercept)
    elif x2 > xmax:
        x2 = xmax
        y2 = int(x2 * slope + intercept)
    else:
        y2 = int(y2)
    return((x1, y1),(x2, y2))

def get_lane_lines(img, lines):
    left_avg, right_avg = get_lines_slope_intecept(lines)
    
    y1 = img.shape[0] - 1
    y2 = img.shape[0] * 0.6
    
    left_lane = convert_slope_intercept_to_line(y1, y2, left_avg, img.shape[1]-1)
    right_lane = convert_slope_intercept_to_line(y1, y2, right_avg, img.shape[1]-1)

    # Lanes are two close meaning that only one lane is detected
    if left_lane is not None and right_lane is not None and abs(left_lane[0][0]-right_lane[0][0])<img.shape[1]/2:
        return None, None

    return left_lane, right_lane

def draw_weighted_lines(img, lines, color=[255, 0, 0], thickness=2, alpha = 1.0, beta = 0.95, gamma= 0):
    mask_img = np.zeros_like(img)
    for line in lines:
        if line is not None:
            cv2.line(mask_img, *line, color, thickness)            
    return weighted_img(mask_img, img, alpha, beta, gamma)

class LaneDetector(Detector):
    def __init__(self, agent: Agent, mem_size: int = 5, **kwargs):
        super().__init__(agent, **kwargs)
        self.left_lane = None # lastest left lane coordinates in world frame
        self.right_lane = None # lastest right lane coordinates in world frame
        self.lane_center = None
        self.dist_to_lane_center = 0 # distance to lane center, positive when car is on the right side of the lane center
        # self.left_mem = deque(mem_size)
        # self.right_mem = deque(mem_size)

    def run_in_series(self, **kwargs) -> Any:
        rgb_img = self.agent.front_rgb_camera.data
        self.dist_to_lane_right,self.dist_to_lane_left = self.process_image(rgb_img, visualize=False)
        # if self.dist_to_lane_center is None:
        #     self.lane_center = None
        return self.dist_to_lane_right,self.dist_to_lane_left

    def run_in_threaded(self, **kwargs):
        pass

    def process_image(self, image, visualize=False, **kwargs):
        # NOTE: The output you return should be a color image (3 channel) for processing video below
        # TODO: put your pipeline here,
        # you should return the final output (image where lines are drawn on lanes)
        if image is None or len(image.shape) != 3:
            return None,None

        original_img = np.copy(image)
        # cv2.imshow("original img", original_img)

        if visualize:
            original_aoi_img = get_aoi(original_img)
        
        # convert to grayscale
        gray_img = grayscale(image)
        
        # darken the grayscale
        # darkened_img = adjust_gamma(gray_img, 1)
        # cv2.imshow("darkened img", darkened_img)
        
        # Color Selection
        # white_mask = isolate_color_mask(to_hls(image), np.array([0, 0, 0], dtype=np.uint8), np.array([200, 255, 255], dtype=np.uint8))
        # cv2.imshow("white mask", white_mask)
        # yellow_mask = isolate_color_mask(to_hls(image), np.array([10, 0, 100], dtype=np.uint8), np.array([40, 255, 255], dtype=np.uint8))
        # cv2.imshow("yellow mask", yellow_mask)
        # mask = cv2.bitwise_or(white_mask, yellow_mask)
        # cv2.imshow("mask", mask)
        
        # colored_img = cv2.bitwise_and(darkened_img, darkened_img, mask=mask)
        
        # Apply Gaussian Blur
        # blurred_img = gaussian_blur(colored_img, kernel_size=7)
        
        # Apply Canny edge filter
        canny_img = canny(gray_img, low_threshold=70, high_threshold=140)
        # cv2.imshow("canny_img", canny_img)
        
        # Get Area of Interest
        aoi_img = get_aoi(canny_img)
        
        # Apply Hough lines
        hough_lines = get_hough_lines(aoi_img)
        # hough_img = draw_lines(original_img, hough_lines)
        # cv2.imshow("hough_img", hough_img)

        if hough_lines is None:
            return None,None
        
        # Extrapolation and averaging
        left_lane, right_lane = get_lane_lines(original_img, hough_lines)

        if left_lane is None or right_lane is None:
            return None,None

        # self.calculate_world_cords(np.array(left_lane+right_lane).T[::-1])
        # Convert to wold frame
        if left_lane is not None:
            self.left_lane = self.calculate_world_cords(np.array(left_lane).T[::-1])
        if right_lane is not None:
            self.right_lane = self.calculate_world_cords(np.array(right_lane).T[::-1])
        
        self.lane_center = (self.left_lane + self.right_lane) / 2
        #car_center = self.agent.vehicle.transform.get_matrix()@np.r_[0,self.agent.vehicle.wheel_base/2,0,1]
        #dist_to_lane_center = dist_to_line_2d(np.array([self.agent.vehicle.transform.location.x,self.agent.vehicle.transform.location.z]), self.lane_center[0,[0,2]], self.lane_center[1,[0,2]])
        dist_to_lane_right = dist_to_line_2d(np.array([self.agent.vehicle.transform.location.x,self.agent.vehicle.transform.location.z]), self.right_lane[0,[0,2]], self.right_lane[1,[0,2]])
        dist_to_lane_left = dist_to_line_2d(np.array([self.agent.vehicle.transform.location.x,self.agent.vehicle.transform.location.z]), self.left_lane[0,[0,2]], self.left_lane[1,[0,2]])
        #print(dist_to_lane_left,dist_to_lane_right)
        if visualize:
            processed_img = draw_weighted_lines(original_aoi_img, [left_lane, right_lane], thickness= 10)
            if left_lane is not None:
                for (x,y), coor_world in zip(left_lane, self.left_lane):
                    processed_img = cv2.putText(processed_img, str(coor_world), (x-100,y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1, cv2.LINE_AA)
            if right_lane is not None:
                for (x,y), coor_world in zip(right_lane, self.right_lane):
                    processed_img = cv2.putText(processed_img, str(coor_world), (x-100,y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1, cv2.LINE_AA)
            if left_lane is not None and right_lane is not None:
                center_x1 = (left_lane[0][0] + right_lane[0][0]) // 2
                center_x2 = (left_lane[1][0] + right_lane[1][0]) // 2
                center_y1 = (left_lane[0][1] + right_lane[0][1]) // 2
                center_y2 = (left_lane[1][1] + right_lane[1][1]) // 2
                lane_center = (center_x1, center_y1), (center_x2, center_y2)
                processed_img = draw_weighted_lines(processed_img, [lane_center], thickness= 5, color=[0,255,0])
                for (x,y), coor_world in zip(lane_center, self.lane_center):
                    processed_img = cv2.putText(processed_img, str(coor_world), (x-100,y-40), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1, cv2.LINE_AA)
            processed_img = cv2.putText(processed_img, str(dist_to_lane_center), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            processed_img = cv2.putText(processed_img, str((self.agent.vehicle.transform.location.x,self.agent.vehicle.transform.location.z)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow("processed img", processed_img)

        return dist_to_lane_right,dist_to_lane_left

    def calculate_world_cords(self, coords):
        depth_img = self.agent.front_depth_camera.data
        # cv2.imshow('depth', np.minimum(depth_img,0.01)*100)
        raw_p2d = np.reshape(self._pix2xyz(depth_img=depth_img, i=coords[0], j=coords[1]), (3, np.shape(coords)[1])).T

        cords_y_minus_z_x = np.linalg.inv(self.agent.front_depth_camera.intrinsics_matrix) @ raw_p2d.T
        cords_xyz_1 = np.vstack([
            cords_y_minus_z_x[2, :],
            -cords_y_minus_z_x[1, :],
            cords_y_minus_z_x[0, :],
            np.ones((1, np.shape(cords_y_minus_z_x)[1]))
        ])
        points: np.ndarray = self.agent.vehicle.transform.get_matrix() @ self.agent.front_depth_camera.transform.get_matrix() @ cords_xyz_1
        points = points.T[:, :3]
        return points

    @staticmethod
    def _pix2xyz(depth_img, i, j):
        return [
            depth_img[i, j] * j * 1000,
            depth_img[i, j] * i * 1000,
            depth_img[i, j] * 1000
        ]