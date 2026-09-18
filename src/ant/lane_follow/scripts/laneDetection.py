# uncompyle6 version 3.8.0
# Python bytecode 2.7 (62211)
# Decompiled from: Python 3.6.9 (default, Dec  8 2021, 21:08:43) 
# [GCC 8.4.0]
# Embedded file name: /home/huazai/work/catkin-lane/src/lane_follow/scripts/laneDetection.py
# Compiled at: 2021-11-06 23:43:06
import cv2, rospy, numpy as np, os
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors
import datetime, math

from dynamic_reconfigure.server import Server
from lane_follow.cfg import Detect_rangeConfig

high_ = 720
width_ = 1080
polyfit_val = 2
ym_per_pix = 30.0 / high_
xm_per_pix = 3.7 / width_
CWD_PATH = os.getcwd()

hls_val = rospy.get_param('/lane_follow/hls_val', 160)
gray_threshold = rospy.get_param('/lane_follow/gray_threshold', 50)
h_size = rospy.get_param('/lane_follow/h_size', 50)
h_off = rospy.get_param('/lane_follow/h_off', 0)
h_top = rospy.get_param('/lane_follow/h_top', 0)
v_size = rospy.get_param('/lane_follow/v_size', 50)
v_off = rospy.get_param('/lane_follow/v_off', 0)

def readVideo():
    inpImage = cv2.VideoCapture(os.path.join(CWD_PATH, 'video/challenge.mp4'))
    return inpImage


def processImage(inpImage):
    global hls_val,gray_threshold
    hls = cv2.cvtColor(inpImage, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, hls_val, 10])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(inpImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inpImage, inpImage, mask=mask)
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, gray_threshold, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return (
     '', hls_result, gray, thresh, blur, canny)


def get_points(w, h):
    global h_size,h_off,h_top,v_size,v_off
    cols, rows = w, h
    horizontal_size = int(cols / 100 * h_size)
    horizontal_offset = int((cols - horizontal_size) / 2 / 100 * h_off)
    vertical_size = int(rows / 100 * v_size)
    vertical_offset = int(vertical_size / 100 * v_off)
    horizontal_top = int(horizontal_size / 2 / 100 * h_top)
    point1 = [
     int(cols / 2) - int(horizontal_size / 2) + horizontal_offset, rows - vertical_offset]
    point2 = [int(cols / 2) + horizontal_offset - horizontal_top, rows - vertical_size]
    point3 = [int(cols / 2) + horizontal_offset + horizontal_top, rows - vertical_size]
    point4 = [int(cols / 2) + int(horizontal_size / 2) + horizontal_offset, rows - vertical_offset]
    return (point1, point2, point3, point4)


def perspectiveWarp(inpImage):
    img_size = (
     inpImage.shape[1], inpImage.shape[0])
    cols, rows = img_size
    point1, point2, point3, point4 = get_points(cols, rows)
    src = np.float32([point2, point3, point1, point4])
    dst = np.float32([[0, 0], [cols, 0], [0, rows], [cols, rows]])
    matrix = cv2.getPerspectiveTransform(src, dst)
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)
    height, width = birdseye.shape[:2]
    birdseyeLeft = birdseye[0:height, 0:width // 2]
    birdseyeRight = birdseye[0:height, width // 2:width]
    return (
     birdseye, birdseyeLeft, birdseyeRight, minv)


def plotHistogram(inpImage):
    histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis=0)
    midpoint = np.int(histogram.shape[0] / 2)
    leftxBase = np.argmax(histogram[:midpoint])
    rightxBase = np.argmax(histogram[midpoint:]) + midpoint
    plt.xlabel('Image X Coordinates')
    plt.ylabel('Number of White Pixels')
    return (
     histogram, leftxBase, rightxBase)


uni_direction = -1

def slide_window_search(binary_warped, histogram):
    global uni_direction
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    nwindows = 9
    window_height = np.int(binary_warped.shape[0] / nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (
         win_xleft_high, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (
         win_xright_high, win_y_high), (0, 255, 0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    if not len(leftx) and not len(rightx):
        uni_direction = -1
    else:
        if not len(leftx):
            uni_direction = 0
        elif not len(rightx):
            uni_direction = 1
        if len(left_lane_inds) == len(right_lane_inds):
            uni_direction = 0
    left_fit = np.polyfit(lefty, leftx, polyfit_val)
    right_fit = np.polyfit(righty, rightx, polyfit_val)
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    out_img[(nonzeroy[left_lane_inds], nonzerox[left_lane_inds])] = [
     255, 0, 0]
    out_img[(nonzeroy[right_lane_inds], nonzerox[right_lane_inds])] = [0, 0, 255]
    return (
     ploty, left_fit, right_fit, ltx, rtx)


def general_search(binary_warped, left_fit, right_fit):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = (nonzerox > left_fit[0] * nonzeroy ** 2 + left_fit[1] * nonzeroy + left_fit[2] - margin) & (nonzerox < left_fit[0] * nonzeroy ** 2 + left_fit[1] * nonzeroy + left_fit[2] + margin)
    right_lane_inds = (nonzerox > right_fit[0] * nonzeroy ** 2 + right_fit[1] * nonzeroy + right_fit[2] - margin) & (nonzerox < right_fit[0] * nonzeroy ** 2 + right_fit[1] * nonzeroy + right_fit[2] + margin)
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    left_fit = np.polyfit(lefty, leftx, polyfit_val)
    right_fit = np.polyfit(righty, rightx, polyfit_val)
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    window_img = np.zeros_like(out_img)
    out_img[(nonzeroy[left_lane_inds], nonzerox[left_lane_inds])] = [255, 0, 0]
    out_img[(nonzeroy[right_lane_inds], nonzerox[right_lane_inds])] = [0, 0, 255]
    left_line_window1 = np.array([
     np.transpose(np.vstack([left_fitx - margin, ploty]))])
    left_line_window2 = np.array([
     np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([
     np.transpose(np.vstack([right_fitx - margin, ploty]))])
    right_line_window2 = np.array([
     np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))
    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty
    return ret


def measure_lane_curvature(ploty, leftx, rightx):
    leftx = leftx[::-1]
    rightx = rightx[::-1]
    y_eval = np.max(ploty)
    left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, polyfit_val)
    right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, polyfit_val)
    left_curverad = (1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5 / np.absolute(2 * left_fit_cr[0])
    right_curverad = (1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5 / np.absolute(2 * right_fit_cr[0])
    if leftx[0] - leftx[(-1)] > 60:
        curve_direction = 'Left Curve'
    elif leftx[(-1)] - leftx[0] > 60:
        curve_direction = 'Right Curve'
    else:
        curve_direction = 'Straight'
    return ((left_curverad + right_curverad) / 2.0, curve_direction)


def draw_lane_lines(original_image, warped_image, Minv, draw_info):
    leftx = draw_info['leftx']
    rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']
    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([
     np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))
    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)
    return (
     pts_mean, result)


def offCenter(meanPts, inpFrame):
    mpts = meanPts[(-1)][(-1)][(-2)].astype(int)
    pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    direction = 'left' if deviation < 0 else 'right'
    return (
     deviation, direction)


def addText(img, radius, direction, deviation, devDirection):
    font = cv2.FONT_HERSHEY_TRIPLEX
    if direction != 'Straight':
        text = 'Radius of Curvature: ' + ('{:04.0f}').format(radius) + 'm'
        text1 = 'Curve Direction: ' + direction
    else:
        text = 'Radius of Curvature: ' + 'N/A'
        text1 = 'Curve Direction: ' + direction
    cv2.putText(img, text, (50, 100), font, 0.8, (0, 100, 200), 2, cv2.LINE_AA)
    cv2.putText(img, text1, (50, 150), font, 0.8, (0, 100, 200), 2, cv2.LINE_AA)
    deviation_text = 'Off Center: ' + str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
    cv2.putText(img, deviation_text, (50, 200), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0,
                                                                                100,
                                                                                200), 2, cv2.LINE_AA)
    return img

def reconfigureCB(config, level):

    global hls_val,gray_threshold,h_size,h_off,h_top,v_size,v_off
    hls_val = config.hls_val
    gray_threshold = config.gray_threshold
    h_size = config.h_size
    h_off = config.h_off
    h_top = config.h_top
    v_size = config.v_size
    v_off = config.v_off

    if (config.save_config == True):
        os.system("rosparam set  /lane_follow/save_config false");
        os.system("rosparam dump $(rospack find lane_follow)/params/lane_follow.yaml /lane_follow/");
        # os.system("rosnode kill /lane_follow");
    if(config.recover_default == True):
        os.system("rosparam set  /lane_follow/recover_default false");
        os.system("rosparam load $(rospack find lane_follow)/params/lane_follow_default.yaml /lane_follow/");
        os.system("rosparam dump $(rospack find lane_follow)/params/lane_follow.yaml /lane_follow/");
        # os.system("rosnode kill /lane_follow");
    return config
    
def init_detection():
    srv = Server(Detect_rangeConfig, reconfigureCB)

def loop(frame):
    try:
        birdView, birdViewL, birdViewR, minverse = perspectiveWarp(frame)
        img, hls, grayscale, thresh, blur, canny = processImage(birdView)
        hist, leftBase, rightBase = plotHistogram(thresh)
        ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)
        draw_info = general_search(thresh, left_fit, right_fit)
        curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)
        meanPts, result = draw_lane_lines(frame, thresh, minverse, draw_info)
        deviation, directionDev = offCenter(meanPts, frame)
        finalImg = addText(result, curveRad, curveDir, deviation, directionDev)
    except Exception as e:
        finalImg = frame
        deviation = 10001
        if uni_direction == 0:
            deviation = -1
        elif uni_direction == 1:
            deviation = 1

    cv2.imshow('lane', birdView)
    cv2.imshow('canny', canny)
    cv2.waitKey(1)
    point1, point2, point3, point4 = get_points(finalImg.shape[1], finalImg.shape[0])
    cv2.line(finalImg, tuple(point1), tuple(point2), color=(255, 0, 0), thickness=1)
    cv2.line(finalImg, tuple(point2), tuple(point3), color=(255, 0, 0), thickness=1)
    cv2.line(finalImg, tuple(point3), tuple(point4), color=(255, 0, 0), thickness=1)
    return (finalImg, deviation)
# okay decompiling laneDetection.pyc
