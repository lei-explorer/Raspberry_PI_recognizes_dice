# -*- coding:utf-8 -*-

import cv2
import numpy as np
# import math
# import RPi.GPIO as GPIO
import time
import cv2.aruco as aruco

video = cv2.VideoCapture(0)

def pring_img_massgae():
    global video
    fps = video.get(cv2.CAP_PROP_FPS)
    print(fps)
    size = (int(video.get(cv2.CAP_PROP_FRAME_WIDTH)), int(video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(size)

def get_img():
    global video
    ret, frame = video.read()
    return frame


def show_img(frame):
    cv2.imshow("A video", frame)
    c = cv2.waitKey(30)


def release_camera():
    global video
    video.release()
    cv2.destroyAllWindows()


def img_preprocess(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # blur_image = cv2.medianBlur(gray_image, 5)  # 中值滤波
    # (_, threshold_image) = cv2.threshold(blur_image, 0, 255, cv2.THRESH_OTSU)  # OTSU二值化
    # erode_image = cv2.erode(threshold_image, None, iterations=2)  # 图像腐蚀
    return gray_image


def det_ID(frame, gray):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return ids
    # aruco.drawDetectedMarkers(frame, corners, ids)
    # cv2.imshow("frame", frame)
    # cv2.imwrite("ID.jpg", frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def check_QR_code_exists(original_img, img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hierarchy = hierarchy[0]
    vin = []
    for i in range(0, len(contours)):
        if hierarchy[i][2] == -1:
            continue
        else:
            tempindex1 = hierarchy[i][2]
        if hierarchy[tempindex1][2] == -1:
            continue
        else:
            tempindex2 = hierarchy[tempindex1][2]
            vin.append((i, tempindex1, tempindex2))
    for v in vin:
        contour1 = contours[v[1]]
        contour2 = contours[v[2]]
        lenth1 = cv2.arcLength(contour1, True)
        lenth2 = cv2.arcLength(contour2, True)
        # 排除过大轮廓
        if abs(lenth1 / lenth2 - 2) > 1:
            vin.remove(v)
    # 位置探测图形中心点坐标
    points = []
    for v in vin:
        m = cv2.moments(contours[v[1]])
        cx = np.int0(m['m10'] / m['m00'])
        cy = np.int0(m['m01'] / m['m00'])
        points.append((cx, cy))
    if len(points) < 3:
        return False
    else:
        for i in range(0, 3):
            cv2.line(original_img, points[i % 3], points[(i + 1) % 3], (0, 0, 255), 5)
        cv2.imshow("OR", original_img)
        cv2.waitKey(0)  # 等有键输入或者1000ms后自动将窗口消除，0表示只用键输入结束窗口
        cv2.destroyAllWindows()  # 关闭所有窗口
        return True


# def led_flash():
#     BCM_pin = 18
#     GPIO.setmode(GPIO.BCM)  # 如果是把BCM换成BOARD，下方则为GPIO.setup（物理编码，XX）
#     GPIO.setup(BCM_pin, GPIO.OUT)  # 这里的18就表示BCM编码
#
#     # 闪1次
#     GPIO.output(BCM_pin, GPIO.HIGH)
#     time.sleep(0.1)
#     GPIO.output(BCM_pin, GPIO.LOW)
#     time.sleep(0.1)
#     GPIO.cleanup()  # 每次退出时都用cleanup设置GPIO引脚为低电平状态


if __name__ == '__main__':
    while True:
        frame = get_img()
        gray = img_preprocess(frame)
        id = det_ID(frame, gray)
        if id == 1 or id == 2 or id ==3:
            print("发现敌方，亮灯")
            # led_flash()
        else:
            print("灭灯")