# -*- coding:utf-8 -*-

import cv2
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
    return gray_image


def det_ID(gray):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return ids

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
        id = det_ID(gray)
        if id == 1 or id == 2 or id ==3:
            print("发现敌方，亮灯")
            # led_flash()
        else:
            print("灭灯")