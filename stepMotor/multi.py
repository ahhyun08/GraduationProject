from __future__ import print_function
import RPi.GPIO as GPIO
import time
import multiprocessing


import numpy as np
import cv2 as cv
import multiprocessing

# local modules
from video import create_capture
from common import clock, draw_str

# 초음파 센서 거리 측정 함수


def sonicSensor(trigger, echoh):
    GPIO.output(trigger, False)
    time.sleep(0.5)
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    while GPIO.input(echoh) == 0:
        pulse_start = time.time()

    while GPIO.input(echoh) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = round(distance, 2)

    return distance

# 모터 이진 코드 변환 함수


def gpioSig(motorPin, sig):
    if sig == 1:
        GPIO.output(motorPin, GPIO.HIGH)
    else:
        GPIO.output(motorPin, GPIO.LOW)


# 스위치 인터럽트 콜백 함수 두가지

def switchInterrupt(channel):
    global negative
    global active
    if negative == 0:
        negative = 1
        time.sleep(2)
        active = 0
    elif negative == 1:
        negative = 0
        time.sleep(2)
        active = 0
    print("switch pressed!!")


def deactivate(channel):
    global active
    if active == 1:
        active = 0
        print("deactivated!!")
    elif active == 0:
        active = 1
        print("activated!!")


# 모터 작동함수


def motorActive(q):

    def switchInterrupt(channel):
        global negative
        global active
        if negative == 0:
            negative = 1
            time.sleep(2)
            active = 0
        elif negative == 1:
            negative = 0
            time.sleep(2)
            active = 0
        print("switch pressed!!")

    def deactivate(channel):
        global active
        if active == 1:
            active = 0
            print("deactivated!!")
        elif active == 0:
            active = 1
            print("activated!!")

    global active
    global negative
    active = 1
    negative = 1

    # 모터 이진 코드
    mot = [[1, 0, 0, 0],
           [1, 1, 0, 0],
           [0, 1, 0, 0],
           [0, 1, 1, 0],
           [0, 0, 1, 0],
           [0, 0, 1, 1],
           [0, 0, 0, 1],
           [1, 0, 0, 1]]

    # 모터 핀넘버
    out1 = 27
    out2 = 17
    out3 = 22
    out4 = 18

    # 초음파 핀넘버
    trig = 23
    echo = 24

    # 스위치 인터럽트 핀넘버
    limitSwitch = 4
    stopSwitch = 21

    # GPIO핀 설정
    GPIO.setmode(GPIO.BCM)
    # 모터 신호 핀 설정
    GPIO.setup(out1, GPIO.OUT)
    GPIO.setup(out2, GPIO.OUT)
    GPIO.setup(out3, GPIO.OUT)
    GPIO.setup(out4, GPIO.OUT)
    # 초음파센서 트리거, 에코 핀 설정
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    # 두가지 스위치 인터럽트 핀 설정
    GPIO.setup(limitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(stopSwitch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # 인터럽트 신호 감지
    GPIO.add_event_detect(limitSwitch, GPIO.RISING,
                          callback=switchInterrupt, bouncetime=500)
    GPIO.add_event_detect(stopSwitch, GPIO.RISING,
                          callback=deactivate, bouncetime=500)

#    while True:
#        x = sonicSensor(trig, echo)
#        print(x)

#        if x < 60:
#            # activate변수가 1일때만 활성화
#            while True:
#                que = q.get()
#                GPIO.output(out1, GPIO.LOW)
#                GPIO.output(out2, GPIO.LOW)
#                GPIO.output(out3, GPIO.LOW)
#                GPIO.output(out4, GPIO.LOW)
#                # negative 변수가 0일때 내려가고 1일때 올라감
#                if negative == 0:
#                    for i in range(7):
#                        gpioSig(out1, mot[i][0])
#                        gpioSig(out2, mot[i][1])
#                        gpioSig(out3, mot[i][2])
#                        gpioSig(out4, mot[i][3])
#                        time.sleep(0.002)
#                elif negative == 1:
#                    for i in range(7, 0, -1):
#                        gpioSig(out1, mot[i][0])
#                        gpioSig(out2, mot[i][1])
#                        gpioSig(out3, mot[i][2])
#                        gpioSig(out4, mot[i][3])
#                        time.sleep(0.002)
#                if que == 'center':
#                    print("done")
#                    GPIO.cleanup()
#                    break
#                else:
#                    pass

    while True:
        que = q.get()
        GPIO.output(out1, GPIO.LOW)
        GPIO.output(out2, GPIO.LOW)
        GPIO.output(out3, GPIO.LOW)
        GPIO.output(out4, GPIO.LOW)
        # negative 변수가 0일때 내려가고 1일때 올라감
        if negative == 0:
            for i in range(7):
                gpioSig(out1, mot[i][0])
                gpioSig(out2, mot[i][1])
                gpioSig(out3, mot[i][2])
                gpioSig(out4, mot[i][3])
                time.sleep(0.002)
        elif negative == 1:
            for i in range(7, 0, -1):
                gpioSig(out1, mot[i][0])
                gpioSig(out2, mot[i][1])
                gpioSig(out3, mot[i][2])
                gpioSig(out4, mot[i][3])
                time.sleep(0.002)
        if que == 'center':
            print("done")
            GPIO.cleanup()
            break
        else:
            pass


# 얼굴인식


def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.05, minNeighbors=4, minSize=(30, 30),
                                     flags=cv.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:, 2:] += rects[:, :2]
    return rects


def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv.rectangle(img, (x1, y1), (x2, y2), color, 2)


def main(q):
    import sys
    import getopt

    args, video_src = getopt.getopt(
        sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    try:
        video_src = video_src[0]
    except:
        video_src = 0
    args = dict(args)
    cascade_fn = args.get(
        '--cascade', "/home/pi/opencv/data/haarcascades/haarcascade_frontalface_alt.xml")
    nested_fn = args.get('--nested-cascade',
                         "/home/pi/opencv/data/haarcascades/haarcascade_eye.xml")

    cascade = cv.CascadeClassifier(cv.samples.findFile(cascade_fn))
    nested = cv.CascadeClassifier(cv.samples.findFile(nested_fn))

    cam = create_capture(video_src, fallback='synth:bg={}:noise=0.05'.format(
        cv.samples.findFile('/home/pi/opencv/samples/data/lena.jpg')))

    while True:
        _ret, img = cam.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = cv.equalizeHist(gray)

        t = clock()
        rects = detect(gray, cascade)
        vis = img.copy()
        draw_rects(vis, rects, (0, 255, 0))

        if not nested.empty():
            for x1, y1, x2, y2 in rects:
                roi = gray[y1:y2, x1:x2]
                vis_roi = vis[y1:y2, x1:x2]
                subrects = detect(roi.copy(), nested)
                draw_rects(vis_roi, subrects, (255, 0, 0))
        dt = clock() - t

        draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
        cv.imshow('facedetect', vis)

        if type(rects) != list:
            if (rects[0, 1] + (rects[0, 3]-rects[0, 1])/2) in range(210, 270):
                q.put('center')
                break

        if cv.waitKey(5) == 27:
            break

    print('Done')


# 본 코드 시작
if __name__ == '__main__':

    try:

        q = multiprocessing.Queue()

        p1 = multiprocessing.Process(target=main, args=(q,))
        p2 = multiprocessing.Process(target=motorActive, args=(q,))

        p1.start()
        p2.start()

        p1.join()
        p2.join()

        print("done")

    except KeyboardInterrupt:
        GPIO.remove_event_detect(4)
        GPIO.remove_event_detect(21)
        p1.terminate
        p2.terminate
        GPIO.cleanup()
        print("bye")
