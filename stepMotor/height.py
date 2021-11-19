import RPi.GPIO as GPIO
import time

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

# 모터 작동함수


def motorActive(motorPin1, motorPin2, motorPin3, motorPin4, mot):
    global negative
    global active
    # activate변수가 1일때만 활성화
    while active == 1:
        GPIO.output(motorPin1, GPIO.LOW)
        GPIO.output(motorPin2, GPIO.LOW)
        GPIO.output(motorPin3, GPIO.LOW)
        GPIO.output(motorPin4, GPIO.LOW)
        # negative 변수가 0일때 내려가고 1일때 올라감
        if negative == 0:
            for i in range(7):
                gpioSig(motorPin1, mot[i][0])
                gpioSig(motorPin2, mot[i][1])
                gpioSig(motorPin3, mot[i][2])
                gpioSig(motorPin4, mot[i][3])
                time.sleep(0.002)
        elif negative == 1:
            for i in range(7, 0, -1):
                gpioSig(motorPin1, mot[i][0])
                gpioSig(motorPin2, mot[i][1])
                gpioSig(motorPin3, mot[i][2])
                gpioSig(motorPin4, mot[i][3])
                time.sleep(0.002)


# 스위치 인터럽트 콜백 함수 두가지

def switchInterrupt(channel):
    global negative
    if negative == 0:
        negative = 1
    elif negative == 1:
        negative = 0
    print("reverse switch pressed!!")


def deactivate(channel):
    global active
    if active == 1:
        active = 0
        print("deactivated!!")
    elif active == 0:
        active = 1
        print("activated!!")


# 인터럽트 기능에 쓰이는 전역변수 설정
global active
global negative
active = 1
negative = 1

# 모터 이진 코드
motor = [[1, 0, 0, 0],
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

# 본 코드 시작
try:
    while True:
        x = sonicSensor(trig, echo)
        print(x)
        # 거리가 60cm 미만으로 가까울 때 액추에이터 활성화
        if x < 60:
            motorActive(out1, out2, out3, out4, motor)
            negative = 1
        else:
            pass

except KeyboardInterrupt:
    GPIO.remove_event_detect(4)
    GPIO.remove_event_detect(21)
    GPIO.cleanup()
    print("bye")
