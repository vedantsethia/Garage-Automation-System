import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
###################################
Motor1A = 36
Motor1B = 38
Motor1E = 40
Motor2A = 33
Motor2B = 35
Motor2E = 37
R=22
G=18
B=16
###################################
a = 120
b = 90
lm=(int)(3 * a / 8)
rm=(int)(5 * a / 8)
###################################
lower = np.array([0, 0, 66])
upper = np.array([255, 255, 255])
###################################
GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)
left = GPIO.PWM(Motor1E, 100)
right = GPIO.PWM(Motor2E, 100)
###################################
frame = None
cx = (int)(a/2)
cy = (int)(b/2)
speed = 20
d = 0.90
e = 0.90

def stop_complete():
    left.stop()
    right.stop()
    GPIO.output(Motor1E, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.LOW)
def stop():
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.LOW)
    GPIO.output(Motor1E, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.LOW)


def start(pwm, factor):
    left.start(pwm)
    right.start(int(pwm * factor))
    GPIO.output(Motor1E, GPIO.HIGH)
    GPIO.output(Motor2E, GPIO.HIGH)


def forward(pwm=speed):
    start(pwm, d)
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)


def backward(pwm=speed):
    start(pwm, e)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    GPIO.output(Motor2A, GPIO.HIGH)


def rotater(pwm=speed):
    start(pwm, e)
    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    GPIO.output(Motor2A, GPIO.HIGH)


def rotatel(pwm=speed):
    start(pwm, d)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)


def change_speed(pwm=speed):
    left.ChangeDutyCycle(pwm)
    left.ChangeDutyCycle(pwm)


def centroid_frame(frame):
    global cx, cy,count
    blur = cv2.GaussianBlur(frame, (0, 0), 4)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(gray, lower, upper)
    ret, thresh = cv2.threshold(mask, 90, 255, cv2.THRESH_BINARY_INV)
    for i in range(int(a / 4), int(3 * a / 4), int(a / 32)):
        if (thresh[(int)( b / 3)][i] == 255):  # check range value
            count+=1
    print ("count:",count)
    cv2.imshow("thresh", thresh)
    __, contours, __ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    l = len(contours)
    try:
        if (contours is not None):
            max = cv2.contourArea(contours[0]);
            pos = 0
            if l >= 2:
                for i in range(l - 1):
                    next = cv2.contourArea(contours[i + 1])
                    if next > max:
                        max = next;
                        pos = i + 1
            M = cv2.moments(contours[pos])
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
    except(Exception):
        pass
    return cx, cy

def pixl_move(pwm=speed,f=0,l=lm,r=rm):
    global cx,cy
    if (cy > (int)(9 * b / 10) and f==0):
        backward(int(pwm * 0.85))
    elif (cx < l):
        rotatel(int(11))
    elif (cx > r):
        rotater(int(11))
    elif (f==0):
        forward(pwm)

def led_ZI_blink(color,num):
    if color == 'R':
        for i in range(num):
            GPIO.setup(R, GPIO.OUT)
            GPIO.output(R, GPIO.HIGH)  # high >> on,low >> off
            time.sleep(0.5)  # led stays on for .5 sec
            GPIO.output(R, GPIO.LOW)
            time.sleep(1)  # delay of 1 second betwen 2 blinks
    elif color == 'G':
        for i in range(num):
            GPIO.setup(G, GPIO.OUT)
            GPIO.output(G, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(G, GPIO.LOW)
            time.sleep(1)
    else:
        for i in range(num):
            GPIO.setup(B, GPIO.OUT)
            GPIO.output(B, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(B, GPIO.LOW)
            time.sleep(1)

def led_SHED_blink():
    for i in scl:
        if (i == "B"):
            i = 18
        elif (i == "G"):
            i = 13
        else:
        	i = 11
        GPIO.setup(i, GPIO.OUT)
        GPIO.output(i, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(i, GPIO.LOW)
        time.sleep(1)
img1=cv2.imread("./overlay.png",-1)
img2=cv2.imread("./overlay1.png",-1)
cam = PiCamera()
cam.resolution = (a, b)
cam.framerate = 20
raw_cap = PiRGBArray(cam, (a, b))
time.sleep(0.5)
flag=0;zi=0
forward()
time.sleep(0.3)
stop()
runs=0
for frame in cam.capture_continuous(raw_cap, format="bgr", use_video_port=True, splitter_port=2, resize=(a, b)):
    count = 0
    color_image = frame.array
    cx, cy = centroid_frame(color_image)
    print cx, cy
    color_image = cv2.circle(color_image, (cx, cy), 3, (0, 255, 255), -1)
    cv2.imshow("Frames", color_image)
    if flag==1 and t>0.28:
        pixl_move(13,1,(int)(5 * a / 12),(int)(7 * a / 12))
	print("Inside slow motion")
    else:
        pixl_move()
	print("Inside fast")

    if (count>=10 or flag==1):
        if flag==0:
            e1 = cv2.getTickCount()
        flag=1
        e2 = cv2.getTickCount()
	t=(e2 - e1) / cv2.getTickFrequency()
        if t> 0.8 and cx>(int)(5 * a / 12) and cx < (int)(7 * a / 12):
            flag=0
	    zi+=1
            stop()
	    print("Inside")
	    time.sleep(0.5)
            #Keshav fn.
	    if (zi==1):
            	led_ZI_blink("B",2)
	    else:
	    	led_ZI_blink("G",1)
    if (zi==2):
    	stop()	
        break
    k = cv2.waitKey(1) & 0xff
    if (k == 27):
        cam.stop_preview()
        cam.close()
        cv2.destroyAllWindows()
        break
    raw_cap.truncate(0)
stop_complete()
GPIO.cleanup()
cv2.imshow("overlay",img2)
cv2.waitKey(0)
cv2.destroyAllWindows()
