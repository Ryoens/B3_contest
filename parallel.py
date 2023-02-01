#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# line_follower.py -- PID line follower
# carp-lover.py -- search and chase a red object
# Released under the MIT License
# Copyright (c) 2017 Hiroshima City University
#
import time
import math
import r3pi
import cv2
import numpy as np
import sys
import RPi.GPIO as GPIO
import threading
import multiprocessing
import os

################################

print('Python {0}.{1}.{2}'.format(sys.version_info.major, sys.version_info.minor, sys.version_info.micro))
print('OpenCV {0}'.format(cv2.__version__))

################################

# Global parameters
MJPGSTREAMER = False # True # False if you use webcam
GUI = True # False if you use CUI
MJPGURI = "http://localhost:8081/?action=stream"
# Image Size
# 160x120
#WIDTH = 160
#HEIGHT = 120
# 320x240
WIDTH = 320
HEIGHT = 240
# 640x480
#WIDTH = 640
#HEIGHT = 480
BARSIZE = 60
RED   = (0,0,255) # in BGR, rather than RGB
GREEN = (0,255,0) # in BGR, rather than RGB
BLUE  = (255,0,0) # in BGR, rather than RGB
LOWER_BLACK= np.array([0,0,0]) # in HSV
UPPER_BLACK= np.array([180,255,50])
LOWER_WHITE= np.array([0,0,180]) # in HSV
UPPER_WHITE= np.array([180,255,255])

LOWER_RED = np.array([0,70,50]) # HSV
UPPER_RED = np.array([10,255,255]) # HSV
# black line detection window
LY = HEIGHT-int(HEIGHT/10)
LX = 0
LH = int(HEIGHT/10)
LW = int(WIDTH/3)
# left white line detection window
#LH=int(HEIGHT*0.?)
#LW=int(WIDTH*0.?)        
#LX=int(WIDTH*0.?)
#LY=int(HEIGHT-LH)
WH = int(HEIGHT/5)
WW = int(WIDTH/5)
WY = int(HEIGHT*1/3)
WX = int(WIDTH/2 - WW/2)

XH = int(HEIGHT/5)
XW = int(WIDTH/5)
XY = int(HEIGHT*2/3)
XX = int(WIDTH/2 - XW/2)

#KERNEL = np.ones((5, 5), np.uint8) # 320x240,640x480
#KERNEL = np.ones((3, 3), np.uint8) # 160x120
KSIZE=int(WIDTH/53) # WIDTH=320->KSIZE=6,WIDTH=160->KSIZE=3
KERNEL = np.ones((KSIZE, KSIZE), np.uint8)

# not use
FF = 0.0 # Forgetting factor

#MAX = 0.3
#MAX = 0.4
MAX = 0.5
MIN = 0.0
#P_TERM = 0.2
#I_TERM = 0.0
#D_TERM = 1.0
P_TERM = 0.1
I_TERM = 0.0
D_TERM = 0.5
right = 0.0
left = 0.0
cur_pos = 0.0
prev_pos = 0.0
deriv = 0.0
prop = 0.0
intg = 0.0
power = 0.0
speed = 0.2
#speed = 0.3
#speed = 0.35
#speed = 0.4 # OK

#固体差で車体ごとに、多少はカメラの中心が違うので、調整する
CENTER=0.5

# Relative target direction, which ranges in [0,1]
gcx = CENTER

cap = None
frame = None
frame0 = None

# Utility functions
def getMomentX(c):
    m = cv2.moments(c)
    x = float(m['m10'] / m['m00'])
    return x

def printBar(value):
    i = int(value * BARSIZE + 0.5)
    bar = "|" + "-" * i + "*" + "-" * (BARSIZE-i) + "|"
    print(bar)

#フレーム数を指定し、カメラの読み出し＆表示をしながら待つ 30frame=1sec.
def cap_sleep(n):
    for i in range(n):
        _, frame0 = cap.read()
        cv2.imshow('frame0', frame0)
        k = cv2.waitKey(1) & 0xFF

# Setup r3pi
lock = threading.Lock()
r3pi.init()
r3pi.cls()
r3pi.play("O5 T120 L16 ceg>crg>c8")
################

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("白いボタンを押すとスタートします\n（オレンジ色のシャットダウンボタンの右にあります）")
while (GPIO.input(12) == 1): # 押されるまで待機
    time.sleep(0.1)
r3pi.play("c")
while (GPIO.input(12) == 0): # 離されるまで待機（チャタリング対策） 
    time.sleep(0.1)
r3pi.play("g")
print("白いボタンが押されました")

time.sleep(1.0)

################
if MJPGSTREAMER == False:
    # Use webcam
    cap = cv2.VideoCapture(0)
    cap.set(3, WIDTH)
    cap.set(4, HEIGHT)
else:
    cap = cv2.VideoCapture(MJPGURI)

# dummy read 30frames=1sec. for OpenCV3
cap_sleep(30)

f = 0
start = time1 = time.time()

def motor_play():
    global lock
    global r3pi
    global gcx
    global prev_pos
    global intg
    global f
    global time1
    line = True
    roll = False

    while(1):
        lock.acquire()
        # Take a frame
        _, frame0 = cap.read()
        
        # trim low part only
        frame = frame0[LY:LY+LH, LX:LX+LW]
        frame_sub = frame0[WY:WY+WH+1, WX:WX+WW-1]
        frame_rec = frame0[XY:XY+XH+1, XX:XX+XW-1]

        # # Convert BGR to HSV
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame_sub, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(frame_rec, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray1 = cv2.cvtColor(frame_sub, cv2.COLOR_BGR2GRAY)
        #gray2 = cv2.cvtColor(frame_rec, cv2.COLOR_BGR2GRAY)

        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 41, -15)
        #mask1 = cv2.adaptiveThreshold(gray1, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 41, -15)
        #mask2 = cv2.adaptiveThreshold(gray2, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 41, -15)

        # # Threshold the HSV image to only detect black colors
        # mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
        mask1 = cv2.inRange(hsv1, LOWER_RED, UPPER_RED)
        mask2 = cv2.inRange(hsv2, LOWER_RED, UPPER_RED)

        # # Bitwise-AND mask and original image
        # res = cv2.bitwise_and(frame, frame, mask = mask)
        res1 = cv2.bitwise_and(frame_sub, frame_sub, mask = mask1)
        res2 = cv2.bitwise_and(frame_rec, frame_rec, mask = mask2)

        # Eliminate holes and dots 
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, KERNEL)
        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, KERNEL)

        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, KERNEL)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, KERNEL)

        # Find the masked area
        # opencv3
        #image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # opencv4
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, RED, 3)

        contours1, hierarchy1 = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame_sub, contours1, -1, RED, 3)

        contours2, hierarchy2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame_rec, contours2, -1, RED, 3)

        print(line)
        if len(contours) >= 1 and line:
            # Black object(s) detected
            # focus on the first one
            cx = getMomentX(contours[0]) / LW # normalization

            clen = len(contours)
            while clen > 1:
                # find most center cx
                clen -= 1
                ccx = getMomentX(contours[clen]) / LW # normalization
                if abs(cx - CENTER) > abs(ccx - CENTER):
                    cx = ccx

            # Forgetting-factor-based smoothing
            gcx = (1.0 - FF) * cx + FF * gcx
            printBar(gcx)

            # gcx -> line position [-1.0,1.0]
            cur_pos = (gcx - CENTER) * 2.0

            prop = cur_pos
            deriv = cur_pos - prev_pos
            intg += prop
            prev_pos = cur_pos

            power = P_TERM * prop + I_TERM * intg + D_TERM * deriv

            left = speed + power
            right = speed - power

            if left < MIN :
                left = MIN
            elif left > MAX:
                left = MAX

            if right < MIN :
                right = MIN
            elif right > MAX:
                right = MAX

            r3pi.left_motor(left)
            r3pi.right_motor(right)

            #print(cur_pos, power, speed, intg, deriv, left, right)

        else:
            print('len(contours) < 1')

        # time.sleepで実行済み # 時計周り　→　反時計回り*2 -> 時計回り
        if len(contours2) >= 1: # 障害物を認識したとき
            print("detection")
            r3pi.stop()
            cap_sleep(3)
            r3pi.left_motor(left)
            cap_sleep(30)
            r3pi.forward(speed)
            cap_sleep(20)
            r3pi.stop()
            cap_sleep(3)
            print("half")
            r3pi.right_motor(right)
            cap_sleep(30)
            r3pi.forward(speed)
            cap_sleep(40)
            r3pi.stop()
            cap_sleep(3)
            r3pi.right_motor(right)
            cap_sleep(15)
            print("go")
            # r3pi.right_motor(right)
            # cap_sleep(30)
            r3pi.left_motor(left)
            cap_sleep(20)
            #r3pi.stop()

        for cnt in contours1:
            x, y, w, h, = cv2.boundingRect(cnt)
            print("x={0}, y={1}, w={2}, h={3}, WW={4}" .format(x, y, w, h, WW))
            if w > int(WW * 0.8): # 上の検出窓
                print("next car")
            
                # r3pi.right_motor(0.3)
                # print("centre")
                # cap_sleep(30)
                # r3pi.stop()
                roll = True
                break

        if roll:
            print("roll")
            r3pi.stop()
            time.sleep(2)
            r3pi.left(0.3) # 一回転
            time.sleep(1.2)
            r3pi.stop()
            line = False
            print(line)


        if not line: # バトンパス完了後
            print("kill")
            
            # os.system("pkill -9 -n python3")
            raise KeyboardInterrupt

                

        # for c in contours2:
        #     xx, xy, xw, xh, = cv2.boundingRect(c)
        #     print("xx={0}, xy={1}, xw={2}, xh={3}, XW={4}" .format(xx, xy, xw, xh, XW))
        #     if xw > int(XW * 0.5):
        #         print("detection")
        #         r3pi.stop()

        # if red_line:
        #     r3pi.stop()
        #     cap_sleep(15)
        #     r3pi.right_motor(right)
        #     cap_sleep(15)
        #     r3pi.stop()
        #     red_line = False

        # if stop_line == 1:
        #     r3pi.left_motor(left)
        #     # r3pi.stop()
        #     # cap_sleep(30)
        # else stop_line == 2:
        #     r3pi.stop()    
        if GUI:
            cv2.rectangle(frame,(0,0),(LW-1,LH-1),(0,255,0),1)
            cv2.rectangle(frame_sub, (0,0),(WW-1,WH-1),(0,255,0), 1)
            cv2.rectangle(frame_rec, (0,0),(XW-1,XH-1),(0,255,0), 1)
            cv2.imshow('frame0', frame0)

        # Press ESC key to exit
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            print("ctrl c")
            break

        f += 1
        time2 = time.time()
        av_fps = f / (time2 - start)
        fps = 1 / (time2 - time1)
        time1 = time2
        print ("{0} : fps={1:.4}, av_fps={2:.4}".format(f, fps, av_fps))

        lock.release()

def music():
    global lock
    global r3pi
    
    while True:
        lock.acquire()
        r3pi.play("! V12 T180 crdrerfrgrarbr>cr")
        lock.release()
        time.sleep(8)

try:
    thread1 = multiprocessing.Process(target=music)
    thread1.start()
    time.sleep(0.5)
    motor_play()
    thread1.join()
except KeyboardInterrupt:
    r3pi.left_motor(speed)
    thread1.kill()
    cv2.destroyAllWindows()
    r3pi.stop()

