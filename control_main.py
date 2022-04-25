# -*- coding: utf-8 -*-

import numpy as np
import math
import time
import serial
import binascii
import threading
import numpy as np
import pyperclip

import sys
import random
import os
import socket
# import tkinter
import requests

from pygame.locals import *
import pygame

from pyproj import Geod
from contextlib import closing

# from msvcrt import getch

# gps_PORT = "/dev/ttyUSB0" # テキストで指定
# whill_PORT = "/dev/ttyUSB1" # テキストで指定


pygame.init()  # Pygameを初期化
screen = pygame.display.set_mode((800, 600))  # 画面を作成
pygame.display.set_caption("keyboard event")  # タイトルを作成
nodeS = np.array([  # S
    [33.32009737126935, 130.38062811621978],
    [33.320281157131646, 130.3808279407914],
    [33.320429642377604, 130.381027765363],
    [33.320503243333334, 130.38101967166668],
    [33.320557986666664, 130.380907945],
    [33.320642575, 130.38073984],
    [33.32065904, 130.38072736666666],
])

nodeT = np.array([  # T
    [33.320765525, 130.38058003],
    [33.32104003333333, 130.38043794],
    [33.32125212, 130.38046574166665],
    [33.321365996666664, 130.38052031666666],
    [33.32144092833333, 130.38057997666667],
    [33.32151210166667, 130.38064723833332],
])

nodeU2 = np.array([  # U
    [33.321442268333335, 130.38058260666668],
    [33.321380686666664, 130.380536975],
    [33.32123150666666, 130.38046044333333],
    [33.32101020333333, 130.380459975],
    [33.32077934, 130.38057943],
    [33.32068140333333, 130.38070883166668],
])

nodeU = np.array([  # U
    [33.32144752666667, 130.38058757166667],
    [33.321416115, 130.38056172],
    [33.32129899666667, 130.38048279333333],
    [33.32110610833333, 130.38043553833333],
    [33.32086165166667, 130.38051553833333],
    [33.32071480333333, 130.38065914],
])

nodeV = np.array([  # S
    [33.32060998833333, 130.380829345],
    [33.32052367166666, 130.380992745],
    [33.32046932666667, 130.38106599666668],
    [33.32039247666667, 130.38101239],
    [33.32029203, 130.38087579333333],
    [33.32015592166667, 130.38072403],
    [33.320051013333334, 130.38061162333332],
])

nodeE = np.array([
    [33.32018289340305, 130.38077215653442],
    [33.32033866299999, 130.38093845349334],
    [33.320445124075135, 130.38107792836212],
    [33.320529172200516, 130.38128177624725],
    [33.32060873768452, 130.38151646953605],
    [33.32066028711427, 130.38176591497444],
    [33.32071519842981, 130.38199792605423],
    [33.32075217950037, 130.38210923772834],
    [33.32080597012044, 130.38228626352333],
    [33.32089001789784, 130.38244049054168],
    [33.3209617386038, 130.38254643779777],
    [33.32103345925075, 130.38265238505386],
    [33.32119931302073, 130.38281734090828],
    [33.32136852836619, 130.3829836378672],
    [33.32141223293846, 130.38299839001678]
])

nodeF = np.array([
    [33.32129456673244, 130.3829112182238],
    [33.32113095441055, 130.38276369672798],
    [33.32098639258288, 130.38259337645553],
    [33.32087881153224, 130.38241098624252],
    [33.3207790748146, 130.382227254925],
    [33.3207286460937, 130.3820287714579],
    [33.32067821734361, 130.3818302879908],
    [33.32062778856436, 130.3816318045237],
    [33.32057735975593, 130.3814333210566],
    [33.32051684514729, 130.3812375197985],
    [33.32043391765239, 130.3810551295855],
    [33.32031512948169, 130.3809116314032],
    [33.32019634114912, 130.3807681332209],
    [33.32006858747865, 130.38063402277015],
])

## global
whillAcc = 0
whillRev = 0
whillPhi = 0
whillTarget = 0
whillDistance = 0

current = 0
count = 0
angle = 0
distance = 0
nodeLen = 0
node = []

emg = 0  # emergency

goal = 0

detect = 0
center = 0

long = 0
lati = 0
azim = 999

## GPS-RTK
# gps_BAURATE = 57600
# gps_Serial = serial.Serial(
# port = gps_PORT,
# baudrate = gps_BAURATE,
# parity = serial.PARITY_NONE,
# bytesize = serial.EIGHTBITS,
# stopbits = serial.STOPBITS_TWO,
# timeout = None,
# xonxoff = 0,
# rtscts = 0,
# )

## whill
whill_BAURATE = 38400
whill_PROTCOLSIGN = "AF"
ID = {"startSendingData": "00",
      "stopSendingData": "01",
      "powerON": "02",
      "powerOFF": "02",
      "setJoystick": "03",
      "setSpeedProfile": "04",
      "setBatteryOutON": "05",
      "setBatteryOutOFF": "05", }


# ctrlSerial = serial.Serial(
# port = whill_PORT,
# baudrate = whill_BAURATE,
# parity = serial.PARITY_NONE,
# bytesize = serial.EIGHTBITS,
# stopbits = serial.STOPBITS_TWO,
# timeout = None,
# xonxoff = 0,
# rtscts = 0,
# )

def getCoordinate():
    global long
    global lati
    coordinate = (lati, long)
    return coordinate


def copyCoordinate():
    global long
    global lati
    coordinate = f'[{lati}, {long}],'
    return coordinate


def sendSerial(k):
    # print(k)
    for l in k:
        b = binascii.a2b_hex(l)
        ctrlSerial.write(b)


def addCheckSum(message):
    cs = "00"
    for j in message:
        cs = int(cs, 16) ^ int(j, 16)  # 10進数に変換，XORを取る
        cs = hex(cs)  ##文字列に変換
    cs = cs[2:]  ##文字列3文字目から読み取る 0xaf→af
    message.append(cs)
    sendSerial(message)


def genCmd(id, a, r):
    message = [whill_PROTCOLSIGN]
    if (id == "startSendingData"):
        print("WHILLから情報取得 開始")
        message.append("06")
        message.append(ID[id])
    elif (id == "stopSendingData"):
        print("WHILLから情報取得 停止")
        message.append("02")
        message.append(ID[id])
    elif (id == "powerON"):
        print("電源ON")
        message.append("03")
        message.append(ID[id])
        message.append("01")  ##ON
    elif (id == "powerOFF"):
        print("電源OFF")
        message.append("03")
        message.append(ID[id])
        message.append("00")  ##OFF
    elif (id == "setJoystick"):
        # print("WHILLにジョイティック制御量送信")
        message.append("05")
        message.append(ID[id])
        message.append("00")  # 00 disable / 01 enable joystick
        message.append(a)  # front/back
        message.append(r)  # left/right
    elif (id == "setSpeedProfile"):
        print("WHILLのスピード設定")
        message.append("0B")  # length
        message.append(ID[id])
        message.append("04")  # rs232c
        message.append("20")  # maxSpeed f 8~60 unit 0.1km/h
        message.append("10")  # acc f
        message.append("20")  # deacc f
        message.append("20")  # maxSpeed b 8~30 unit 0.1km/h
        message.append("10")  # acc b
        message.append("20")  # deacc b
        message.append("20")  # maxSpeed t 8~35 unit 0.1km/h
        message.append("10")  # acc t
        message.append("20")  # deacc t
    elif (id == "setBatteryOutON"):
        print("電源取得 ON")
        message.append("03")
        message.append("05")
        message.append("01")

    elif (id == "setBatteryOutOFF"):
        print("電源取得 OFF")
        message.append("03")
        message.append(ID[id])
        message.append("00")
    # print(message)
    addCheckSum(message)


def keyInput():
    ## ascii code 0-9:48-57
    #             a-z:97-122
    global goal
    global node
    global nodeLen
    global emg

    while True:
        for event in pygame.event.get():  # 何かしらの入力があったとき
            if event.type == KEYDOWN:  # キーを押したとき
                # ESCキーならスクリプトを終了
                if event.key == K_c:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeC
                    nodeLen = len(node)
                if event.key == K_d:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeD
                    nodeLen = len(node)
                if event.key == K_e:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeE
                    nodeLen = len(node)
                if event.key == K_f:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeF
                    nodeLen = len(node)
                if event.key == K_s:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeS
                    nodeLen = len(node)
                if event.key == K_t:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeT
                    nodeLen = len(node)
                if event.key == K_u:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeU
                    nodeLen = len(node)
                if event.key == K_v:
                    print(("スタート GOAL = 5 NODE C"))
                    goal = 5
                    node = nodeV
                    nodeLen = len(node)
                if event.key == K_SPACE:
                    if emg == 0:
                        emg = 1
                    else:
                        emg = 0
            pygame.display.update()

        # key = ord(getch())
        # if(49 == key):# 1
        #     pyperclip.copy(copyCoordinate())
        # elif(50 == key):# 2
        #     pass
        # elif(51 == key):# 3
        #     pass
        # elif(52 == key):# 4
        #     pass
        # elif(53 == key):# 5
        #     pass
        # elif(54 == key):# 6
        #     pass
        # elif(55 == key):# 7
        #     pass
        # elif(97 == key):# a
        #     print(("スタート GOAL = 5 NODE A"))
        #     goal = 5
        #     node = nodeA
        #     nodeLen = len(node)
        # elif(98 == key):# b
        #     print(("スタート GOAL = 5 NODE B"))
        #     goal = 5
        #     node = nodeB
        #     nodeLen = len(node)
        # elif(99 == key):# c
        #     print(("スタート GOAL = 5 NODE C"))
        #     goal = 5
        #     node = nodeC
        #     nodeLen = len(node)
        # elif(100 == key):# d
        #     print(("スタート GOAL = 5 NODE D"))
        #     goal = 5
        #     node = nodeD
        #     nodeLen = len(node)
        # elif(101 == key):# e
        #     print(("スタート GOAL = 5 NODE E"))
        #     goal = 5
        #     node = nodeE
        #     nodeLen = len(node)
        # # elif(98 == key):#b
        # #     sendCmd("任意初期位置同定要求")
        # elif(57 == key):# 9
        #     pass
        # else:
        #     pass


def main():
    global current
    global count
    genCmd("setBatteryOutON", "", "")
    global goal
    global emg
    i = 0

    while (True):
        # genCmd("setBatteryOutON","","")
        if goal == 0:
            print(getCoordinate())
            pass
        elif (goal != 0 and detect != 1):
            # if( goal != 0):
            acce = 0
            revo = 0
            targetA = 0
            distance = 0
            #############################################################
            x0, y0 = getCoordinate()
            x1 = node[current, 0]
            y1 = node[current, 1]
            acce, revo, deltaA, distance = angleAdj(x0, y0, x1, y1)

            whillAcc = acce * power_control  # 速度の項？？  acce(固定値)×距離に対する速度のパーセンテージ値＝距離に応じた速度値
            whillRev = revo
            ## 前・後進
            acce = acce.to_bytes(1, byteorder='little', signed=True)  # int2bytes
            ## 左右
            revo = revo.to_bytes(1, byteorder='little', signed=True)
            acce = '{:02X}'.format(acce[0])
            revo = '{:02X}'.format(revo[0])

            if emg == 0 and goal != 0:  # emergency = 1
                genCmd("setJoystick", acce, revo)
                print(current)
                # print(printInfo)
            elif (emg == 1):
                pass
                # print(printInfo + " :emgStop")
            elif (emg == 2):
                pass
            else:
                pass
                # print(printInfo + " :else")

            # time.sleep(0.05)

    # 終了処理
    genCmd("stopSendingData", "", "")
    genCmd("setBatteryOutOFF", "", "")
    genCmd("powerOFF", "", "")


def calcGeo(x0, y0, x1, y1):
    # 基準点(p1)となる緯度(p1_latitude)、経度(p1_longitude)
    # 単位はDegree
    p1_latitude = x0
    p1_longitude = y0

    # 飛行物体は緯度(obj_latitude)、経度(obj_longitude)、高度(obj_altitude)のところにある
    obj_latitude = x1
    obj_longitude = y1
    obj_altitude = 0  # 単位は(m)

    # ellpsは赤道半径。GPSはWGS84を使っている。距離は6,378,137m
    g = Geod(ellps='WGS84')

    # 必要なものだけ欲しいなら以下でも可
    result = g.inv(p1_longitude, p1_latitude, obj_longitude, obj_latitude)
    azimuth = result[0]
    # back_azimuth = result[1]
    distance_2d = result[2]

    # inv()で求めた距離が分かれば、GPSの高度と合わせて仰角(elevation)が分かる
    # math.degrees()はmath.atan2()の戻り値がRadianなので、Degree(°)に変換している。
    elevation = math.degrees(math.atan2(obj_altitude, distance_2d))

    # 飛行物体までの直線距離(distance_3d)はmathを使ってピタゴラスの定理
    # math.pypotで求められる。
    distance_3d = math.hypot(distance_2d, obj_altitude)

    # print( '基準となる位置から飛行物体までの地図上距離は' + str(distance_2d) + "m")
    # print( '基準となる位置から飛行物体までの直線距離は' + str(distance_3d) + "m")
    # print( '基準となる位置から飛行物体までの方位角は' + str(azimuth) + "°")
    # print( '基準となる位置から飛行物体までの反方位角は' + str(back_azimuth) +"°")
    # print( '基準となる位置から飛行物体までの仰角は' + str(elevation) + "°")
    return distance_2d, azimuth


def angleAdj(x0, y0, x1, y1):
    global current
    global count
    global nodeLen
    global goal
    global azim

    turn = 0
    front = 0

    # 距離と方位の計算
    distance, houi = calcGeo(x0, y0, x1, y1)

    # 右か左か
    direction = direct(azim, houi)

    # ステアの量
    c = amountSteering(azim, houi)

    # ポイント接近判定
    if distance <= 1:
        if current >= nodeLen - 1:
            front = 0
            turn = 0
            goal = 0
            node = []
            nodeLen = 0
            current = 0
        else:
            if (count == 0 and goal != 0):
                current = current + 1
                # count = count +1
            elif (count > 0):
                # count = count +1
                pass

    if c > 8:
        # c : 目標までの角度
        front = 10
        turn = 20
    # elif ( c <= 10 and distance > 200):
    elif c <= 8 and distance > 0.5:
        front = 45
        turn = 5
        count = 0
    else:
        front = 0
        turn = 0
    ## 操舵
    turn = turn * direction

    return front, turn, direction, distance


def readSerial():
    # 暁旦文字まで読む readserial
    tmp = gps_Serial.readline()
    return tmp


def direct(hedding, target):
    print(f'目標の方位 {target} 今の向き {hedding}')
    d = 0
    a = amountSteering(hedding, target)
    print(a)
    if target > hedding:
        # 目標のほうが大きかったら 右
        d = 1
    elif target < hedding:
        # 車両の向きのほうが大きかったら 左
        d = -1

    if a > 180:
        d *= -1
    return d


# 差の大きさ
def amountSteering(hedding, target):
    hedding = hedding + 180
    target = target + 180
    amount = abs(hedding - target)
    return amount


def gps_read():
    global long
    global lati
    global azim
    while (True):
        nmea = readSerial()
        nmea = nmea.strip().decode('UTF-8')
        if (nmea != ''):
            split_nmea = nmea.split(',')
            if (split_nmea[0] == '$GPGGA'):
                lat0 = float(split_nmea[2][:2])
                lat1 = float(split_nmea[2][2:]) / 60

                lon0 = float(split_nmea[4][:3])
                lon1 = float(split_nmea[4][3:]) / 60

                lati = lat0 + lat1
                long = lon0 + lon1

                # print(f'lat {lat}, lon {lon}')
            elif split_nmea[0] == '$HEHDT':
                azi = float(split_nmea[1])
                if azi > 180:
                    azi = azi - 360
                print(round(azi))
                azim = round(azi)

    # 二種類のメッセージの可能性がある
    # $GPGGA
    # $GPGGA,085120.307,3541.1493,N,13945.3994,E,1,08,1.0,6.9,M,35.9,M,,0000*5E
    # $GPGGA,時間      ,北緯        ,東経
    # $HEHDT
    # 行頭がおかしい場合は読み捨てる


def udpRasp():
    global emg  # emg=o sefty
    global power_control

    # UDP Init
    SrcIP = "192.168.128.176"
    SrcPort = 22222
    SrcAddr = (SrcIP, SrcPort)
    BUFSIZE = 1024
    udpServSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpServSock.bind(SrcAddr)
    while True:
        data, addr = udpServSock.recvfrom(BUFSIZE)
        power_control = data.decode()
        # print(mode)

    if power_control == '0':  # 緊急停止
        emg = 1

    else:
        emg = 0

    # threading.Thread(target=gps_read, args=()).start()


# threading.Thread(target=keyInput, args=()).start()
# threading.Thread(target=main, args=()).start()
threading.Thread(target=udpRasp, args=()).start()
