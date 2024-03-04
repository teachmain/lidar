import serial
import time
import math
import matplotlib.pyplot as plot
import numpy
import pyautogui
port = serial.Serial('/dev/ttyUSB0',115200)
def readSection()->bytearray:
    section = bytearray()
    while True:
        if port.read()[0] == 0xAA:
            if port.read()[0] == 0x55:
                section.append(0xAA)
                section.append(0x55)
                section.append(port.read()[0])
                lsn = port.read()[0]
                section.append(lsn)
                remain_byte_number = 6 + 2*lsn
                for i in range(remain_byte_number):
                    section.append(port.read()[0])
                return section
    

def print_hex(bytes):
  l = [hex(int(i)) for i in bytes]
  print(" ".join(l))

def is_header(section)->bool:
    if section[2] & 1 == 1:
        return True
    else:
        return False

def readFrame(buff):
    frame = []
    if len(buff) !=0 :
        frame.append(buff.pop(0))
    else:
        temp_section = readSection()
        while not is_header(temp_section):
            temp_section = readSection()
        frame.append(temp_section)
    
    temp_section = readSection()
    while not is_header(temp_section):
        frame.append(temp_section)
        temp_section = readSection()
    
    buff.append(temp_section)
    return frame

def decodeSection(section):
    pointNum = section[3]
    startAngle = (section[4] + section[5] * 256.0) / 128.0
    endAngle = (section[6] + section[7] * 256.0) / 128.0
    while endAngle < startAngle:
        endAngle += 360
    diffAngle = endAngle - startAngle
    if pointNum > 1 :
        stepAngle = diffAngle / (pointNum-1)
    else:
        stepAngle = 0
    result = []
    for i in range(pointNum):
        if section[10+2*i] & 3 > 1:
            continue
        else:
            distance = (section[10+2*i] >> 2) + section[11+2*i] * 64
            if distance < 100:
                continue
            angle = startAngle + i*stepAngle

            if distance != 0:
                angle += math.atan(21.8 * (155.3 - distance) / (155.3 * distance))

            angle = -angle
            angle += 90

            while angle<0 or angle >= 360:
                if angle >= 360 :
                    angle -= 360

                if angle < 0:
                    angle += 360

            angle /= 180
            angle *= math.pi
            x = - math.cos(angle) * distance
            y = - math.sin(angle) * distance
            result.append([x,y])
    return result

def decodeFrame(frame):
    result = []
    for section in frame:
        section_result = decodeSection(section)
        result += section_result
    return result

def combine(points,ref_distance):
    result = []
    flag = 0
    sumx=0
    sumy=0
    count=0
    for point in points:
        temp_x = point[0]
        temp_y = point[1]
        if count > 0:
            avg_x = sumx/count
            avg_y = sumy/count
            temp_dist = math.sqrt((avg_x - temp_x)**2 + (avg_y-temp_y)**2)
            if temp_dist > ref_distance:
                result.append([avg_x,avg_y])
                sumx = temp_x
                sumy = temp_y
                count = 1
        else:
                sumx = temp_x
                sumy = temp_y
                count = 1
    if count != 0:
        avg_x = sumx/count
        avg_y = sumy/count
        result.append([avg_x,avg_y])
    return result

def clamp(input,min,max):
    if input < min:
        return min
    if input > max:
        return max
    return input

def clampPoint(low_x,high_x,low_y,high_y,points):
    result = []
    for point in points:
        if point[0] < low_x or point[0] > high_x or point[1] < low_y or point[1] > high_y:
            continue
        new_x = (point[0] - low_x) / (high_x - low_x)
        new_y = (point[1] - low_y) / (high_y - low_y)
        result.append([new_x,new_y])
    return result

        


        

plot.ion()
fig,ax = plot.subplots()
while True:
    buff = []
    frame = readFrame(buff)
    result = decodeFrame(frame)
    points = numpy.array(result)
    if len(points) == 0:
        continue
    points = clampPoint(-280,300,150,500,points)
    points = combine(points,0.2)
    if len(points) == 0:
        continue
    # points = numpy.array(points)
    #points = points.T
    #plot.cla()
    #ax.set_xlim(0,1)
    #ax.set_ylim(0,1)
    #plot.scatter(points[0],points[1])
    #plot.pause(0.001)
    #print(pyautogui.position())
    pyautogui.moveTo(3839*points[0][0], 2159*(1-points[0][1]))
    
    
    


