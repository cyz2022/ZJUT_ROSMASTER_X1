# coding:UTF-8
# Version: V1.0.1
import serial
import time
import numpy as np


class imuSub:
 
    def __init__(self):
        self.ACCData = [0.0] *8
        self.GYROData = [0.0] *8
        self.AngleData = [0.0] *8
        self.FrameState = 0  # What is the state of the judgment
        self.Bytenum = 0  # Read the number of digits in this paragraph
        self.CheckSum = 0  # Sum check bit			
        self.a = [0.0] *3			
        self.w = [0.0] *3
        self.Angle = [0.0] *3
        self.acc = 0,0,0
        self.gyro = 0,0,0

    def get_acc(self,datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def DueData(self,inputdata):  # New core procedures, read the data partition, each read to the corresponding array 
        #global FrameState    # Declare global variables
        #global Bytenum
        #global CheckSum
        #global acc
        #global gyro
    	#global Angle
        #print("inputdata",inputdata)
        for data in inputdata:  # Traversal the input data
            #print(self.FrameState)
            if self.FrameState == 0:  # When the state is not determined, enter the following judgment
                if data == 0x55 and self.Bytenum == 0:  # When 0x55 is the first digit, start reading data and increment bytenum
                    self.CheckSum = data
                    self.Bytenum = 1
                    continue
                elif data == 0x51 and self.Bytenum == 1:  # Change the frame if byte is not 0 and 0x51 is identified
                    self.CheckSum += data
                    self.FrameState = 1
                    self.Bytenum = 2
                elif data == 0x52 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 2
                    self.Bytenum = 2
                elif data == 0x53 and self.Bytenum == 1:
                    self.CheckSum += data

                    self.FrameState = 3
                    self.Bytenum = 2
            elif self.FrameState == 1:  # acc

                if self.Bytenum < 10:            # Read 8 data
                    self.ACCData[self.Bytenum-2] = data  # Starting from 0
                    self.CheckSum += data
                    self.Bytenum += 1
                #print(1)
                else:
                    if data == (self.CheckSum & 0xff):  # verify check bit
                        self.acc = self.get_acc(self.ACCData)

                    #print("acc",acc) 
                    self.CheckSum = 0  # Each data is zeroed and a new circular judgment is made
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 2:  # gyro

                if self.Bytenum < 10:
                    self.GYROData[self.Bytenum-2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.gyro = self.get_gyro(self.GYROData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 3:  # angle
                #print(2)
                if self.Bytenum < 10:
                    self.AngleData[self.Bytenum-2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.Angle = self.get_angle(self.AngleData)
                        #print(1)
                        result = self.acc + self.gyro + self.Angle                    
                    #print(
                        #"acc:%10.3f %10.3f %10.3f \ngyro:%10.3f %10.3f %10.3f \nangle:%10.3f %10.3f %10.3f" % result)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
        
        return self.acc,self.gyro,self.Angle

    def get_gyro(self,datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z


    def get_angle(self,datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z

    def return_angle(self):
        port = '/dev/gyroscope' # USB serial port #/dev/ttyS3
        baud = 115200   # Same baud rate as the INERTIAL navigation module
        ser = serial.Serial(port, baud, timeout=0.1)
        #img_sub = imuSub()
        #print("Serial is Opened:", ser.is_open)
        #time.sleep(0.1)
        datahex = ser.read(33)        
        _,_,angle = self.DueData(datahex)
        #time.sleep(0.1)
        #if angle != [0.0] *3 :
            #print("angle:",angle)
        return angle
    
    def return_acc(self):
        port = '/dev/ttyUSB1' # USB serial port #/dev/ttyS3
        baud = 115200   # Same baud rate as the INERTIAL navigation module
        ser = serial.Serial(port, baud, timeout=0.1)
        #img_sub = imuSub()
        #print("Serial is Opened:", ser.is_open)
        #time.sleep(0.1)
        datahex = ser.read(33)        
        acc,_,_ = self.DueData(datahex)
        #time.sleep(0.1)
        #if angle != [0.0] *3 :
            #print("angle:",angle)
        return acc

if __name__ == '__main__':
    #port = '/dev/ttyUSB0' # USB serial port #/dev/ttyS3
    #baud = 9600   # Same baud rate as the INERTIAL navigation module
    #ser = serial.Serial(port, baud, timeout=0.5)
    img_sub = imuSub()
    #print("Serial is Opened:", ser.is_open)
    #time.sleep(0.5)
    #i = 0
    #a = np.zeros(100)
   # while(i<100):
        #a[i] = img_sub.return_pitch()
        #i+=1
        #print(a[i])
    #print(a.sum()/100)
        #img_sub.return_angle()
       # datahex = ser.read(33)        
        #angle = img_sub.DueData(datahex)
       # print("angle:",angle)
        #angle_x, angle_y, angle_z = get_angle(datahex)
        #print("angle:",angle_x, angle_y, angle_z)





