#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import crcmod.predefined
import serial
from struct import unpack
import serial.tools.list_ports
import threading
from PIL import Image
import cv2
import base64
from io import BytesIO


class EvoThermal:
    def __init__(self):
        ### Search for Evo Thermal port and open it ###
        ports = list(serial.tools.list_ports.comports())
        portname = None
        for p in ports:
            if ":5740" in p[2]:
                print("EvoThermal found on port " + p[0])
                portname = p[0]
        if portname is None:
            print("Sensor not found. Please Check connections.")
            exit()
        self.port = serial.Serial(
            port=portname,  # To be adapted if using UART backboard
            baudrate=115200,  # 460800 for UART backboard
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.port.isOpen()
        self.serial_lock = threading.Lock()
        ### CRC functions ###
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        ### Activate sensor USB output ###
        self.activate_command = (0x00, 0x52, 0x02, 0x01, 0xDF)
        self.deactivate_command = (0x00, 0x52, 0x02, 0x00, 0xD8)
        self.send_command(self.activate_command)

        self.MinAvg = []
        self.MaxAvg = []

        r = []
        g = []
        b = []
        with open('thermals/colormap.txt', 'r') as f:
            for i in range(256):
                x, y, z = f.readline().split(',')
                r.append(x)
                g.append(y)
                b.append(z.replace(";\n", ""))
        self.colormap = np.zeros((256, 1, 3), dtype=np.uint8)
        self.colormap[:, 0, 0] = b
        self.colormap[:, 0, 1] = g
        self.colormap[:, 0, 2] = r

    def update_GUI(self):

        frame = self.rounded_array.astype(np.uint8)
        frame = cv2.applyColorMap(frame, self.colormap)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # frame = cv2.resize(frame, (self.canvas_width, self.canvas_height), interpolation=cv2.INTER_NEAREST)

        buffered = BytesIO()
        Image.fromarray(frame).save(buffered, format="PNG")
        # reset file pointer to start
        buffered.seek(0)
        img_bytes = buffered.read()

        base64_encoded_result_bytes = base64.b64encode(img_bytes)
        base64_encoded_result_str = base64_encoded_result_bytes.decode('ascii')

        return base64_encoded_result_str

    def array_2_image(self, frame):
        '''
        This function is creating an Image from numpy array
        '''
        thermal_img = frame
        im = Image.fromarray(np.uint8(thermal_img), mode="P")
        im = im.resize(size=(self.canvas_width, self.canvas_height), resample=Image.NEAREST)
        return im

    def get_thermal_data(self):
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                ### Polls for header ###
                header = self.port.read(2)
                # header = unpack('H', str(header))
                header = unpack('H', header)
                if header[0] == 13:
                    ### Header received, now read rest of frame ###
                    data = self.port.read(2068)
                    ### Calculate CRC for frame (except CRC value and header) ###
                    calculatedCRC = self.crc32(data[:2064])
                    data = unpack("H" * 1034, data)
                    receivedCRC = (data[1032] & 0xFFFF) << 16
                    receivedCRC |= data[1033] & 0xFFFF
                    TA = data[1024],
                    data = data[:1024],
                    data = np.reshape(data, (32, 32))
                    ### Compare calculated CRC to received CRC ###
                    if calculatedCRC == receivedCRC:
                        got_frame = True
                    else:
                        print("Bad CRC. Dropping frame")
        self.port.flushInput()
        ### Data is sent in dK, this converts it to celsius ###
        data = (data / 10.0) - 273.15
        return data

    def get_forehead_data(self):
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                ### Polls for header ###
                header = self.port.read(2)
                # header = unpack('H', str(header))
                header = unpack('H', header)
                if header[0] == 13:
                    ### Header received, now read rest of frame ###
                    data = self.port.read(2068)
                    ### Calculate CRC for frame (except CRC value and header) ###
                    calculatedCRC = self.crc32(data[:2064])
                    data = unpack("H" * 1034, data)
                    receivedCRC = (data[1032] & 0xFFFF) << 16
                    receivedCRC |= data[1033] & 0xFFFF
                    TA = data[1024],
                    data = data[:1024]
                    data = np.reshape(data, (32, 32))

                    result = data[6][10:20]
                    result = np.vstack((data[7][10:20], result))
                    result = np.vstack((data[8][10:20], result))
                    result = np.vstack((data[9][10:20], result))
                    result = np.vstack((data[10][10:20], result))
                    result = np.vstack((data[11][10:20], result))
                    result = np.vstack((data[12][10:20], result))

                    ### Compare calculated CRC to received CRC ###
                    if calculatedCRC == receivedCRC:
                        got_frame = True
                    else:
                        print("Bad CRC. Dropping frame")
        self.port.flushInput()
        ### Data is sent in dK, this converts it to celsius ###
        result = (result / 10.0) - 273.15
        return result

    def get_max_thermal(self):
        data = self.get_forehead_data()
        # Get Data from upper half of image
        return 0.5553 * np.max(data) + 17.3893

    def get_thermals(self):
        data = self.get_thermal_data()

        ### Get min/max/TA for averaging ###
        frameMin, frameMax = data.min(), data.max()
        self.MinAvg.append(frameMin)
        self.MaxAvg.append(frameMax)

        ### Need at least 10 frames for better average ###
        if len(self.MaxAvg) >= 10:
            AvgMax = sum(self.MaxAvg) / len(self.MaxAvg)
            AvgMin = sum(self.MinAvg) / len(self.MinAvg)
            ### Delete oldest insertions ###
            self.MaxAvg.pop(0)
            self.MinAvg.pop(0)
        else:
            ### Until list fills, use current frame min/max/ptat ###
            AvgMax = frameMax
            AvgMin = frameMin

        # Scale data
        data[data <= AvgMin] = AvgMin
        data[data >= AvgMax] = AvgMax
        multiplier = 255 / (AvgMax - AvgMin)
        data = data - AvgMin
        data = data * multiplier

        return data

    def send_command(self, command):
        ### This avoid concurrent writes/reads of serial ###
        with self.serial_lock:
            self.port.write(command)
            ack = self.port.read(1)
            ### This loop discards buffered frames until an ACK header is reached ###
            while ord(ack) != 20:
                ack = self.port.read(1)
            else:
                ack += self.port.read(3)
            ### Check ACK crc8 ###
            crc8 = self.crc8(ack[:3])
            if crc8 == ack[3]:
                ### Check if ACK or NACK ###
                if ack[2] == 0:
                    print("Command acknowledged")
                    return True
                else:
                    print("Command not acknowledged")
                    return False
            else:
                print("Error in ACK checksum")
                return False

    def get_thermal_img(self):
        ### Get frame and print it ###
        frame = self.get_thermals()
        self.rounded_array = np.round(frame, 0)
        return self.update_GUI()

    def get_forehead_img(self):
        ### Get frame and print it ###
        frame = self.get_forehead_data()
        self.rounded_array = np.round(frame, 0)
        return self.update_GUI()

    def stop(self):
        ### Deactivate USB VCP output and close port ###
        self.send_command(self.deactivate_command)
        self.port.close()