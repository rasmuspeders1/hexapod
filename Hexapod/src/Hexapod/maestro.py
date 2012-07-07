#! /usr/bin/env python

import serial
import struct
import time
import threading
import logging


class Maestro(threading.Thread):
    """
    Python class that handles communication with the Pololu Maestro series
    servo controller.
    The class uses the "Compact Protocol" as described in the Maestro manual.
    """
    def __init__(self, get_pos, port='/dev/ttyACM0', baudrate=115200, updateFreq=50, logInterval=1):
        self.maestroLogger = logging.getLogger('Maestro')
        self.logInterval = logInterval
        self.Positions = {}
        self.oldPositions = {}
        self.PositionList = []
        try:
            self.SerialPort = serial.Serial(port, baudrate)
            self.SerialPort.open()
        except Exception, exc:
            self.maestroLogger.error('Failed to Initiate connection on serial port %s with baud rate %d. Error: %s', port, baudrate, exc)
            self.stop()

        self.stopEvent = threading.Event()

        self.stopEvent.clear()
        #angle to pulse width ratio. eg. ms. per degree
        self.d2pRatio = 10
        #angle to pulse width offset. eg 0 degrees is 600 ms.
        self.d2pOffset = 600
        #centering offset. used to center the servos precisely.
        self.centerOffsets = {}
        #frequency with which the positions are sent to the controller
        self.updateFreq = updateFreq
        self.frequency = 0
        self.frequencyHist = []

        self.maestroLogger.info('Maestro servo controller interface initialized.')
        self.started = False
        self.get_pos = get_pos
        threading.Thread.__init__(self)

    def set_pos(self, Servo, PulseWidth):

        command = struct.pack('BB', 0x84, Servo)
        command += struct.pack('B', PulseWidth * 4 & 0x7f)
        command += struct.pack('B', PulseWidth * 4 >> 7 & 0x7f)
        self.SerialPort.write(command)

    def set_group_pos(self, StartServo, PulseWidthList):
        command = struct.pack('BBB', 0x9F, len(PulseWidthList), StartServo)
        for PulseWidth in PulseWidthList:
            command += struct.pack('B', PulseWidth * 4 & 0x7f)
            command += struct.pack('B', PulseWidth * 4 >> 7 & 0x7f)

        self.SerialPort.write(command)

    def go_home_all(self):
        self.SerialPort.write(chr(0xA2))

    def get_errors(self):
        command = struct.pack('B', 0xA1)
        self.SerialPort.write(command)
        return struct.unpack('BB', self.SerialPort.read())

    def print_hex(self, string):
        hexstring = ''
        for ch in string:
            hexstring += format(ord(ch), '02x')

        print(hexstring)

    def print_binary(self, string):
        binstring = ''
        for ch in string:
            binstring += format(ord(ch), '08b')

        print(binstring)

    def generate_position_list(self):
        """
        Method that generates self.PositionList from self.Positions
        the output is a list with servo positions at the indices corresponding
        to the servo address of the Maestro controller.
        """
        if not self.Positions:
            return

        if not len(self.PositionList) == max(self.Positions.keys()) + 1 :
            self.PositionList.extend([0] * (max(self.Positions.keys()) + 1 - len(self.PositionList)))

        for index, value in self.Positions.items():
            if index in self.centerOffsets.keys():
                self.PositionList[index] = value + self.centerOffsets[index]
            else:
                self.PositionList[index] = value

    def run(self):
        lastLog = time.time()
        self.maestroLogger.info('Maestro servo controller interface running.')
        self.started = True
        self.frequency = self.updateFreq
        while not self.stopEvent.isSet():
            begin = time.time()
            self.Positions = self.get_pos()
            if time.time() - lastLog > self.logInterval:
                lastLog = time.time()
                self.maestroLogger.debug('Servo Positions: %s' % str(self.Positions))
                self.maestroLogger.info('Average Update Frequency: %s' % ('?' if len(self.frequencyHist) == 0 else (float(sum(self.frequencyHist)) / float(len(self.frequencyHist)))) )

            #update positionList from Position and centerOffset dictionaries
            self.generate_position_list()
            ServoPWList = [int(pos * self.d2pRatio + self.d2pOffset) for pos in self.PositionList]
            #send position command to servo controller
            self.set_group_pos(0, ServoPWList)

            #sleep for remaining time of update tick
            sleeptime = 1.0 / self.updateFreq - (time.time() - begin)
            if sleeptime > 0:
                time.sleep(sleeptime)
            self.frequency = 1.0 / (time.time() - begin)
            #Save start time
            self.frequencyHist.append(self.frequency)
            if len(self.frequencyHist) > 1000:
                del self.frequencyHist[0]

        self.started = False


    def stop(self):
        self.maestroLogger.info('Maestro servo controller interface stopping.')
        self.stopEvent.set()
        while self.started:
            time.sleep(0.1)
        self.go_home_all()
        self.join()
        self.maestroLogger.info('Maestro servo controller interface stopped.')



