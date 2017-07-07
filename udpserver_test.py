#!/usr/bin/python2

import socket
import subprocess
import os
import time
import sys

accessCode = 'GET'
localPort = 5868
bufferLength = 2048

def getCurrentLogData():
    dataToSend='''196011 4663 116720 116720 0 33 4663 120531 120531 0 16 4663 120541 120541 0 39 4663 116674 116674 0 25 4663 120503 120503 0 -77 4663 120503 120503 0 -228 4663 116720 116720 0 135 4663 120541 120541 0 -80 4663 120531 120531 0 -112 4663 116720 116720 0 131 4663 120541 120541 0 -110 4663 120531 120531 0 -154 4663 116674 116674 0 162 4663 120503 120503 0 -30 4663 120503 120503 0 4 4663 116720 116720 0 80 4663 120531 120531 0 26 4663 120541 120541 0 -22 '''
    return dataToSend

def main():
    localAddr = ('0.0.0.0', localPort)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(localAddr)

    print('Data monitor server started')

    while True:
        msgRecv, remoteAddr = sock.recvfrom(bufferLength)
        if msgRecv:
            print('Data:',msgRecv)
            print('remoteAddr:', remoteAddr)

            if msgRecv.strip('\r\n ') == accessCode:
                dataToSend = getCurrentLogData()

                msgToSend = 'BEG ' + dataToSend + ' END'
                sock.sendto(msgToSend, remoteAddr)
                print('DataLength:', len(msgToSend))

if __name__ == '__main__':
    main()

