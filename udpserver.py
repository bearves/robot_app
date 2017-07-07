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
    dataFile = subprocess.check_output(
            'find /var/log/aris -regex ".*data.txt" | xargs ls -t | head -1',
            shell=True)

    data = subprocess.check_output(
            'tail -n 1 %s' % dataFile,
            shell=True)

    dataToSend = data.strip('\r\n')
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

