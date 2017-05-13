#!/usr/bin/python2

import subprocess
import os
import time
import sys

def main():
    eleNum = 5
    count = 0

    os.system('clear')

    while True:
        # update the watch target file every 25 cycles
        if count % 25 == 0:
            dataFile = subprocess.check_output(
                    'find `pwd` -regex ".*data.txt" | xargs ls -t | head -1',
                    shell=True)

        count += 1
        data = subprocess.check_output(
            'tail -n 1 %s' % dataFile,
            shell=True)

        dataArray = data.strip('\r\n').split(' ')

        timeStamp = dataArray.pop(0)

        print('\033[H File: %s\n Count: %s' % (dataFile.strip('\r\n'), timeStamp))

        print('%4s%6s%12s%12s%12s%8s' %
                ('MOT', 'SW', 'TPOS', 'APOS', 'AVEL', 'ACUR'))

        dataLen = len(dataArray)
        readableMotorNum = int(dataLen / eleNum)

        for i in range(readableMotorNum):
            print('%4s%6s%12s%12s%12s%8s' % (
                i,
                dataArray[i*eleNum+0],
                dataArray[i*eleNum+1],
                dataArray[i*eleNum+2],
                dataArray[i*eleNum+3],
                dataArray[i*eleNum+4]))

        time.sleep(0.2)


if __name__ == '__main__':
    main()



