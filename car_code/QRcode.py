#!/usr/bin/env python3.7
# coding:utf8

from cv2 import cv2
from pyzbar import pyzbar

def QRcode_recognizition():
    capture = cv2.VideoCapture(0)
    while(1):
        ret, frame = capture.read()
        test = pyzbar.decode(frame)
        for tests in test:
            testdate = tests.data.decode('utf-8')
            print(testdate)
        cv2.imshow('QRcode', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
if __name__ == '__main__':
    QRcode_recognizition()

