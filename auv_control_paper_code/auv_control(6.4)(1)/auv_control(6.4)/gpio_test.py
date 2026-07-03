#!/usr/bin/env python3
# -*- coding:UTF-8 -*-

# author: Yu xing
# contact: 17824845379@163.com
# datetime:2023/10/18 8:14
# software: PyCharm

from periphery import GPIO

# 根据具体板卡的LED灯和按键连接修改使用的Chip和Line
LED_CHIP = "/dev/gpiochip0"
LED_LINE_OFFSET = 8

BUTTON_CHIP = "/dev/gpiochip1"
BUTTON_LINE_OFFSET = 1cd 

led = GPIO(LED_CHIP, LED_LINE_OFFSET, "out")
button = GPIO(BUTTON_CHIP, BUTTON_LINE_OFFSET, "in")

try:
    while True:
        led.write(button.read())
finally:
    led.write(True)
    led.close()
