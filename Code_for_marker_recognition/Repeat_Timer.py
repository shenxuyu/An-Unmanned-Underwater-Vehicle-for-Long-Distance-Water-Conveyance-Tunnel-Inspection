#!/usr/bin/env python3
# coding:utf-8

import time
from threading import Timer


class RepeatingTimer(Timer):
    def run(self):
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)


def print_timer():
    """
    打印当前时间，测试用的
    :return:
    """
    current_time = int(time.time())
    local_time = time.localtime(current_time)
    dt = time.strftime("%Y/%m/%d %H:%M:%S", local_time)
    print(dt)


if __name__ == '__main__':
    t = RepeatingTimer(1, print_timer)  # 每隔10秒执行一次 print_timer 函数
    t.start()
