# 编辑人:努力学习的小李
# 开发时间:2024/1/14 10:25
# 座右铭:今天不学习，明天变废物！
from image_reciever import ImageReciever
from resnet18 import ResNet18MNIST
from Repeat_Timer import RepeatingTimer as RTimer
import threading
import cv2
import torch
import numpy as np
import socket


class IdentifySignal():
    def __init__(self):
        self.img_reciver = ImageReciever()
        self.img_reciver.connect()
        threading.Thread(target=self.img_reciver.receive_image()).start()

        # 导入GPU设备和训练好的模型
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = ResNet18MNIST()
        self.checkpoint = torch.load('Sonar_identify/trained_model/ACC_97.39.pth')
        self.model.load_state_dict(self.checkpoint)
        self.net = self.model.to(self.device)

        # 创建定时识别图像定时器
        self.identify_image_timer = RTimer(0.1,self.idenfy_image)
        self.identify_image_timer.start()

        # 建立UDP通讯
        self.socket_sonar_identify = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.local_addr_sonar_identify = ('192.168.1.5', 6666)
        self.socket_sonar_identify.bind(self.local_addr_sonar_identify)
        self.local_addr_Qt = ('192.168.1.5', 6665)

    def idenfy_image(self):
        img = self.img_reciver.image_queue.get()
        if img is not None:
            image = cv2.resize(img, (480, 480))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = image.astype(np.float32)
            image = image / 255.0 - 0.5
            image = np.transpose(image, (2, 0, 1))
            image = torch.Tensor(image)

            image = image.to(self.device)

            out = self.net(image.unsqueeze(0))
            pred = out.argmax(dim=1)
            print(f"图片所属的label分别是:", pred.item())
            # 方案一：直接向Qt界面发送图片识别的结果
            self.socket_sonar_identify.sendto(pred.item(),self.local_addr_Qt)
            # 方案一：直接向Qt界面发送图片识别的结果
            # 方案二：只有图片识别到标志物才向Qt发送数据报
            # if pred.item() == 1:
            #     self.socket_sonar_identify.sendto(pred.item(), self.local_addr_Qt)
            # 方案二：只有图片识别到标志物才向Qt发送数据报

if __name__ == '__main__':
    try:
        print("The Sonar is identifying markers!")
        IS = IdentifySignal()
    except KeyboardInterrupt as e:
        print("The Sonar stop recognition marker!")
        print(e)