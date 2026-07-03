import socket
import struct
import numpy as np
import cv2
from queue import Queue
def recvall(sock, n):
    # Helper function to receive n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


class ImageReciever:
    def __init__(self, host='127.0.0.1', port=8765):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.image_queue = Queue(maxsize=3)

    def connect(self):
        self.client_socket.connect((self.host, self.port))

    def disconnect(self):
        self.client_socket.close()

    def receive_image(self):
        
        try:
            while True:
                # 接收帧头
                frame_header = recvall(self.client_socket, 2)
                if frame_header != bytes([0x5A, 0xA5]):
                    print("Invalid frame header")
                    self.client_socket.close()
                    exit()

                # 接收功能码
                function_code = recvall(self.client_socket, 1)
                if function_code != bytes([0x61]):
                    print("Invalid function code")
                    self.client_socket.close()
                    exit()

                # 接收长度，年月日毫秒（小端格式）
                length_data = recvall(self.client_socket, 12)
                length,year,month,day,milliseconds = struct.unpack('<IHBBL', length_data)

                content_dataframe_num = recvall(self.client_socket, 4)
                dataframe_num = struct.unpack('<L', content_dataframe_num)[0]
                # 接收图像分辨率
                content_image_resolution = recvall(self.client_socket, 4)
                image_resolution = struct.unpack('<f', content_image_resolution)[0]
            
                content_row_column_num = recvall(self.client_socket, 4)
                row_column_num = struct.unpack('<i', content_row_column_num)[0]
                img_data_num = row_column_num*row_column_num * 4
                
                content_image_data_bytes = recvall(self.client_socket, img_data_num)
                # 将字节数据解析为 float32 类型的数组
                #n = length - 7 - 20
                image_data_array = np.frombuffer(content_image_data_bytes, dtype=np.float32).reshape(row_column_num, row_column_num)

                # 数据归一化，将数据缩放到 0-255 的 uint8 范围
                image_data_array[image_data_array>4000000]=4000000
                min_val = np.min(image_data_array)
                max_val = np.max(image_data_array)
                scaled_data = (image_data_array - min_val) / (max_val - min_val)  # 归一化到 0-1
                image_data = (scaled_data * 255).astype(np.uint8)  # 转换为 uint8

                # 应用伪彩色映射
                pseudo_color_image = cv2.applyColorMap(image_data, cv2.COLORMAP_JET)
                # # 接收CRC校验值
                if self.image_queue.full():
                    self.image_queue.get()
                    self.image_queue.put(pseudo_color_image.copy())
                else:
                    self.image_queue.put(pseudo_color_image.copy())
                crc_value = recvall(self.client_socket, 2)
                # self.image_queue.put(pseudo_color_image.copy())

        except Exception as e:
            print(e)
            return None
            #self.client_socket.close()
            #exit()
        # finally:
        #     # 关闭连接
        #     self.client_socket.close()
        #     print("Connection closed.")

# # 服务器的主机名和端口
# host = '127.0.0.1'
# port = 8765

# # 创建一个socket对象
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# # 连接到服务器
# client_socket.connect((host, port))

# try:
#     # 接收帧头
#     frame_header = recvall(client_socket, 2)
#     if frame_header != bytes([0x5A, 0xA5]):
#         print("Invalid frame header")
#         client_socket.close()
#         exit()

#     # 接收功能码
#     function_code = recvall(client_socket, 1)
#     if function_code != bytes([0x61]):
#         print("Invalid function code")
#         client_socket.close()
#         exit()

#     # 接收长度，年月日毫秒（小端格式）
#     length_data = recvall(client_socket, 12)
#     length,year,month,day,milliseconds = struct.unpack('<IHBBL', length_data)[0]

#     # # 接收日期：年
#     # content_year = recvall(client_socket, 2)
#     # year = struct.unpack('<H', content_year)[0]
#     # # 接收日期：月
#     # content_month = recvall(client_socket, 1)
#     # month = struct.unpack('<B', content_month)[0]
#     # # 接收日期：日
#     # content_day = recvall(client_socket, 1)
#     # day = struct.unpack('<B', content_day)[0]
#     # # 接收日期：当天经过的毫秒数
#     # content_millisecond = recvall(client_socket, 4)
#     # millisecond = struct.unpack('<L', content_millisecond)[0]
#     # # 接收数据帧号
#     content_dataframe_num = recvall(client_socket, 4)
#     dataframe_num = struct.unpack('<L', content_dataframe_num)[0]
#     # 接收图像分辨率
#     content_image_resolution = recvall(client_socket, 4)
#     image_resolution = struct.unpack('<f', content_image_resolution)[0]
#     # 接收图像行列数
#     content_row_column_num = recvall(client_socket, 4)
#     row_column_num = struct.unpack('<i', content_row_column_num)[0]
#     img_data_num = row_column_num*row_column_num * 4
#     # 接收直角坐标图像数据
#     # image_data = []
#     # for i in range(row_column_num):
#     #     row = []
#     #     for j in range(row_column_num):
#     #         content_image_data = recvall(client_socket, 4)
#     #         image_data0 = struct.unpack('<f', content_image_data)[0]
#     #         row.append(image_data0)
#     #     image_data.append(row)
#     content_image_data_bytes = recvall(client_socket, img_data_num)
#     # 将字节数据解析为 float32 类型的数组
#     #n = length - 7 - 20
#     image_data_array = np.frombuffer(content_image_data_bytes, dtype=np.float32).reshape(row_column_num, row_column_num)

#     # 数据归一化，将数据缩放到 0-255 的 uint8 范围
#     image_data_array[image_data_array>4000000]=4000000
#     min_val = np.min(image_data_array)
#     max_val = np.max(image_data_array)
#     scaled_data = (image_data_array - min_val) / (max_val - min_val)  # 归一化到 0-1
#     image_data = (scaled_data * 255).astype(np.uint8)  # 转换为 uint8

#     # 应用伪彩色映射
#     pseudo_color_image = cv2.applyColorMap(image_data, cv2.COLORMAP_JET)

#     # 显示图像
#     cv2.imshow('Pseudo-color Image', pseudo_color_image)
#     cv2.waitKey(0)  # 等待按键后关闭窗口
#     #cv2.destroyAllWindows()
#     # # 接收内容
#     # content = recvall(client_socket, 2)
#     # # 接收CRC校验值
#     crc_value = recvall(client_socket, 2)


# finally:
#     # 关闭连接
#     client_socket.close()
#     print("Connection closed.")