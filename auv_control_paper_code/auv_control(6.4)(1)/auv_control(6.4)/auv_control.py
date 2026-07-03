#!/usr/bin/env python3
# -*- coding:UTF-8 -*-

# author: Yu xing. Wang weiwei. Li chang
# contact: 17824845379@163.com
# datetime:2023/8/28 15:30
# software: PyCharm

# ==============分工================= #
"""
1234部分由王维伟编写
56789部分由余星编写
10111213部分由李昌编写
"""
# ================================== #
import socket
import struct
import math
import csv
import time
import copy
import threading
from Repeat_Timer import RepeatingTimer as RTimer
from Modbus import Modbus_Code as Mc
from thruster_distribute import ThrustDis as Td  # 推力分配
from course_pid import CoursePID
from under_depth_pid import UnderDepthPID  # 不带fuzzy的欠驱动定深PID
# from under_depth_pid_with_fuzzy import UnderDepthPID
from full_depth_pid import FullDepthPID
from pitch_pid import PitchPID
from roll_pid import RollPID
from under_depth_backstep import UnderDepthBackStep #反步法欠驱动定深控制
from FullCourseMPC import FullCourseMPC

class AuvControl:
    def __init__(self):
        self.socket_main_board = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_host_computer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_sonar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # 绑定声纳socket
        self.local_addr_main_board = ('192.168.1.4', 20000)  # 英伟达绑定本机IP和port,用于与主控板收发数据
        self.local_addr_host_computer = ('192.168.1.4', 8887)  # 英伟达绑定本机IP和port,用于与上位机收发数据
        self.local_addr_sonar = ('192.168.1.4', 8848)
        self.up_addr = ('192.168.1.5', 8886)  # 上位机QT界面IP和port
        self.down_addr = ('192.168.1.6', 20000)  # 主控板IP和port
        self.sonar_add = ('192.168.1.7', 8321) # 声纳ip和port
        self.socket_main_board.bind(self.local_addr_main_board)
        self.socket_host_computer.bind(self.local_addr_host_computer)
        self.socket_sonar.bind(self.local_addr_sonar) # 绑定声纳地址
        self.Mc = Mc()
        self.Td = Td()
        # 定义全局变量，用于接收传感器数据
        self.depth_data = 0.00
        self.distance_list = [0.00, 0.00, 0.00, 0.00]
        self.angleVel_list = [0.00, 0.00, 0.00, 0.00]
        self.angle_list = [0.00, 0.00, 0.00]
        self.yaw = 0.00
        self.InsVel_list = [0.00, 0.00, 0.00]
        self.longitude = 0.0000000
        self.latitude = 0.0000000
        self.Ins_flag = 0
        self.Dvl_flag = 0
        self.DvlVel_list = [0.00, 0.00, 0.00]
        self.Dvl_voyage = 0.00
        self.Dvl_depth = 0.00
        self.Dvl_height = 0.00
        self.DvlToInsAngle_list = [0.00, 0.00, 0.00]
        self.recv_modbus_code = b''

        self.pre_feedback_dvl_x = 0.0  # 上一时刻的 feedback_dvl_x
        self.pre_feedback_dvl_y = 0.0  # 上一时刻的 feedback_dvl_y
        # 控制力矩
        self.Fx = 0
        self.Fy = 0
        self.Fz = 0
        self.Fn = 0
        self.Fm = 0
        # 定义9个全局变量，用于定时下发
        self.main_thruster = 0
        self.f_h_thruster = 0
        self.f_v_thruster = 0
        self.b_h_thruster = 0
        self.b_v_thruster = 0
        self.left_steer = 0
        self.right_steer = 0
        self.up_steer = 0
        self.down_steer = 0
        # flag
        self.function_flag = True       # 功能码标志位
        # self.course_pid_flag = False    # 定向PID标志，防止多次打开定时器
        self.depth_pid_flag = False
        self.pitch_pid_flag = False
        self.roll_pid_flag = False      # 横滚PID标志，防止多次打开定时器
        self.auto_course_flag = False
        self.control_mode_flag = 0  # 切换控制模式后初始化flag
        self.next_control_mode_flag = 0  # 切换控制模式后初始化flag
        self.run_time_counter_cal = 0
        self.wireless_flag = 0  # 0-有线模式 ，1-无线模式
        self.run_time_counter = 0  # 欠驱动定深定向运行时间计数
        self.under_auto_start_flag = 0  # 判断加在欠驱动定向里，改变值加在测航速时欠驱动定深定向功能码函数里
        self.under_depth_pid_flag = True
        # ==========sxy===============
        self.full_course_mpc_flag = False
        # ===========================
        # 数据保存文件编号
        self.file_num = 0
        # PID
        self.exp_course = 0
        self.exp_depth = 0
        self.exp_roll = 0  # 肯定是0
        self.alpha4 = 0  # 用于靠一边的自主航行的计算
        self.positive_angle = 0  # 自主航行正前方角度
        # self.course_ctl_mode = None
        self.depth_ctl_mode = None
        self.set_pid_ctl_mode = None
        self.auto_ctl_mode = None
        self.auto_ctl_direction = None
        self.direction_ctl_mode = None
        # =====sxy定义的mpc航向控制模式======
        self.mpc_course_ctl_mode = None
        # =================================
        # 惯性环节
        self.last_system_output = 0
        self.system_output = 0
        # roll惯性环节
        self.last_roll_system_output = 0
        self.system_roll_output = 0
        # 自主航行规划角度
        self.res_plan = 0
        # 剔除野值
        self.last_exp_course = 0
        # 安全函数初始化
        self.safe_flag = 10
        # 定向 定深 定速
        self.set_course_depth_speed_mode = None
        self.ac_p = 0.0  # 取名ac表示自主航向auto_course
        self.ac_r = 0.0
        self.ac_k1 = 0.0
        self.ac_k2 = 0.0
        self.delta_l = 0.115  # 左右测距仪的距离
        self.ac_d = 1.59  # 前后测距仪的距离
        # 编程顺序：先欠驱动 后全驱动；先定向，后定深
        #self.under_course_pid_controller = \
        #    CoursePID(Kp=1.2, Ki=0.05, Kd=1, sample_time=0.1)       # 航向PID控制器（欠驱动）
        # self.full_course_pid_controller = \
        #    CoursePID(Kp=2, Ki=0.05, Kd=1, sample_time=0.1)       # 航向PID控制器（全驱动）
        # self.under_depth_pid_controller = \
        #     UnderDepthPID(Kp=3, Ki=0, Kd=0, sample_time=0.1)      # 深度PID控制器（欠驱动）
        # # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制2024.1.4添加￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
        self.under_depth_backstep_controller = \
            UnderDepthBackStep(K1=1, K2=2, K3=2, sample_time=0.1)      # 深度反步法控制器（欠驱动）
        # # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制2024.1.4添加￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
        self.full_depth_pid_controller = \
            FullDepthPID(Kp=25, Ki=0.5, Kd=6, sample_time=0.1)    # 深度PID控制器（全驱动）d=2
        self.full_pitch_pid_controller = \
            PitchPID(Kp=2, Ki=0.05, Kd=2, sample_time=0.1)
            # PitchPID(Kp=2, Ki=0.05, Kd=1, sample_time=0.1)         # 俯仰PID控制器（全驱动）11月12上午12点17之前ki=0.1
        self.roll_pid_controller = \
            RollPID(Kp=1, Ki=0.05, Kd=1, sample_time=0.1)          # 横滚PI控制器
        
        # =================sxy写的MPC定向控制===================
        # 初始化 FullCourseMPC 控制器
        self.full_course_mpc_controller = FullCourseMPC(sample_time=0.1)
        #======================================================

        # 定时器
        # self.course_pid_timer = RTimer(0.2, self.course_pid_calculate)
        # =================sxy写的定时器===================
        self.full_course_mpc_timer = RTimer(0.2, self.full_course_mpc_calculate)  # 添加定时器
        # ================================================
        self.depth_pid_timer = RTimer(0.2, self.depth_pid_calculate)
        self.auto_course_timer = RTimer(0.2, self.plan_angle_calculate)
        self.auv_func_decide_timer = RTimer(0.2, self.function_decide)
        self.auv_func_decide_timer.start()
        self.hybrid_drive_data_timer = RTimer(0.2, self.hybrid_drive_data)
        self.hybrid_drive_data_timer.start()
        
        # 数据存储
        self.save_sensor_data_file = None
        self.file_writer = None
        self.save_data_flag = False
        # 读传感器数据
        self.my_thread = threading.Thread(target=self.read_sensor_data, args=(0.1,))
        self.read_date_lock = threading.Lock()  # 创建一个锁
        self.my_thread.start()
        # 上传传感器数据
        self.upload_sensor_timer = RTimer(0.2, self.upload_sensor_data)
        self.upload_sensor_timer.start()

    # 功能码判断函数
    def function_decide(self):
        """
        主要是用于判断上位机下发数据的标志位
        :return:
        """
        print("进入功能码决定函数！")
        while self.function_flag:
            # 退出线程
            if not self.function_flag:
                break
            recv_content = self.recv_udp_msg(self.socket_host_computer)  # 从上位机收到的数据
            print(f"从上位机接收到数据：{recv_content}")
            function_num = int(recv_content[0])
            # 安全函数
            self.safe_flag = 1
            if function_num == 17:
                self.wireless_flag = 1  # 无线模式
                self.auto_start_function(recv_content)
            else:
                self.wireless_flag = 0  # 无线模式
                if function_num == 1:  # Function control enable
                    self.control_enable_function(recv_content)
                elif function_num == 2:  # Function full_drive or under_drive control
                    self.control_mode_function(recv_content)
                elif function_num == 3:  # Function light brightness control
                    self.led_function(recv_content)
                elif function_num == 4:  # function INS init and Set parameters
                    self.ins_init_function(recv_content)
                elif function_num == 5:  # function full_drive control with velocity
                    self.full_drive_test_function(recv_content)
                # elif function_num == 6:  # function course control
                #    self.course_control_function(recv_content)
                # ===================sxy写的MPC定向控制功能码=====================
                elif function_num == 6:  # FullCourseMPC 控制器功能码
                    self.full_course_mpc_function(recv_content)
                # ==============================================================
                elif function_num == 7:  # function depth control
                    self.depth_control_function(recv_content)
                elif function_num == 8:  # function autonomous course
                    self.auto_course_function(recv_content)
                elif function_num == 9:  # function set PID parameters
                    self.set_pid_parameters_function(recv_content)
                elif function_num == 10:  # function make file
                    self.save_data_function(recv_content)
                elif function_num == 12:  # Function enable Sonar
                    self.sonar_enable_function(recv_content)
                elif function_num == 15:  # MainBoard restart
                    self.main_board_restart_function(recv_content)
                elif function_num == 16:  # CAN restart
                    self.can_restart_function(recv_content)
                # elif function_num == 17:  # 无线状态下启动欠驱动定深和定向
                #     self.auto_start_function(recv_content)

                elif function_num == 0:  # Function break the connection
                    print("break connection!")
                    break
            time.sleep(0.1)  # 每0.1s循环一次

    # 1 推进器、舵机始能
    def control_enable_function(self, msg):
        """
        控制始能数据的具体内容
        :param msg: 去掉标志位之后的具体数据即 主推 侧垂推 舵机
        :return:
        """
        # self.wireless_flag = 0  # 有线模式
        main_thruster_enable = int(msg[1])
        lat_thruster_enable = int(msg[2])
        steer_control_enable = int(msg[3])
        enable_flag = None
        if main_thruster_enable == 1 and lat_thruster_enable == 0 \
                and steer_control_enable == 1:
            enable_flag = 0  # 欠驱动
        elif main_thruster_enable == 1 and lat_thruster_enable == 1 \
                and steer_control_enable == 0:
            enable_flag = 1  # 全驱动
        elif main_thruster_enable == 0 and lat_thruster_enable == 0 \
                and steer_control_enable == 0:
            enable_flag = 2  # stop
        if enable_flag == 0:
            print('欠驱动始能')
            close_init_ctl_code = self.Mc.encode(1, '15', 1, 3, [1, 0, 1])
            self.send_udp_code(close_init_ctl_code, self.down_addr, self.socket_main_board)
        elif enable_flag == 1:
            print('全驱动始能')
            close_init_ctl_code = self.Mc.encode(1, '15', 1, 3, [1, 1, 1])
            self.send_udp_code(close_init_ctl_code, self.down_addr, self.socket_main_board)
        elif enable_flag == 2:
            print("始能关闭")
            close_init_ctl_code = self.Mc.encode(1, '15', 1, 3, [0, 0, 0])
            self.send_udp_code(close_init_ctl_code, self.down_addr, self.socket_main_board)
            self.hybrid_drive_data_initialize()  # 使能关闭后全局变量清0

    # 2 全驱动、欠驱动模式选择
    def control_mode_function(self, msg):
        """
        欠驱动全驱动模式下的推进器速度数据
        :param msg:
        :return:
        """
        # self.wireless_flag = 0  # 有线模式
        control_mode = int(msg[1])  # 1-全驱动 0-欠驱动
        if self.control_mode_flag != control_mode:
            self.hybrid_drive_data_initialize()
        self.control_mode_flag = control_mode
        if control_mode == 0:  # under_drive control mode
            next_control_mode = int(msg[2])  # 1-手柄控制模式 2-自主航行模式 3-定深调试 4-定向调试
            if self.next_control_mode_flag != next_control_mode:
                self.hybrid_drive_data_initialize()
            self.next_control_mode_flag = next_control_mode
            print("欠驱动模式")
            if next_control_mode == 1:  # 手柄控制
                # 20230915初始化
                # self.hybrid_drive_data_initialize()
                self.main_thruster = int(msg[3])
                self.left_steer = float(msg[4])
                self.right_steer = float(msg[5])
                self.up_steer = float(msg[6])
                self.down_steer = float(msg[7])
                # 20230920欠驱动保护
                # self.under_drive_protection()
                print('欠驱动手柄控制')
                print([int(self.left_steer / 10), int(self.right_steer / 10),
                       int(self.up_steer / 10), int(self.down_steer / 10)])
            elif next_control_mode == 2:  # 欠驱动自主控制
                # 待补充写完整的部分，欠驱动定深和定向，现在只给主推转速
                # 20230915初始化
                # self.hybrid_drive_data_initialize()
                self.main_thruster = int(msg[3])
                # 20230920欠驱动保护
                # self.under_drive_protection()
                print('定深定向手动主推转速')
            elif next_control_mode == 3:  # 欠驱动定深调试模式
                # 定深模式下，界面输入主推转速，手柄输入上下舵，
                # PID定深输入左舵右舵
                self.main_thruster = int(msg[3])
                self.up_steer = float(msg[6])
                self.down_steer = float(msg[7])
                # 20230920欠驱动保护
                # self.under_drive_protection()
                print('定深手动主推转速：', self.main_thruster)
                print('定深手动上下舵：', self.up_steer, self.down_steer)
            elif next_control_mode == 4:  # 欠驱动定向调试模式
                self.main_thruster = int(msg[3])
                self.left_steer = float(msg[4])
                self.right_steer = float(msg[5])
                # 20230920欠驱动保护
                # self.under_drive_protection()
                print('定向手动左右舵：', self.left_steer, self.right_steer)
            # self.control_flag = 0
        elif control_mode == 1:  # full_drive control mode
            # 20230915初始化
            # self.hybrid_drive_data_initialize()
            print("全驱动模式")
            fulldrive_next_control_mode = int(msg[2])
            if fulldrive_next_control_mode == 0:  # 手柄控制
                self.Fx = int(msg[3])
                self.Fy = int(msg[4])
                self.Fz = int(msg[5])
                self.Fn = int(msg[6])
                self.Fm = int(msg[7])
                # 20230920，下位机全驱动手柄控制保护
                # self.full_drive_protection()
                self.thrust_distribute_assign(self.Fx, self.Fy, self.Fz, self.Fn, self.Fm)
            else:  # 自主控制
                self.Fx = int(msg[3])
                self.thrust_distribute_assign(self.Fx, self.Fy, self.Fz, self.Fn, self.Fm)
    # 3 灯始能
    def led_function(self, msg):
        """
        灯亮度数据的具体内容
        :param msg: 始能 左灯亮度 右灯亮度
        :return:
        """
        led_control_enable = int(msg[1])  # 灯始能标志
        led_l = int(msg[2])
        led_r = int(msg[3])
        led_list = [led_l, led_r]  # 亮度
        # 灯始能
        led_modbus_code = self.Mc.encode(3, '5', 4, 1, led_control_enable)
        self.send_udp_code(led_modbus_code, self.down_addr, self.socket_main_board)
        # 始能了，再发亮度
        if led_control_enable == 1:
            modbus_code = self.Mc.encode(8, '16', 80, 2, led_list)
            self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)

    # 4 INS始能以及设定初始纬度
    def ins_init_function(self, msg):
        """
        惯导始能及初始纬度数据
        :param msg:
        :return:
        """
        ins_enable_flag = int(msg[1])
        init_latitude = int(float(msg[2]) * 100)
        # INS enable
        modbus_code = self.Mc.encode(4, '5', 5, ins_enable_flag)
        self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        # DVL enable
        modbus_code = self.Mc.encode(5, '5', 10, 1)
        self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        # Init INS Latitude
        modbus_code = self.Mc.encode(9, '6', 96, init_latitude)
        self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        print("惯导、DVL使能、纬度零点设置成功!")

    # 5 全驱动单调试功能
    def full_drive_test_function(self, msg):
        # self.wireless_flag = 0  # 有线模式
        ctl_mode = int(msg[1])
        main_thruster = int(msg[2])
        f_h_thruster = int(msg[3])
        f_v_thruster = int(msg[4])
        b_h_thruster = int(msg[5])
        b_v_thruster = int(msg[6])
        full_drive_test_list = [f_h_thruster, f_v_thruster,
                                b_h_thruster, b_v_thruster,
                                main_thruster]
        if ctl_mode == 2:
            modbus_code = self.Mc.encode(6, '16', 64, 5, full_drive_test_list)
            self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
            # self.control_flag = 0

    # 6 定向控制
    '''
    def course_control_function(self, msg):

        """
        定向控制
        :param msg: 欠/全 开/关 期望角度值
        :return: 
        """
        # self.wireless_flag = 0  # 有线模式
        # 控制模式
        self.course_ctl_mode = int(msg[1])  # 1-全驱动;0-欠驱动
        # 定深PID状态-开启or关闭
        course_pid_status = int(msg[2])  # 1-开启;0-关闭
        # 期望深度
        self.exp_course = float(msg[3])
        if course_pid_status == 1:
            if not self.course_pid_flag:
                # 定时器启动，开始每0.1秒执行 self.course_pid_calculate 一次
                self.course_pid_timer.start()
                self.course_pid_flag = True
                self.roll_pid_flag = True
        else:  # 定向关闭
            # modbus_code = self.Mc.encode(7, '16', 64, 9, [0, 0, 0, 0, 0, 0, 0, 0, 0])
            # # 定向关闭后把全局变量赋0
            # # self.hybrid_drive_data_initialize()
            # self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
            self.course_pid_timer.cancel()
            self.course_pid_timer = RTimer(0.2, self.course_pid_calculate)
            # 将全局变量置零,并将PID三项清0
            self.f_h_thruster = 0
            self.b_h_thruster = 0
            self.Fn = 0
            self.main_thruster = 0
            self.up_steer = 0
            self.down_steer = 0
            self.course_pid_flag = False
            self.roll_pid_flag = False
            self.full_course_pid_controller.clear()   # 定向关闭后，将历史数据清零
            self.under_course_pid_controller.clear()
            self.roll_pid_controller.clear()
        print(f'定向flag：{self.course_pid_flag}')
        print(f'横滚flag：{self.roll_pid_flag}')


    def course_pid_calculate(self):
        # 欠驱动定向
        if self.course_ctl_mode == 0:
            self.under_course_pid_controller.set_course(self.exp_course)
            self.under_course_pid_controller.update(self.yaw,
                                                    round(self.DvlVel_list[0] / 1000, 3))
            theta = self.under_course_pid_controller.output * 10
            if theta <= -450:
                theta = -450
            elif theta >= 450:
                theta = 450
            print(f'定向输出theta：{theta / 10}')
            self.up_steer = theta
            self.down_steer = theta
        # 全驱动定向
        elif self.course_ctl_mode == 1:
            self.full_course_pid_controller.set_course(self.exp_course)  # 输入期望角度
            # 两个参数传入：yaw x向速度
            self.full_course_pid_controller.update(self.yaw, round(self.DvlVel_list[0] / 1000, 3))
            # 在定向中加入RollPID，使用左右舵去控制横滚角
            self.roll_pid_controller.set_roll(0)  # 输入期望横滚角
            # 横滚角 ：round(self.angle_list[1] / 100, 2)
            self.roll_pid_controller.update(round(self.angle_list[1] / 100, 2), round(self.DvlVel_list[0] / 1000, 3))
            self.Fn = self.full_course_pid_controller.output  # 输出结果
            print(f'定向输出Fn：{self.Fn}')
            self.thrust_distribute_assign(self.Fx, self.Fy, self.Fz, self.Fn, self.Fm)
            # 全驱动定向时，为了减小roll角度，打舵角
            # if self.roll_pid_controller.output <= -45:
            #     self.roll_pid_controller.output = -45
            # elif self.roll_pid_controller.output >= 45:
            #     self.roll_pid_controller.output = 45
            # print(f'$$$$$$$$$$$$ {self.roll_pid_controller.output} $$$$$$$$$$$$$$$$$$$$$')

            # 横滚PID
            # self.roll_set_inertia_time(1, 0.3, self.roll_pid_controller.output)
            # self.roll_pid_controller.output = self.system_roll_output
            # if self.roll_pid_controller.output <= -45:
            #     self.roll_pid_controller.output = -45
            # elif self.roll_pid_controller.output >= 45:
            #     self.roll_pid_controller.output = 45
            # print(f'$$$$$$$$$$$$ {self.roll_pid_controller.output} $$$$$$$$$$$$$$$$$$$$$')
            # self.left_steer = self.roll_pid_controller.output * 10  # 输出结果,注意要X10
            # self.right_steer = -1*self.roll_pid_controller.output * 10

            # 打固定角度
            if round(self.DvlVel_list[0] / 1000, 3) > 1:
                self.left_steer = 13 * 10  # 输出结果,注意要X10
                self.right_steer = -1 * 13 * 10
            else:
                self.left_steer = 7 * 10  # 输出结果,注意要X10
                self.right_steer = -1 * 7 * 10

            print(f"定向并控制Roll,输出舵角---左舵:{self.left_steer},右舵:{self.right_steer}")

    '''
     # ========sxy:定义方法 full_course_mpc_function() 来处理功能码(18)=========
    def full_course_mpc_function(self, msg):
        """
        FullCourseMPC 控制器的功能实现
        :param msg: 从上位机接收到的消息
        :return:
        """
        # 控制模式
        self.mpc_course_ctl_mode = int(msg[1])  # 1-开启;0-关闭
        # 定深MPC状态-开启or关闭
        course_mpc_status = int(msg[2])  # 1-开启;0-关闭
        # 期望航向和角速度
        self.exp_course = float(msg[3])
    
        if course_mpc_status == 1:
            if not self.full_course_mpc_flag:
                # 定时器启动，开始每0.2秒执行 self.full_course_mpc_calculate 一次
                self.full_course_mpc_timer.start()
                print(f'mpc的定时器已打开')
                self.full_course_mpc_flag = True

        else:  # 关闭 FullCourseMPC 控制
            self.full_course_mpc_timer.cancel()
            self.full_course_mpc_timer = RTimer(0.2, self.full_course_mpc_calculate)
            self.f_h_thruster = 0
            self.b_h_thruster = 0
            self.Fn = 0
            self.main_thruster = 0
            self.up_steer = 0
            self.down_steer = 0
            self.full_course_mpc_flag = False
            self.full_course_mpc_controller.clear()   # 控制关闭后，将历史数据清零
        print(f'FullCourseMPC flag：{self.full_course_mpc_flag}')
        

    def full_course_mpc_calculate(self):
        """
        更新 FullCourseMPC 控制器
        """
        self.full_course_mpc_controller.set_course(self.exp_course, self.yaw)
        self.pre_feedback_dvl_x = round(self.DvlVel_list[0] / 1000, 3)
        self.pre_feedback_dvl_y = round(self.DvlVel_list[1] / 1000, 3)
        pre_feedback_course = self.yaw
        pre_feedback_r = round(self.angleVel_list[2] / 1000, 3)
        self.full_course_mpc_controller.mpc_update(pre_feedback_course, pre_feedback_r, self.pre_feedback_dvl_x,self.pre_feedback_dvl_y) 
        self.Fn = self.full_course_mpc_controller.output
        self.full_course_mpc_controller.gp_model_update(round(self.angleVel_list[2] / 1000, 3), self.pre_feedback_dvl_x, self.pre_feedback_dvl_y)
        print(f'定向输出Fn:{self.Fn}')
        self.thrust_distribute_assign(self.Fx, self.Fy, self.Fz, self.Fn, self.Fm)
        
         # 打固定角度
        if round(self.DvlVel_list[0] / 1000, 3) > 1:
            self.left_steer = 13 * 10  # 输出结果,注意要X10
            self.right_steer = -1 * 13 * 10
        else:
            self.left_steer = 7 * 10  # 输出结果,注意要X10
            self.right_steer = -1 * 7 * 10

        print(f"定向并控制Roll,输出舵角---左舵:{self.left_steer},右舵:{self.right_steer}")
    # =======================================================================



    # 7 定深控制
    def depth_control_function(self, msg):

        """
        定深控制
        :param msg: 欠/全 开/关 期望深度 主推速度
        :return:
        """
        # self.wireless_flag = 0  # 有线模式
        # 控制模式
        self.depth_ctl_mode = int(msg[1])  # 1-全驱动;0-欠驱动
        # 定深PID状态-开启or关闭
        depth_pid_status = int(msg[2])  # 1-开启;0-关闭
        # 期望深度
        self.exp_depth = float(msg[3])
        if depth_pid_status == 1:
            if not self.depth_pid_flag:  # 防止多次启动定时器
                self.depth_pid_timer.start()
                self.depth_pid_flag = True
                self.pitch_pid_flag = True
        else:
            self.depth_pid_timer.cancel()
            self.depth_pid_timer = RTimer(0.2, self.depth_pid_calculate)
            # 将全局变量置零
            self.f_v_thruster = 0
            self.b_v_thruster = 0
            self.Fz = 0
            self.Fm = 0
            self.main_thruster = 0
            self.left_steer = 0
            self.right_steer = 0
            self.full_depth_pid_controller.clear()  # 定深关闭后，将历史数据清零
            # self.under_depth_pid_controller.clear()
            # # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制2024.1.4添加￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
            self.under_depth_backstep_controller.clear()
            # # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制2024.1.4添加￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
            self.depth_pid_flag = False
            self.pitch_pid_flag = False
        print('定深flag：', self.depth_pid_flag)
        print('俯仰flag：', self.pitch_pid_flag)

    def depth_pid_calculate(self):
        # 欠驱动定深
        if self.depth_ctl_mode == 0:
            # 在下潜到0.5m之前，以左右舵8度，主推500转的速度运行
            # -------------------------------------------------------------
            # if self.depth_data <= 0.4:
            #     if self.under_depth_pid_flag:
            #         self.main_thruster = 500
            #         self.left_steer = self.right_steer = -80
            #         print(f"以固定速度{self.main_thruster}转,固定舵角{self.left_steer / 10}度下潜")
            #     # -------------------------------------------------------------
            # else:
            #     self.under_depth_pid_flag = False

            print("进入定深PID计算下潜角度下潜")
            # self.under_depth_pid_controller.set_depth(self.exp_depth)
            # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
            self.under_depth_backstep_controller.set_depth(self.exp_depth)
            # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥

            # 当前俯仰角
            pitch_data = round(self.angle_list[0] / 100, 2)

            # self.under_depth_pid_controller.update(self.depth_data, pitch_data)
            # theta = self.under_depth_pid_controller.output * 10
            # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
            # 当前速度
            dvl_x_vel = round(self.DvlVel_list[1] / 1000, 3)
            dvl_y_vel = round(self.DvlVel_list[0] / 1000, 3)
            # 做处理，得到AUV真实的前向和横向速度
            auv_x_vel = dvl_y_vel * math.cos(math.pi / 4) - dvl_x_vel * math.cos(math.pi / 4)
            self.under_depth_backstep_controller.update(self.depth_data, pitch_data,auv_x_vel)
            theta = self.under_depth_backstep_controller.output * 10
            # ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥反步法控制￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥
            # 加7度舵角调节横滚角
            self.left_steer = theta + 7 * 10
            self.right_steer = theta - 7 * 10

            #  Remove redundant parentheses 删除掉多余的括号
            if self.left_steer <= -450:
                self.left_steer = -450
            elif self.left_steer >= 450:
                self.left_steer = 450
            if self.right_steer <= -450:
                self.right_steer = -450
            elif self.right_steer >= 450:
                self.right_steer = 450
            print('定深输出theta：', theta / 10)
            # if round(self.DvlVel_list[0] / 1000, 3) > 1:
            #     self.left_steer = 13 * 10  # 输出结果,注意要X10
            #     self.right_steer = -1 * 13 * 10
            # else:
            #     self.left_steer = 7 * 10  # 输出结果,注意要X10
            #     self.right_steer = -1 * 7 * 10

            # 脱缆后欠驱动定深定向
            # self.run_time_counter += 1
            # if self.run_time_counter > self.run_time_counter_cal:
            #     self.save_data_flag = 0  # 保存数据关闭
            #     self.under_auto_start_flag = 1  # 使能关闭
            #     self.depth_pid_timer.cancel()
            #     self.depth_pid_timer = RTimer(0.2, self.depth_pid_calculate)
            #     # print("使能关闭")
            #     print('导航状态：', self.Ins_flag)
            #     # 将全局变量置零
            #     self.main_thruster = 0
            #     self.left_steer = 0
            #     self.right_steer = 0
            #     self.full_depth_pid_controller.clear()  # 定深关闭后，将历史数据清零
            #     self.under_depth_pid_controller.clear()
            #     self.depth_pid_flag = False
            #     self.pitch_pid_flag = False
            #     self.course_pid_timer.cancel()
            #     self.course_pid_timer = RTimer(0.2, self.course_pid_calculate)
            #     # 将全局变量置零,并将PID三项清0
            #     self.course_pid_flag = False
            #     self.roll_pid_flag = False
            #     self.full_course_pid_controller.clear()  # 定向关闭后，将历史数据清零
            #     self.under_course_pid_controller.clear()
            #     self.roll_pid_controller.clear()
            #     self.depth_pid_status = 0  # 1-开启;0-关闭
            #     self.course_pid_status = 0  # 1-开启;0-关闭
            #     # self.run_time_counter_cal = 0
            #     self.run_time_counter = 0
            #     self.save_data_flag = False
            #     if self.save_sensor_data_file is not None and not self.save_sensor_data_file.closed:
            #         self.save_sensor_data_file.close()
            # if self.under_auto_start_flag == 1:
            #     print("始能关闭")
            #     close_init_ctl_code = self.Mc.encode(1, '15', 1, 3, [0, 0, 0])
            #     self.send_udp_code(close_init_ctl_code, self.down_addr, self.socket_main_board)
            #     self.hybrid_drive_data_initialize()  # 使能关闭后全局变量清0


        # 全驱动定深
        elif self.depth_ctl_mode == 1:
            self.full_depth_pid_controller.set_depth(self.exp_depth)
            self.full_pitch_pid_controller.set_pitch_course(0)  # 默认设置目标“pitch”角度为0
            self.full_depth_pid_controller.update(self.depth_data, round(self.DvlVel_list[0] / 1000, 3))
            # self.angle_list[0]为反馈的pitch角度
            self.full_pitch_pid_controller.update(round(self.angle_list[0] / 100, 2), round(self.DvlVel_list[0] / 1000, 3))
            self.Fz = self.full_depth_pid_controller.output
            self.Fm = self.full_pitch_pid_controller.output
            print('定深输出Fz：', self.Fz)
            print('定深输出Fm：', self.Fm)
            # 20230920日全驱动定深保护
            # self.full_drive_protection()
            self.thrust_distribute_assign(self.Fx, self.Fy, self.Fz, self.Fn, self.Fm)

    # 8 自主航行
    def auto_course_function(self, msg):
        """
        自主航向控制回调
        :param msg:P R K1 K2 
        :return: 
        """
        # self.wireless_flag = 0  # 有线模式
        self.auto_ctl_mode = int(msg[1])  # 0-欠驱动；1-全驱动
        auto_course_status = int(msg[2])  # 1-开启;0-关闭
        self.auto_ctl_direction = int(msg[3])  # 1-前进，0-后退
        # 这4个参数，虽然当前函数中未使用，但给全局变量赋值了
        self.ac_p = float(msg[4])  # P 目标点
        self.ac_r = float(msg[5])  # R 隧道半径
        self.ac_k1 = float(msg[6])  # K1
        self.ac_k2 = float(msg[7])  # K2
        self.last_exp_course = float(msg[8])
        self.positive_angle = float(msg[8])  # 正向角度
        if auto_course_status == 1:
            print("自主航行开启！")
            if not self.auto_course_flag:
                self.auto_course_timer.start()
                self.course_ctl_mode = self.auto_ctl_mode
                self.direction_ctl_mode = self.auto_ctl_direction
                self.course_pid_timer.start()
                self.auto_course_flag = True
        else:
            self.auto_course_timer.cancel()
            self.course_pid_timer.cancel()
            self.hybrid_drive_data_initialize()
            self.auto_course_timer = RTimer(0.2, self.plan_angle_calculate)
            self.course_pid_timer = RTimer(0.2, self.course_pid_calculate)
            self.auto_course_flag = False
            self.full_course_pid_controller.clear()  # 定向关闭后，将历史数据清零
            self.under_course_pid_controller.clear()
            self.roll_pid_controller.clear()
        print(self.auto_course_flag)

    def plan_angle_calculate(self):
        """
        自主航行算法，
        :return: 期望角度,赋给self.exp_course
        """

        # self.alpha4 = abs(self.positive_angle - self.yaw)
        # print('^%$#%#%#$@#$#!$^&#&#alpha4',self.alpha4)
        l1 = self.distance_list[2] / 1000  # 前右
        l2 = self.distance_list[1] / 1000  # 后右S
        # l3 = self.distance_list[3] / 1000  # 前左
        # l3 = (self.distance_list[3] / 1000 - abs(2 / math.cos(self.alpha4 / math.pi * 180)))
        # if l3 >= l1:
        #     l3 = (self.distance_list[3] / 1000 - abs(2 / math.cos(self.alpha4 / math.pi * 180)))
        # else:
        #     l3 = self.distance_list[3] / 1000
        # l3 = abs(self.distance_list[3]/1000 - abs(2/math.cos(self.alpha4 / math.pi * 180)))
        # print('**********************l3:',l3)
        l4 = self.distance_list[0] / 1000  # 后左
        # l4 = self.distance_list[0] / 1000 - abs(2 / math.cos(self.alpha4 / math.pi * 180))  # 后左
        # if l4 >= l2:
        #     l4 = self.distance_list[0] / 1000 - abs(2 / math.cos(self.alpha4 / math.pi * 180))  # 后左
        # else:
        #     l4 = self.distance_list[0] / 1000
        # print('---------------', self.direction_ctl_mode)

        if self.direction_ctl_mode == 1:
            print("进入前进的自主航行代码")

            alpha1 = abs((math.atan((l1 - l2) / self.ac_d)))
            # alpha2 = (math.acos((self.ac_p - self.ac_d * math.cos(alpha1)) /
            #                     math.sqrt(((l2 + self.delta_l) * math.cos(alpha1)
            #                                - self.ac_r + self.ac_d * math.sin(alpha1)) ** 2
            #                               + (self.ac_p - self.ac_d * math.cos(alpha1)) ** 2)))
            # if l3 == 0:
            #     l3 = ((2 * 6) / math.cos(alpha1)) - l1 - self.delta_l  # self.real_r待定，真实半径，一起修改
            # else:
            #     l3 = self.distance_list[3] / 1000

            l3 = ((2 * self.ac_r) / math.cos(alpha1)) - l1 - self.delta_l  # self.real_r待定，真实半径，一起修改
            alpha2 = math.acos((self.ac_p - self.ac_d * math.cos(alpha1)) /
                               math.sqrt((-(l1 + self.delta_l) * math.cos(alpha1)
                                          + self.ac_r) ** 2
                                         + (self.ac_p - self.ac_d * math.cos(alpha1)) ** 2))

            beta = (math.acos(self.ac_p / math.sqrt(((l2 + self.delta_l) *
                                                     math.cos(alpha1) - self.ac_r) ** 2 + self.ac_p ** 2)))
            # if the value of l1 or l2 is wrong; turn to calculate with l3 and l4
            # ===============================================================================
            # if (l1 <= 0.004) or (l2 <= 0.004):
            #     alpha1 = abs((math.atan((l3 - l4) / self.ac_d)))
            #     alpha2 = (math.acos((self.ac_p - self.ac_d * math.cos(alpha1)) / math.sqrt(
            #         ((l4 + self.delta_l) * math.cos(alpha1) -
            #          self.ac_r + self.ac_d * math.sin(alpha1)) ** 2
            #         + (self.ac_p - self.ac_d * math.cos(alpha1)) ** 2)))
            #     beta = (math.acos(
            #         self.ac_p / math.sqrt(((l4 + self.delta_l) * math.cos(alpha1) -
            #                                self.ac_r) ** 2 + self.ac_p ** 2)))
            # ================================================================================

            if l1 > l3:  # AUV 在隧洞中轴线以左
                if l1 > l2:  # AUV艏向向左，需要向右偏转;
                    alpha3 = alpha1 + alpha2
                elif l1 < l2:  # AUV艏向向右，根据以下三种情形判断;
                    if alpha1 > beta:
                        alpha3 = -1 * abs(alpha1 - alpha2)
                    elif alpha1 < beta:
                        alpha3 = abs(alpha1 - alpha2)
                    else:
                        alpha3 = 0
                else:  # AUV 艏向在中轴线上
                    alpha3 = alpha2
            elif l1 < l3:  # AUV 在隧洞中轴线以右
                if l1 > l2:  # AUV艏向向左，根据以下三种情形判断;
                    if alpha1 > beta:
                        alpha3 = abs(alpha1 - alpha2)
                    elif alpha1 < beta:
                        alpha3 = -1 * abs(alpha1 - alpha2)
                    else:
                        alpha3 = 0
                elif l1 < l2:  # AUV艏向向右，需要向左偏转;
                    alpha3 = -1 * (alpha1 + alpha2)
                else:  # AUV 艏向在中轴线上
                    alpha3 = -1 * alpha2
            else:
                alpha3 = 0
            self.res_plan = alpha3 / math.pi * 180  # 规划角度结果
        elif self.direction_ctl_mode == 0:
            print('进入后退的自主航行')
            alpha1 = abs((math.atan((l1 - l2) / self.ac_d)))
            alpha2 = math.acos((self.ac_p - self.ac_d * math.cos(alpha1)) /
                               math.sqrt((-(l4 + self.delta_l) * math.cos(alpha1)
                                          + self.ac_r) ** 2
                                         + (self.ac_p - self.ac_d * math.cos(alpha1)) ** 2))
            beta = (math.acos(self.ac_p / math.sqrt(((l2 + self.delta_l) *
                                                     math.cos(alpha1) - self.ac_r) ** 2 + self.ac_p ** 2)))
            if l2 > l4:  # AUV 在隧洞中轴线以左
                if l1 < l2:  # AUV艏向向左，需要向右偏转;
                    # print('flag1')
                    alpha3 = -1 * (alpha1 + alpha2)
                elif l1 > l2:  # AUV艏向向右，根据以下三种情形判断;
                    if alpha1 > beta:
                        alpha3 = abs(alpha1 - alpha2)
                        # print('flag2')
                    elif alpha1 < beta:
                        alpha3 = -1 * abs(alpha1 - alpha2)
                        # print('flag3')
                    else:
                        alpha3 = 0
                        # print('flag4')
                else:  # AUV 艏向在中轴线上
                    alpha3 = -1 * alpha2
                    # print('flag5')
            elif l2 < l4:  # AUV 在隧洞中轴线以右
                if l1 < l2:  # AUV艏向向左，根据以下三种情形判断;
                    if alpha1 > beta:
                        alpha3 = -1 * abs(alpha1 - alpha2)
                        # print('flag6')
                    elif alpha1 < beta:
                        alpha3 = abs(alpha1 - alpha2)
                        # print('flag7')
                    else:
                        alpha3 = 0
                        # print('flag8')
                elif l1 > l2:  # AUV艏向向右，需要向左偏转;
                    alpha3 = (alpha1 + alpha2)
                    # print('flag9')
                    print('alpha2:', alpha2)
                    print(alpha2 / math.pi * 180)
                    print(alpha1 / math.pi * 180)
                else:  # AUV 艏向在中轴线上
                    alpha3 = alpha2
                    # print('flag10')
            else:
                alpha3 = 0
                # print('flag11')
            self.res_plan = alpha3 / math.pi * 180  # 规划角度结果
        if abs(self.res_plan) <= 10:
            self.exp_course = round(self.yaw + self.res_plan, 3)
        else:
            self.exp_course = round(self.yaw + self.res_plan, 3)
        if self.exp_course < 0:
            self.exp_course = 360 + self.exp_course
        elif self.exp_course > 360:
            self.exp_course = self.exp_course - 360
        # 添加左右的推力保护，当机器人靠近一端过近时，将其向中轴线推进；
        # self.Fy = -80 * math.sin((1 - (l1 + l2) / 2 * self.ac_r) * math.pi / 2)
        # print(self.Fy)
        # 剔除野值
        if abs(self.exp_course - self.last_exp_course) >= 30:
            self.exp_course = self.last_exp_course

        # if l1 == 0 or l2 == 0:
        #     self.exp_course = self.positive_angle
        self.last_exp_course = self.exp_course
        # 惯性环节
        self.set_inertia_time(1, 0.4, self.exp_course)
        self.exp_course = self.system_output
        print('当前航向：', round(self.yaw))
        print('规划角度：', round(self.res_plan))
        print('期望航向：', self.exp_course)
        return self.exp_course

        # 设置一阶惯性环节系统  其中InertiaTime为惯性时间常数
    def set_inertia_time(self, inertia_time, sample_time, output):
        self.system_output = (inertia_time * self.last_system_output
                              + sample_time * (output-self.last_system_output))
        # self.system_output = (inertia_time * self.last_system_output
        #                       + sample_time * output) / (sample_time + inertia_time)
        self.last_system_output = self.system_output

        # 设置一阶惯性环节系统  其中InertiaTime为惯性时间常数

    def roll_set_inertia_time(self, inertia_time, sample_time, output):
        self.system_roll_output = (inertia_time * self.last_roll_system_output
                              + sample_time * (output - self.last_roll_system_output))
        # self.system_output = (inertia_time * self.last_system_output
        #                       + sample_time * output) / (sample_time + inertia_time)
        self.last_roll_system_output = self.system_roll_output


    # 9 PID参数
    def set_pid_parameters_function(self, msg):
        # 控制模式
        self.set_pid_ctl_mode = int(msg[1])
        # 定向 定深 定速
        self.set_course_depth_speed_mode = int(msg[2])
        kp = float(msg[3])  # P
        ki = float(msg[3])  # I
        kd = float(msg[3])  # D
        if self.set_course_depth_speed_mode == 0:  # 定向PID
            if self.set_pid_ctl_mode == 0:  # 欠驱动
                self.under_course_pid_controller.setKp(kp)
                self.under_course_pid_controller.setKi(ki)
                self.under_course_pid_controller.setKd(kd)
            elif self.set_pid_ctl_mode == 1:  # 全驱动
                self.full_course_pid_controller.setKp(kp)
                self.full_course_pid_controller.setKi(ki)
                self.full_course_pid_controller.setKd(kd)
        if self.set_course_depth_speed_mode == 1:  # 定深PID
            if self.set_pid_ctl_mode == 0:  # 欠驱动
                # self.under_depth_pid_controller.setKp(kp)
                # self.under_depth_pid_controller.setKi(ki)
                # self.under_depth_pid_controller.setKd(kd)

                #￥￥￥￥￥￥￥￥￥￥￥￥￥反步法￥￥￥￥￥￥￥￥￥￥￥
                self.under_depth_backstep_controller.setK1(kp)
                self.under_depth_backstep_controller.setK2(ki)
                self.under_depth_backstep_controller.setK3(kd)
                # ￥￥￥￥￥￥￥￥￥￥￥￥￥反步法￥￥￥￥￥￥￥￥￥￥￥

            elif self.set_pid_ctl_mode == 1:  # 全驱动
                self.full_depth_pid_controller.setKp(kp)
                self.full_depth_pid_controller.setKi(ki)
                self.full_depth_pid_controller.setKd(kd)
        if self.set_course_depth_speed_mode == 2:  # 定速PID
            pass

    # 10 数据保存
    def save_data_function(self, msg):
        print("进入数据保存函数")
        save_status = int(msg[1])
        # print(save_status, type(save_status))
        current_time = time.strftime('%Y%m%d_%H-%M-%S', time.localtime())
        if save_status:
            print("进入数据保存打开判断")
            pwd = f'/home/nvidia/disk/sensor_save_data/{current_time}_sensor_data{self.file_num}.csv'
            self.save_sensor_data_file = open(pwd, 'a', encoding='utf-8-sig', newline="")
            self.file_writer = csv.writer(self.save_sensor_data_file)
            self.file_writer.writerow(['时间', '功能码', '当前深度',
                                       '前左测距仪', '前右测距仪', '后左测距仪', '后右测距仪',
                                       '惯导角速度x', '惯导角速度y', '惯导角速度z',
                                       '纵倾角pitch', '横滚角roll', '航向角yaw',
                                       'east_vel', 'north-vel', '惯导速度ins_vel',
                                       '经度', '纬度', '惯导状态ins_status', 'DVL_status',
                                       'X向速度', 'Y向速度', 'Z向速度', 'DVL_Voyage',
                                       'DVL_Depth', 'DVL_Height', 'DVLToInsAngeleX',
                                       'DVLToInsAngeleY', 'DVLToInsAngeleZ',
                                       '前侧转速', '前垂转速', '后侧转速', '后垂转速',
                                       '主推转速', '左舵', '右舵', '上舵', '下舵', '自主航行期望航向',
                                       'Fx', 'Fy', 'Fz', 'Fn', 'Fm', 'e', 'ec', 'fuzzy_res_kp', ])
            self.save_data_flag = True
            self.file_num += 1
        else:
            self.save_data_flag = False
            if self.save_sensor_data_file is not None and not self.save_sensor_data_file.closed:
                self.save_sensor_data_file.close()

    def read_sensor_data(self, t):
        """
        从主控板读取传感器数据，赋值给全局变量
        :return:
        """

        while True:
            self.read_date_lock.acquire()  # 上锁
            modbus_code = self.Mc.encode(11, '4', 144, 28)
            self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
            #  加入了deepcopy(),防止被更改
            self.recv_modbus_code = copy.deepcopy(self.socket_main_board.recvfrom(1024)[0])
            # print("￥￥" * 16)
            # print(f'读到的传感器数据：\n {self.recv_modbus_code}')
            # print("￥￥" * 16)
            if self.recv_modbus_code != b'' and struct.unpack("!H", self.recv_modbus_code[:2])[0] == 11:
                recv_msg = self.Mc.decode(self.recv_modbus_code)[6:]  # 使用的是self.Mc
                self.depth_data = round(recv_msg[0] / 100, 2)  # bar to m
                self.distance_list = recv_msg[1:5]
                self.angleVel_list = recv_msg[5:8]
                self.angle_list = recv_msg[8:11]
                # 角度从-180~180转变为0~360
                # ==========================================================
                if self.angle_list[2] <= 0:
                    self.yaw = self.angle_list[2] = \
                        round((-1 * self.angle_list[2]) / 100, 2)
                else:
                    self.yaw = self.angle_list[2] = \
                        round((36000 - self.angle_list[2]) / 100, 2)
                # ==========================================================
                self.InsVel_list = recv_msg[11:14]
                self.latitude = struct.unpack("!i", self.recv_modbus_code[37:41])[0]
                self.longitude = struct.unpack("!i", self.recv_modbus_code[41:45])[0]
                self.Ins_flag = struct.unpack("B", self.recv_modbus_code[45:46])[0]
                self.Dvl_flag = struct.unpack("B", self.recv_modbus_code[46:47])[0]
                self.DvlVel_list = recv_msg[19:22]
                self.Dvl_voyage = round(recv_msg[22] / 100, 2)
                self.Dvl_depth = round(recv_msg[23] / 100, 2)
                self.Dvl_height = round(recv_msg[24] / 100, 2)
                self.DvlToInsAngle_list = recv_msg[25:28]
            else:
                print("数据不是传感器反馈数据或为空")
            self.recv_modbus_code = b''  # 置空缓冲区
            # print("缓冲区已置空！！！")
            self.read_date_lock.release()  # 解锁
            time.sleep(t)

    def upload_sensor_data(self):
        """
        将传感器数据上传
        :return:
        """
        # depth sensor
        depth = self.depth_data
        # sonar-ranging sensor
        f_l_distance = round(self.distance_list[3] / 1000, 3)
        f_r_distance = round(self.distance_list[2] / 1000, 3)
        b_l_distance = round(self.distance_list[0] / 1000, 3)
        b_r_distance = round(self.distance_list[1] / 1000, 3)
        # INS
        ins_angle_vel_x = round(self.angleVel_list[0] / 100, 2)
        ins_angle_vel_y = round(self.angleVel_list[1] / 100, 2)
        ins_angle_vel_z = round(self.angleVel_list[2] / 100, 2)
        ins_pitch = round(self.angle_list[0] / 100, 2)
        ins_roll = round(self.angle_list[1] / 100, 2)
        ins_yaw = self.angle_list[2]
        ins_east_vel = round(self.InsVel_list[0] / 1000, 3)
        ins_north_vel = round(self.InsVel_list[1] / 1000, 3)
        ins_ins_vel = round(self.InsVel_list[2] / 1000, 3)
        ins_longitude = round(self.longitude / 10000000, 7)
        ins_latitude = round(self.latitude / 10000000, 7)
        ins_status = self.Ins_flag
        dvl_status = self.Dvl_flag
        # DVL
        # dvl_longitudinal_vel = round(self.DvlVel_list[0] / 1000, 3)
        # dvl_lateral_vel = round(self.DvlVel_list[1] / 1000, 3)
        # dvl_ground_vel = round(self.DvlVel_list[2] / 1000, 3)

        # 注意DVL的坐标系偏离了45度，详见DVL说明书page29
        # 所以auv的前向速度，需要做处理
        # 以下三个为dvl直接读到的数据
        dvl_y_vel = round(self.DvlVel_list[0] / 1000, 3)  # 这个y对应于DVL说明书上的坐标定义
        dvl_x_vel = round(self.DvlVel_list[1] / 1000, 3)  # 这个x对应于DVL说明书上的坐标定义
        dvl_ground_vel = round(self.DvlVel_list[2] / 1000, 3)  # z方向不改动
        # 做处理，得到AUV真实的前向和横向速度
        auv_x_vel = dvl_y_vel * math.cos(math.pi / 4) - dvl_x_vel * math.cos(math.pi / 4)
        auv_y_vel = dvl_y_vel * math.cos(math.pi / 4) + dvl_x_vel * math.cos(math.pi / 4)

        dvl_voyage = self.Dvl_voyage
        dvl_depth = self.Dvl_depth
        dvl_height = self.Dvl_height
        dvl_dvl2ins_angle_x = round(self.DvlToInsAngle_list[0] / 100, 2)
        dvl_dvl2ins_angle_y = round(self.DvlToInsAngle_list[1] / 100, 2)
        dvl_dvl2ins_angle_z = round(self.DvlToInsAngle_list[2] / 100, 2)
        # **************************李昌添加******************************
        # 当前时间
        current_time = time.strftime('%H:%M:%S', time.localtime())
        # 深度计
        send_time = f'{str(current_time)},'  # 注意末尾要加逗号！！ , ！！
        send_msg1 = f'11,{depth},'
        # 测距仪4个数据
        send_msg2 = f'{f_l_distance},{f_r_distance},' \
                    f'{b_l_distance},{b_r_distance},'
        # 惯导
        send_msg3 = f'{ins_angle_vel_x},{ins_angle_vel_y},{ins_angle_vel_z},' \
                    f'{ins_pitch},{ins_roll},{ins_yaw},' \
                    f'{ins_east_vel},{ins_north_vel},{ins_ins_vel},' \
                    f'{ins_longitude},{ins_latitude},{ins_status},'
        send_msg4 = f'{dvl_status},{auv_x_vel},{auv_y_vel},' \
                    f'{dvl_ground_vel},{dvl_voyage},{dvl_depth},{dvl_height},' \
                    f'{ dvl_dvl2ins_angle_x},{dvl_dvl2ins_angle_y},{dvl_dvl2ins_angle_z},'
        # 定时下发的9个数据
        send_msg5 = f'{self.f_h_thruster},{self.f_v_thruster},{self.b_h_thruster},' \
                    f'{self.b_v_thruster},{self.main_thruster},{self.left_steer/10},' \
                    f'{self.right_steer/10},{self.up_steer/10},{self.down_steer/10},'
        send_msg6 = f'{self.exp_course},'
        send_msg7 = f'{self.Fx},{self.Fy},{self.Fz},{self.Fn},{self.Fm},'
        # send_msg8 = f'{self.under_depth_pid_controller.e_pitch_save},' \
        #             f'{self.under_depth_pid_controller.ec_pitch_save},' \
        #             f'{self.under_depth_pid_controller.fuzzy_res_kp},'  # 欠驱动定深中内环俯仰的e、ec与模糊PID输出的KP
        # 合并
        send_msg = send_msg1 + send_msg2 + send_msg3 + send_msg4 + \
                   send_msg5 + send_msg6 + send_msg7
        # 保存传感器数据
        if self.save_data_flag:
            # 将按逗号分隔后的数据写入已经创建的文件
            send_msg_save_data = send_time + send_msg
            write_data = send_msg_save_data.split(',')
            self.file_writer.writerow(write_data)
            # # ***********************采用pandas库写入表格*****************
            # df = pd.DataFrame([write_data])
            # df.to_csv(self.pwd,mode='a', header=False, index=False)
            # # ***********************采用pandas库写入表格*****************
        # 编码并使用UDP上传到上位机
        udp_code = send_msg.encode()  # encode()函数是Modbus.py里的
        self.send_udp_code(udp_code, self.up_addr, self.socket_host_computer)
        #发送给声纳
        self.send_udp_code(udp_code,self.sonar_add,self.socket_sonar)

    # 12 声呐始能
    def sonar_enable_function(self, msg):
        """
        声呐始能函数
        """
        sonar_enable_flag = int(msg[1])
        modbus_code = self.Mc.encode(12, '5', 6, sonar_enable_flag)
        self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        print("Sonar enable !")

    # 主控板重启
    def main_board_restart_function(self, msg):
        """
        发1重启
        """
        main_board_restart_flag = int(msg[1])
        modbus_code = self.Mc.encode(5, '5', 30, 1, main_board_restart_flag)  # 第一位事物标识符是乱写的，没关系
        self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        print("Main board has already restarted !")

    # CAN重启
    def can_restart_function(self, msg):
        """
        发1重启
        """
        can_restart_flag = int(msg[1])
        modbus_code = self.Mc.encode(5, '5', 29, 1, can_restart_flag)  # 第一位事物标识符是乱写的，没关系
        self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        print("CAN has already restarted !")

    def send_udp_code(self, modbus_code, target_addr, target_socket):
        """
        将编码后的数据发送到目标IP和port
        :param modbus_code: 编码后的需要发送的内容
        :param target_addr: 目标IP和Port
        :param target_socket: 哪一个socket
        :return:
        """
        target_socket.sendto(modbus_code, target_addr)

    def hybrid_drive_data(self):
        """
        发送主推侧推舵机数据，注意顺序
        """
        under_drive_ctl_list = [self.f_h_thruster, self.f_v_thruster,
                                self.b_h_thruster, self.b_v_thruster,
                                self.main_thruster, int(self.left_steer),
                                int(self.right_steer), int(self.up_steer),
                                int(self.down_steer)]
        print("=="*30)
        print(f'前侧:{self.f_h_thruster},前垂:{self.f_v_thruster},'
              f'后侧:{self.b_h_thruster},后垂:{self.b_v_thruster},主推:{self.main_thruster},'
              f'\n左舵:{self.left_steer/10},右舵:{self.right_steer/10},上舵:{self.up_steer/10},'
              f'下舵:{self.down_steer/10}')
        print("==" * 30)
        # modbus_code = self.Mc.encode(6, '16', 64, 9, under_drive_ctl_list)
        # self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        if self.wireless_flag == 0:  # 0-有线模式
            if self.safe_flag < 6:
                print('=============flag小于6================')
                modbus_code = self.Mc.encode(6, '16', 64, 9, under_drive_ctl_list)
                self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
                self.safe_flag += 1
            elif self.safe_flag >= 6:
                print('--------------flag大于6-----------------')
                close_init_ctl_code = self.Mc.encode(1, '15', 1, 3, [0, 0, 0])
                self.send_udp_code(close_init_ctl_code, self.down_addr, self.socket_main_board)
                self.hybrid_drive_data_initialize()  # 使能关闭后全局变量清0
        else:
            modbus_code = self.Mc.encode(6, '16', 64, 9, under_drive_ctl_list)
            self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
            # self.safe_flag += 1


    def hybrid_drive_data_initialize(self):
        self.f_h_thruster = 0
        self.f_v_thruster = 0
        self.b_h_thruster = 0
        self.b_v_thruster = 0
        self.main_thruster = 0
        self.left_steer = 0
        self.right_steer = 0
        self.up_steer = 0
        self.down_steer = 0
        self.Fx = 0
        self.Fy = 0
        self.Fz = 0
        self.Fn = 0
        self.Fm = 0

    def recv_udp_msg(self, recv_socket, num=1024):
        """
        接收消息函数
        :param recv_socket:
        :param num:
        :return: 消息列表
        """
        recv_msg = recv_socket.recvfrom(num)[0].decode()
        recv_msg_list = recv_msg.split(',')  # 按逗号分隔
        return recv_msg_list
        # f_r_distance = self.distance_list[2] / 1000  # 前右
        # b_r_distance = self.distance_list[1] / 1000  # 后右
        # f_l_distance = self.distance_list[3] / 1000  # 前左
        # b_l_distance = self.distance_list[0] / 1000  # 后左
        # set_distance = 0.7
        # # if f_r_distance <= set_distance :
        # if f_r_distance <= set_distance or b_r_distance <= set_distance \
        #         or f_l_distance <= set_distance or b_l_distance <= set_distance:
        #     # 启动防撞
        #     main_thrust = 0
        #     send_list = [main_thrust]
        #     modbus_code = self.Mc.encode(7, '16', 68, 1, send_list)
        #     self.send_udp_code(modbus_code, self.down_addr, self.socket_main_board)
        #     
        #     print("防撞启动，主推已置0")
        # else:


    def auto_start_function(self, msg):

        self.wireless_flag = 1  # 无线模式
        #欠驱动使能
        print('欠驱动始能')
        close_init_ctl_code = self.Mc.encode(1, '15', 1, 3, [1, 0, 1])
        self.send_udp_code(close_init_ctl_code, self.down_addr, self.socket_main_board)
        self.save_data_flag = 1
        self.save_data_function([1, 1])  # 保存数据
        self.exp_depth = float(msg[1])
        self.exp_course = float(msg[2])
        main_thruster = int(msg[3])
        self.run_time_counter_cal = int(msg[4]) * 5
        # print("航行时间计数非全局变量：",run_time_counter)
        # print('航行时间计数全局变量：', self.run_time_counter)
        # run_time_counter = 0  # 欠驱动定深定向运行时间计数

        # 欠驱动定深开启
        self.depth_ctl_mode = 0  # 1-全驱动;0-欠驱动
        # 定深PID状态-开启or关闭
        self.depth_pid_status = 1  # 1-开启;0-关闭
        # 期望深度
        # self.exp_depth = float(msg[3])
        if self.depth_pid_status == 1:
            if not self.depth_pid_flag:  # 防止多次启动定时器
                self.depth_pid_timer.start()
                self.depth_pid_flag = True
                self.pitch_pid_flag = True

        print('定深flag：', self.depth_pid_flag)
        print('俯仰flag：', self.pitch_pid_flag)
        # 欠驱动定向启动
        self.course_ctl_mode = 0  # 1-全驱动;0-欠驱动
        # 定深PID状态-开启or关闭
        self.course_pid_status = 1  # 1-开启;0-关闭
        self.under_auto_start_flag = 0  # 0-开始，1-关闭
        # 期望角度
        if self.course_pid_status == 1:
            if not self.course_pid_flag:
                # 定时器启动，开始每0.1秒执行 self.course_pid_calculate 一次
                self.course_pid_timer.start()
                self.course_pid_flag = True
                self.roll_pid_flag = True

        #主推开始给转速
        self.main_thruster = main_thruster
        self.wireless_flag = 1  # 无线模式


        print(f'定向flag：{self.course_pid_flag}')

        print(f'横滚flag：{self.roll_pid_flag}')


    def full_drive_protection(self):
        if self.Fx <= -400:
            self.Fx = -400
        elif self.Fx >= 400:
            self.Fx = 400
        if self.Fy <= -130:
            self.Fy = -130
        elif self.Fy >= 130:
            self.Fy = 130
        if self.Fz <= -130:
            self.Fz = -130
        elif self.Fz >= 130:
            self.Fz = 130
        if self.Fn <= -117:
            self.Fn = -117
        elif self.Fn >= 117:
            self.Fn = 117
        if self.Fm <= -130:
            self.Fm = -130
        elif self.Fm >= 130:
            self.Fm = 130

    def under_drive_protection(self):
        if self.main_thruster >= 2000:
            self.main_thruster = 2000
        elif self.main_thruster <= -2000:
            self.main_thruster = -2000
        if self.left_steer >= 450:
            self.left_steer = 450
        elif self.left_steer <= -450:
            self.left_steer = -450
        if self.right_steer >= 450:
            self.right_steer = 450
        elif self.right_steer <= -450:
            self.right_steer = -450
        if self.up_steer >= 450:
            self.up_steer = 450
        elif self.up_steer <= -450:
            self.up_steer = -450
        if self.down_steer >= 450:
            self.down_steer = 450
        elif self.down_steer <= -450:
            self.down_steer = -450

    def thrust_distribute_assign(self, fx, fy, fz, fn, fm):
        """
        推力分配，并下发。为什么要单独写一个函数，不直接用self.Td.thrust_distribute？
        因为每次分配完都要下发
        :param fx:
        :param fy:
        :param fz:
        :param fn:
        :param fm:
        :return:
        """
        speed_list = self.Td.thrust_distribute(
            fx, fy, fz, fn, fm)
        # 推力分配返回的速度列表顺序为 主推 前侧 后侧 前垂 后垂
        print("推力分配打印：", speed_list)
        # 会出现IndexError，进而导致功能码决定函数不能接收上位机数据
        # 解决办法：如果遇到索引错误，pass掉
        try:
            self.main_thruster = speed_list[0]
            self.f_h_thruster = speed_list[1]
            self.b_h_thruster = speed_list[2]
            self.f_v_thruster = speed_list[3]
            self.b_v_thruster = speed_list[4]
        except IndexError:
            print("索引错误出现￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥")
            pass


if __name__ == '__main__':
    try:
        print("The AUV is Starting!")
        AC = AuvControl()
    except KeyboardInterrupt as e:
        print("The AUV is Shutting Down!")
        print(e)
