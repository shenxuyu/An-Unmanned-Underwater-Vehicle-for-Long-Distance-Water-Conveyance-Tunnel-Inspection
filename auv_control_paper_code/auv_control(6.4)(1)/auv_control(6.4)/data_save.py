# coding:utf-8
import csv
import time


current_time = time.strftime('%H:%M:%S', time.localtime())
current_time_1 = time.strftime('%Y%m%d_%H_%M_%S', time.localtime())
data = '11, 2 ,3.2, 3.0, 4.2, 4.5'
big_data = str(current_time) + ',' + data
#  1.创建文件对象
file_name = ('{}_date_save.csv'.format(current_time_1))
f = open(file_name, 'w', encoding='utf-8-sig', newline="")

#  2.基于文件对象构建csv写入对象
csv_write = csv.writer(f)

#  3.构建列表头
# csv_write.writerow(['深度值', '前左距离', '前右距离', '后左距离', '后右距离',
#                     'X轴角速度', 'y轴角速度', 'z轴角速度', 'pitch', 'roll', 'yaw',
#                     '东向速度', '北向速度', '天向速度', '经度', '纬度', '惯导状态',
#                     'DVL状态', 'DVL纵向速度', 'DVL横向速度', 'DVL地向速度',
#                     'DVL航程', 'DVL深度', 'DVL高度', 'DVL与惯导间绕X角度',
#                     'DVL与惯导间绕Y角度', 'DVL与惯导间绕Z角度'])
csv_write.writerow(['时间', '功能码', '深度值', '前左距离', '前右距离', '后左距离', '后右距离'])
#  4.写入csv文件
write_data = big_data.split(',')
csv_write.writerow(write_data)
#  5.关闭文件
f.close()
