#!/usr/bin/env python3
#coding:utf-8

import struct

class Modbus_Code():
    def __init__(self):
        self.mbap = b''
        self.pdu = b''
        self.mbap_msg = []
        self.pdu_msg = []
        
    def MBAP_encode(self,transFlag,PDU):
        transflag = struct.pack("!H",transFlag)
        protoflag = b'\x00\x00'
        length = struct.pack("!H",(1+len(PDU)))
        unitflag = b'\x01'
        self.mbap = transflag + protoflag + length + unitflag
        return self.mbap

    def PDU_Read_encode(self,func,addr,num):
        func_List = {'1':b'\x01',
                     '2':b'\x02',
                     '3':b'\x03',
                     '4':b'\x04'}
        func_code = func_List[func]
        start_addr = struct.pack('!H',addr)
        register_num = struct.pack('!H',num)
        self.pdu = func_code + start_addr + register_num
        # print(len(pdu))
        return self.pdu

    def PDU_Write_encode(self,func,addr,num,data=None):
        func_List = {'5':b'\x05',
                     '6':b'\x06',
                     '15':b'\x0f',
                     '16':b'\x10'}
        func_code = func_List[func]
        start_addr = struct.pack('!H',addr)
        # register_num = struct.pack('!H',num)
        register_num = struct.pack('!h', num)
        if int(func) <= 6:
            self.pdu = func_code + start_addr + register_num
        else:
            data_length = struct.pack('B',(2*num))
            data_code = b''
            # data is a list of int
            for i in range(len(data)):
                item_code = struct.pack('!h',data[i])
                data_code += item_code
            self.pdu = func_code + start_addr + register_num + data_length + data_code
        # print(pdu)
        return self.pdu
    
    def encode(self,transFlag,func,addr,num,data=None):
        if int(func) <= 4:
            self.pdu = self.PDU_Read_encode(func,addr,num)
        else:
            self.pdu = self.PDU_Write_encode(func,addr,num,data)
        self.mbap = self.MBAP_encode(transFlag,self.pdu)
        modbus_encode = self.mbap + self.pdu
        return modbus_encode

    def MBAP_decode(self,msg):
        self.mbap_msg.clear()
        transflag = struct.unpack("!H",msg[:2])[0]
        protoflag = struct.unpack("!H",msg[2:4])[0]
        length = struct.unpack("!H",msg[4:6])[0]
        unitflag = struct.unpack("B",msg[6:7])[0]
        self.mbap_msg.append(transflag)
        self.mbap_msg.append(protoflag)
        self.mbap_msg.append(length)
        self.mbap_msg.append(unitflag)
        return self.mbap_msg

    def PDU_Read_decode(self,msg):
        self.pdu_msg.clear()
        func = struct.unpack("B",msg[7:8])[0]
        data_length = struct.unpack("B",msg[8:9])[0]
        data = []
        for i in range(int(data_length/2)):
            data.append(struct.unpack("!h",msg[9+2*i:9+2*i+2])[0])
        self.pdu_msg.append(func)
        self.pdu_msg.append(data_length)
        for i in range(len(data)):
            self.pdu_msg.append(data[i])
        return self.pdu_msg

    def PDU_Write_decode(self,msg):
        self.pdu_msg.clear()
        func = struct.unpack("B",msg[7:8])[0]
        start_addr = struct.unpack("!H",msg[8:10])[0]
        if int(func) <= 6:
            register_data = struct.unpack("!H",msg[10:])[0]
            self.pdu_msg.append(func)
            self.pdu_msg.append(start_addr)
            self.pdu_msg.append(register_data)
        else:
            register_num = struct.unpack("!H",msg[10:])[0]
            # data_length = struct.unpack("B",msg[12:13])[0]
            # data = []
            # for i in range(int(data_length/2)):
            #     data.append(struct.unpack("!h",msg[13+2*i:13+2*i+2])[0])
            self.pdu_msg.append(func)
            self.pdu_msg.append(start_addr)
            self.pdu_msg.append(register_num)
            # self.pdu_msg.append(data_length)
            # for i in range(len(data)):
            #     self.pdu_msg.append(data[i])
        return self.pdu_msg
    
    def decode(self,msg):
        self.mbap_msg = self.MBAP_decode(msg)
        func = struct.unpack("B", msg[7:8])[0]
        if func <= 4:
            self.pdu_msg = self.PDU_Read_decode(msg)
        else:
            self.pdu_msg = self.PDU_Write_decode(msg)
        modbus_decode = self.mbap_msg + self.pdu_msg
        return modbus_decode

if __name__ == '__main__':
    MBC = Modbus_Code()
    # modbus_encode = MBC.encode(3,'1',4,1)
    # modbus_encode = MBC.encode(8,'6',80,30)   # write left brightness
    modbus_encode = MBC.encode(11, '4', 144, 28)
    print(modbus_encode)
    recv_msg = b'\x00\x0b\x00\x00\x00\x06\x01\x04\x00\x90\x00\x1c'

    # recv_msg = b'\x00\x06\x00\x00\x00\x06\x01\x10\x00@\x00\x05'
    modbus_decode = MBC.decode(recv_msg)
    print(modbus_decode)