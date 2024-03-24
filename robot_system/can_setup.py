import time
import can
import numpy as np

CAN_ID = [0x401, 0x402, 0x403, 0x404]
CAN_ID_feedback = [897, 898, 899, 890]


class setup():
    def __init__(self):

        self.can_open = False
        self.motor_velocity_rpm = np.zeros(4).astype(int)
        self.motor_velocity_position = np.zeros(4).astype(int)

        try:
            self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
            self.can_open = True
        except OSError:
            self.can_open = False
            exit()

    def send_data_can(self, data):
        for i in range(len(data)):
            self.bus.send(can.Message(arbitration_id=CAN_ID[i], data=[0x0f, 0x00, 0x03, 
                                       int(hex(data[i] & 0xff), 16), 
                                       int(hex(data[i] >> 8 & 0xff), 16), 
                                       int(hex(data[i] >> 16 & 0xff), 16), 
                                       int(hex(data[i] >> 32 & 0xff), 16), 0x00], is_extended_id=False))
            if self.can_open == True:
                msg_recv = self.bus.recv()
                if msg_recv.arbitration_id == CAN_ID_feedback[i]:
                    a = msg_recv.data[msg_recv.dlc - 6]
                    b = msg_recv.data[msg_recv.dlc - 5] * 0x100
                    c = msg_recv.data[msg_recv.dlc - 4] * 0x10000
                    d = msg_recv.data[msg_recv.dlc - 3] * 0x1000000
                    self.motor_velocity_position[i] = a + b + c + d
        
        print(self.motor_velocity_position)

    def read_position_data_can(self):
        if self.can_open == True:
            self.bus.send(can.Message(arbitration_id=CAN_ID[0], data=[0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False))
            msg_recv = self.bus.recv()
            if msg_recv.arbitration_id == 1410:
                a = msg_recv.data[msg_recv.dlc - 4]
                b = msg_recv.data[msg_recv.dlc - 3] * 0x100
                c = msg_recv.data[msg_recv.dlc - 2] * 0x10000
                d = msg_recv.data[msg_recv.dlc - 1] * 0x1000000
                data = a + b + c + d
                print(data)
