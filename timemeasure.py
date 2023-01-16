import time
from udpsocket import *
from tlvmessage import *
import numpy as np


class TimeMeasureClient:
    """
    只对UDP协议有效
    """

    def __init__(self, server_addr: tuple, sock: UDPSocket):
        self.server_address = server_addr
        self.sock = sock
        self.delta_t = 0
        self.synchronize_finish = False

    def synchronize(self):
        temp_t = np.array([0] * 10)
        for i in range(10):
            t1 = int(time.time() * 1000000)  # 转化为us单位，并取整
            t1_tlv = TLVMessage(b'\x01' + t1.to_bytes(7, 'big'), SEND, new_msg=True,  # 7字节，大端格式
                                config=(b'\x00\x00\x00\x70',  # aid
                                        b'\x00\x00\x00\x01',  # traffic_period
                                        b'\x00\x00\x00\x7f',  # priority
                                        b'\x00\x00\xff\xff'))  # traffic_id
            self.sock.sendto(t1_tlv.get_tlv_raw_message(), self.server_address, False)
            t2_t3_raw = self.sock.recv(recv_len=2048)
            t4 = int(time.time() * 1000000)
            t2_t3_tlv = TLVMessage(t2_t3_raw, RECEIVE)
            t2_t3_bytes = t2_t3_tlv.get_payload()
            t2 = int.from_bytes(t2_t3_bytes[1:8], 'big')
            t3 = int.from_bytes(t2_t3_bytes[8:15], 'big')
            temp_t[i] = (t2 - t1 + t3 - t4) / 2
            time.sleep(0.2)
        print(temp_t)
        temp_t[np.argmax(temp_t)] = 0  # 删除最大值和最小值
        temp_t[np.argmin(temp_t)] = 0
        self.delta_t = np.sum(temp_t) / 8
        print("delta_t: %d " % self.delta_t)
        self.synchronize_finish = True

    def get_syn_time(self) -> int:
        if self.synchronize_finish:
            return int(time.time() * 1000000 + self.delta_t)
        else:
            self.synchronize()
            return int(time.time() * 1000000 + self.delta_t)


class TimeMeasureServer:
    def __init__(self, client_addr: tuple, sock: UDPSocket):
        self.client_address = client_addr
        self.sock = sock
        self.delta_t = 0

    def response(self):
        for _ in range(10):
            while True:
                t1_raw = self.sock.recv(recv_len=2048)
                t2 = int(time.time() * 1000000)
                t1_tlv = TLVMessage(t1_raw, RECEIVE)
                t1_bytes = t1_tlv.get_payload()
                if t1_bytes[0:1] == b'\x01':
                    t3 = int(time.time() * 1000000)
                    t2_t3_tlv = TLVMessage(b'\x02' + t2.to_bytes(7, 'big') + t3.to_bytes(7, 'big'),  # 7字节，大端格式
                                           SEND, new_msg=True,
                                           config=(b'\x00\x00\x00\x70',  # aid
                                                   b'\x00\x00\x00\x01',  # traffic_period
                                                   b'\x00\x00\x00\x7f',  # priority
                                                   b'\x00\x00\xff\xff'))  # traffic_id
                    self.sock.sendto(t2_t3_tlv.get_tlv_raw_message(), self.client_address, False)
                    break
