# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import PyQt5
# from PyQt5.QtWidgets import QApplication
# from PyQt5.QtCore import QT_VERSION_STR
# from PyQt5.Qt import PYQT_VERSION_STR
# from sip import SIP_VERSION_STR
import sys
# import sip
from PyQt5 import QtWidgets, QtCore
from udpsocket import UDPSocket
from tlvmessage import *
from timemeasure import *


def tlv_message_sending_test():
    remote_host = '192.168.62.199'
    remote_port = 30299

    sock = UDPSocket()
    tlv_message = TLVMessage(b'\x88\x88\x88\x88', SEND, new_msg=True, config=(b'\x00\x00\x00\x70',
                                                                              b'\x00\x00\x00\x01',
                                                                              b'\x00\x00\x00\x7f',
                                                                              b'\x00\x00\xff\xff'))
    print(tlv_message.get_tlv_raw_message())
    print(sock.sendto(tlv_message.get_tlv_raw_message(), (remote_host, remote_port), False))
    sock.close()


def tlv_message_recv_test():
    remote_host = '192.168.62.199'
    remote_port = 30299

    sock = UDPSocket()
    rec = sock.recv(2048)
    print(b'Bytes data:\n' + rec + b'\n')
    tlv_rec = TLVMessage(rec, RECEIVE)
    print(tlv_rec)
    sock.close()


def time_measure_client_test():
    remote_host = '192.168.62.199'
    remote_port = 30299
    sock = UDPSocket()
    time_client = TimeMeasureClient((remote_host, remote_port), sock)
    time_client.synchronize()
    # time.sleep(1)
    for _ in range(100):
        current_time = time_client.get_syn_time()
        # print('current_time: %d ' % current_time)
        current_time_tlv = TLVMessage(b'\x03' + current_time.to_bytes(7, 'big'), SEND, new_msg=True,  # 7字节，大端格式
                                      config=(b'\x00\x00\x00\x70',  # aid
                                              b'\x00\x00\x00\x01',  # traffic_period
                                              b'\x00\x00\x00\x7f',  # priority
                                              b'\x00\x00\xff\xff'))  # traffic_id
        sock.sendto(current_time_tlv.get_tlv_raw_message(), (remote_host, remote_port), False)
        time.sleep(0.2)


def time_measure_server_test():
    remote_host = '192.168.62.199'
    remote_port = 30299
    sock = UDPSocket()
    time_server = TimeMeasureServer((remote_host, remote_port), sock)
    time_server.response()
    for _ in range(100):
        client_time_raw = sock.recv(recv_len=2048)
        server_time = int(time.time() * 1000000)
        client_time_tlv = TLVMessage(client_time_raw, RECEIVE)
        client_time_bytes = client_time_tlv.get_payload()
        client_time = int.from_bytes(client_time_bytes[1:8], 'big')
        print('Transmission delay: %f ms' % ((server_time - client_time)/1000))
        # time.sleep(0.2)


if __name__ == '__main__':
    time_measure_client_test()

# Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     app = QtWidgets.QApplication(sys.argv)
#     widget = QtWidgets.QWidget()
#     widget.resize(400, 400)
#     widget.setWindowTitle("Hi, PyQt5")
#     widget.show()
#     sys.exit(app.exec_())

# import socket
# import sys
# from _thread import *
#
# if __name__ == '__main__':
#     HOST = ''
#     PORT = 8888
#     AT_HOST = '192.168.62.199'
#     AT_PORT = 50700
#
#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     print('Socket created')
#
#     try:
#         s.bind((HOST, PORT))
#     except socket.error as msg:
#         print('Bind failed. Error Code:' + str(msg[0]) + 'Message' + msg[1])
#         sys.exit()
#
#     print('Socket bind complete')
#
#     s.listen(10)
#     print('Socket now listening')


# def clientthread(conn):
#     conn.send(b'Welcome to the server. Type something and hit enter\n')
#
#     while True:
#         data = conn.recv(1024)
#         reply = b'OK...' + data
#         if not data:
#             break
#
#         conn.sendall(reply)
#
#     conn.close()
#     # address = conn.getpeername()
#     print('Connection close')
#
# while 1:
#     conn, addr = s.accept()
#     print('Connected with' + addr[0] + ':' + str(addr[1]))
#
#     start_new_thread(clientthread, (conn,))
#
# s.close()
