import socket
# import sys
# from _thread import *


class TCPSocket:
    """
    TCP Socket
    """
    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connect(self, host, port):
        self.sock.connect((host, port))

    def close(self):
        self.sock.close()

    def at_send(self, msg):
        sent = self.sock.send(msg)
        if sent == 0:
            raise RuntimeError("socket connection broken")
        recv = self.sock.recv(1024)
        if recv == b'':
            raise RuntimeError("socket connection broken")
        return recv


def tcpsocket_test():
    # host = ''
    # port = 8888
    at_host = '192.168.62.199'
    at_port = 50700

    at_sock = TCPSocket()
    # with TCPSocket() as at_sock:
    at_sock.connect(at_host, at_port)
    print(at_sock.at_send(b'AT\r\n'))
    at_sock.close()


if __name__ == '__main__':
    tcpsocket_test()
