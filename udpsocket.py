import socket


class UDPSocket:
    """
    UDP Socket
    """

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('192.168.62.224', 30301))
        else:
            self.sock = sock

    def close(self):
        self.sock.close()

    def sendto(self, msg, address, require_response=False, recv_len=2048):
        send = self.sock.sendto(msg, address)
        if send == 0:
            raise RuntimeError("Address can not reach")

        if require_response:
            recv = self.sock.recv(recv_len)
            return recv
        else:
            return ''

    def recv(self, recv_len=2048):
        return self.sock.recv(recv_len)


def udpsocket_test():
    remote_host = '192.168.10.140'
    remote_port = 30301

    sock = UDPSocket()
    print(sock.sendto(b'Test', (remote_host, remote_port), True))
    sock.close()


if __name__ == '__main__':
    udpsocket_test()
