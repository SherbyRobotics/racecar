#!/usr/bin/env python3

import socket


HOST = str("127.0.0.1")
PORT = int(65432)


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))


if __name__ == "__main__":
    main()
