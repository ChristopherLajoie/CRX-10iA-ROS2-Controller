#!/usr/bin/env python3

import socket
import re

def setup_socket(ip_address, port, logger):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip_address, port))
    sock.listen(1)
    logger.info('Waiting for connection...')
    conn, addr = sock.accept()
    logger.info(f'Connected by {addr}')
    return conn, addr

def socket_listener(conn, logger, plan_and_execute_callback):
    data = conn.recv(1024)
    if data:
        message = data.decode('utf-8', errors='ignore')
        # Remove non-printable characters
        message = re.sub(r'[^\x20-\x7E]+', '', message).strip().lower()
        logger.info(f"Received raw data: {repr(message)}")
        if message == 'home':
            logger.info(f"Received command: {message}")
       
            plan_and_execute_callback()
        else:
            logger.info(f"Received unrecognized command: {message}")