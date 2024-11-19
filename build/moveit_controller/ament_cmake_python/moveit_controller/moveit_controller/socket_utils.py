#!/usr/bin/env python3

import socket
import re
import threading

def setup_socket(ip_address, port, logger):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((ip_address, port))
        sock.listen(1)
        logger.info('Socket server set up and listening')
        return sock
    except Exception as e:
        logger.error(f"Socket setup failed: {e}")
        return None


def socket_server(sock, logger, command_queue):
    while True:
        logger.info('Waiting for connection...')
        try:
            conn, addr = sock.accept()
            logger.info(f'Connected by {addr}')
            conn.settimeout(1.0)  # Set a timeout on the socket

            while True:
                try:
                    data = conn.recv(1024)
                    if data:
                        message = data.decode('utf-8', errors='ignore')
                        # Remove non-printable characters
                        message = re.sub(r'[^\x20-\x7E]+', '', message).strip().lower()
                        logger.info(f"Received raw data: '{message}'")
                        # Put the message into the command queue
                        command_queue.put(message)
                    else:
                        # Connection closed
                        logger.info('Connection closed by client.')
                        break
                except socket.timeout:
                    # No data received, continue the loop
                    continue
                except Exception as e:
                    logger.error(f"Socket error: {e}")
                    break
            conn.close()
        except Exception as e:
            logger.error(f"Socket accept failed: {e}")
            break
    sock.close()

