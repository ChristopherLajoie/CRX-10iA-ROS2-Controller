#!/usr/bin/env python3

import socket
import re
import time, queue

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


def socket_server(sock, logger, command_queue, response_queue):
    while True:
        logger.info('Waiting for connection...')
        try:
            conn, addr = sock.accept()
            logger.info(f'Connected by {addr}')
            conn.settimeout(0.1)  # Set a short timeout for responsiveness

            while True:
                # Handle incoming data
                try:
                    data = conn.recv(1024)
                    if data:
                        message = data.decode('utf-8', errors='ignore')
                        # Remove non-printable characters
                        message = re.sub(r'[^\x20-\x7E]+', '', message).strip().lower()
                        logger.info(f"Received raw data: '{message}'")
                        command_queue.put(message)
                    else:
                        # Connection closed by client
                        logger.info('Connection closed by client.')
                        break
                except socket.timeout:
                    # No data received, proceed to check for messages to send
                    pass
                except Exception as e:
                    logger.error(f"Socket error: {e}")
                    break

                # Handle outgoing data
                try:
                    response_message = response_queue.get_nowait()
                    if response_message:
                        conn.sendall(response_message.encode('utf-8'))
                        logger.info(f"Sent response: '{response_message}'")
                except queue.Empty:
                    # No message to send
                    pass
                except Exception as e:
                    logger.error(f"Error sending response: {e}")
                    break

                # Sleep briefly to avoid busy looping
                time.sleep(0.01)
            conn.close()
        except Exception as e:
            logger.error(f"Socket accept failed: {e}")
            continue
    # sock.close()  # Do not close the socket unless you intend to stop the server

