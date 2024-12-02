#!/usr/bin/env python3

import socket
import re
import time, queue

from pycomm3 import CIPDriver

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
            conn.settimeout(0.1) 

            while True:
                try:
                    data = conn.recv(1024)
                    if data:
                        message = data.decode('utf-8', errors='ignore')
                        # Remove non-printable characters
                        message = re.sub(r'[^\x20-\x7E]+', '', message).strip().lower()
                        logger.info(f"Received raw data: '{message}'")
                        command_queue.put(message)
                    else:
                        logger.info('Connection closed by client.')
                        break
                except socket.timeout:
                    pass
                except Exception as e:
                    logger.error(f"Socket error: {e}")
                    break

                try:
                    response_message = response_queue.get_nowait()
                    if response_message:
                        conn.sendall(response_message.encode('utf-8'))
                        logger.info(f"Sent response: '{response_message}'")
                except queue.Empty:
                    pass
                except Exception as e:
                    logger.error(f"Error sending response: {e}")
                    break

                time.sleep(0.01)
            conn.close()
        except Exception as e:
            logger.error(f"Socket accept failed: {e}")
            continue
    # sock.close()  

class eip_comm:

    def __init__(self, logger, ip_address, command_queue, response_queue):
        self.logger = logger
        self.ip_address = ip_address
        self.command_queue = command_queue
        self.response_queue = response_queue

        try:
            with CIPDriver(self.ip_address) as self.conn:
                if self.conn.connected:
                    self.logger.info(f'Connected to : {self.ip_address}')
                    self.main()
                else:
                    self.logger.error('Failed to connect to robot.')
        except Exception as e:
            self.logger.error(f'An exception occurred: {e}')

    def readR_Register(self, RegNum):
         
        response = self.conn.generic_message(
            service=0x0E,       
            class_code=0x6B,    
            instance=1,         
            attribute=RegNum,        
            connected=False
        )

        value_bytes = response.value
        value = int.from_bytes(value_bytes, byteorder='little', signed=True)
        self.command_queue.put(f'{value}')
        #self.logger.info(f'Read value from R[5]: {value}')

    def writeR_Register(self, RegNum):
        try:
            Value = self.response_queue.get_nowait()
        
        except queue.Empty:
            #self.logger.info('Response queue is empty')
            return  

        try:
            Value = int(Value)
            myBytes = Value.to_bytes(4, 'little')

            tag = self.conn.generic_message(
                service=0x10,
                class_code=0x6B,
                instance=1,
                attribute=RegNum,
                request_data=myBytes,
                connected=True
            )

            #self.logger.info(f'Sent value to R[{RegNum}]: {Value}')
            return tag.error
        except Exception as e:
            self.logger.error(f'Failed to write value to R[{RegNum}]: {e}')
    
    def main(self):
        while True:
            self.readR_Register(5)
            time.sleep(1)
            self.writeR_Register(6)
