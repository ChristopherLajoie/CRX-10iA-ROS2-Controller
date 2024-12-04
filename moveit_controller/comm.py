#!/usr/bin/env python3

import socket, re, time, queue, struct, yaml, os

from pycomm3 import CIPDriver, Services
from ament_index_python.packages import get_package_share_directory

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
        
        self.package_share_directory = get_package_share_directory('moveit_controller')
        config_path = os.path.join(self.package_share_directory, 'config', 'config.yaml')
        
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
        
        self.read_register = config_data['read_register']
        self.write_register = config_data['write_register']
        self.part_handling_register = config_data['part_handling_register']
    
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
        self.command_queue.put({'RegNum': RegNum, 'Value': value})
     
    def writeR_Register(self, RegNum):
        try:
            Value = self.response_queue.get_nowait()
        
        except queue.Empty:
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

            return tag.error
        except Exception as e:
            self.logger.error(f'Failed to write value to R[{RegNum}]: {e}')
    
    def readDO(self, Num):

        response = self.conn.generic_message(
            service=Services.get_attribute_single,
            class_code=0x04,
            instance=0x321,
            attribute=0x03,
            data_type=None,
            connected=False,
        )

        output = list(response.value)
        register = ((Num-1) // 8)
        value = output[register] >> ((Num-1) % 8)
        value = value & 1

        if value:
            return True
        elif not value:
            return False
    
    def readCURPOS(self):

        response = self.conn.generic_message(
            service=0x0E,
            class_code=0x7E,
            instance=1,
            attribute=1,
            connected=False
        )

        output = response.value

        num_joints = 6
        start_offset = 4 
        float_size = 4    

        joint_positions = []

        for i in range(num_joints):
            idx_start = start_offset + i * float_size
            idx_end = idx_start + float_size
            joint_value = struct.unpack('f', output[idx_start:idx_end])[0]
            joint_positions.append(joint_value)

        # Rviz and Fanuc J3 are different
        if len(joint_positions) >= 3:
            joint_positions[2] = joint_positions[1] + joint_positions[2]

        self.command_queue.put({'RegNum': 0, 'Value': joint_positions})
     

    def periodic_task(self):
        while True:
            do_state = self.readDO(103)
            if do_state and not executed:
                self.readCURPOS()
                executed = True  
            elif not do_state:
                executed = False  

            self.writeR_Register(self.write_register)
            self.readR_Register(self.read_register)
            self.readR_Register(self.part_handling_register)
            time.sleep(0.1)

    def main(self):
        self.periodic_task()