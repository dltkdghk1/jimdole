
import rclpy
from rclpy.node import Node  
import time
import os
import socket
import threading
import struct
import binascii

# iot_udp 노드는 udp 통신을 이용해 iot로 부터 uid를 얻어 접속, 제어를 하는 노드입니다.
# sub1,2 에서는 ros 메시지를 이용해 쉽게 제어했지만, advanced iot 제어에서는 정의된 통신프로토콜을 보고 iot와 직접 데이터를 주고 받는 형식으로 제어하게 됩니다.
# 통신 프로토콜은 명세서를 참조해주세요.


# 노드 로직 순서
# 1. 통신 소켓 생성
# 2. 멀티스레드를 이용한 데이터 수신
# 3. 수신 데이터 파싱
# 4. 데이터 송신 함수
# 5. 사용자 메뉴 생성 
# 6. iot scan 
# 7. iot connect
# 8. iot control

# 통신프로토콜에 필요한 데이터입니다. 명세서에 제어, 상태 프로토콜을 참조하세요. 
params_status = {
    (0xa,0x25 ) : "IDLE" ,
    (0xb,0x31 ) : "CONNECTION",
    (0xc,0x51) : "CONNECTION_LOST" ,
    (0xb,0x37) : "ON",
    (0xa,0x70) : "OFF",
    (0xc,0x44) : "ERROR"
}


params_control_cmd= {
    "TRY_TO_CONNECT" : (0xb,0x31 )  ,
    "SWITCH_ON" : (0xb,0x37 ) ,
    "SWITCH_OFF" : (0xa,0x70),
    "RESET" : (0xb,0x25) ,
    "DISCONNECT" : (0x00,0x25) 
}


class iot_udp(Node):

    def __init__(self):
        super().__init__('iot_udp')

        self.ip='127.0.0.1'
        self.port=7502
        self.send_port=7401

        # 로직 1. 통신 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip,self.port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        
        # 로직 2. 멀티스레드를 이용한 데이터 수신
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

        self.is_recv_data=False

        os.system('cls')
        while True:
            '''
            로직 5. 사용자 메뉴 생성
            '''
            print('Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures ] ')

            menu=input()
            if menu=='0' :
                self.scan()
            if menu=='1' :
                self.connect()               
            if menu=='2' :
                self.control()   
            if menu=='3' :
                self.disconnect()
            if menu=='4' :
                self.all_procedures()


    def data_parsing(self,raw_data) :
        # print(raw_data)
        
        '''
        로직 3. 수신 데이터 파싱
        '''
        header=raw_data[0:19].decode()
        data_length=struct.unpack('i',raw_data[19:23])
        aux_data=struct.unpack('iii',raw_data[23:35])


        if header == '#Appliances-Status$' and data_length[0] ==20:
            uid_pack=struct.unpack('16B',raw_data[35:51])
            uid=self.packet_to_uid(uid_pack)
        
            nework_status=struct.unpack('2B',raw_data[51:53])
            device_status=struct.unpack('2B',raw_data[53:55])
            
            self.is_recv_data=True
            self.recv_data=[uid,nework_status,device_status]

            
 
    def send_data(self,uid,cmd):
        
        '''
        로직 4. 데이터 송신 함수 생성
        '''
        

        header='#Ctrl-command$'.encode()
        data_length=struct.pack('i',18)
        aux_data=struct.pack('iii',0,0,0)
        self.upper=header+data_length+aux_data
        self.tail='\r\n'.encode()


        uid_pack=self.uid_to_packet(uid)
        cmd_pack=bytes([cmd[0],cmd[1]])

        send_data=self.upper+uid_pack+cmd_pack+self.tail
        self.sock.sendto(send_data,(self.ip,self.send_port))


    def recv_udp_data(self):
        while True :
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)
            



        
            
    def uid_to_packet(self,uid):
        uid_pack=binascii.unhexlify(uid)
        return uid_pack

        
    def packet_to_uid(self,packet):
        uid=""
        for data in packet:
            if len(hex(data)[2:4])==1:
                uid+="0"
            
            uid+=hex(data)[2:4]
            
            
        return uid


    def scan(self):
        
        print('SCANNING NOW.....')
        print('BACK TO MENU : Ctrl+ C')
        '''
        로직 6. iot scan
        '''
        while True:
            try:
                if self.is_recv_data==True :
                    print(' uid :{0} , network status : {1} , device status : {2}'.format(self.recv_data[0],params_status[self.recv_data[1]],params_status[self.recv_data[2]]))
                    self.is_recv_data=False
                    time.sleep(0.5)
            except KeyboardInterrupt:
                break
        os.system('cls')
        
        
            

    def connect(self):
        '''
        로직 7. iot connect
        '''
        if self.is_recv_data==True :
            if params_status[self.recv_data[1]] == "CONNECTION_LOST" :
                print('Current devices status is ERROR so try to reset')
                self.send_data(self.recv_data[0],params_control_cmd["RESET"])
                time.sleep(0.5)
            self.send_data(self.recv_data[0],params_control_cmd["TRY_TO_CONNECT"])


    
    def control(self):

        '''
        로직 8. iot control
        '''
        if self.is_recv_data==True :
            if params_status[self.recv_data[2]] == "ON":
                self.send_data(self.recv_data[0],params_control_cmd["SWITCH_OFF"])
                print('{0} : OFF'.format(self.recv_data[0]))
            elif params_status[self.recv_data[2]] == "OFF":
                self.send_data(self.recv_data[0],params_control_cmd["SWITCH_ON"])
                print('{0} : ON'.format(self.recv_data[0]))


    def disconnect(self):
        if self.is_recv_data==True :
            self.send_data(self.recv_data[0],params_control_cmd["DISCONNECT"])
        

    def all_procedures(self):
        self.connect()
        time.sleep(0.5)
        self.control()
        time.sleep(0.5)
        self.disconnect()


           
    def __del__(self):
        self.sock.close()
        print('del')



def main(args=None):
    rclpy.init(args=args)
    iot = iot_udp()
    rclpy.spin(iot)
    iot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()