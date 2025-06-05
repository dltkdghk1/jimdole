import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Int8MultiArray
import geometry_msgs.msg
from ssafy_msgs.msg import EnviromentStatus

import socketio
sio = socketio.Client()
global air_con_control_cmd
air_con_control_cmd=0

@sio.event
def connect():
    print('connection established')

@sio.on('sendAirConOn')
def aircon_on(data):
    global air_con_control_cmd
    air_con_control_cmd+=1
    print(air_con_control_cmd)
    # print(data)


@sio.on('sendAirConOff')
def aircon_off(data):
    global air_con_control_cmd
    air_con_control_cmd-=1
    print(air_con_control_cmd)
    # print('message received with ', data)


@sio.event
def disconnect():
    print('disconnected from server')    

def get_global_var():
    return air_con_control_cmd 
class awsClient(Node):

    def __init__(self):
        super().__init__('aws_client')
        self.envir_msg=EnviromentStatus()
        self.subscription = self.create_subscription(EnviromentStatus,'/envir_status',self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Int8MultiArray, 'app_control', 10)

        self.prev_ctrl_cmd = 0

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        sio.connect('http://localhost:12001')

    
    def listener_callback(self, msg):
        self.envir_msg=msg

      

    def timer_callback(self):

        ctrl_msg = Int8MultiArray()
        time_info_msg = '%d:%d'%(self.envir_msg.hour,self.envir_msg.minute)
        weather_info_msg=self.envir_msg.weather
        temperature_info_msg='%d C'%(self.envir_msg.temperature)

        sio.emit('sendTime',time_info_msg)
        sio.emit('sendWeather',weather_info_msg)
        sio.emit('sendTemperature',temperature_info_msg)
        
        ctrl_cmd = get_global_var()
        if ctrl_cmd-self.prev_ctrl_cmd ==1  :
            print('on')
            control_data=[]
            for a in range(17):
                control_data.append(1)
   
            ctrl_msg.data=control_data
            self.publisher_.publish(ctrl_msg)
        elif ctrl_cmd-self.prev_ctrl_cmd==-1  :
            print('off')
            air_con_control_cmd=0
            control_data=[]
            for a in range(17):
                control_data.append(2)
  
            ctrl_msg.data=control_data
            self.publisher_.publish(ctrl_msg)

        self.prev_ctrl_cmd=ctrl_cmd
 
def main(args=None):
    rclpy.init(args=args)
    aws = awsClient()
    rclpy.spin(aws)
    aws.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()









