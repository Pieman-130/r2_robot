import serial
import struct
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32

PACKET_SIZE = 28 #7 4byte ints
STRUCT_READ = '<7i>' #7 ints little endian

'''
Decipher the serial data:
    0 - Left Distance Sensor
    1 - Right Distance Sensor
    2 - Rear Distance Sensor
    3 - Front Cliff Sensor
    4 - Rear Cliff Sensor
    5 - Right Hall Sensor
    6 - Left Hall Sensor

To convert values:
    Distance:
        value*0.0343/2/100 = dist in meters
    Clif:
        x = value*0.0048828125
        13*pow(x,-1)/100
    Hall:
        RPS = 1/((value/1000000)*90)
        M/S = dia * pi * RPS
'''

class SensorRead(Node):
    '''For reading sensors connected via arduino'''
    def __init__(self):
        super().__init__('Sensors')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        #create sensor buffer dict for each of the 7 sensors

        #TODO Add publishers for each sensor stream
        #3 publishers Speed, Cliff, and Sides
        
        #self.imu_publisher = self.create_publisher(Imu, 'boat/imu', 10)
        #self.imu_timer = self.create_timer(pub_rate, self.imu_tmr_calbk)

    def read_sensors(self):
        if self.ser.in_waiting >= PACKET_SIZE:
            data = ser.read(PACKET_SIZE)

            rcvd_data = struct.unpack(STRUCT_READ,data)

            print("Data: ", rcvd_data)
            #TODO don't print, convert to something that can be
            #parsed for sending out ros messages for each
            #sensor feed


def main():

    rclpy.init()
    sensors = SensorRead()
    while True:
        sensors.read_sensors()
    sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
       def imu_tmr_calbk(self):
        msg = Imu()
        data = self.mav.moi['ATTITUDE']

        if data: #None signifies no data has been populated yet

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_frame'

            quat = quaternion_from_euler(data['roll'],
                                        data['pitch'],
                                        data['yaw'])

            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]

            msg.angular_velocity.x = data['rollspeed']
            msg.angular_velocity.y = data['pitchspeed']
            msg.angular_velocity.z = data['yawspeed']

            self.imu_publisher.publish(msg)
            self.get_logger().debug(f"Publishing: {msg.orientation}, {msg.angular_velocity}")'''
