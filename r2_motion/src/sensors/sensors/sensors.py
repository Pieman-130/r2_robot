import serial
import struct
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32

PAYLOAD_SIZE = 28 #7 4byte ints
STRUCT_READ = '<7i>' #7 ints little endian
PREAMBLE = bytes([0xAA,0x55])
POSTAMBLE = bytes([0x55,0xAA])
CHECKSUM_SIZE = 1
PACKET_SIZE = len(PREAMBLE) + PAYLOAD_SIZE + CHECKSUM_SIZE + len(POSTAMBLE) #33 bytes

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
    def __init__(self,port='/dev/ttyUSB0',data_rate=115200):
        super().__init__('Arduino_Sensors')
        try:
            self.ser = serial.Serial(port, data_rate)
        except:
            self.get_logger().error("Failed to connect to Arduino!")
            rclpy.shutdown()

        self.current_sensor_data = {lt_dist_sensor: None,
                                    rt_dist_sensor: None,
                                    rr_dist_sensor: None,
                                    ft_clif_sensor: None,
                                    rr_clif_sensor: None,
                                    lt_hall_sensor: None,
                                    rt_hall_sensor: None}

        #TODO Add publishers for each sensor stream
        #3 publishers Speed, Cliff, and Sides
        
        #self.imu_publisher = self.create_publisher(Imu, 'boat/imu', 10)
        #self.imu_timer = self.create_timer(pub_rate, self.imu_tmr_calbk)

    def _validate_chunk(self,chunk):
        skip = 2+1+PAYLOAD_SIZE
        if chunk[0:2] == PREAMBLE and chunk[skip:skip+2] == POSTAMBLE:
            payload = chunk[2:PAYLOAD_SIZE+2]
            checksum = chunk[2+PAYLOAD_SIZE]
            calc_cksum = 0
            for b in payload:
                calc_cksum ^= b

            if calc_cksum == checksum:
                return payload
            else:
                self.get_logger().debug("Serial packet data bad checksum.")
                return False
        else:
            return False


    def read_sensors(self):
        if self.ser.in_waiting >= PACKET_SIZE:
            data = ser.read(PACKET_SIZE)

            payload = self._validate_chunk(data)
            if payload:
                try:
                    rcvd_data = struct.unpack(STRUCT_READ, payload)
                except:
                    self.get_logger().warn("Failed to decode data from arduino sensors.")


def main():

    rclpy.init()
    sensors = SensorRead()
    while not rclpy.is_shutdown():
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
