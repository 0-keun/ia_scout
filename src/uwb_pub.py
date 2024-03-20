import serial
from rospy import Publisher, Rate, init_node
from std_msgs.msg import Float32
import rospy
import re

port0 = serial.Serial("/dev/ttyUWB2", "115200")
port1 = serial.Serial("/dev/ttyUWB3", "115200")

if __name__ == '__main__':
    init_node("UWB_pub")

    publisher_uwb0 = Publisher(name="uwb0_pub", data_class=Float32, queue_size=1)
    publisher_uwb1 = Publisher(name="uwb1_pub", data_class=Float32, queue_size=1)

    r = Rate(100)
    while not rospy.is_shutdown():


        msg0 = Float32()
        msg1 = Float32()

        uwb_pattern = r'DIST: (\d+\.\d+) m'

        if port0:
            uwb0 = port0.readline()
            match_1 = re.search(uwb_pattern, uwb0.decode('utf-8'))
        if match_1:
            uwb0_value = match_1.group(1)
            msg0.data = float(uwb0_value)
            publisher_uwb0.publish(msg0)
            port0.reset_input_buffer()

        if port1:
            uwb1 = port1.readline()
            match_2 = re.search(uwb_pattern, uwb1.decode('utf-8'))
        if match_2:
            uwb1_value = match_2.group(1)
            msg1.data = float(uwb1_value)
            publisher_uwb1.publish(msg1)
            port1.reset_input_buffer()

        r.sleep()
