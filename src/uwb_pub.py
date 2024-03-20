#!/usr/bin/python4

import serial
from rospy import Publisher, Rate, init_node
from std_msgs.msg import Float32
import rospy
import re

port0 = serial.Serial("/dev/ttyUWB2", "115200")
port1 = serial.Serial("/dev/ttyUWB3", "115200")

'''while True:
        try:
            data0 = port0.readline()
            data1 = port1.readline()
 
            print("0: ", data0)  
            print("1: ", data1) 
                        
            
            
 
        except KeyboardInterrupt:
                break
                
port.close()'''

if __name__ == '__main__':
    init_node("UWB_pub")

    #uwb0 = port0.readline()
    #uwb1 = port1.readline()
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

        #uwb0_ = uwb0.strip().decode('utf-8')

        #uwb0_value = float(uwb0.strip().decode('utf-8'))
        #uwb0_value = float(match_1)

        if port1:
            uwb1 = port1.readline()
            match_2 = re.search(uwb_pattern, uwb1.decode('utf-8'))
        if match_2:
            uwb1_value = match_2.group(1)
            msg1.data = float(uwb1_value)
            publisher_uwb1.publish(msg1)
            port1.reset_input_buffer()

        #uwb1_value = float(uwb1.strip().decode('utf-8'))
        #uwb1_value = float(match_2)

        #msg0.data = uwb0_value
        #msg1.data = uwb1_value
        #while(publisher_uwb0.get_num_connections == 0 or publisher_uwb1.get_num_connections == 0):
            #pass
        #while(publisher_uwb0.get_num_connections == 0):
        #    pass
        #rospy.loginfo(msg)

        #publisher_uwb0.publish(msg0)
        #publisher_uwb1.publish(msg1)

        r.sleep()
