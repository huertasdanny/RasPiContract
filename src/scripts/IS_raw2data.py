#!/usr/bin/env python
import rospy
import serial
import sys
sys.path.append(r'/home/pi/catkin_ws/src/inertialsense/src/scripts')
import checksum
from serial import SerialException
from std_msgs.msg import String
from sensor_msgs.msg import Imu

#get args from parameter set
def openPort():
    portname = '/dev/ttyUSB0'
    ser = serial.Serial(timeout = 10)
    ser.port = portname
    ser.baudrate = 30000000
    try:
        ser.open()
        rospy.loginfo('%s port opened', portname)
        #send message to ask for ASCII data instead of binary
        #format is: $ASCB,[imu period], [ins1 period],[ins2 period],
        #[gps pos period], [gps vel period]*XX\r\n where XX is the checksum
        # calculated according to NMEA standard
        sentence = '$ASCB,2,4,4,200,200*'
        chksum = hex(checksum.cal_chksum(sentence))
        #print(sentence + chksum[chksum.find('x')+1:] +'\r\n')
        chksum_pad = checksum.padzero(chksum[chksum.find('x')+1:])
        nmea = sentence + chksum_pad +'\r\n'
        print nmea
        ser.write(nmea)            
        return True, ser
    except SerialException:
        rospy.logfatal('could not open port %s', portname)
        return False, ser    

def parseraw(ser):
    # parse the bit data
    bitstream = ser.readline() #read data until eol /n characted received
    #print(repr(bitstream))
    data = inertialsensefunction(bitstream) 
    rospy.logdebug('recieved data %s', repr(data))
    return data

def inertialsensefunction(rawdata):
    #convert bit data to readable data
    #count to 100 for now
    n = 0
    while n <100:
        n+=1
    imuMsg = Imu()
    imuMsg.orientation_covariance = [ 9999, 0, 0,
                                      0, 9999,0,
                                      0,0, 9999]
    imuMsg.angular_velocity_covariance = [ 9999, 0, 0,
                                      0, 9999,0,
                                      0,0, 9999]
    imuMsg.linear_acceleration_covariance = [ 1, 0, 0,
                                      0, 1,0,
                                      0,0, 1]
    imuMsg.header.stamp = rospy.Time.now()
    return imuMsg, rawdata

def readData(ser):
    rospy.init_node('is_dataparser', anonymous=True)
    rate = rospy.Rate(1000) # 1000hz
    #init publishere here (IMU, GPS, MAG etc)
    pub = rospy.Publisher('raw_IS_data', String, queue_size=10)
    imupub = rospy.Publisher('IS_IMU', Imu, queue_size = 10)
    while not rospy.is_shutdown():
        datamsg, rawdata = parseraw(ser)
        rospy.loginfo('Publishing %s', repr(rawdata))
        imupub.publish(datamsg)
        pub.publish(repr(rawdata))
        #rate.sleep()


if __name__ == '__main__':    
    try:
        op, ser = openPort()
        if op:
            readData(ser)
        else:
            #port is closed abort
            print('failed')
    except rospy.ROSInterruptException:
        pass
