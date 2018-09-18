'''
    Example for how to receive live data and display live video (without gaze overlay) from glasses.
    gstreamer 0.10 required in order to display live video.

    Note: This example program is *only* tested with Python 2.7 on Ubuntu 12.04 LTS
          and Ubuntu 14.04 LTS (running natively).
'''
import time
import socket
import threading
import signal
import sys
#import pygst
import rospy
from std_msgs.msg import String

#pygst.require('0.10')
#import gst

timeout = 1.0
running = True


GLASSES_IP = "192.168.71.50"
PORT = 49152


# Keep-alive message content used to request live data and live video streams
KA_DATA_MSG = "{\"type\": \"live.data.unicast\", \"key\": \"some_GUID\", \"op\": \"start\"}"


# Create UDP socket
def mksock(peer):
    iptype = socket.AF_INET
    if ':' in peer[0]:
        iptype = socket.AF_INET6
    return socket.socket(iptype, socket.SOCK_DGRAM)


# Callback function
def send_keepalive_msg(socket, msg, peer):
    while running:
        print("Sending " + msg + " to target " + peer[0] + " socket no: " + str(socket.fileno()) + "\n")
        socket.sendto(msg, peer)
        time.sleep(timeout)


def signal_handler(signal, frame):
    stop_sending_msg()
    sys.exit(0)


def stop_sending_msg():
    global running
    running = False

def talker(data):
    pub = rospy.Publisher('gaze_tracker', String, queue_size=10)
    rospy.init_node('gaze', anonymous=True)
    rate = rospy.Rate(50) # 50hz is the frequency of the eye tracker
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        # Read live data
        data, address = data_socket.recvfrom(1024)
        print(data)
        print(address)
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    peer = (GLASSES_IP, PORT)

    # Create socket which will send a keep alive message for the live data stream
    data_socket = mksock(peer)
    td = threading.Timer(0, send_keepalive_msg, [data_socket, KA_DATA_MSG, peer])
    td.start()

    #while running:
    try:
        talker(data_socket)
    except rospy.ROSInterruptException:
        pass
        #stop_sending_msg()

        # Read live data
        #data, address = data_socket.recvfrom(1024)
        #print (data)
