#!/usr/bin/python2

import rospy
from datetime import datetime
from mavros_msgs.msg import RTCM
from base64 import b64encode
from threading import Thread, Event
from httplib import HTTPConnection, IncompleteRead



class ntripconnect(Thread):
    def __init__(self, ntc):
        print("Initialize!")
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.cnt_reconnection = 0
        self.non_rtcm_count = 0.0 
        self.rtcm_count = 0.0
        self.data_count = 0.0
        self.stop_event = Event()
        self.log_thread = Thread(target=self.log_status_periodically)
        self.log_thread.daemon = True  

    def log_status_periodically(self):
        while not rospy.is_shutdown():
            if self.data_count==0:
                continue
            rospy.loginfo(
                "------- NTRIP MODULE ------\nStatus Update: \nTotal disconnections = {}\nNon-RTCM msgs received ratio = {:.2f}\nRTCM msgs received ration = {:.2f}\n---------------------"
                .format(self.cnt_reconnection, (self.non_rtcm_count/self.data_count)*100.0, (self.rtcm_count/self.data_count)*100.0)
            )
            rospy.sleep(30)


    def run(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass))
        }
        rospy.loginfo("Attempting to connect to NTRIP server...")
        connection = HTTPConnection(self.ntc.ntrip_server, timeout=self.ntc.timeout)
        # Start the periodic logging thread
        self.log_thread.start()
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                now = datetime.utcnow()
                gga_sentence = self.ntc.nmea_gga % (now.hour, now.minute, now.second)
                connection.request('GET', '/' + self.ntc.ntrip_stream, gga_sentence, headers)

                response = connection.getresponse()
                
                
                
                if response.status != 200:
                    raise Exception("Failed to connect to NTRIP server: {}".format(response.reason))

                # Log server and mount point information
                rospy.loginfo("Connected to NTRIP server: {} (IP/Domain)".format(self.ntc.ntrip_server))
                rospy.loginfo("Mount point: {}".format(self.ntc.ntrip_stream))
                buf = ""
                rmsg = RTCM()

                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    data = response.read(1)  # Read in chunks for efficiency
                    if not data:
                        rospy.logwarn("No data received. Attempting to reconnect...")
                        raise Exception("No data received.")
                    self.data_count += 1.0
                    if ord(data[0]) == 211:  # RTCM message start
                        # Count RTCM messages
                        self.rtcm_count += 1.0
                        l1 = ord(response.read(1))
                        l2 = ord(response.read(1))
                        pkt_len = ((l1 & 0x3) << 8) + l2
                        pkt = response.read(pkt_len)
                        parity = response.read(3)

                        if len(pkt) != pkt_len:
                            rospy.logerr("Packet length mismatch: expected {}, got {}".format(pkt_len, len(pkt)))
                            continue

                        rmsg.header.seq += 1
                        rmsg.header.stamp = rospy.get_rostime()
                        rmsg.data = data + chr(l1) + chr(l2) + pkt + parity
                        self.ntc.pub.publish(rmsg)
                    else:
                        # Count and log non-RTCM messages
                        self.non_rtcm_count += 1.0
                        # Print non-RTCM data in a user-friendly way
                        try:
                            message = data.decode("utf-8")  # Attempt to decode as UTF-8
                            rospy.logwarn("Non-RTCM Message: {}".format(message.strip()))
                        except UnicodeDecodeError:
                            rospy.logwarn("Non-RTCM Message (binary): {}".format(data.encode("hex")))
                            rospy.sleep(0.5)


                rospy.sleep(0.1)

            except Exception as e:
                rospy.logerr("Connection error: {}. Retrying...".format(e))
                connection.close()
                self.cnt_reconnection += 1 
                rospy.loginfo("Try to reconnect (2 sec)...[{}]".format(self.cnt_reconnection))
                connection = HTTPConnection(self.ntc.ntrip_server, timeout=self.ntc.timeout)
                rospy.sleep(2)


        connection.close()
        rospy.loginfo("NTRIP connection thread stopped.")

    def stop(self):
        self.stop_event.set()


class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic',"rtcm")
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')
        self.timeout = rospy.get_param('~timeout', 3)

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=50)
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        try:
            rospy.spin()
        finally:
            rospy.loginfo("Shutting down NTRIP client...")
            self.connection.stop()


if __name__ == '__main__':
    client = ntripclient()
    client.run()
