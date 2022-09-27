#!/usr/bin/python2

import rospy
from datetime import datetime

# from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM
import datetime
from base64 import b64encode
from threading import Thread

from httplib import HTTPConnection
from httplib import IncompleteRead

''' This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import httplib


def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except httplib.IncompleteRead as e:
            return e.partial
    return inner


httplib.HTTPResponse.read = patch_http_response_read(httplib.HTTPResponse.read)


class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):

        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass))
        }
        connection = HTTPConnection(self.ntc.ntrip_server, timeout=3)

        print("\n")

        while True:
            try:
                print("connecting")
                # connection.request('GET', '/'+self.ntc.ntrip_stream,
                #                    self.ntc.nmea_gga, headers)
                now = datetime.datetime.utcnow()
                connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga %
                                   (now.hour, now.minute, now.second), headers)

                print("\tsuccess")
                response = connection.getresponse()
                if response.status != 200:
                    raise Exception("blah")
                buf = ""
                rmsg = RTCM()
                restart_count = 0
                while not self.stop:
                    '''
                    data = response.read(100)
                    pos = data.find('\r\n')
                    if pos != -1:
                        rmsg.message = buf + data[:pos]
                        rmsg.header.seq += 1
                        rmsg.header.stamp = rospy.get_rostime()
                        buf = data[pos+2:]
                        self.ntc.pub.publish(rmsg)
                    else: buf += data
                    '''

                    ''' This now separates individual RTCM messages and publishes each one on the same topic '''
                    data = response.read(1)
                    if len(data) != 0:
                        if ord(data[0]) == 211:
                            l1 = ord(response.read(1))
                            l2 = ord(response.read(1))
                            pkt_len = ((l1 & 0x3) << 8)+l2

                            pkt = response.read(pkt_len)
                            parity = response.read(3)
                            if len(pkt) != pkt_len:
                                rospy.logerr("Length error: {} {}".format(len(pkt), pkt_len))
                                continue
                            rmsg.header.seq += 1
                            rmsg.header.stamp = rospy.get_rostime()
                            rmsg.data = data + chr(l1) + chr(l2) + pkt + parity
                            self.ntc.pub.publish(rmsg)
                        else:
                            # print(data)
                            pass
                    else:
                        ''' If zero length data, close connection and reopen it '''
                        restart_count = restart_count + 1
                        print("Zero length data...Restart count: ", restart_count)
                        connection.close()
                        connection = HTTPConnection(
                            self.ntc.ntrip_server, timeout=3)
                        print("Wrong data")
            except:
                print("\tconnection failed")
                connection.close()
                rospy.sleep(1)
                connection = HTTPConnection(self.ntc.ntrip_server, timeout=3)
                continue

        print("function finished")
        connection.close()


class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', 'rtcm')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.run()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True


if __name__ == '__main__':
    c = ntripclient()
    c.run()
