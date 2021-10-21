#!/usr/bin/env python3

import rospy
from ati_sensor_ros_driver.rpi_ati_net_ft import NET_FT
import argparse
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger, TriggerResponse

class ATIDriver(object):
    def __init__(self, host) -> None:
        super().__init__()

        self.host = host
        self.ati_obj = NET_FT(host)
        self.ati_obj.set_tare_from_ft()
        print(self.ati_obj.read_ft_http())
        print(self.ati_obj.try_read_ft_http())

        # service server
        self.tare_srv = rospy.Service('set_ati_tare',Trigger,self.set_tare)
        self.clr_tare_srv = rospy.Service('clear_ati_tare',Trigger,self.set_tare)

        # publisher
        self.ft_pub = rospy.Publisher("ati_ft",WrenchStamped,queue_size=1)
    
    def get_ft(self):

        res, ft, status = self.ati_obj.try_read_ft_streaming(.1)
        fstamp = rospy.Time.now()

        w = WrenchStamped()
        w.header.stamp = fstamp
        w.header.frame_id = 'ati_frame'
        w.wrench.torque.x = ft[0]
        w.wrench.torque.y = ft[1]
        w.wrench.torque.z = ft[2]
        w.wrench.force.x = ft[3]
        w.wrench.force.y = ft[4]
        w.wrench.force.z = ft[5]

        self.ft_pub.publish(w)

    def set_tare(self, req):
        
        self.ati_obj.set_tare_from_ft()
        return TriggerResponse(
            success=True,
            message="ATISensor: The tare is set."
        )
    
    def clear_tare(self, req):

        self.ati_obj.clear_tare()
        return TriggerResponse(
            success=True,
            message="ATISensor: The tare is cleared."
        )

def main():
    parser = argparse.ArgumentParser(description="ATI force torque sensor driver service for Robot Raconteur")
    parser.add_argument("--sensor-ip", type=str, default="192.168.50.65", help="the ip address of the ati sensor")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args,_ = parser.parse_known_args()
    
    rospy.init_node('ati_driver')
    ati_driver_obj = ATIDriver(args.sensor_ip)

    while not rospy.is_shutdown():
        ati_driver_obj.get_ft()

if __name__ == "__main__":
    main()