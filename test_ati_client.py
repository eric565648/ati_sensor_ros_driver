
from RobotRaconteur.Client import *
import time
import numpy
import sys

def main():

    # url='rr+local:///?nodeid=8570e7c7-b75e-4234-ae69-185caf0c59df&service=ati_sensor'
    url='rr+tcp://localhost:59823?service=ati_sensor'
    if (len(sys.argv)>=2):
        url=sys.argv[1]

    #Connect to the service
    cli = RRN.ConnectService(url)

    #Connect a wire connection
    wrench_wire = cli.wrench_sensor_value.Connect()

    #Add callback for when the wire value change
    wrench_wire.WireValueChanged += wrench_wire_cb

    if (sys.version_info > (3, 0)):
        input("Server started, press enter to quit...")
    else:
        raw_input("Server started, press enter to quit...")


def wrench_wire_cb(w,value,time):
    

    print("==============")
    print("torque.x",value['torque']['x'])
    print("torque.y",value['torque']['y'])
    print("torque.z",value['torque']['z'])
    print("force.x",value['force']['x'])
    print("force.y",value['force']['y'])
    print("force.z",value['force']['z'])
    print("==============")

    

if __name__=='__main__':
    main()