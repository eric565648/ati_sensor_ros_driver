#!/usr/bin/env python


import sys
import time
import rpi_ati_net_ft

def main():
    sys.path.append('../src')
    

    try:
        if (len(sys.argv) < 2):
            raise Exception('IP address of ATI Net F/T sensor required')
        host=sys.argv[1]
        netft=rpi_ati_net_ft.NET_FT(host)
        netft.set_tare_from_ft()
        print(netft.read_ft_http())
        print(netft.try_read_ft_http())
        
        netft.start_streaming()
        
        while(True):
            #print(netft.read_ft_streaming(.1))
            res, ft, status = netft.try_read_ft_streaming(.1)
            print("res: ", res)
            print("ft: ", ft)
            print("status: ", status)
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
