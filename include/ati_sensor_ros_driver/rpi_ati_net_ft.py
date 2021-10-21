# Copyright (c) 2018, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Rensselaer Polytechnic Institute, or Wason 
#       Technology LLC, nor the names of its contributors may be used to 
#       endorse or promote products derived from this software without 
#       specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import absolute_import

import socket
import select
import requests
from bs4 import BeautifulSoup
import numpy as np
from collections import namedtuple
import struct
import time

NET_FT_device_settings=namedtuple('net_ft_settings', ['ft', 'conv', 'maxrange', 'bias', 'ipaddress', 'rdt_rate', 'device_status'])

class NET_FT(object):
    
    def __init__(self, net_ft_host='192.168.1.1'):
        self.host=net_ft_host
        self.base_url='http://' + net_ft_host
        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('',0))
        self.port=self.socket.getsockname()[1]
                
        self.device_settings=self.read_device_settings()
        
        self.tare=np.ndarray([6])
        
        self._streaming=False
        self._last_streaming_command_time=0
                
    def _read_netftapi2(self):
        
        url="/".join([self.base_url, 'netftapi2.xml'])
        res=requests.get(url)
        res.raise_for_status()
        
        soup=BeautifulSoup(res.text)
        
        return soup
    
    def read_device_settings(self):
        
        soup=self._read_netftapi2()
        
        device_status = int(soup.find('runstat').text,16)
                
        if soup.find('scfgfu').text != 'N':
            raise Exception('ATI Net F/T must use MKS units')
        
        if soup.find('scfgtu').text != 'N-m':
            raise Exception('ATI Net F/T must use MKS units')
        
        if soup.find('comrdte').text != "Enabled":
            raise Exception('ATI Net F/T must have RDT enabled')
        
        cfgcpf=float(soup.find('cfgcpf').text)
        cfgcpt=float(soup.find('cfgcpt').text)
        
        def _to_array(s):
            return np.fromstring(soup.find(s).text, dtype=np.float64, sep=';' )
        
        conv=np.asarray([cfgcpt, cfgcpt, cfgcpt, cfgcpf, cfgcpf, cfgcpf], dtype=np.float64)
        maxrange=_to_array('cfgmr')
        bias=np.divide(_to_array('setbias'), conv)
        ft1=_to_array('runft')
        ft=np.divide(np.append(ft1[3:6],ft1[0:3]), conv)
        ipaddress=soup.find('netip').text
        rdt_rate=int(soup.find('comrdtrate').text)
        
        return NET_FT_device_settings(ft, conv, maxrange, bias, ipaddress, rdt_rate, device_status)
        
    def set_tare_from_ft(self):
        settings=self.read_device_settings()
        self.tare=settings.ft
        
    def clear_tare(self):
        self.tare=np.ndarray([6])
        
    def read_ft_http(self):
        settings=self.read_device_settings()
        if settings.device_status != 0:
            raise Exception('ATI Net F/T returning error status code: ' + str(settings.device_status))
        
        return settings.ft-self.tare
    
    def try_read_ft_http(self):
        try:
            settings=self.read_device_settings()        
            return settings.ft-self.tare, settings.device_status
        except:
            return None, 0x80000000
    
    def start_streaming(self):
        sample_count=10*self.device_settings.rdt_rate
        dat=struct.pack('>HHI',0x1234, 0x0002, sample_count)
        self.socket.sendto(dat, (self.host, 49152))
        self._streaming=True
        self._last_streaming_command_time=time.time()
    
    def stop_streaming(self):
        dat=struct.pack('>HHI',0x1234, 0x0000, 0)
        self.socket.sendto(dat, (self.host, 49152))
        self._streaming=False
        self._last_streaming_command_time=time.time()
    
    def try_read_ft_streaming(self, timeout=0):
        
        #Re-up the streaming if running out of packets
        if (time.time() - self._last_streaming_command_time) > 5:
            if (self._streaming):                
                self.start_streaming()
        
        s=self.socket
        s_list=[s]
        
        buf=None
        
        timeout1=timeout
        drop_count=0
        while(True):        
            res=select.select(s_list, [], s_list, timeout1)            
            if len(res[0]) == 0 and len(res[2])==0:
                break            
            try:
                (buf, addr)=s.recvfrom(1024)
            except:            
                return False, None, 0
            
            if (drop_count > 100):
                break
        
            timeout1=0
            drop_count+=1            
            
        if (buf is None):
            return False, None, 0
        
        rdt_sequence, ft_sequence, status, Fx, Fy, Fz, Tx, Ty, Tz \
            =struct.unpack('>IIIiiiiii', buf)
                
        ft=np.divide(np.asarray([Tx, Ty, Tz, Fx, Fy, Fz]), self.device_settings.conv)-self.tare
                
        return True, ft, status
        
    def read_ft_streaming(self, timeout=0):
        
        ret, ft, status = self.try_read_ft_streaming(timeout)
        if not ret:
            return False, None
        
        if (status != 0):
            raise Exception('ATI Net F/T returning error status code: ' + str(status))
        
        return True, ft
        
    def __del__(self):
        if (self._streaming):
            try:
                self.stop_streaming()
            except: pass
