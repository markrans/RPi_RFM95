#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 07:44:50 2017

Example for receiving data packets over a network of RFM95 modules.

@author: markrans
"""

from RFM95 import *
import time

from RFM95 import *
import time

freqBand=434.0
nodeID=0
networkID=1
intPin=18
rstPin=24
spiBus=0
spiDevice=0
print('Radio initialising...')
rfm95=RFM95(freqBand=freqBand,nodeID=nodeID,networkID=networkID,intPin=intPin,rstPin=rstPin,spiBus=spiBus,spiDevice=spiDevice)
print('Success!')
print('RF95 node #'+str(nodeID)+' init OK @ '+str(freqBand)+'MHz')
print('Listening packet...')
while True:
    buf=[]
    if rfm95.available():
        if rfm95.recv(buf,RF95_MAX_MESSAGE_LEN):
            print('Packet['+str(len(rfm95.lastMessage))+'] #'+str(rfm95.rxHeaderFrom)+' => #'+str(rfm95.rxHeaderTo)+' '+str(rfm95.lastRSSI)+'dB: '+rfm95.lastMessage)
            time.sleep(1)
            
