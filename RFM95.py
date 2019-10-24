#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 29 19:35:19 2017

Python RFM95/96/97/w library for Raspberry Pi. Adapted from RadioHead by Airspayce.


@author: markrans
"""

import spidev
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
# No IRQ used on Raspberry PI
RF95_IRQLESS = True
# Define the maximum number of interrupts the driver can support
RF95_NUM_INTERRUPTS = 3
# Define max number of octets the LORA Rx/Tx FIFO can hold
RF95_FIFO_SIZE = 255
# Define the maximum number of bytes that can be carried by the LORA.
RF95_MAX_PAYLOAD_LEN = RF95_FIFO_SIZE
# Define the length of headers added to LoRa's payload
RF95_HEADER_LEN = 4
# Define the maximum message length that can be supported by this driver. 
# Can be pre-defined to a smaller size (to save SRAM) prior to including this header
# Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
RF95_MAX_MESSAGE_LEN = (RF95_MAX_PAYLOAD_LEN - RF95_HEADER_LEN)
# Define crystal oscillator frequency of the module
RF95_FXOSC = 32000000.0
# The Frequency Synthesizer step = RF95_FXOSC / 2^^19
RF95_FSTEP  = (RF95_FXOSC / 524288)


# Register names (LoRa Mode, from table 85)
RF95_FIFO                                =0x00
RF95_OP_MODE                             =0x01
RF95_RESERVED                            =0x02
RF95_RESERVED                            =0x03
RF95_RESERVED                            =0x04
RF95_RESERVED                            =0x05
RF95_FRF_MSB                             =0x06
RF95_FRF_MID                             =0x07
RF95_FRF_LSB                             =0x08
RF95_PA_CONFIG                           =0x09
RF95_PA_RAMP                             =0x0a
RF95_OCP                                 =0x0b
RF95_LNA                                 =0x0c
RF95_FIFO_ADDR_PTR                       =0x0d
RF95_FIFO_TX_BASE_ADDR                   =0x0e
RF95_FIFO_RX_BASE_ADDR                   =0x0f
RF95_FIFO_RX_CURRENT_ADDR                =0x10
RF95_IRQ_FLAGS_MASK                      =0x11
RF95_IRQ_FLAGS                           =0x12
RF95_RX_NB_BYTES                         =0x13
RF95_RX_HEADER_CNT_VALUE_MSB             =0x14
RF95_RX_HEADER_CNT_VALUE_LSB             =0x15
RF95_RX_PACKET_CNT_VALUE_MSB             =0x16
RF95_RX_PACKET_CNT_VALUE_LSB             =0x17
RF95_MODEM_STAT                          =0x18
RF95_PKT_SNR_VALUE                       =0x19
RF95_PKT_RSSI_VALUE                      =0x1a
RF95_RSSI_VALUE                          =0x1b
RF95_HOP_CHANNEL                         =0x1c
RF95_MODEM_CONFIG1                       =0x1d
RF95_MODEM_CONFIG2                       =0x1e
RF95_SYMB_TIMEOUT_LSB                    =0x1f
RF95_PREAMBLE_MSB                        =0x20
RF95_PREAMBLE_LSB                        =0x21
RF95_PAYLOAD_LENGTH                      =0x22
RF95_MAX_PAYLOAD_LENGTH                  =0x23
RF95_HOP_PERIOD                          =0x24
RF95_FIFO_RX_BYTE_ADDR                   =0x25
RF95_MODEM_CONFIG3                       =0x26

RF95_DIO_MAPPING1                        =0x40
RF95_DIO_MAPPING2                        =0x41
RF95_VERSION                             =0x42

RF95_TCXO                                =0x4b
RF95_PA_DAC                              =0x4d
RF95_FORMER_TEMP                         =0x5b
RF95_AGC_REF                             =0x61
RF95_AGC_THRESH1                         =0x62
RF95_AGC_THRESH2                         =0x63
RF95_AGC_THRESH3                         =0x64

# RF95_OP_MODE                             0x01
RF95_LONG_RANGE_MODE                       =0x80
RF95_ACCESS_SHARED_REG                     =0x40
RF95_LOW_FREQUENCY_MODE                    =0x08
RF95_MODE                                  =0x07
RF95_MODE_SLEEP                            =0x00
RF95_MODE_STDBY                            =0x01
RF95_MODE_FSTX                             =0x02
RF95_MODE_TX                               =0x03
RF95_MODE_FSRX                             =0x04
RF95_MODE_RXCONTINUOUS                     =0x05
RF95_MODE_RXSINGLE                         =0x06
RF95_MODE_CAD                              =0x07

# RF95_PA_CONFIG                           0x09
RF95_PA_SELECT                             =0x80
RF95_MAX_POWER                             =0x70
RF95_OUTPUT_POWER                          =0x0f

# RF95_PA_RAMP                             0x0a
RF95_LOW_PN_TX_PLL_OFF                     =0x10
RF95_PA_RAMP                               =0x0f
RF95_PA_RAMP_3_4MS                         =0x00
RF95_PA_RAMP_2MS                           =0x01
RF95_PA_RAMP_1MS                           =0x02
RF95_PA_RAMP_500US                         =0x03
RF95_PA_RAMP_250US                         =0x0
RF95_PA_RAMP_125US                         =0x05
RF95_PA_RAMP_100US                         =0x06
RF95_PA_RAMP_62US                          =0x07
RF95_PA_RAMP_50US                          =0x08
RF95_PA_RAMP_40US                          =0x09
RF95_PA_RAMP_31US                          =0x0a
RF95_PA_RAMP_25US                          =0x0b
RF95_PA_RAMP_20US                          =0x0c
RF95_PA_RAMP_15US                          =0x0d
RF95_PA_RAMP_12US                          =0x0e
RF95_PA_RAMP_10US                          =0x0f

# RF95_OCP                                 0x0b
RF95_OCP_ON                                =0x20
RF95_OCP_TRIM                              =0x1f

# RF95_LNA                                 0x0c
RF95_LNA_GAIN                              =0xe0
RF95_LNA_GAIN_G1                           =0x20
RF95_LNA_GAIN_G2                           =0x40
RF95_LNA_GAIN_G3                           =0x60                
RF95_LNA_GAIN_G4                           =0x80
RF95_LNA_GAIN_G5                           =0xa0
RF95_LNA_GAIN_G6                           =0xc0
RF95_LNA_BOOST_LF                          =0x18
RF95_LNA_BOOST_LF_DEFAULT                  =0x00
RF95_LNA_BOOST_HF                          =0x03
RF95_LNA_BOOST_HF_DEFAULT                  =0x00
RF95_LNA_BOOST_HF_150PC                    =0x11

# RF95_IRQ_FLAGS_MASK                      0x11
RF95_RX_TIMEOUT_MASK                       =0x80
RF95_RX_DONE_MASK                          =0x40
RF95_PAYLOAD_CRC_ERROR_MASK                =0x20
RF95_VALID_HEADER_MASK                     =0x10
RF95_TX_DONE_MASK                          =0x08
RF95_CAD_DONE_MASK                         =0x04
RF95_FHSS_CHANGE_CHANNEL_MASK              =0x02
RF95_CAD_DETECTED_MASK                     =0x01

# RF95_IRQ_FLAGS                           0x12
RF95_RX_TIMEOUT                            =0x80
RF95_RX_DONE                               =0x40
RF95_PAYLOAD_CRC_ERROR                     =0x20
RF95_VALID_HEADER                          =0x10
RF95_TX_DONE                               =0x08
RF95_CAD_DONE                              =0x04
RF95_FHSS_CHANGE_CHANNEL                   =0x02
RF95_CAD_DETECTED                          =0x01

# RF95_MODEM_STAT                          0x18
RF95_RX_CODING_RATE                        =0xe0
RF95_MODEM_STATUS_CLEAR                    =0x10
RF95_MODEM_STATUS_HEADER_INFO_VALID        =0x08
RF95_MODEM_STATUS_RX_ONGOING               =0x04
RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      =0x02
RF95_MODEM_STATUS_SIGNAL_DETECTED          =0x01

# RF95_HOP_CHANNEL                         0x1c
RF95_PLL_TIMEOUT                           =0x80
RF95_RX_PAYLOAD_CRC_IS_ON                  =0x40
RF95_FHSS_PRESENT_CHANNEL                  =0x3f

# RF95_MODEM_CONFIG1                       0x1d
RF95_BW                                    =0xf0

RF95_BW_7_8KHZ                             =0x00
RF95_BW_10_4KHZ                            =0x10
RF95_BW_15_6KHZ                            =0x20
RF95_BW_20_8KHZ                            =0x30
RF95_BW_31_25KHZ                           =0x40
RF95_BW_41_7KHZ                            =0x50
RF95_BW_62_5KHZ                            =0x60
RF95_BW_125KHZ                             =0x70
RF95_BW_250KHZ                             =0x80
RF95_BW_500KHZ                             =0x90
RF95_CODING_RATE                           =0x0e
RF95_CODING_RATE_4_5                       =0x02
RF95_CODING_RATE_4_6                       =0x04
RF95_CODING_RATE_4_7                       =0x06
RF95_CODING_RATE_4_8                       =0x08
RF95_IMPLICIT_HEADER_MODE_ON               =0x01

# RF95_MODEM_CONFIG2                       0x1e
RF95_SPREADING_FACTOR                      =0xf0
RF95_SPREADING_FACTOR_64CPS                =0x60
RF95_SPREADING_FACTOR_128CPS               =0x70
RF95_SPREADING_FACTOR_256CPS               =0x80
RF95_SPREADING_FACTOR_512CPS               =0x90
RF95_SPREADING_FACTOR_1024CPS              =0xa0
RF95_SPREADING_FACTOR_2048CPS              =0xb0
RF95_SPREADING_FACTOR_4096CPS              =0xc0
RF95_TX_CONTINUOUS_MOE                     =0x08

RF95_PAYLOAD_CRC_ON                        =0x04
RF95_SYM_TIMEOUT_MSB                       =0x03

# RF95_TCXO                                0x4b
RF95_TCXO_TCXO_INPUT_ON                    =0x10

# RF95_PA_DAC                              0x4d
RF95_PA_DAC_DISABLE                        =0x04
RF95_PA_DAC_ENABLE                         =0x07

SPI_WRITE_MASK                             =0x80


                     
class RFM95(object):
    # Initialise the Driver transport hardware and software. Make sure the Driver is 
    # properly configured before calling init(). 
    def __init__(self, freqBand, nodeID, networkID, intPin=18, rstPin=24, spiBus=0, spiDevice=0):
        self.freqBand = freqBand
        self.address = nodeID
        self.networkID = networkID
        self.intPin = intPin
        self.rstPin = rstPin
        self.spiBus = spiBus
        self.spiDevice = spiDevice
        self.mode = ""
        self.config=[]
        self.rxBufValid=False
        self.lastMsg=""
        self.bufLen=0
        self.promiscuous=True
        self.txHeaderTo=0
        self.buf=[]
        self.lastRSSI=0

        
        
        # Defines register values for a set of modem configuration registers
        # that can be passed to setModemRegisters() if none of the choices in
        # ModemConfigChoice suit your need setModemRegisters() writes the
        # register values from this structure to the appropriate registers
        # to set the desired spreading factor, coding rate and bandwidth
        self.ModemConfig=['reg_1d', # Value for register RF95_MODEM_CONFIG1
                          'reg_1e', # Value for register RF95_MODEM_CONFIG2
                          'reg_26'] # Value for register RF95_MODEM_CONFIG3


        self.ModemConfigChoice = ['Bw125Cr45Sf128', # Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
                                  'Bw500Cr45Sf128', # Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
                                  'Bw31_25Cr48Sf512',	# Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
                                  'Bw125Cr48Sf4096' # Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
                                  ]             

        self.MODEM_CONFIG_TABLE = [[0x72,   0x74,    0x00], # Bw125Cr45Sf128 (the chip default)
                                  [0x92,   0x74,    0x00], # Bw500Cr45Sf128
                                  [0x48,   0x94,    0x00], # Bw31_25Cr48Sf512
                                  [0x78,   0xc4,    0x00]] # Bw125Cr48Sf4096
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.intPin, GPIO.IN)
        GPIO.setup(self.rstPin, GPIO.OUT)
        
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(self.spiBus, self.spiDevice)
        self.spi.max_speed_hz = 4000000
        
        # Hard reset the RFM module
        GPIO.output(self.rstPin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.rstPin, GPIO.LOW)
        time.sleep(0.1)
        
        
        # Get the device type and check it. This also tests whether we are really connected to a device.
        deviceVersion = self.spiRead(RF95_VERSION)
        if deviceVersion == 00 or deviceVersion == 0xff:
            return False
        # Set sleep mode, so we can also set LORA mode:
        self.spiWrite(RF95_OP_MODE, RF95_MODE_SLEEP | RF95_LONG_RANGE_MODE)
        # Wait for sleep mode to take over from say, CAD
        time.sleep(0.01)
        if (self.spiRead(RF95_OP_MODE) != (RF95_MODE_SLEEP | RF95_LONG_RANGE_MODE)):
            print('No device present?')
            #return False # No device present?
            
        # Set up FIFO
        # We configure so that we can use the entire 256 byte FIFO for either receive
        # or transmit, but not both at the same time
        self.spiWrite(RF95_FIFO_TX_BASE_ADDR, 0)
        self.spiWrite(RF95_FIFO_RX_BASE_ADDR, 0)
        
        # Packet format is preamble + explicit-header + payload + crc
        # Explicit Header Mode
        # payload is TO + FROM + ID + FLAGS + message data
        # RX mode is implmented with RXCONTINUOUS
        # max message data length is 255 - 4 = 251 octets
        self.setModeIdle()
        
        # Set up default configuration
        # No Sync Words in LORA mode.
        self.setModemConfig(0) # Radio default
        #self.setModemConfig(Bw125Cr48Sf4096) # slow and reliable?
        self.setPreambleLength(8) # Default is 8
        # An innocuous ISM frequency, same as RF22's
        self.setFrequency(self.freqBand)
        # Lowish power
        self.setTxPower(13)
        
        #return True
    # Reads a single register from the SPI device
    # \param[in] reg Register number
    # \return The value of the register
    def spiRead(self,reg):
        return self.spi.xfer([reg & 0x7F, 0])[1]
    
    # Writes a single byte to the SPI device
    # \param[in] reg Register number
    # \param[in] val The value to write
    
    def spiWrite(self,reg, val):
        if type(val)==str:
            val=ord(val)
        self.spi.xfer([reg | 0x80, val])

    # Reads a number of consecutive registers from the SPI device using burst read mode
    # \param[in] reg Register number of the first register
    # \param[in] dest Array to write the register values to. Must be at least len bytes
    # \param[in] len Number of bytes to read
    # \return Some devices return a status byte during the first data transfer. This byte is returned.
    # it may or may not be meaningfule depending on the the type of device being accessed.
    def spiBurstRead(self,reg,dest,length):
        message=[]
        for i in range(length):
            message.append(self.spi.xfer([reg & 0x7F, 0])[1])
        self.buf=message
        lastMessage=''
        for i in self.buf:
            lastMessage+=chr(i)
        self.lastMessage=lastMessage
        #self.lastRxID=message[1]
        #self.lastRxDest=message[0]
        
    def getRSSI(self):
        self.lastRSSI=self.spi.xfer([RF95_PKT_RSSI_VALUE & 0x7F, 0])[1]-137
        
    def spiBurstWrite(self,reg,src,length):
        for i in list(src):
            i=ord(i)
            #self.spi.xfer([reg | 0x80, len(src) + 3, toAddress, self.address, ack] + [int(ord(i)) for i in list(buff)])
            self.spi.xfer([reg | 0x80, i])
        
    
    # Prints the value of all chip registers.
    def printRegisters(self):
        registers = [ 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]
        for i in range(len(registers)):
            print(registers[i],': ',self.spiRead(registers[i]))
        return True
        
    
    # Sets all the registers required to configure the data modem in the RF95/96/97/98, 
    # including the bandwidth, spreading factor etc. You can use this to configure the 
    # modem with custom configurations if none of the canned configurations in ModemConfigChoice suit you. 
    # \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    def setModemRegisters(self,config):
        self.spiWrite(RF95_MODEM_CONFIG1, config[0])
        self.spiWrite(RF95_MODEM_CONFIG2, config[1])
        self.spiWrite(RF95_MODEM_CONFIG3, config[2])
    
    # Select one of the predefined modem configurations. If you need a modem 
    # configuration not provided here, use setModemRegisters() with your own ModemConfig.
    # \param[in] index The configuration choice.
    # \return true if index is a valid choice.
    def setModemConfig(self,index):
        if (index > (len(self.MODEM_CONFIG_TABLE) / len(self.ModemConfig))):
            return False
        self.setModemRegisters(self.MODEM_CONFIG_TABLE[index])
        return True
    
    # Get the values of one of the predefined modem configurations. 
    # \param[in] index The configuration choice.
    # \param[in] config A ModemConfig structure that will contains values of the modem configuration values.
    # \return true if index is a valid choice and config has been filled with values
    def getModemConfig(self,index, config):
        if (index > (len(self.MODEM_CONFIG_TABLE) / len(self.ModemConfig))):
            return False
        self.config=self.MODEM_CONFIG_TABLE[index]
        return True
        
    
    # Tests whether a new message is available from the driver. 
    # On most drivers, this will also put the Driver into RHModeRx mode until 
    # a message is actually received by the transport, when it wil be returned 
    # to RHModeIdle. This can be called multiple times in a timeout loop.
    # \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    def available(self):
        # Read the interrupt register
        irq_flags = self.spiRead(RF95_IRQ_FLAGS)
        if (self.mode == 'RHModeRx' and irq_flags and RF95_RX_DONE):
            # Have received a packet
            packetLen = self.spiRead(RF95_RX_NB_BYTES)
            # Reset the fifo read ptr to the beginning of the packet
            self.spiWrite(RF95_FIFO_ADDR_PTR, self.spiRead(RF95_FIFO_RX_CURRENT_ADDR))
            self.spiBurstRead(RF95_FIFO, self.buf, packetLen)
            self.bufLen = packetLen
            self.spiWrite(RF95_IRQ_FLAGS, 0xff) # Clear all IRQ flags
            
            # Remember the RSSI of this packet
            # this is according to the doc, but is it really correct?
            # weakest receiveable signals are reported RSSI at about -66
            #self.lastRssi = self.spiRead(RF95_PKT_RSSI_VALUE) - 137
            #self.lastRxID=self.buf[1]
            #self.lastRxDest=self.buf[0]
            self.getRSSI()
            #We have received a message.
            self.validateRxBuf()
            if (self.rxBufValid):
                self.setModeIdle() # Got one.
            elif (self.mode == 'RHModeCad' and irq_flags and RF95_CAD_DONE):
                self.cad = irq_flags and RF95_CAD_DETECTED
                self.setModeIdle()
            
            self.spiWrite(RF95_IRQ_FLAGS, 0xff) # Clear all IRQ flags
        if (self.mode == 'RHModeTx'):
            return False
        self.setModeRx()
        return self.rxBufValid # Will be set by the interrupt handler when a good message is received

    
    # Turns the receiver on if it not already on. If there is a valid message 
    # available, copy it to buf and return true else return false. If a message 
    # is copied, *length is set to the length (Caution, 0 length messages are permitted). 
    # You should be sure to call this function frequently enough to not miss any messages. 
    # It is recommended that you call it in your main loop.
    # \param[in] buf Location to copy the received message
    # \param[in,out] length Pointer to available space in buf. Set to the actual number of octets copied.
    # \return true if a valid message was copied to buf.
    def recv(self,buf,length):
        
        if (self.available()==False):
            return False
        if (buf and packetLen):
            #self.lastRxID=
            # Skip the 4 headers that are at the beginning of the rxBuf
            if (packetLen > length-RF95_HEADER_LEN):
                packetLen = length-RF95_HEADER_LEN
        self.clearRxBuf() # This message accepted and cleared
        return True
        

    
    
    # Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    # Then optionally waits for Channel Activity Detection (CAD) 
    # to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    # Then loads a message into the transmitter and starts the transmitter. Note that a message length
    # of 0 is permitted. 
    # \param[in] data Array of data to be sent
    # \param[in] length Number of bytes of data to send
    # specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
    # \return true if the message length was valid and it was correctly queued for transmit. Return false
    # if CAD was requested and the CAD timeout timed out before clear channel was detected.
    def send(self,data,length):
        if (length > RF95_MAX_MESSAGE_LEN):
            return False
        self.waitPacketSent() # Make sure we dont interrupt an outgoing message
        self.setModeIdle()
        if (self.waitCAD()==False): # Check channel activity
            return False
        # Position at the beginning of the FIFO
        self.spiWrite(RF95_FIFO_ADDR_PTR, 0)
        # The headers
        self.spiWrite(RF95_FIFO, self.txHeaderTo)
        self.spiWrite(RF95_FIFO, self.address)
        self.spiWrite(RF95_FIFO, self.address)
        self.spiWrite(RF95_FIFO, 0)
        # The message data
        self.spiBurstWrite(RF95_FIFO, data, length)
        self.spiWrite(RF95_PAYLOAD_LENGTH, length + RF95_HEADER_LEN)
        
        self.setModeTx() # Start the transmitter
        # when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
        return True
    
    # Blocks until the current message (if any) has been transmitted.
    # \return true on success, false if the chip is not in transmit mode or other transmit failure
    def waitPacketSent(self):
        if (self.mode != 'RHModeTx'):
            return False
        while (self.spiRead(RF95_IRQ_FLAGS)==False and RF95_TX_DONE):
            pass
        # A transmitter message has been fully sent
        #txGood+=1
        self.setModeIdle() # Clears FIFO
        return True
            
    
    # Sets the length of the preamble in bytes. Caution: this should be set to the same
    # value on all nodes in your network. Default is 8. Sets the message preamble length in RF95_REG_??_PREAMBLE_?SB
    # \param[in] numbytes Preamble length in bytes.  
    def setPreambleLength(self,numbytes):
        self.spiWrite(RF95_PREAMBLE_MSB, numbytes >> 8)
        self.spiWrite(RF95_PREAMBLE_LSB, numbytes & 0xff);
    
    # Returns the maximum message length available in this Driver.
    # \return The maximum legal message length
    def maxMessageLength(self):
        return RF95_MAX_MESSAGE_LEN
    
    
    # Sets the transmitter and receiver centre frequency.
    # \param[in] centre Frequency in MHz. 137.0 to 1020.0. Caution: RFM95/96/97/98 comes in several
    # different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    # \return true if the selected frquency centre is within range
    def setFrequency(self,centre):
        frf = int((centre * 1000000.0) / RF95_FSTEP)
        self.spiWrite(RF95_FRF_MSB, (frf >> 16) & 0xff)
        self.spiWrite(RF95_FRF_MID, (frf >> 8) & 0xff)
        self.spiWrite(RF95_FRF_LSB, frf & 0xff)
        return True
    
    # If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, disables them.
    def setModeIdle(self):
        if (self.mode != 'RHModeIdle'):
            self.spiWrite(RF95_OP_MODE, RF95_MODE_STDBY)
            self.mode = 'RHModeIdle'
    
    # If current mode is Tx or Idle, changes it to Rx. Starts the receiver in the RF95/96/97/98.
    def setModeRx(self):
        if (self.mode != 'RHModeRx'):
            self.spiWrite(RF95_OP_MODE, RF95_MODE_RXCONTINUOUS)
            self.spiWrite(RF95_DIO_MAPPING1, 0x00) # Interrupt on RxDone
            self.mode = 'RHModeRx'
    
    # If current mode is Rx or Idle, changes it to Rx. Starts the transmitter in the RF95/96/97/98.
    def setModeTx(self):
        if (self.mode != 'RHModeTx'):
            self.spiWrite(RF95_OP_MODE, RF95_MODE_TX)
            self.spiWrite(RF95_DIO_MAPPING1, 0x40) # Interrupt on TxDone
            self.mode = 'RHModeTx'
    
    # Sets the transmitter power output level, and configures the transmitter pin.
    # Be a good neighbour and set the lowest power level you need.
    # Some SX1276/77/78/79 and compatible modules (such as RFM95/96/97/98) 
    # use the PA_BOOST transmitter pin for high power output (and optionally the PA_DAC)
    # while some (such as the Modtronix inAir4 and inAir9) 
    # use the RFO transmitter pin for lower power but higher efficiency.
    # You must set the appropriate power level and useRFO argument for your module.
    # Check with your module manufacturer which transmtter pin is used on your module
    # to ensure you are setting useRFO correctly. Failure to do so will result in very low 
    # transmitter power output. Caution: legal power limits may apply in certain countries.
    # After init(), the power will be set to 13dBm, with useRFO false (ie PA_BOOST enabled).
    # \param[in] power Transmitter power level in dBm. For RFM95/96/97/98 LORA with useRFO false, 
    # valid values are from +5 to +23.
    # For Modtronix inAir4 and inAir9 with useRFO true (ie RFO pins in use), 
    # valid values are from -1 to 14.
    # \param[in] useRFO If true, enables the use of the RFO transmitter pins instead of
    # the PA_BOOST pin (false). Choose the correct setting for your module.
    def setTxPower(self,power, useRFO=False):
        # Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
        # for the transmitter output
        if (useRFO):
            if (power > 14):
                power = 14
            if (power < -1):
                power = -1
            self.spiWrite(RF95_PA_CONFIG, RF95_MAX_POWER | (power + 1))
        else:
            if (power > 23):
                power = 23
            if (power < 5):
                power = 5
            # For RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
            # RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
            # for 21, 22 and 23dBm
            if (power > 20):
                self.spiWrite(RF95_PA_DAC, RF95_PA_DAC_ENABLE)
                power -= 3
            else:
                self.spiWrite(RF95_PA_DAC, RF95_PA_DAC_DISABLE)
            # RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
            # pin is connected, so must use PA_BOOST
            # Pout = 2 + OutputPower.
            # The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
            # but OutputPower claims it would be 17dBm.
            # My measurements show 20dBm is correct
            self.spiWrite(RF95_PA_CONFIG, RF95_PA_SELECT | (power-5))
    
    # Sets the radio into low-power sleep mode. If successful, the transport will stay in sleep mode until woken by 
    # changing mode to idle, transmit or receive (eg by calling send(), recv(), available() etc)
    # Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    # \return true if sleep mode was successfully entered.
    def sleep(self):
        if (self.mode != 'RHModeSleep'):
            self.spiWrite(RF95_OP_MODE, RF95_MODE_SLEEP)
            self.mode = 'RHModeSleep'
        return True
    
    # Bent G Christensen (bentor@gmail.com), 08/15/2016
    # Use the radio's Channel Activity Detect (CAD) function to detect channel activity.
    # Sets the RF95 radio into CAD mode and waits until CAD detection is complete.
    # To be used in a listen-before-talk mechanism (Collision Avoidance) with a reasonable time backoff algorithm.
    # This is called automatically by waitCAD().
    # \return true if channel is in use.  
    def isChannelActive(self):
        # Set mode RHModeCad
        if (self.mode != 'RHModeCad'):
            self.spiWrite(RF95_OP_MODE, RF95_MODE_CAD)
            self.spiWrite(RF95_DIO_MAPPING1, 0x80) # Interrupt on CadDone
            self.mode = 'RHModeCad'
        #while self.mode=='RHModeCad':
        #    pass
        return False
        
    
    # Enable TCXO mode.
    # Call this immediately after init(), to force your radio to use an external 
    # frequency source, such as a Temperature Compensated Crystal Oscillator (TCXO).
    # See the comments in the main documentation about the sensitivity of this radio to
    # clock frequency especially when using narrow bandwidths.
    # Leaves the module in sleep mode. Caution, this function has not been tested by us.
    def enableTCXO(self):
        while ((self.spiRead(RF95_TCXO) & RF95_TCXO_TCXO_INPUT_ON) != RF95_TCXO_TCXO_INPUT_ON):
            self.sleep()
            self.spiWrite(RF95_TCXO, (self.spiRead(RF95_TCXO) | RF95_TCXO_TCXO_INPUT_ON))
            
    
    #### PROTECTED FUNCTIONS (DO NOT CALL AS USER)
    
    # This is a low level function to handle the interrupts for one instance of RF95.
    # Called automatically by isr*()
    def handleInterrupt(self):
        pass
    
    # Examine the revceive buffer to determine whether the message is for this node.
    def validateRxBuf(self):
        if(self.bufLen<4):
            return
        # Extract the 4 headers
        self.rxHeaderTo    = self.buf[0]
        self.rxHeaderFrom  = self.buf[1]
        self.rxHeaderId    = self.buf[2]
        self.rxHeaderFlags = self.buf[3]
        if (self.promiscuous or self.rxHeaderTo == self.thisAddress or self.rxHeaderTo == BROADCAST_ADDRESS):
            #rxGood+=1
            self.rxBufValid = True
            
    # Clear our local receive buffer
    def clearRxBuf(self):
        self.rxBufValid = False
        self.bufLen = 0

    # Low level interrupt service routine for device connected to interrupt 0
    def isr0(self):
        pass
    
    
    # Low level interrupt service routine for device connected to interrupt 1
    def isr1(self):
        pass
    
    # Low level interrupt service routine for device connected to interrupt 1
    def isr2(self):
        pass
    
    def waitCAD(self):
        now=time.time()
        while self.isChannelActive():
            if time.time()-now>self.timeout:
                return False
        return True
        
    
    
    
    
