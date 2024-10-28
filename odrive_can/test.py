
# This is to test if we could send message in CAN bus by USB port using can library of Python

import can

bus = can.Bus(
    interface='socketcan', # On Linux, CAN firmware is Candlelight, so the interface is socketcan
    channel='can0', # in cmd, use 'ip link show'(?) to find the can device (usually can0)
    bitrate=500000 # the bitrate of CAN bus, default 500k
    )

msg = can.Message( 
    arbitration_id=0x00D, # arbitration ID is the combination of the device ID we want to send the message and the type of message.
                          # In this code, the device ID is 0x00, and the message ID is 0x0D (velocity control), so the arbitration ID is 0x00 << 5 | 0x0D = 0x0D                
    data=[0x00, 0x00, 0x20, 0x41, 0x00, 0x00, 0x00, 0x00], # The data we send in velocity control mode is float, and we must convert it to hexadecimal using IEEE 754
                                                           # The value is 10.0 that mean 0x00002041 in hexadecimal little-endian, and follow by 4 0x00 byte                   
    is_extended_id=False # the CAN message is standard, not extended
    )

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent")