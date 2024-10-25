
import can

# Candlelight firmware on Linux
bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)

msg = can.Message(arbitration_id=0x00D,                  
                data=[0x00, 0x00, 0x20, 0x41, 0x00, 0x00, 0x00, 0x00],                  
                is_extended_id=False)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent")