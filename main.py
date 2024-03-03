import serial
port = serial.Serial('/dev/ttyUSB0',115200)
def readFrame()->bytearray:
    frame = bytearray()
    while True:
        if port.read()[0] == 0xAA:
            if port.read()[0] == 0x55:
                frame.append(0xAA)
                frame.append(0x55)
                frame.append(port.read()[0])
                lsn = port.read()[0]
                frame.append(lsn)
                remain_byte_number = 6 + 2*lsn
                print(remain_byte_number)
                for i in range(remain_byte_number):
                    frame.append(port.read()[0])
                return frame
    

def print_hex(bytes):
  l = [hex(int(i)) for i in bytes]
  print(" ".join(l))

frame = readFrame()
print_hex(frame)
frame = readFrame()
print_hex(frame)

