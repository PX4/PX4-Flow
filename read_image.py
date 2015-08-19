import usb.core, usb.util
import numpy as np
import matplotlib.pyplot as plt
import struct

dev = usb.core.find(idVendor=0x26ac, idProduct=0x0015)
endpoint = dev[0][(2,0)][0]

SIZE = 352
image = np.zeros((SIZE, SIZE), dtype='uint8')
imshow = plt.imshow(image, cmap=plt.cm.gray)
imshow.norm.vmin = 0
imshow.norm.vmax = 255
plt.ion()
plt.show()

HEADER_SIZE = 16
READ_SIZE = HEADER_SIZE + SIZE*SIZE

while True:
    
    data = endpoint.read(64 * (1 + (READ_SIZE) / 64), timeout=20000) # Round up to include the zero-length packet
    
    if len(data) != READ_SIZE:
        # We read a partial image from killing a previous instance of This
        # program in the middle of a transfer. We should be using an interface
        # alternate setting to enable and reset the endpoint.
        print "Ignoring partial buffer of", len(data)
        continue
        
    (flags, timestamp, exposure, reserved) = struct.unpack('<IIII', data[:HEADER_SIZE])
    
    print "Time: ", timestamp, "Exposure:", exposure
        
    image = np.frombuffer(data[HEADER_SIZE:], dtype='uint8').reshape(SIZE, SIZE)
    imshow.set_data(image)
    plt.pause(1e-9)
