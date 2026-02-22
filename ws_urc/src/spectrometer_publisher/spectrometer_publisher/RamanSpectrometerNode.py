import rclpy
import os
import serial
import time
import numpy as np
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory

from custom_interfaces.msg import Spectrometer

#making node for raman spectrometer using spectro.py to work with BTC100 spectrometer
#!/bin/python3

# Spectro.py
# Tom Trebisky  2-20-2025
#
# B and W Tech Model BTC100-S spectrometer.

# Used in srp, for the spectrometer, heavy thanks to Tom for his documentation and work

save_path = "data/spectrum"  # Adjust this path as needed

# First do: dnf install python3-pyserial

# When powered up, it spits out a bunch of characters
# that so far make no sense, then prints a nice
# greeting at 9600 baud.

# Sending 'a' is the command to tell it to communicate in ascii.
# It should respond with "ACK"
# The 'A' command is different (it sets averaging)

# Baudrate codes for the BTC100 "K" command
BAUD_115200 = 0
BAUD_38400 = 1
BAUD_19200 = 2
BAUD_9600 = 3
BAUD_4800 = 4
BAUD_2400 = 5
BAUD_1200 = 6
BAUD_600 = 7

class SpectroError ( Exception ) :
    pass

# At one time I was having trouble getting Python to
# set up the serial port baud rate correctly, but I was
# able to run picocom to set up the port, then inherit
# the setup.  This no longer seems necessary.
#  setup_cmd = "picocom --noreset -b 9600 /dev/ttyUSB0"
#  setup_cmd = "picocom -qrX -b 9600 " + device
#  print ( setup_cmd )

# The python pyserial "read()" method reads one byte by default.
# You can ask for more, but will need to wait for the timeout
# if that many are not available.
# There is also read_until () which reads until a certain
# sequence is found ('\n' by default).
# You could do:  read_until ( "\r\n", 32 )
# HOWEVER, asking for more than you will actually get
# means that you will wait for a timeout

class Spectro () :
    def __init__ ( self, device ) :

        self.ser = self.connect ( device )

        # returning None here is useless, the caller still
        # gets a "Spectro" object.
        # The caller should check self.ser
        if self.ser == None :
            print ( "Init fails" )
            return None

        self.average ( 1 )
        self.average_val = 1

        self.integ ( 50 )
        self.integ_val = 50

    def connect ( self, device ) :
        baud = 115200
        TIMEOUT = 2

        try:
            ser = serial.Serial ( device, baud, timeout=TIMEOUT )
        except serial.SerialException as e:
            print ( f"Cannot open serial port {device}: {e}")
            return None
        except FileNotFoundError as e:
            print ( f"Cannot open serial port {device}: {e}")
            return None

        print ( "Using port " + ser.name)

        ser.write ( "a\n".encode('ascii') )
        buf = ser.read ( 8 )

        if len(buf) == 8 :
            print ( "init found device at 115200" )
            print ( "Init OK" )
            return ser

        print ( "Init trouble, probably at 9600 :: ", len(buf) )

        print ( "init resetting port to 9600" )
        ser.baudrate = 9600

        # this will fail, but it gets things in
        # the mood to accept the baud rate change
        # We typically see a 3 byte response
        # It will timeout, but that is OK in init
        # and it only happens when the baud is wrong
        ser.write ( "a\n".encode('ascii') )
        buf = ser.read ( 8 )
        #print ( "cleanup got ", len(buf) )

        # OK, change baud rate
        # This gets an 8 bytes response
        #spec_baud ( BAUD_115200 )
        rate = BAUD_115200
        cmd = f"K{rate}\n"
        ser.write ( cmd.encode('ascii') )
        buf = ser.read_until ( "\r\n", 9 )
        ser.baudrate = 115200

        # check again with the "a" command
        ser.write ( "a\n".encode('ascii') )
        buf = ser.read ( 8 )
        if len(buf) == 8 :
            print ( "Init OK" )
            return ser

        return None

    def finish ( self ) :
        self.ser.close()
        print ( "Done" )

    def set_integ ( self, val ) :
        if val < 50 :
            val = 50
        if val > 65000 :
            val = 65000
        self.integ_val = val
        self.integ ( val )

    def get_integ ( self ) :
        return self.integ_val

    def set_average ( self, val ) :
        self.average_val = val
        self.average ( val )

    def get_average ( self ) :
        return self.average_val

    # Tell the BTC100 to change baud rate
    # (this is a bit of a cat and mouse game as we
    #  must also change our baud rate to match)
    #
    # K{int}: Set the baud rate
    # We always get the echo
    # We get only part of the ACK when the rate changes
    # (the last byte gets mangled)

    def baud ( self, rate ) :
        ser = self.ser

        cmd = f"K{rate}\n"
        ser.write ( cmd.encode('ascii') )
        buf = ser.read_until ( "\r\n", 9 )
        # We get 9 bytes if going from 9600 to 9600
        # We get 8 bytes when actually changing
        #print ( "Baud got ", len(buf) )
        #monitor ()

        if rate == BAUD_9600 :
            ser.baudrate = 9600
        if rate == BAUD_19200 :
            ser.baudrate = 19200
        if rate == BAUD_115200 :
            ser.baudrate = 115200

    # useful during debug, no longer used.
    # read from serial port until timeout
    def monitor ( self ) :
        # loop ends on a read timeout
        while True:
            reply = ser.read()
            if len(reply) == 0 :
                print ( "Timeout" )
                break

            try :
                areply = reply.decode('ascii')
            except UnicodeDecodeError :
                print ( "received: ", len(reply), reply, " EX" )
                break

            print ( "received: ", len(reply), reply, areply )

    # Set how many spectra to average
    def average ( self, val ) :
        cmd = f"A{val}\n"
        #print ( "set averaging: ", cmd )
        expect = len(cmd) + 1 + 5
        self.ser.write ( cmd.encode('ascii') )
        buf = self.ser.read_until ( "\r\n", expect )

    # Set integration time in ms (50-65000)
    def integ ( self, val ) :
        cmd = f"I{val}\n"
        #print ( "set integ: ", cmd )
        expect = len(cmd) + 1 + 5
        self.ser.write ( cmd.encode('ascii') )
        buf = self.ser.read_until ( "\r\n", expect )

    # reset the spectrometer (never used)
    def reset ( self ) :
        cmd = "Q\n"
        self.ser.write ( cmd.encode('ascii') )
        buf = self.ser.read_until ( "\r\n", 8 )

    # We get an 8 byte reply
    # 61 0d 0a 41 43 4b 0d 0a
    # First we get an echo as "a\r\n"
    # Then we get an ACK as "ACK\r\n"
    def ascii ( self ) :
        # Read until wants byte array
        term = '\r\n'.encode('ascii')
        ser = self.ser

        # We must encode to bytes
        ser.write ( "a\n".encode('ascii') )

        # read echo
        buf = ser.read_until ( term, 8 )
        if len(buf) != 3 :
            print ( "Trouble in spec_ascii() :: ", len(buf) )

        # read ACK
        buf = ser.read_until ( term, 8 )
        if len(buf) != 5 :
            print ( "Trouble in spec_ascii() :: ", len(buf) )

        #print ( "Response gives us ", len(buf), " bytes" )
        #print ( buf )
        #print ( buf.decode('ascii') )
        # monitor ()

    def binary ( self ) :
        ser = self.ser

        ser.write ( "b\n".encode('ascii') )
        buf = ser.read ( 8 )
        if len(buf) != 8 :
            print ( "Trouble in spec_binary()" )

    # for windows, do we need to add '\r\n' ?
    # I don't think so, but we should experiment and see
    # making sure it does the right thing.
    def asave ( self, path, lines ) :
        with open ( path, 'w' ) as file :
            for line in lines :
                file.write ( line + "\n" )

    def bsave ( self, path, vals ) :
        with open ( path, 'w' ) as file :
            for val in vals :
                file.write ( f"{val:05d}\n" )

    def read1 ( self ) :
        raw = self.ser.read ()
        if raw == None :
            print ( "bscan/read1 trouble A" )
            raise SpectroError ( "bscan2/read1" )
        if len(raw) != 1 :
            print ( "bscan/read1 trouble B :: ", len(raw) )
            raise SpectroError ( "bscan2/read1" )

        return raw[0]

    # "scan" the spectrum, but read out in binary
    # Must be upper case S
    # I typically see 2091 bytes or so in the raw image.
    #  (compage to 14336 for the ascii scan)
    # There is no null byte at the end.
    # We get 2049 values -- I have no idea what to make of that.

    # In binary mode, each pixel is compared to the previous value.
    # If the value is +/- 127, a single signed 8-bit int encodes the difference.
    # If the difference is greater, a three-byte sequence is sent:
    #  0x80 (flag value that the next byte is the full value),
    #  {high order bits of the 16-bit uint}, {low order bits of the 16-bit uint}.

    # Improved version of bscan.
    # Because the binary protocol does nothing to show us termination
    # We decode the bytes on the fly to know when to stop, thus avoiding
    # the penalty of a timeout.
    # I time 3 seconds to call this 10 times,
    #  so 0.3 seconds per scan

    # 2-21-2025  I was having trouble when I set averaging and/or
    # integration values too high.  The issue is that the serial read
    # times out waiting for the data to start.  So a delay is in
    # order before we expect data.

    def bscan ( self ) :
        ser = self.ser

        # Python sleep allows floats
        my_delay = (3 * self.average_val * self.integ_val) / (10 * 50)
        if my_delay > 1.0 :
            time.sleep ( my_delay )

        self.binary ()
        ser.write ( "S\n".encode('ascii') )
        # discard the echo
        buf = ser.read ( 3 )
        #print ( len(buf) )
        # discard the ACK
        buf = ser.read ( 5 )

        # Covers weird cases
        val = 0

        try:
            out = []
            while ( True ) :
                bb = self.read1 ()
                if bb == 128 :
                    b1 = self.read1 ()
                    b2 = self.read1 ()
                    val = (b1 << 8) + b2
                    #print ( " --- >> ", val )
                else :
                    if bb > 127 :
                        bb = bb - 256
                    val += bb
                    #print ( bb, " >> ", val )
                out.append ( val )
                if len(out) > 2048 :
                    break
        except SpectroError as e:
            return None
        
        # XXX - here we delete the first item to get 2048
        # but maybe we should delete the last??
        return out[1:]

    # Read out via ascii.
    # -- there is little reason to use this with bscan available.
    #
    # An ascii spectrum file is 14336 bytes
    # This is 2048 * 7 (5 digits + "\r\n"
    # Must be upper case S
    # Takes about 2 seconds at 115200 baud
    def ascan ( self ) :
        self.ascii ()
        ser = self.ser

        ser.write ( "S\n".encode('ascii') )
        # discard the echo
        buf = ser.read ( 3 )
        # discard the ACK
        buf = ser.read ( 5 )

        # this avoids a timeout (the until zero byte)
        raw_image = ser.read_until ( b'\x00', 16000 )
        #raw_image = ser.read ( 16000 )
        # print ( raw_image )
        # print ( len(raw_image) )
        if len(raw_image) != 14337 :
            print ( "spec_scan fishy 1 : ", len(raw_image) )


        # dump the null byte at the end
        # and the last \r\n
        image = raw_image[:-3].decode ( 'ascii' )
        im = image.split ( "\r\n" )
        # print ( len(im) )
        if len(im) != 2048 :
            print ( "spec_scan fishy 2 : ", len(im) )
        return im

# def main () :
#     device = "/dev/ttyUSB0"
#     spec = Spectro ( device )

#     if spec.ser == None :
#         print ( "Cannot connect to spectrometer" )
#         return

#     #spec.set_average ( 10 )
#     #spec.set_integ ( 100 )

#     #lines = spec.ascan ()
#     vals = spec.bscan ()

#     #print ( lines )
#     print ( vals )

#     #spec.asave ( save_path + ".txt", lines )
#     spec.bsave ( save_path + ".txt", vals )

#     spec.finish ()


# if __name__ == "__main__" :
#     main ()
# # THE END


class RamanSpectrometerNode(Node):
    def __init__(self):
        super().__init__('raman_spectrometer_node')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('timer_period', 1.0)
        
        device = self.get_parameter('device').value
        timer_period = self.get_parameter('timer_period').value
        
        # Initialize the spectrometer with device path
        try:
            self.spectrometer = Spectro(device)
            if self.spectrometer.ser is None:
                self.get_logger().error(f'Failed to connect to spectrometer on {device}')
                raise RuntimeError('Spectrometer connection failed')
        except Exception as e:
            self.get_logger().error(f'Error initializing spectrometer: {e}')
            raise
        
        # Load wavelength calibration or use pixel indices
        self.wavelengths = self.load_wavelengths()
        
        self.publisher_ = self.create_publisher(Spectrometer, 'raman_spectrum', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Raman spectrometer node initialized on {device}')
    
    def load_wavelengths(self):
        """Load or generate wavelength array for the 2048 pixels"""
        return list(range(2048))

        # Try to load calibration data
        self.get_logger().warn('Using pixel indices as wavelengths')
        return list(range(2048))
    
    def timer_callback(self):
        try:
            # Use bscan() to get spectrum intensities (2048 values)
            self.get_logger().info('1')
            intensities = self.spectrometer.bscan()
            self.get_logger().info('2')
            
            if intensities is None or len(intensities) != 2048:
                self.get_logger().error('Failed to get valid spectrum data')
                return
            
            # Publish the spectrum message
            self.get_logger().info('3')
            msg = Spectrometer()
            self.get_logger().info('4')
            msg.wavelengths = self.wavelengths
            self.get_logger().info('5')
            msg.intensity = [int(x) for x in intensities]
            self.get_logger().info('6')
            self.publisher_.publish(msg)
            self.get_logger().info('Published Raman spectrum data')
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

def main(args=None):

    rclpy.init(args=args)
    raman_spectrometer_node = RamanSpectrometerNode()
    rclpy.spin(raman_spectrometer_node)
    
    # Clean up and shutdown
    raman_spectrometer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()