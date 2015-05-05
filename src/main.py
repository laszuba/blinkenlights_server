#!/usr/bin/python

from OSC import *
import serial
import sys
import time
import threading
import re
from math import *
import subprocess

class ChannelData:
    def __init__(self, name, address):
        self.name = name
        self.address = address
        self.state = {
            'beat_gate' : {
                'enabled'   : False,
                'changed'   : True
            },
            'led' : {
                'setpoint'  : 127,
                'changed'   : True
            },
            'enabled' : False
        }

class OutputData:
    def __init__(self, address, outputs):
        self.lock = threading.Lock()
        self.address = 0
        self.outputs = outputs

class Sender:

    command_map = {
        'off'       : 0x01,
        'on'        : 0x02,
        'set'       : 0x03,
        'bg_off'    : 0x11,
        'bg_on'     : 0x12,
        'bg_set'    : 0x13
    }

    address_map = {
        'local'     : 0x00,
        'device'    : 0x40,
        'global'    : 0x80
    }

    header = chr(0xFF)

    # Time between updating remote device
    wait_time = 0.1

    def __init__(self, port, rate):
        self.port = port
        self.rate = rate
        self.ser = None
        self.connected = False

    def connect(self):
        self.ser = serial.Serial(self.port, self.rate, timeout=0)
        self.connected = True

    def transmit(self, device):
        while True:
            with device.lock:
                for output in device.outputs:
                    if output.state['led']['changed']:
                        tx_data = 0
                        if output.state['enabled'] == True:
                            tx_data = output.state['led']['setpoint']
                        else:
                            tx_data = 0
                        self.send_packet(device.address, output.address, 'set', data=tx_data)
                        output.state['led']['changed'] = False

                    if output.state['beat_gate']['changed']:
                        bg_enabled = output.state['beat_gate']['enabled']
                        self.send_packet(device.address, output.address, ('bg_off', 'bg_on')[bg_enabled])
                        output.state['beat_gate']['changed'] = False

            time.sleep(Sender.wait_time)

    def send_packet(self, dev_addr, out_addr, cmd, data=0):
        dev_addr = chr(dev_addr & 0x7F)
        out_addr = chr(out_addr & 0x7F)
        byte_cmd = chr(Sender.command_map[cmd] & 0x7F)

        d_high, d_low = chr(0x7F & (data >> 8)), chr(0xFF & data)

        packet = Sender.header + dev_addr + out_addr + byte_cmd + d_high + d_low + '\n'
        self.print_packet(packet)
        if self.connected:
            self.ser.write(packet)

    def print_packet(self, packet):
        text_packet = ":".join("{:02x}".format(ord(c)) for c in packet)
        print "The contents of the packet: {}".format(text_packet)

def xyz_to_hsv((x,y,z)):
    x = (x - 0.5) * 2
    y = (y - 0.5) * 2
    h = degrees(atan2(y, x)) + 180
    s = hypot(x, y)
    v = z

    s = min(max(s, 0.0), 1.0)

    return (h,s,v)

def hsv_to_xyz((h,s,v)):
    h = h - 180
    x = (s * cos(radians(h)) / 2) + 0.5
    y = (s * sin(radians(h)) / 2) + 0.5
    z = v

    x = min(max(x, 0.0), 1.0)
    y = min(max(y, 0.0), 1.0)

    return (x,y,z)

def rgb_to_hsv((r,g,b)):
    M = max(r, g, b)
    m = min(r, g, b)
    chroma = M - m

    if chroma == 0:
        scaled_hue = 0.0
    elif M == r:
        scaled_hue = ((g - b) / chroma) % 6
    elif M == g:
        scaled_hue = ((b - r) / chroma) + 2.0
    elif M == b:
        scaled_hue = ((r - g) / chroma) + 4.0
    else:
        scaled_hue = 0.0

    h = 60.0 * scaled_hue

    if M == 0:
        s = 0.0
    else:
        s = chroma / M

    v = M

    return (h,s,v)

def hsv_to_rgb((h,s,v)):
    # hue is from 0 to 360
    # sat is from 0 to 1
    # val is from 0 to 1

    chroma = v * s
    scaled_hue = h/60
    x = chroma * (1 - abs((scaled_hue % 2) - 1))

    if 0 <= scaled_hue < 1:
        (r1,g1,b1) = (chroma, x, 0.0)
    elif 1 <= scaled_hue < 2:
        (r1,g1,b1) = (x, chroma, 0.0)
    elif 2 <= scaled_hue < 3:
        (r1,g1,b1) = (0.0, chroma, x)
    elif 3 <= scaled_hue < 4:
        (r1,g1,b1) = (0.0, x, chroma)
    elif 4 <= scaled_hue < 5:
        (r1,g1,b1) = (x, 0.0, chroma)
    elif 5 <= scaled_hue <= 6:
        (r1,g1,b1) = (chroma, 0.0, x)
    else:
        (r1,g1,b1) = (0.0,0.0,0.0)

    match_value = v - chroma
    (r,g,b) = (r1 + match_value, g1 + match_value, b1 + match_value)

    return (r,g,b)

def test_colour_funcs():
    tol = 0.0001
    v = 1
    s = 0
    for h in range(0,360, 10):
        #for s in range(0,10):
            #s = s/10.0
        (x,y,z) = hsv_to_xyz((h,s,v))
        (h2,s2,v2) = xyz_to_hsv((x,y,z))
        dh, ds = h - h2, s - s2
        if (dh > tol) or (ds > tol):
            print "error: dh={:3.2f}, h={:3.2f}, h2={:3.2f}, x={:3.2f}, y={:3.2f}".format(dh,h,h2,x,y)

def update_channel(channel, value=None, enabled=None, beat_gate=None):
    with my_device.lock:
        for output in my_device.outputs:
            if output.name == channel:
                if value != None:
                    output.state['led']['setpoint'] = int(round(255 * value))
                    output.state['led']['changed'] = True
                if enabled != None:
                    output.state['enabled'] = enabled
                    output.state['led']['changed'] = True
                if beat_gate != None:
                    output.state['beat_gate']['enabled'] = beat_gate
                    output.state['beat_gate']['changed'] = True

def get_input(default=''):
    usr_input = raw_input().strip()
    if usr_input == '':
        usr_input = default
    return usr_input

def get_channel(prefix, path):
    if path == "{}r".format(prefix):
        c = 'red'
    elif path == "{}g".format(prefix):
        c = 'green'
    elif path == "{}b".format(prefix):
        c = 'blue'
    else:
        c = ''
    return c

def set_colour_callback(path, tags, args, source):
    channel = get_channel("/1/set_", path)

    data = float(args[0])
    last_rgb[channel] = data

    (h,s,v) = rgb_to_hsv((last_rgb['red'], last_rgb['green'], last_rgb['blue']))
    (x,y,z) = hsv_to_xyz((h,s,v))

    #print "XYZ: {:1.2f},{:1.2f},{:1.2f} | HSV: {:3f},{:1.2f},{:1.2f} | RGB: {:1.2f},{:1.2},{:1.2f}".format(x,y,z, h,s,v, r,g,b)

    # hs_msg = OSCMessage("/1/hue_sat")
    # hs_msg.append(x)
    # hs_msg.append(y)
    # v_msg = OSCMessage("/1/value")
    # v_msg.append(z)

    # messages = [hs_msg, v_msg]

    # for msg in messages:
    #     client.send(msg)

    update_channel(channel, value=data)

def set_xyz_callback(path, tags, args, source):
    if path == "/1/hue_sat":
        x = float(args[0])
        y = float(args[1])
        z = last_xyz['z']

    if path == "/1/value":
        x = last_xyz['x']
        y = last_xyz['y']
        z = float(args[0])

    last_xyz['x'], last_xyz['y'], last_xyz['z'] = x, y, z

    (h,s,v) = xyz_to_hsv((x,y,z))
    (r,g,b) = hsv_to_rgb((h,s,v))

    #print "XYZ: {:1.2f},{:1.2f},{:1.2f} | HSV: {:3f},{:1.2f},{:1.2f} | RGB: {:1.2f},{:1.2},{:1.2f}".format(x,y,z, h,s,v, r,g,b)

    r_msg = OSCMessage("/1/set_r")
    r_msg.append(r)
    g_msg = OSCMessage("/1/set_g")
    g_msg.append(g)
    b_msg = OSCMessage("/1/set_b")
    b_msg.append(b)
    messages = [r_msg,g_msg,b_msg]

    for msg in messages:
        client.send(msg)

    for (channel, value) in [('red',r),('green',g),('blue',b)]:
        update_channel(channel, value=value)

def set_enable_callback(path, tags, args, source):
    channel = get_channel("/1/en_", path)

    if args[0] == 1.0:
        enabled = True
    else:
        enabled = False

    update_channel(channel, enabled=enabled)

def set_beat_gate_callback(path, tags, args, source):
    channel = get_channel("/1/bg_", path)

    if args[0] == 1.0:
        enabled = True
    else:
        enabled = False

    update_channel(channel, beat_gate=enabled)

def print_settings():
    print ""
    print "Device Port     : {}".format(ser_port)
    print "Device Baud     : {}".format(ser_rate)
    print "OSC Server IP   : {}".format(serv_address)
    print "OSC Server Port : {}".format(serv_port)
    print "OSC Client IP   : {}".format(client_address)
    print "OSC Client Port : {}".format(client_port)
    print ""

def get_ip():
    try:
        console_output = subprocess.check_output(['ifconfig', 'eth0'])
    except subprocess.CalledProcessError:
        try:
            console_output = subprocess.check_output(['ifconfig', 'en0'])
        except:
            return None

    ip_pattern = re.compile(r"\sinet (\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b)")

    ip = ip_pattern.findall(console_output)

    return ip[0]

ser_port, ser_rate = 'usb', 115200
serv_address, serv_port = "10.0.1.37", 7000
client_address, client_port = "10.0.1.10", 9000

# LED power supply on GPIO 4
led_ps = 4

my_device_channels = [
    ChannelData('blue', 1),
    ChannelData('red', 2),
    ChannelData('green', 3)
]

last_xyz = {'x':0.0, 'y':0.0, 'z':0.5}
last_rgb = {'red':0.0, 'green':0.0, 'blue':0.0}

print "\nOpen OSC BlinkenServe"
print   "====================="
print   "Author : Lee Szuba"
print   "Date   : April 2015\n"

try:
    import RPi.GPIO as gpio
    gpio.setmode(gpio.BCM)
    gpio.setwarnings(False)
    gpio.setup(led_ps, gpio.OUT)
except:
    gpio = None

my_ip = get_ip()
if my_ip is not None:
    print "Found my ip: {}".format(my_ip)
    serv_address = my_ip

if gpio is not None:
    print "Detected Raspberry Pi"
else:
    print "Unable to open gpio, Raspberry Pi features disabled"

try:
    ser_port = serial.serial_for_url("hwgrep://{}".format(ser_port)).port
except:
    print "Hardware grep found no matching ports"
    ser_port = ''

print_settings()
print "Accept the default settings (Y/n) :",

choice = get_input('y').lower()

if choice == 'n':
    print "Enter device port (default: {}):".format(ser_port),
    ser_port = get_input(ser_port)

    print "Enter device baud (default: {}):".format(ser_rate),
    ser_rate = int(get_input(ser_rate))

    print "Enter OSC Server IP (default: {}):".format(serv_address),
    serv_address = get_input(serv_address)

    print "Enter OSC Server Port (default: {}):".format(serv_port),
    serv_port = int(get_input(serv_port))

    print "Enter OSC Client IP (default: {}):".format(client_address),
    client_address = get_input(client_address)

    print "Enter OSC Client Port (default: {}):".format(client_port),
    client_port = int(get_input(client_port))

    print ''
    print_settings()
    print ''
elif (choice == '') or (choice == 'y'):
    pass
else:
    print "Exiting..."
    exit()

try:
    ser_port = serial.serial_for_url("hwgrep://{}".format(ser_port)).port
except:
    print "Hardware grep found no matching ports"
    ser_port = ''

my_device = OutputData(0, my_device_channels)
my_sender = Sender(ser_port, ser_rate)

client_enabled = True

if client_enabled:
    client = OSCClient()
    serv = OSCServer((serv_address, serv_port))#, client=client)
    client.connect((client_address, client_port))
    print "OSC client connected on: osc://{}:{}".format(client.address()[0], client.address()[1])
else:
    client = None
    serv = OSCServer((serv_address, serv_port))

serv.addDefaultHandlers()

print client
print "OSC server started on: osc://{}:{}".format(serv.address()[0], serv.address()[1])

for colour in ['r','g','b']:
    serv.addMsgHandler("/1/set_{}".format(colour), set_colour_callback)
    serv.addMsgHandler("/1/en_{}".format(colour), set_enable_callback)
    serv.addMsgHandler("/1/bg_{}".format(colour), set_beat_gate_callback)

serv.addMsgHandler("/1/hue_sat", set_xyz_callback)
serv.addMsgHandler("/1/value", set_xyz_callback)

#my_message = OSCMessage("/1/set_r")
#my_message.append(0.5)

#client.send(my_message)

try:
    my_sender.connect()
    print "Serial object: {}".format(str(my_sender.ser))
except:
    print "Could not open port: {}".format(my_sender.port)
    print "Continuing in debug mode, packets will not be transmitted."

if gpio is not None:
    # Enable 12V LED power supply
    gpio.output(led_ps, 1)


# Start the serial communcation daemon in another thread
t = threading.Thread(target=Sender.transmit, args=(my_sender, my_device,))
t.daemon = True
t.start()

serv_t = threading.Thread(target=serv.serve_forever)
serv_t.start()

try:
    while True:
        time.sleep(30)

except KeyboardInterrupt:
    print "\nClosing BlinkenServe."
    if gpio is not None:
        # Shut off the LED power supply
        gpio.output(led_ps, 0)
        gpio.cleanup()
    serv.close()
    print "Waiting for Server-thread to finish"
    serv_t.join()
    print "Closing OSCClient"
    client.close()
    print "Done"

sys.exit(0)
