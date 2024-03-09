#!/usr/bin/python
import serial
#from serial.tools import list_ports
import os
from optparse import OptionParser
import io
import time
import random
import sys
from smtp_stuff import sendMail 
from imap_stuff import checkMessages
import socket
import struct
import asyncore
import requests
import threading
from BaseHTTPServer import BaseHTTPRequestHandler
import SimpleHTTPServer 
import SocketServer
import Queue 
import signal 
from collections import deque
from sbd_packets import assemble_mo_directip_packet
from sbd_packets import parse_mt_directip_packet
from sbd_packets import assemble_mt_directip_response

AVERAGE_SBDIX_DELAY = 1     #TODO: implement randomness, average is ~30s
STDEV_SBDIX_DELAY = 1 
AVERAGE_SBDIX_SUCCESS = 0.9

AVERAGE_CSQ_DELAY = 1
STDEV_CSQ_DELAY = 1

EOL_CHAR = 13
BACKSPACE_CHAR = 8

REG_STATUS_DETACHED = 0
REG_STATUS_NOT_REGISTER = 1
REG_STATUS_REGISTERED = 2
REG_STATUS_DENIED = 3

LOCKED = 1
NOT_LOCKED = 0

echo = True
binary_rx = False
binary_rx_incoming_bytes = 0

ring_enable = False

mt_buffer = ''
mo_buffer = ''
mo_set = False
mt_set = True

momsn = 0
mtmsn = 0

locked = NOT_LOCKED

registered = REG_STATUS_NOT_REGISTER

ser = 0

lat = 0.0
lon = 0.0

user = ''
recipient = ''
incoming_server = ''
outgoing_server = ''
password = ''

mo_ip = '127.0.0.1'
mo_port = 10801
mt_port = 10800

imei = 300234060379270

email_enabled = False
ip_enabled = False
http_post_enabled = False

mt_messages = deque()

#Set-up a queue
http_queue = Queue.Queue(0)
webhook_server_endpoint = ""
SocketServer.TCPServer.allow_reuse_address = True

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    global http_queue
    def do_POST(self):
        print("Handling post request")
        content_len = int(self.headers['Content-Length'])
        body = str(self.rfile.read(content_len))
        rec_msg = body 
        http_queue.put(rec_msg)
        self.send_response(200)
        self.end_headers()


class runServer(threading.Thread):
    def __init__(self, port):
        threading.Thread.__init__(self)
        self.port = port
    def run(self):
        global httpd
        print "Serving at port", self.port
        httpd = SocketServer.TCPServer(("", self.port), SimpleHTTPRequestHandler)
        httpd.serve_forever()

def signal_handler(sig, frame):
    global httpd 
    print('Server closing...')
    httpd.shutdown()
    sys.exit(0)

def send_mo_email():
    global lat
    global lon
    global mo_buffer
    global momsn
    global mtmsn
    global email
    global incoming_server
    global outgoing_server
    global password
    global imei

    #put together body
    body = \
'MOMSN: %d\r\n\
MTMSN: %d\r\n\
Time of Session (UTC): %s\r\n\
Session Status: TRANSFER OK\r\n\
Message Size: %d\r\n\
\r\n\
Unit Location: Lat = %8.6f Long = %8.6f\r\n\
CEPRadius = 3\r\n\
\r\n\
Message is Attached.'\
    % (momsn, mtmsn, time.asctime(), len(mo_buffer), lat, lon)
            
    #subject
    subject = 'SBD Msg From Unit: %d' % (imei)
            
    #message is included as an attachment
    attachment = 'text.sbd'
    fd = open(attachment, 'wb')
    fd.write(mo_buffer)
    fd.close()
    
    sendMail(subject, body, user, recipient, password, outgoing_server, attachment)

# def list_serial_ports():
#     # Windows
#     if os.name == 'nt':
#         # Scan for available ports.
#         available = []
#         for i in range(256):
#             try:
#                 s = serial.Serial(i)
#                 available.append('COM'+str(i + 1))
#                 s.close()
#             except serial.SerialException:
#                 pass
#         return available
#     else:
#         # Mac / Linux
#         return [port[0] for port in list_ports.comports()]

def write_text(cmd,start_index):
    global mo_set
    global mo_buffer
    text = cmd[start_index:len(cmd)-1]
    mo_buffer = text
    mo_set = True
    send_ok()

def sbdi():
    print 'AT+SBDI is not currently supported.  Still need to write this function.  Use AT+SBDIX instead'
    send_error()

def sbdix():
    global mo_set
    global momsn
    global mtmsn
    global ser
    global incoming_server
    global user
    global password
    global imei
    global mt_buffer
    global mo_ip
    global mo_port
    global mt_set
    global mo_buffer
    global momsn
    global mtmsn
    global http_queue
    global webhook_server_endpoint

    has_incoming_msg = False
    received_msg = 0
    received_msg_size = 0
    unread_msgs = 0
    time.sleep(AVERAGE_SBDIX_DELAY)
    success = True#(bool(random.getrandbits(1)))


    if success:
        
        #use e-mail interface if specified
        if email_enabled:
            #send e-mail if outgoing data is present
            if mo_set and not mo_buffer == "":
                if email_enabled:
                    send_mo_email()
                mo_set = False
                momsn += 1
                 
            
            #check e-mail for messages
            temp, received_msg, unread_msgs  = checkMessages(incoming_server,user,password,imei)
            if received_msg:
                #mtmsn += 1
                received_msg_size = len(temp)
                mt_buffer = temp
                mt_set = True
            else:
                received_msg_size = 0
    
        elif ip_enabled:
            if mo_set and not mo_buffer == "":
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                momsn += 1
                try:
                    s.connect((mo_ip, mo_port))
                    s.send(assemble_mo_directip_packet(imei, momsn, mtmsn, mo_buffer))
                    s.close()
                except socket.error as msg:
                    print "Failed to open {}:{}".format(mo_ip, mo_port)
                    s.close()                
                mo_set = False
            if len(mt_messages) is not 0:
                mtmsn += 1
                mt_set = True
                mt_buffer = mt_messages.popleft()
                unread_msgs = len(mt_messages)
                received_msg = mt_set
                received_msg_size = len(mt_buffer)
        elif http_post_enabled:
            if mo_set and not mo_buffer == "":
                r = requests.post(webhook_server_endpoint, data=mo_buffer)
                mo_set = False 
                momsn += 1
            elif http_queue.qsize() > 0:
                temp = http_queue.get()
                received_msg_size = len(temp)
                mt_buffer = temp
                mtmsn += 1
                mt_set = True 
                received_msg = mt_set
            unread_msgs = http_queue.qsize()

    #TODO: generate result output
    if success: rpt = 0
    else: rpt = 18 #TODO: add more sophisticated behavior for error msgs
    
    return_string = "\r\n+SBDIX: %d, %d, %d, %d, %d, %d\r\n" % (rpt,momsn,received_msg,mtmsn,received_msg_size,unread_msgs)
    #+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MT queued>
    print "Sent:",return_string
    ser.write(return_string)
    send_ok()
        
    mo_set = False
    if received_msg:
        mtmsn += 1

def sbd_reg():
    global registered
    
    success = (bool(random.getrandbits(1)))
    if registered == REG_STATUS_REGISTERED:
        print 'Already registered'
        error_text = ',0'
    else:
        if success:
            registered = REG_STATUS_REGISTERED
            error_text = ',0'
        else:
            registered = REG_STATUS_NOT_REGISTER
            error_text = ',17' #TODO: add more sophisticated failures
    
    ser.write("\nSBDREG:%d%s\r\n" % (registered,error_text))
    send_ok()
    
def check_reg_status():
    ser.write("\n+SBDREG:%d\r\n" % (registered))
    send_ok()
    
def sbd_det():
    print 'Detached'
    registered = True
    send_ok()
    
def read_text():
    global mt_buffer
    print(mt_buffer)
    ser.write("\n+SBDRT:\r\n%s\r\n" % (mt_buffer))
    send_ok()

def read_binary():
    global mt_buffer
    print "Device is reading binary from MT buffer: ",mt_buffer
    ser.write("\n+SBDRB:\r\n%s\r\n" % (mt_buffer))
    send_ok()
    

def send_ok():
    global ser
    ser.write('\r\nOK\r\n')
    print "Sending OK"
    
def send_error():
    global ser
    ser.write('\r\nERROR\r\n')

def send_ready():
    global ser
    ser.write('\r\nREADY\r\n')

def do_ok():
    print 'Received blank command'
    send_ok()

def clear_buffers(buffer):
    global mo_set
    global mt_set
    global mo_buffer
    global mt_buffer
    
    if buffer == 0:
        mo_buffer = ''
        mo_set = False
        ser.write('\r\n0\r\n')
        send_ok()
    elif buffer == 1:
        mt_buffer = ''
        mt_set = False
        ser.write('\r\n0\r\n')
        send_ok()
    elif buffer == 2:
        mt_buffer = ''
        mo_buffer = ''
        mo_set = False
        mt_set = False
        ser.write('\r\n0\r\n')
        send_ok()
    else:
        send_error()
    

def clear_momsn():
    momsn = 0
    ser.write('\r\n0\r\n')

def get_sbd_status():
    global mt_set
    global mo_set
    global momsn
    global mtmsn
    
    if mt_set:
        mt_flag = 1
    else:
        mt_flag = 0
        
    if mo_set:
        mo_flag = 1
    else:
        mo_flag = 0
    
    if mt_set:
        reported_mtmsn = mtmsn
    else:
        reported_mtmsn = -1
        
        
    return_string = "\nSBDS:%d,%d,%d,%d\r\n" % (mo_flag, momsn, mt_flag, mtmsn)

    ser.write(return_string)
    send_ok()

def copy_mo_to_mt():
    global mo_buffer
    global mt_buffer
    
    mt_buffer = mo_buffer
    
    return_string = "\nSBDTC: Outbound SBD Copied to Inbound SBD: size = %d\r\n" % (len(mo_buffer))
    ser.write(return_string)
    
    send_ok()
    
def which_gateway():
    return_string = "\rSBDGW:EMSS\r\n"

    ser.write(return_string)
    send_ok()

def get_system_time():
    return_string = "\r\n---MSSTM: 01002000\r\n"
    ser.write(return_string)
    send_ok()
    print 'We havent actually implemented MSSTM this yet.'
    
def set_ring_indicator(cmd,start_index):
    global ring_enable

    text = cmd[start_index:len(cmd)-1]
    
    if len(text) == 1:
        if text[0] == '0':
            send_ok()
        elif text[0] == '1':
            send_ok()
        else:
            send_error()
    else:
        send_error()

    
def get_signal_strength():
    return_string = "\r\n+CSQ:%d\r\n" % 5#(random.randint(0,5))
    time.sleep(AVERAGE_SBDIX_DELAY)
    ser.write(return_string)
    send_ok()

def get_valid_rssi():
    return_string = "\n+CSQ:(0-5)\r\n"
    ser.write(return_string)
    send_ok()

def get_lock_status():
    global locked
    
    return_string = "\n+CULK:%d\r\n" % ( locked ) 
    ser.write(return_string)
    send_ok()    
    
def get_manufacturer():
    return_string = "\n+Iridium\r\n" 
    ser.write(return_string)
    send_ok() 
    
def get_model():
    return_string = "\nIRIDIUM 9600 Family SBD Transceiver\r\n"
    ser.write(return_string)
    send_ok() 
    
def get_gsn():
    return_string = "\n300234060604220\r\n"
    ser.write(return_string)
    send_ok() 
    
def get_gmr():
    return_string = "\n3Call Processor Version: Long string\r\n"
    print 'Warning: get_gmr function not fully implemented'
    ser.write(return_string)
    send_ok() 
    
def write_binary_start(cmd,start_index):
    global binary_rx_incoming_bytes
    global binary_rx 
    
    text = cmd[start_index:len(cmd)-1]
    try:
        binary_rx_incoming_bytes = int(text)
        if (binary_rx_incoming_bytes > 340):
            ser.write('\r\r\n3\r\n')
            send_ok()
            binary_rx_incoming_bytes = 0
        else:
            print 'Ready to receive {} bytes'.format(binary_rx_incoming_bytes)
            send_ready()
            binary_rx = True
    except:
        send_error()

def parse_cmd(cmd):
    global echo
    #get string up to newline or '=' 
    index = cmd.find('=')
    if index == -1:
        index = cmd.find('\r')
    cmd_type = cmd[0:index].lower()
    
    #print cmd_type
    
    if cmd_type == 'at' : do_ok()
    elif cmd_type == 'at+csq'       : get_signal_strength()
    elif cmd_type == 'at+csq=?'     : get_valid_rssi()
    elif cmd_type == 'at+culk?'     : get_lock_status()
    elif cmd_type == 'at+gmi'       : get_manufacturer()
    elif cmd_type == 'at+gmm'       : get_model()
    elif cmd_type == 'at+gsn'       : get_gsn()
    elif cmd_type == 'at+gmr'       : get_gmr()
    elif cmd_type == 'at+sbdwt'     : write_text(cmd,index + 1)
    elif cmd_type == 'at+sbdwb'     : write_binary_start(cmd,index + 1)
    elif cmd_type == 'at+sbdi'      : sbdi()
    elif cmd_type == 'at+sbdix'     : sbdix()
    elif cmd_type == 'at+sbdreg'    : sbd_reg()
    elif cmd_type == 'at+sbdreg?'   : check_reg_status()
    elif cmd_type == 'at+sbddet'    : sbd_det()
    elif cmd_type == 'at+sbdrt'     : read_text()
    elif cmd_type == 'at+sbdrb'     : read_binary()
    elif cmd_type == 'at+sbdd0'     : clear_buffers(0)
    elif cmd_type == 'at+sbdd1'     : clear_buffers(1)
    elif cmd_type == 'at+sbdd2'     : clear_buffers(2)
    elif cmd_type == 'at+sbdc'      : clear_momsn()
    elif cmd_type == 'at+sbds'      : get_sbd_status()
    elif cmd_type == 'at+sbdtc'     : copy_mo_to_mt()
    elif cmd_type == 'at+sbdgw'     : which_gateway()
    elif cmd_type == 'at-msstm'     : get_system_time()
    elif cmd_type == 'at+sbdmta'    : set_ring_indicator(cmd,index + 1)
    elif cmd_type == 'ate0' or cmd_type == 'ate': 
        echo = False
        do_ok()
    elif cmd_type == 'ate1'    : do_ok()
    elif cmd_type == 'at&d0'    : do_ok()
    elif cmd_type == 'at&k0'    : do_ok()
    else : send_error()
    

def open_port(dev,baudrate):
    ser = serial.Serial(dev, 19200, timeout=.1, parity=serial.PARITY_NONE)
    return ser

class MobileTerminatedHandler(asyncore.dispatcher_with_send):
    def __init__(self, sock, addr):
        asyncore.dispatcher_with_send.__init__(self, sock)
        self.client = None
        self.addr = addr
	self.data = ""
        self.msg_length = 0
        self.preheader_fmt = '!bH'
        self.preheader_size = struct.calcsize(self.preheader_fmt)

    def handle_read(self):
        global mt_messages

        if len(self.data) < self.preheader_size:
            self.data += self.recv(self.preheader_size)
            preheader = struct.unpack(self.preheader_fmt, self.data)
            self.msg_length = preheader[1]
        else:
            self.data += self.recv(self.msg_length)
        
        print self.msg_length
        print self.data.encode("hex")
            
        if len(self.data) >= self.msg_length:
            mt_packet = None
            try: 
                mt_packet = parse_mt_directip_packet(self.data, mt_messages)
            except:
                print 'MT Handler: Invalid message'
            # response message
            self.send(assemble_mt_directip_response(mt_packet, mt_messages))
            self.handle_close()

    def handle_close(self):
        print 'MT Handler: Connection closed from %s' % repr(self.addr)
        sys.stdout.flush()
        self.close()


class MobileTerminatedServer(asyncore.dispatcher):

    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print 'MT Handler: Incoming connection from %s' % repr(addr)
            sys.stdout.flush()
        try:
            handler = MobileTerminatedHandler(sock, addr)
        except: 
            print "MT Handler: Unexpected error:", sys.exc_info()[0]

    


def main():
    
    global ser
    global mo_buffer
    global mo_set
    global binary_rx_incoming_bytes
    global binary_rx

    global user
    global recipient 
    global incoming_server
    global outgoing_server
    global password
    
    global email_enabled
    global ip_enabled
    global http_post_enabled
    
    global mo_ip
    global mo_port
    global mt_port

    global echo

    global http_queue
    global webhook_server_endpoint


    parser = OptionParser()
    parser.add_option("-d", "--dev", dest="dev", action="store", help="tty dev(ex. '/dev/ttyUSB0'", metavar="DEV")
    parser.add_option("--passwd", dest="passwd", action="store", help="Password", metavar="PASSWD")
    parser.add_option("-u", "--user", dest="user", action="store", help="E-mail account username", metavar="USER")
    parser.add_option("-r", "--recipient", dest="recipient", action="store", help="Destination e-mail address.", metavar="USER")
    parser.add_option("-i", "--in_srv", dest="in_srv", action="store", help="Incoming e-mail server url", metavar="IN_SRV")
    parser.add_option("-o", "--out_srv", dest="out_srv", action="store", help="Outging e-mail server", metavar="OUT_SRV")
    parser.add_option("--mo_ip", dest="mo_ip", action="store", help="Mobile-originated DirectIP server IP address", metavar="MO_IP", default="127.0.0.1")
    parser.add_option("--mo_port", dest="mo_port", action="store", help="Mobile-originated DirectIP server Port", metavar="MO_PORT", default=10801)
    parser.add_option("--mt_port", dest="mt_port", action="store", help="Mobile-terminated DirectIP server Port", metavar="MT_PORT", default=10800)
    parser.add_option("-m", "--mode", dest="mode", action="store", help="Mode: EMAIL,HTTP_POST,IP,NONE", default="NONE", metavar="MODE")
    parser.add_option("--imei", dest="imei", action="store", help="IMEI for this modem", default="300234060379270", metavar="MODE")
    
    # HTTP related args
    parser.add_option("--webhook_server_endpoint", dest="webhook_server_endpoint", action="store", help="Where to post webhooks", default=None)
    parser.add_option("--http_server_port", dest="http_server_port", action="store", help="Port number for the http server", default=None)

    (options, args) = parser.parse_args()

    mt_port = int(options.mt_port)

    #check for valid arguments
    if options.mode == "EMAIL":
        if options.passwd is None  or options.user is None or options.recipient is None or options.in_srv is None or options.out_srv is None:
            print 'If you want to use e-mail, you must specify in/out servers, user, password, and recipient address.'
            sys.exit()
        else:
            email_enabled = True
    elif options.mode == "HTTP_POST":
         http_post_enabled = True
         if options.webhook_server_endpoint is None or options.http_server_port is None:
             print "You have missing arguments."
             print "Usage: python2 Iridium9602.py --webhook_server_endpoint <webhook_server_endpoint>\
                     --http_server_port <http_server_port> -d <serial_port> -m HTTP_POST" 
             sys.exit()
    elif options.mode == "IP":
        print 'Using IP mode with MO ({}:{}) and MT (0.0.0.0:{}) servers'.format(options.mo_ip, int(options.mo_port), options.mt_port)
        server = MobileTerminatedServer('0.0.0.0', mt_port)
        print "Started MT Server on port {}".format(mt_port)
        sys.stdout.flush()
        ip_enabled = True
    else:
        print "No valid mode specified"
        sys.exit()
    
    
    user = options.user
    recipient = options.recipient
    incoming_server = options.in_srv
    outgoing_server = options.out_srv
    password = options.passwd

    mo_ip = options.mo_ip
    mo_port = int(options.mo_port)
    imei = options.imei
    
    now_get_checksum_first = False
    now_get_checksum_second = False
    
    try:
        ser = open_port(options.dev,19200)
    except:
        print "Could not open serial port.  Exiting."
#        print "FYI - Here's a list of ports on your system."
#        print list_serial_ports()
        sys.exit()
    
    if http_post_enabled:
        # Runs server 
        try:
            webhook_server_endpoint = options.webhook_server_endpoint
            server_port = int(options.http_server_port)
            server = runServer(server_port)
            server.start()
        except:
            print "Could not start server.  Exiting."
            sys.exit()

    rx_buffer = ''
    
    binary_checksum = 0
    
    while(1):
        signal.signal(signal.SIGINT, signal_handler)

        if ip_enabled:
            asyncore.loop(timeout=0, count=1) # non-blocking loop

        new_char = ser.read() # timeout after .1 seconds to return to asyncore.loop()
        if (len(new_char) == 0):
            continue 
         
        print(new_char)

        if echo and not binary_rx:
            ser.write(new_char)
            
        if not binary_rx:
            rx_buffer = rx_buffer + new_char
            #look for eol char, #TODO figure out what is really devined as EOL for iridium modem
            if new_char == chr(EOL_CHAR):
                if(len(rx_buffer) > 2):
                    print "Here is what I received:%s" % (rx_buffer)
                    parse_cmd(rx_buffer)
                    rx_buffer = ''
                else:
                    rx_buffer = ''

            #process backspace
            elif new_char == chr(BACKSPACE_CHAR) and not binary_rx:
                rx_buffer[:len(rx_buffer)-1]
                rx_buffer = rx_buffer[:len(rx_buffer)-2] #remove char if backspace
        else:
            if now_get_checksum_first:
                checksum_first = ord(new_char)
                now_get_checksum_first = False
                now_get_checksum_second = True
            elif now_get_checksum_second:
                checksum_second = ord(new_char)
                now_get_checksum_first = False
                now_get_checksum_second = False
                #check the checksum
                if (checksum_first * 256 + checksum_second) == (binary_checksum & (2**16-1)):
                    print "Good binary checksum"
                    ser.write('\r\n0\r\n')
                    send_ok()
                    mo_buffer = rx_buffer
                    rx_buffer = ''
                    mo_set = True
                else:
                    print "Bad binary checksum"
                    ser.write('\r\n2\r\n')
                    send_ok()
                    rx_buffer = ''
                    ser.write('\n')            
                binary_checksum = 0
                binary_rx = False
            else:
                if binary_rx_incoming_bytes == 1:
                    now_get_checksum_first = True
                    binary_checksum = binary_checksum + ord(new_char)
                    rx_buffer = rx_buffer + new_char
                else:
                    binary_rx_incoming_bytes -= 1
                    rx_buffer = rx_buffer + new_char
                    binary_checksum = binary_checksum + ord(new_char)
           
         
                
if __name__ == '__main__':
    main()
