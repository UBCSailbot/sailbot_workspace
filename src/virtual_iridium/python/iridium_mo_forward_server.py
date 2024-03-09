#!/usr/bin/python

# splits traffic to Hayes Modem emulator and to an Iridium9602 simulator

import asyncore
import socket
from optparse import OptionParser


class ConditionalForwardClient(asyncore.dispatcher_with_send):

    def __init__(self, server, host, port):
        asyncore.dispatcher_with_send.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect( (host, port) )
        self.server = server
           
    def handle_read(self):
        data = self.recv(64)
        if data:
           self.server.send(data)


class ConditionalForwardHandler(asyncore.dispatcher_with_send):

    def __init__(self, sock, addr):
        asyncore.dispatcher_with_send.__init__(self, sock)
        self.identified_protocol = False
        self.addr = addr
	self.initial_data = ""
        self.buf = ""
        self.hayes_client = ConditionalForwardClient(self, options.hayes_server, int(options.hayes_port))
        self.sbd_client = ConditionalForwardClient(self, options.sbd_server, int(options.sbd_port))
        self.sbd_write = False
        self.sbd_bytes_remaining = 0

    def handle_read(self):
        data = self.recv(256)
        
        print data.encode("hex")

        if not data:
            return
        elif self.sbd_write: # not line mode - raw data
            self.sbd_send_bytes(data)
        elif data == "+++":
            self.hayes_client.send(data)
        else: # line based Command data
            self.buf += data
            line_list = self.buf.split('\r')
            # partial line
            self.buf = line_list[-1]
        
            for line in line_list[0:-1]:
                self.line_process(line)

    def handle_close(self):
        print 'Connection closed from %s' % repr(self.addr)
        sys.stdout.flush()
        self.close()

    def line_process(self, line):
        line_cr = line + '\r'
        
        if line.strip().upper() in ['ATE']:
            self.hayes_client.send(line_cr)
            self.sbd_client.send(line_cr)
        else:
            if len(line) >= 6 and line[2:6].upper() == "+SBD":
                self.sbd_client.send(line_cr)
                if len(line) >= 8 and line[2:8].upper() == "+SBDWB":
                    parts = line.split('=')
                    self.sbd_bytes_remaining = int(parts[1]) + 2 # 2 checksum bytes
                    self.sbd_write = True
            else:
                self.hayes_client.send(line_cr)
    
    def sbd_send_bytes(self, bytes):
        self.sbd_bytes_remaining -= len(bytes)
        self.sbd_client.send(bytes)
        print self.sbd_bytes_remaining
        if self.sbd_bytes_remaining <= 0:
            self.sbd_write = False

class ConditionalForwardServer(asyncore.dispatcher):

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
            print 'Incoming connection from %s' % repr(addr)
            sys.stdout.flush()
        try:
            handler = ConditionalForwardHandler(sock, addr)
        except: 
            print "Unexpected error:", sys.exc_info()[0]
            
import sys





parser = OptionParser()
parser.add_option("-p", "--port", dest="port", action="store", help="bind port", default=4010)
parser.add_option("-a", "--hayes_address", dest="hayes_server", action="store", help="address to connect to Hayes AT emulator", default="127.0.0.1")
parser.add_option("-b", "--hayes_port", dest="hayes_port", action="store", help="address to connect to Hayes AT emulator", default=4001)
parser.add_option("-c", "--sbd_address", dest="sbd_server", action="store", help="address to connect to SBD emulator", default="127.0.0.1")
parser.add_option("-d", "--sbd_port", dest="sbd_port", action="store", help="address to connect to SBD emulator", default=4020)

(options, args) = parser.parse_args()


forward_address = '0.0.0.0'

print "Iridium Port forwarder starting up ..."
print "Listening on port: {}".format(int(options.port))
print "Connecting for Iridium9602 SBD on %s:%d" % (options.sbd_server, int(options.sbd_port))
print "Connecting for Hayes (ATDuck) on %s:%d" % (options.hayes_server, int(options.hayes_port))
sys.stdout.flush()

server = ConditionalForwardServer(forward_address, int(options.port))
asyncore.loop()
