#!/usr/bin/python

# Handles incoming Iridium SBD traffic and port forwards as appropriate based on IMEI 
# Used to allow you to run multiple Iridium9602 simulator instances that can be handled
# by a single MT DirectIP server

import asyncore
import socket
from virtual_iridium.sbd_packets import parse_mt_directip_packet
from collections import deque
import struct

# this script listens (binds) on this port
mt_sbd_address = '0.0.0.0'
mt_sbd_port = 40002

# maps imei to address and port
forward_address = { "300234060379270" : ("127.0.0.1",40010), "300234060379271" : ("127.0.0.1",40011), "300234060379272" : ("127.0.0.1",40012), "300234060379273" : ("127.0.0.1",40013), "300234060379274" : ("127.0.0.1",40014) }

class ConditionalSBDForwardClient(asyncore.dispatcher_with_send):

    def __init__(self, server, host, port):
        asyncore.dispatcher_with_send.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect( (host, port) )
        self.server = server
           
    def handle_read(self):
        data = self.recv(64)
        if data:
           self.server.send(data)


class ConditionalSBDForwardHandler(asyncore.dispatcher_with_send):

    def __init__(self, sock, addr):
        asyncore.dispatcher_with_send.__init__(self, sock)
        self.identified_protocol = False
        self.client = None
        self.addr = addr 
        self.data = ''
        self.preheader_fmt = '!bH'
        self.preheader_size = struct.calcsize(self.preheader_fmt)

    def handle_read(self):
        if len(self.data) < self.preheader_size:
            self.data += self.recv(self.preheader_size)
            if not self.data:
                return
            preheader = struct.unpack(self.preheader_fmt, self.data)
            self.msg_length = preheader[1]
        else:
            self.data += self.recv(self.msg_length)
        
        print self.msg_length
        print self.data.encode("hex")
            
        if len(self.data) >= self.msg_length:
            mt_packet = None
            mt_messages = deque()
            try: 
                mt_packet = parse_mt_directip_packet(self.data, mt_messages)
            except:
                print 'MT Handler: Invalid message'
                sys.stdout.flush()
                
            imei = mt_packet[0][1]
                
            print 'Attempting to forward message for imei: {}' .format(imei)

            if forward_address.has_key(imei):
                self.client = ConditionalSBDForwardClient(self, forward_address[imei][0], forward_address[imei][1])
                self.client.send(self.data)
                self.data = ''
            else:
                print 'No forwarding set up for imei: {}'.format(imei)
                self.close()

    def handle_close(self):
        print 'Connection closed from %s' % repr(self.addr)
        sys.stdout.flush()
        if self.client is not None:
            self.client.close()
        self.close()


class ConditionalSBDForwardServer(asyncore.dispatcher):

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
            handler = ConditionalSBDForwardHandler(sock, addr)
        except: 
            print "Unexpected error:", sys.exc_info()[0]
            
import sys
print "Iridium SBD Port forwarder starting up ..."
print "Listening for SBD on port: %d" % mt_sbd_port


sys.stdout.flush()

sbd_server = ConditionalSBDForwardServer(mt_sbd_address, mt_sbd_port)
asyncore.loop()
