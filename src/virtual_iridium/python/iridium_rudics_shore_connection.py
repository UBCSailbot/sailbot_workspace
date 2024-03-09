#!/usr/bin/python
from optparse import OptionParser
import socket
import time

def close_hayes_socket(hayes_socket):
    hayes_socket.send("+++");        
    time.sleep(2)
    hayes_socket.send("ATH\r\n");  


def main():
    parser = OptionParser()
    parser.add_option("-a", "--hayes_address", dest="hayes_address", action="store", help="Hayes Simulator Address")
    parser.add_option("-p", "--hayes_port", dest="hayes_port", action="store", help="Hayes Simulator Port")

    parser.add_option("-A", "--shore_address", dest="shore_address", action="store", help="Shore driver Address")
    parser.add_option("-P", "--shore_port", dest="shore_port", action="store", help="Shore driver Port")

    (options, args) = parser.parse_args()
    print options

    connected = False
    buffer_size = 1024

    shore_socket = None;
 

    hayes_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    hayes_socket.connect((options.hayes_address, int(options.hayes_port)))
    hayes_socket.settimeout(0.1)
    hayes_socket.send("OK")
    while(1):
        try:       
            if connected:
                try:
                    shore_data = shore_socket.recv(buffer_size)
                    if(len(shore_data) > 0):
                        hayes_socket.send(shore_data)
                    else:
                        print "Zero read"
                        connected = False
                        close_hayes_socket(hayes_socket)
                except socket.timeout:
                    pass
                except socket.error as e:
                    print "Shore socket error: ",e
                    connected = False
                    close_hayes_socket(hayes_socket, connected)

            data = hayes_socket.recv(buffer_size)
            print data

            if("NO CARRIER" in data):
                print "Disconnected!"
                connected = False
                shore_socket.close()
                
            if connected and len(data) > 0:
                try:
                    print "To Shore: ",data.encode("hex")
                    shore_socket.send(data)
                except socket.timeout:
                    print "Timeout sending data"
                except socket.error as e:
                    print "Shore socket error: ",e
                    connected = False
                    hayes_socket.send("+++");        
                    time.sleep(2)
                    hayes_socket.send("ATH\r\n");  
                    
            if(data.strip() == "RING"):
                hayes_socket.send("ATA\r\n");        
            elif("CONNECT" in data):
                print "Connected!"
                shore_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                shore_socket.settimeout(0.1)
                shore_socket.connect((options.shore_address, int(options.shore_port)))
                connected = True

        except socket.timeout:
            time.sleep(0.01)
        except socket.error as e:
            print e


if __name__ == '__main__':
    main()
