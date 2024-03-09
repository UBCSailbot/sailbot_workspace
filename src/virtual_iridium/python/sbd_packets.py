import struct
import random
import time

def assemble_mo_directip_packet(imei, momsn, mtmsn, mo_buffer):

    # ==== MO HEADER ====
    # MO Header IEI           char               0x01
    # MO Header Length        unsigned short
    # CDR Reference (Auto ID) unsigned integer
    # IMEI                    char[] (15 bytes)
    # Session Status          unsigned char
    # MOMSN                   unsigned short
    # MTMSN                   unsigned short
    # Time of Session         unsigned integer
    header_fmt = "!bHI15sBHHI"
    header_iei = 0x01
    # header_length does not include iei and length fields
    header_length = struct.calcsize(header_fmt) - struct.calcsize('!bH')
    cdr_ref = random.getrandbits(32)
    session_status = 0
    header = struct.pack(header_fmt, header_iei, header_length, cdr_ref, str(imei), session_status, momsn, mtmsn, int(time.time()))

    # ==== MO PAYLOAD ====
    # MO Payload IEI         char               0x02
    # MO Payload Length      unsigned short    
    # MO Payload             char 
    payload_iei = 0x02
    payload_length = min(len(mo_buffer), 1960)
    payload = struct.pack('!bH' + str(payload_length) + 's', payload_iei, payload_length, mo_buffer)

    protocol_rev_no = 1    
    overall_msg_length = len(header) + len(payload)    
    preheader = struct.pack('!bH', protocol_rev_no, overall_msg_length)
    return preheader + header + payload

def parse_mt_directip_packet(buffer, mt_messages):

    parse_offset = 0
    preheader_fmt = '!bH'
    ie_header_fmt = '!bH'
    preheader = struct.unpack_from(preheader_fmt, buffer, parse_offset)
    parse_offset += struct.calcsize(preheader_fmt)

    header_iei = 0x41
    payload_iei = 0x42
    prio_iei = 0x46
    header = None
    payload = None

    while parse_offset + struct.calcsize(ie_header_fmt) < len(buffer):
        ie_header = struct.unpack_from(ie_header_fmt, buffer, parse_offset)
        print 'IE Header: ' + str(ie_header)
        parse_offset += struct.calcsize(ie_header_fmt)

        if ie_header[0] == header_iei:
            # ==== MT HEADER ====
            # MT Header IEI                char            0x41
            # MT Header Length             unsigned short
            # Unique Client Message ID     unsigned int
            # IMEI (User ID)               char[] (15 bytes)
            # MT Disposition Flags char    unsigned short
            header_fmt = '!I15sH'
            header = struct.unpack_from(header_fmt, buffer, parse_offset)
            print 'Header: ' + str(header)
        elif ie_header[0] == payload_iei:
            # ==== MT PAYLOAD ====
            # MO Payload IEI         char               2
            # MO Payload Length      unsigned short    
            # MO Payload             char 
            payload_fmt = str(ie_header[1]) + 's'
            payload = struct.unpack_from(payload_fmt, buffer, parse_offset)
            print 'Payload: ' + str(payload)
            mt_messages.append(payload[0])
        else:
            print 'Unknown IEI: %x'.format(ie_header[0])
            
        parse_offset += ie_header[1]
    return (header, payload)

def assemble_mt_directip_response(mt_packet, mt_messages):

    confirm_iei = 0x44
    confirm_fmt = "!bHI15sIh"
    confirm_length = struct.calcsize(confirm_fmt) - struct.calcsize('!bH')
    session_status = 0
    client_id = 0
    imei = '0'*15
    if mt_packet[0] is not None:
        print mt_packet
        session_status = len(mt_messages)
        client_id = mt_packet[0][0]
        imei = mt_packet[0][1]
    else:
        session_status = -7 # violation of MT DirectIP protocol error


    # MT Confirmation Message IEI
    # MT Confirmation Message Length
    # Unique Client Message ID
    # IMEI (User ID)
    # Auto ID Reference
    # MT Message Status
    auto_id = random.getrandbits(32)
    confirm = struct.pack(confirm_fmt, confirm_iei, confirm_length, client_id, imei, auto_id, session_status)
    
    protocol_rev_no = 1    
    overall_msg_length = len(confirm)
    preheader = struct.pack('!bH', protocol_rev_no, overall_msg_length)
    return preheader + confirm
