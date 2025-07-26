import subprocess
import paramiko

# IMPORTANT: 
''' Notes on CAN
- pdb, other serial devices are always sending information to mainframe - use candump to receive the messages and parse them using the id of the message
- Check CAN settings on raspberry pi using ip -details link show can1
- if cansend, candump are not working, reset can settings - set it down and then up according to instructions in CANFD and RPI Setup and Integration
- can use loopback test if necessary - setup two separate ssh sessions on the same raspberry pi, one doing candump, the other doing cansend - the message sent should
    be readable in candump
'''

# White = CAN low
# Red = Power
# Green = CAN high
# Yellow = Ground

# TODO: Simplify program, make it easy to add new functions/frames
# TODO: implement graphing for pdb data - use matplotlib

SAIL_CMD = "sail"
RDR_CMD = "rudder"
PDB_CMD = "pdb"
EXIT_CMD = "exit"
BACK_CMD = "back"

hostname = "raspberrypi.local"
username = "soft"
password = "sailbot"

# EFFECTS: Converts a given decimal number to a string hexadecimal number with num_digits
def convert_to_hex(decimal, num_digits):
    hex = format(decimal, "X")

    if (len(hex) < num_digits):
        add = ""
        for x in range(num_digits - len(hex)):
            add += "0"
        return add + hex

    return hex

# EFFECTS: switches the order of characters in the given string, two at a time (eg. "abcdef" --> "efcdab")
def convert_to_little_endian(hex):
    # note: hex is a string of a hexadecimal number
    new = bytes.fromhex(hex)
    new = new[::-1]
    return new.hex()
        
def send_command(cmd):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname, username=username, password=password)
    stdin, stdout, stderr = client.exec_command(cmd)
    # print("stdout: ", stdout.read().decode())
    # print("stderr: ", stderr.read().decode())
    client.close()

# EFFECTS: Sends can frame for MAIN_TR_TAB
def main_tr_tab():
    msg = "cansend can1 002##0"  
    while True:
        try:
            num = input("\nEnter trim tab angle: ")

            if (num == EXIT_CMD): break

            msg += convert_to_little_endian(convert_to_hex(int(num) * 1000, 8)) # Need 8 hex digits
            print()
            print(msg)
            # subprocess.Popen(msg, shell=True)
            return msg
        except ValueError:
            print("[ERR] Value entered is not an integer")

# EFFECTS: Sends can frame for DEBUG_RUDDER_INPUT
def rdr_heading():
    msg = "cansend can1 200##0"
    while True:
        try:
            num = input("\nEnter rudder angle: ")

            if (num == EXIT_CMD): break

            msg += convert_to_little_endian(convert_to_hex(int(num) * 1000, 8)) # Need 8 hex digits
            print()
            print(msg)
            return msg
        except ValueError:
            print("[ERR] Value entered is not an integer")

# EFFECTS: Get data from can frame BMS_DATA_FRAME
def get_pdb_data():
    # TODO
    return


### MAIN ###
print("\n1.", SAIL_CMD,"\n2.", RDR_CMD, "\n3.", PDB_CMD)

# TODO: error handling - check if msg is valid

while True:
    cmd = input("Enter command: ")
    cmd = cmd.lower()
    msg = ""

    if (cmd == SAIL_CMD):
        msg = main_tr_tab()
        break
    elif (cmd == RDR_CMD):
        msg = rdr_heading()
        break
    elif (cmd == PDB_CMD):
        # TODO
        print("todo")
        break
    elif (cmd == EXIT_CMD):
        print("Exiting program...\n")
        break
    else:
        # TODO
        print("[ERR] Command not recognized")

send_command(msg)
