import subprocess

# White = CAN low
# Red = Power
# Green = CAN high
# Yellow = Ground

SAIL_CMD = "sail"
RDR_CMD = "rudder"
PDB_CMD = "pdb"
EXIT_CMD = "exit"
BACK_CMD = "back"

# EFFECTS: Converts a given decimal number to a string hexadecimal number with num_digits
def convert_to_hex(decimal, num_digits):
    hex = format(decimal, "X")

    if (len(hex) < num_digits):
        add = ""
        for x in range(num_digits - len(hex)):
            add += "0"
        return add + hex

    return hex


### MAIN ###
print("\n1.", SAIL_CMD,"\n2.", RDR_CMD, "\n3.", PDB_CMD)

while True:
    cmd = input("Enter command: ")
    cmd = cmd.lower()
    msg = ""

    if (cmd == SAIL_CMD):
        msg += "cansend can1 002##0"
        
        while True:
            try:
                num = input("\nEnter trim tab angle: ")

                if (num == EXIT_CMD): break

                msg += convert_to_hex(int(num) * 1000, 8) # Need 8 hex digits
                print()
                print(msg)
                # subprocess.run(msg)
                subprocess.run("python test.py")
                break
            except ValueError:
                print("[ERR] Value entered is not an integer")
        break
    elif (cmd == RDR_CMD):
        # TODO
        print("todo")
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

