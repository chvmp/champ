#!/usr/bin/env python
import argparse
import os
import sys
import rospkg

supported_boards = ['teensy31', 'teensy36', 'teensy40']
proj_dir = "$HOME/champ_ws/src/champ/champ_base/firmware"

firmware_includes = os.path.dirname(sys.path[0]) + "/include/firmware"
os.environ["CHAMP_FIRMWARE_INCLUDE_PATH"] = firmware_includes

parser = argparse.ArgumentParser(description='Champ Firmware Tool')
parser.add_argument("--board", default="teensy36", type=str, help="Board: teensy31 teensy36 teensy40")
parser.add_argument("--upload", default=False, help="Upload", action='store_true')
parser.add_argument("--run", default=True, help="Compile", action='store_true')

args = parser.parse_args()

#check if board is correct
try:
    environment =  supported_boards.index(args.board)
except:
    print "Invalid board! Valid Boards are: teensy31, teensy36, teensy40"
    sys.exit()

if args.upload == True:
    command = "--target upload"
else:
    command = ""

environment = args.board
build_command = "pio run " + command + " -e " + environment + " -d " + proj_dir
os.system(build_command)