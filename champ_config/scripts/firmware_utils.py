#!/usr/bin/env python
import os
import sys

proj_dir = "$HOME/champ_ws/src/champ/champ_base/firmware"

firmware_includes = os.path.dirname(sys.path[0]) + "/include/firmware"
os.environ["CHAMP_FIRMWARE_INCLUDE_PATH"] = firmware_includes

build_command = "pio run -d" + proj_dir

if len(sys.argv) > 1:
    firmware_argument = sys.argv[1]

    if firmware_argument == "upload":
        build_command = "pio run -t upload -d" + proj_dir
    
    elif firmware_argument == "run":
        pass

    else:
        print("Invalid argument: %s" % firmware_argument)
        print("\nValid arguments:")
        print("\n python firmware_utils.py upload")
        print("\n python firmware_utils.py run\n")
        sys.exit()

os.system(build_command)