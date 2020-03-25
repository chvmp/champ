#!/usr/bin/env bash

ROBOT_CONFIG_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
export CHAMP_ROBOT_CONFIG_DIR=$ROBOT_CONFIG_DIR/include

bash -i