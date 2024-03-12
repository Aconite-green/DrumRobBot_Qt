#!/bin/bash
echo "Running gdb with sudo..."
sudo /usr/bin/gdb "$@"
echo "gdb has exited with status $?"

