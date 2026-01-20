#!/bin/bash
# Wrapper script to run franky Python scripts with the correct environment

# Set library path for libfranka (now in /usr/local)
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"

# Run the Python script with the venv Python
exec /home/arash/franky/.venv/bin/python "$@"
