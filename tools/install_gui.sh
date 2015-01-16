#!/usr/bin/env bash

# This script will build the cmdr gui and install it on an updroid system.
# Not meant to be used on a host/development system.
# Run with 'sudo'.

echo -n "Building gui..."
cd ./gui
pub build > /dev/null
cd ../
echo "Done."
