#!/usr/bin/env bash

# This script will build and install cmdr on an updroid system.
# Not meant to be run on a host/development system.
# Run with 'sudo'.

echo -n "Building cmdr..."
cd ./cmdr
dart2js --output-type=dart --categories=Server --minify -o cmdr cmdr.dart > /dev/null
echo "Done."

echo -n "Granting exec access to cmdr..."
chmod +x cmdr
echo "Done."

echo -n "Installing cmdr..."
cp cmdr /usr/bin
echo "Done."
