#!/usr/bin/env bash

# Run this script only once, right after the commander repo is cloned.

echo -n "Getting dependencies for cmdr..."
cd ./cmdr
pub get > /dev/null
cd ../
echo "Done."

echo -n "Getting dependencies for gui..."
cd ./gui
pub get > /dev/null
cd ../
echo "Done."
