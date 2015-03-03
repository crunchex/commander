#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

echo -n "Checking system for fpm........."
command -v fpm >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install fpm (effing package management) and restart this script. Aborting."
	exit 1;
}
echo "OK"

cd $DIR/../

### cmdr ###
cd cmdr

echo -n "Getting dependencies for cmdr..."
pub get > /dev/null
echo "OK"

echo -n "Building (minifying) cmdr......."
mkdir bin
dart2js --output-type=dart --categories=Server --minify -o bin/cmdr cmdr.dart > /dev/null
rm bin/cmdr.deps
echo "OK"

cd ../

### gui ###
cd gui

echo -n "Getting dependencies for gui...."
pub get > /dev/null
echo "OK"

echo -n "Building (minifying) gui........"
pub build > /dev/null
echo "OK"

cd ../