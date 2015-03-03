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
mkdir -p bin
dart2js --output-type=dart --categories=Server --minify -o cmdr cmdr.dart > /dev/null
rm cmdr.deps
sed -i '1i#!/usr/bin/env dart' cmdr
chmod +x cmdr
mv cmdr bin/
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

### package ###
echo -n "Packaging......................."
fpm -s dir -t deb -n cmdr -v 0.2 ./gui/build/web=/etc/updroid ./cmdr/bin/cmdr=/usr/bin/cmdr  > /dev/null
echo "OK"