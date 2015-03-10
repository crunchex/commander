#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
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