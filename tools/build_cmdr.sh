#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

### start build ###
echo ""
echo "##### Building Commander... ######"

### check tools ###
echo -n "Checking system for dart-sdk...."
command -v pub >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install dart-sdk, add bin to PATH, and restart this script. Aborting."
	exit 1;
}
echo "OK"

### cmdr ###
cd $TOPDIR/cmdr

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

### gui ###
cd $TOPDIR/gui

echo -n "Getting dependencies for gui...."
pub get > /dev/null
cd web
if [ ! -d "src-min-noconflict" ]; then
	git clone --quiet https://github.com/ajaxorg/ace-builds.git
	cd ace-builds
	# make sure we're using package 03.03.15
	git checkout --quiet beb9ff68e397b4dcaa1d40f79651a063fc917736
	mv src-min-noconflict ../src-min-noconflict
fi
cd $TOPDIR/gui
rm -rf $TOPDIR/gui/web/ace-builds
echo "OK"

cd $TOPDIR/gui

echo -n "Building (minifying) gui........"
pub build > /dev/null
echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Build complete. ############"
