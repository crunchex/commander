#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

function usage {
    echo "usage: build_cmdr [[[-n onlycss ] | [-h]]"
}

# process args
onlycss=0
debug=0
while [ "$1" != "" ]; do
    case $1 in
        -o | --onlycss )        shift
                                onlycss=1
                                ;;
        -d | --debug )       	shift
                                debug=1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

if [ $onlycss == 1 ]; then
	cd $TOPDIR/

	echo -n "Compiling less to css......."
	WEB=$TOPDIR/web
	rm -f $WEB/css/main.css
	lessc $WEB/css/main.less > $WEB/css/main.css

	echo "OK"
	exit 0
fi

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
echo -n "Checking system for lessc......."
command -v lessc >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install less (npm -g install less) and restart this script. Aborting."
	exit 1;
}
echo "OK"
echo -n "Checking system for cleancss...."
command -v cleancss >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install cleancss (npm -g install clean-css) and restart this script. Aborting."
	exit 1;
}
echo "OK"

### global ###
cd $TOPDIR

echo -n "Getting app dependencies........"
pub get > /dev/null
echo "OK"

### gui ###
cd $TOPDIR/

echo -n "Building (minifying) gui........"
WEB=$TOPDIR/web
rm $WEB/css/main.css $WEB/css/cmdr-min.css
lessc $WEB/css/main.less > $WEB/css/main.css
cleancss $WEB/css/main.css -o $WEB/css/cmdr-min.css
if [ $debug == 1 ]; then
	pub build --mode=debug > /dev/null
else
	pub build > /dev/null
fi
echo "OK"

echo -n "Setting up plugins.............."
BUILD=$TOPDIR/build/

BINTABS=$BUILD/bin/tabs
WEBTABS=$BUILD/web/tabs
mkdir -p $BINTABS
mkdir -p $WEBTABS

BINPANELS=$BUILD/bin/panels
WEBPANELS=$BUILD/web/panels
mkdir -p $BINPANELS
mkdir -p $WEBPANELS

# TODO fix these hard-coded paths
cp -r ../upcom-explorer/build/bin $BINPANELS/upcom-explorer
cp -r ../upcom-editor/build/bin $BINTABS/upcom-editor
cp -r ../upcom-console/build/bin $BINTABS/upcom-console
cp -r ../upcom-camera/build/bin $BINTABS/upcom-camera
cp -r ../upcom-shop/build/bin $BINTABS/upcom-shop
# cp -r ../upcom-teleop/build/bin $BINTABS/upcom-teleop
# cp -r ../upcom-learn-demo/build/bin $BINTABS/upcom-learn-demo

cp -r ../upcom-explorer/build/web $WEBPANELS/upcom-explorer
cp -r ../upcom-editor/build/web $WEBTABS/upcom-editor
cp -r ../upcom-console/build/web $WEBTABS/upcom-console
cp -r ../upcom-camera/build/web $WEBTABS/upcom-camera
cp -r ../upcom-shop/build/web $WEBTABS/upcom-shop
# cp -r ../upcom-teleop/build/web $WEBTABS/upcom-teleop
# cp -r ../upcom-learn-demo/build/web $WEBTABS/upcom-learn-demo
echo "OK"

echo -n "Cleaning up gui................."
BUILD=$TOPDIR/build/web
mkdir -p $BUILD/fonts
cp $WEB/packages/bootjack/fonts/glyphicons-halflings-regular.* $BUILD/fonts/
rm -rf $BUILD/css/main.css $BUILD/css/main.less $BUILD/css/glyphicons.css $BUILD/css/src
sed -i 's/main.css/cmdr-min.css/g' $BUILD/index.html
sed -i 's/main.dart/main.dart.js/g' $BUILD/index.html
echo "OK"

### cmdr ###
cd $TOPDIR/bin

BINDIR=$TOPDIR/build/bin
mkdir -p $BINDIR

echo -n "Building (minifying) cmdr......."
if [ $debug == 1 ]; then
	dart2js --output-type=dart --categories=Server -o $BINDIR/cmdr cmdr.dart
else
	dart2js --output-type=dart --categories=Server --minify -o $BINDIR/cmdr cmdr.dart
fi
rm -rf $BINDIR/cmdr.deps
sed -i '1i#!/usr/bin/env dart' $BINDIR/cmdr
chmod +x $BINDIR/cmdr
echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Build complete. ############"
