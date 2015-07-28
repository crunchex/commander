#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

function usage {
    echo "usage: build_cmdr [[[-n onlycss ] | [-h]]"
}

# process args
onlycss=0
while [ "$1" != "" ]; do
    case $1 in
        -o | --onlycss )        shift
                                onlycss=1
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
rm $WEB/css/main.css
lessc $WEB/css/main.less > $WEB/css/main.css
cat $WEB/css/glyphicons.css $WEB/css/main.css | cleancss -o $WEB/css/cmdr.css
pub build > /dev/null
echo "OK"

echo -n "Setting up tabs................."
BUILD=$TOPDIR/build/
BINTABS=$BUILD/bin/tabs
WEBTABS=$BUILD/web/tabs
mkdir -p $BINTABS
mkdir -p $WEBTABS

# TODO fix these hard-coded paths
cp -r ../upcom-editor/build/bin $BINTABS/upcom-editor
cp -r ../upcom-console/build/bin $BINTABS/upcom-console
cp -r ../upcom-camera/build/bin $BINTABS/upcom-camera
cp -r ../upcom-teleop/build/bin $BINTABS/upcom-teleop

cp -r ../upcom-editor/build/web $WEBTABS/upcom-editor
cp -r ../upcom-console/build/web $WEBTABS/upcom-console
cp -r ../upcom-camera/build/web $WEBTABS/upcom-camera
cp -r ../upcom-teleop/build/web $WEBTABS/upcom-teleop
echo "OK"

echo -n "Cleaning up gui................."
BUILD=$TOPDIR/build/web
mkdir -p $BUILD/fonts
cp $WEB/packages/bootjack/fonts/glyphicons-halflings-regular.* $BUILD/fonts/
rm $BUILD/css/main.css $BUILD/css/main.less $BUILD/css/glyphicons.css
sed -i '/glyphicons.css/d' $BUILD/index.html
# sed -i '/bootstrap.min.css/d' $BUILD/index.html
sed -i 's/main.css/cmdr.css/g' $BUILD/index.html
sed -i 's/main.dart/main.dart.js/g' $BUILD/index.html
echo "OK"

### cmdr ###
cd $TOPDIR/bin

BINDIR=$TOPDIR/build/bin
mkdir -p $BINDIR

echo -n "Building (minifying) cmdr......."
dart2js --output-type=dart --categories=Server --minify -o $BINDIR/cmdr cmdr.dart
rm -rf $BINDIR/cmdr.deps
sed -i '1i#!/usr/bin/env dart' $BINDIR/cmdr
chmod +x $BINDIR/cmdr
echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Build complete. ############"
