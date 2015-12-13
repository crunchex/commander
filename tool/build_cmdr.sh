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

BINPLUGINS=$BUILD/bin/plugins
WEBPLUGINS=$BUILD/web/plugins
mkdir -p $BINPLUGINS
mkdir -p $WEBPLUGINS

#cp -r ../upcom-explorer/build/bin $BINPLUGINS/upcom-explorer
#cp -r ../upcom-speak/build/bin $BINPLUGINS/upcom-speak
cp -r ../upcom-launcher/build/bin $BINPLUGINS/upcom-launcher
#cp -r ../upcom-editor/build/bin $BINPLUGINS/upcom-editor
cp -r ../upcom-console/build/bin $BINPLUGINS/upcom-console
# cp -r ../upcom-shop/build/bin $BINPLUGINS/upcom-shop
#cp -r ../upcom-teleop/build/bin $BINPLUGINS/upcom-teleop
#cp -r ../upcom-viz/build/bin $BINPLUGINS/upcom-viz
#cp -r ../upcom-learn-demo/build/bin $BINPLUGINS/upcom-learn-demo

#cp -r ../upcom-explorer/build/web $WEBPLUGINS/upcom-explorer
#cp -r ../upcom-speak/build/web $WEBPLUGINS/upcom-speak
cp -r ../upcom-launcher/build/web $WEBPLUGINS/upcom-launcher
#cp -r ../upcom-editor/build/web $WEBPLUGINS/upcom-editor
cp -r ../upcom-console/build/web $WEBPLUGINS/upcom-console
#cp -r ../upcom-teleop/build/web $WEBPLUGINS/upcom-teleop
#cp -r ../upcom-viz/build/web $WEBPLUGINS/upcom-viz
#cp -r ../upcom-learn-demo/build/web $WEBPLUGINS/upcom-learn-demo
echo "OK"

echo -n "Cleaning up gui................."
BUILD=$TOPDIR/build/web
mkdir -p $BUILD/fonts
cp $WEB/packages/bootjack/fonts/glyphicons-halflings-regular.* $BUILD/fonts/
rm -rf $BUILD/css/main.css $BUILD/css/main.less $BUILD/css/glyphicons.css $BUILD/css/src
if [ $(uname -s) == "Darwin" ]; then
    sed -i '' 's/main.css/cmdr-min.css/' $BUILD/index.html
    sed -i '' 's/main.dart/main.dart.js/' $BUILD/index.html
else
    sed -i 's/main.css/cmdr-min.css/g' $BUILD/index.html
    sed -i 's/main.dart/main.dart.js/g' $BUILD/index.html
fi
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
if [ $(uname -s) == "Darwin" ]; then
    perl -pi -e 'print "#!/usr/bin/env dart\n" if $. == 1' $BINDIR/cmdr
else
    sed -i '1i#!/usr/bin/env dart' $BINDIR/cmdr
fi
chmod +x $BINDIR/cmdr
echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Build complete. ############"
