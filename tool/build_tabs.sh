#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
WORKINGDIR=$DIR/../..

function usage {
    echo "usage: build [[[-d ] | [-h]]"
}

# process args
debug=0
while [ "$1" != "" ]; do
    case $1 in
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

echo "##### Building API ################"
echo -n "Updating upcom-api.............."
cd $WORKINGDIR/upcom-api
git pull >/dev/null 2>&1
echo "OK"

echo "##### Building Panels #############"
echo -n "Building upcom-explorer..........."
cd $WORKINGDIR/upcom-explorer
git pull >/dev/null 2>&1
if [ $debug == 1 ]; then
	./tool/build.sh -d >/dev/null 2>&1
else
	./tool/build.sh >/dev/null 2>&1
fi
echo "OK"

echo "##### Building Tabs ###############"
echo -n "Building upcom-editor..........."
cd $WORKINGDIR/upcom-editor
git pull >/dev/null 2>&1
if [ $debug == 1 ]; then
	./tool/build.sh -d >/dev/null 2>&1
else
	./tool/build.sh >/dev/null 2>&1
fi
echo "OK"

echo -n "Building upcom-console.........."
cd $WORKINGDIR/upcom-console
git pull >/dev/null 2>&1
if [ $debug == 1 ]; then
	./tool/build.sh -d >/dev/null 2>&1
else
	./tool/build.sh >/dev/null 2>&1
fi
echo "OK"

echo -n "Building upcom-camera..........."
cd $WORKINGDIR/upcom-camera
git pull >/dev/null 2>&1
if [ $debug == 1 ]; then
	./tool/build.sh -d >/dev/null 2>&1
else
	./tool/build.sh >/dev/null 2>&1
fi
echo "OK"

echo -n "Building upcom-teleop..........."
cd $WORKINGDIR/upcom-teleop
git pull >/dev/null 2>&1
if [ $debug == 1 ]; then
	./tool/build.sh -d >/dev/null 2>&1
else
	./tool/build.sh >/dev/null 2>&1
fi
echo "OK"

echo "##### Tabs and API done ###########"