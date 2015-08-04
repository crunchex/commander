#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
WORKINGDIR=$DIR/../..

echo "##### Building API ################"
echo -n "Updating upcom-api.............."
cd $WORKINGDIR/upcom-api
git pull >/dev/null 2>&1
echo "OK"

echo "##### Building Panels #############"
echo -n "Building upcom-explorer..........."
cd $WORKINGDIR/upcom-explorer
git pull >/dev/null 2>&1
./tool/build.sh >/dev/null 2>&1
echo "OK"

echo "##### Building Tabs ###############"
echo -n "Building upcom-editor..........."
cd $WORKINGDIR/upcom-editor
git pull >/dev/null 2>&1
./tool/build.sh >/dev/null 2>&1
echo "OK"

echo -n "Building upcom-console.........."
cd $WORKINGDIR/upcom-console
git pull >/dev/null 2>&1
./tool/build.sh >/dev/null 2>&1
echo "OK"

echo -n "Building upcom-camera..........."
cd $WORKINGDIR/upcom-camera
git pull >/dev/null 2>&1
./tool/build.sh >/dev/null 2>&1
echo "OK"

echo -n "Building upcom-teleop..........."
cd $WORKINGDIR/upcom-teleop
git pull >/dev/null 2>&1
./tool/build.sh >/dev/null 2>&1
echo "OK"

echo "##### Tabs and API done ###########"