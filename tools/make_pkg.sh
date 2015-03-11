#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

### start build ###
echo "##### Packaging Commander... #####"

### check dependencies ###
echo -n "Checking system for fpm........."
command -v fpm >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install fpm (effing package management) and restart this script. Aborting."
	exit 1;
}
echo "OK"

cd $TOPDIR

### build ###
$TOPDIR/tools/build_cmdr.sh

### package ###
echo -n "Packaging......................."
mkdir -p $TOPDIR/deploy
fpm -s dir -t deb -n cmdr -v 0.2 -p $TOPDIR/deploy/ ./gui/build/web=/etc/updroid ./cmdr/bin/cmdr=/usr/bin/cmdr  > /dev/null
echo "OK"

### done ###
cd $TOPDIR
echo "##### Packaging complete. ########"
echo "You will find your debian package in deploy/."