#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

echo -n "Checking system for fpm........."
command -v fpm >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install fpm (effing package management) and restart this script. Aborting."
	exit 1;
}
echo "OK"

cd $TOPDIR

### build ###
tools/build_cmdr.sh

### package ###
echo -n "Packaging......................."
fpm -s dir -t deb -n cmdr -v 0.2 ./gui/build/web=/etc/updroid ./cmdr/bin/cmdr=/usr/bin/cmdr  > /dev/null
echo "OK"