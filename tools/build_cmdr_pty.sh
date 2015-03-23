#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

### start build ###
echo ""
echo "###### Building cmdr... ##########"

### check dependencies ###
echo -n "Checking system for golang......"
command -v go >/dev/null 2>&1 || {
	echo "FAIL";
	echo "Please install golang and restart this script. Aborting."
	exit 1;
}
echo "OK"

### build cmdr-pty ###
echo ""
echo "##### Building cmdr-pty ##########"
echo -n "Retrieving cmdr-pty source......"
GO_UPDROID_PATH=${GOPATH:?"Need to set GOPATH non-empty"}/src/bitbucket.org/updroid
GO_CMDRPTY_PATH=$GO_UPDROID_PATH/cmdr-pty
if [ ! -d $GO_CMDRPTY_PATH ]; then
	mkdir -p $GO_UPDROID_PATH
	git clone https://crunchex@bitbucket.org/updroid/cmdr-pty.git $GO_CMDRPTY_PATH > /dev/null
fi
echo "OK"

echo -n "Building cmdr-pty..............."
cd $GO_CMDRPTY_PATH
go build
mkdir -p $TOPDIR/cmdr/bin
mv cmdr-pty $TOPDIR/cmdr/bin/
echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Build complete. ############"