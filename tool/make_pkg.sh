#!/usr/bin/env bash

# Gets the absolute path of the script (not where it's called from)
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TOPDIR=$DIR/..

function usage {
    echo "usage: make_pkg [[[-n nobuild ] | [-h]]"
}

# process args
nobuild=0
while [ "$1" != "" ]; do
    case $1 in
        -n | --nobuild )        shift
                                nobuild=1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

### build ###
if [ $nobuild == 0 ]; then
    $TOPDIR/tool/build_cmdr.sh
    $TOPDIR/tool/build_cmdr_pty.sh
fi

### start packaging ###
echo ""
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

### package ###
echo -n "Packaging......................."
mkdir -p $TOPDIR/deploy
if [ -e "$TOPDIR/deploy/cmdr"* ]; then
	rm deploy/cmdr*
fi

fpm -s dir -t deb -n cmdr -v 0.4.1 -p $TOPDIR/deploy/ \
    --vendor "UpDroid, Inc." \
    --provides cmdr \
    --description "A browser-based IDE and omni-tool for robots." \
    --maintainer "Mike Lewis <mike@updroid.com>" \
    --iteration 1 \
    --url http://www.updroid.com \
    -d 'dart >= 1.9.3' -d 'ffmpeg >= 2.6.2' \
    --before-install=$TOPDIR/tool/packaging/before-install.sh \
    --after-install=$TOPDIR/tool/packaging/after-install.sh \
    ./build/web=/opt/updroid/cmdr \
    ./bin/cmdr=/usr/local/bin/cmdr \
    $GOPATH/bin/cmdr-pty=/usr/local/bin/cmdr-pty #> /dev/null

echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Packaging complete. ########"
echo "You will find your debian package in deploy/."