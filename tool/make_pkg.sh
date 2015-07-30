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
if ls $TOPDIR/deploy/cmdr* 1> /dev/null 2>&1; then
	rm deploy/cmdr*
fi

fpm -s dir -t deb -n cmdr -v 0.5.1 -p $TOPDIR/deploy/ \
    --vendor "UpDroid, Inc." \
    --provides cmdr \
    --description "A browser-based IDE and omni-tool for robotics software development." \
    --maintainer "Mike Lewis <mike@updroid.com>" \
    --iteration 1 \
    --url http://www.updroid.com \
    -d 'dart >= 1.9.3' -d 'ffmpeg >= 2.6.2' \
    --after-install=$TOPDIR/tool/packaging/after-install.sh \
    --after-remove=$TOPDIR/tool/packaging/after-remove.sh \
    --after-upgrade=$TOPDIR/tool/packaging/after-upgrade.sh \
    ./build/bin=/opt/updroid/cmdr \
    ./build/web=/opt/updroid/cmdr > /dev/null
    #./build/bin/cmdr=/usr/bin/cmdr
    #$GOPATH/bin/cmdr-pty=/usr/local/bin/cmdr-pty

echo "OK"

### done ###
cd $TOPDIR
echo ""
echo "##### Packaging complete. ########"
echo "You will find your debian package in deploy/."