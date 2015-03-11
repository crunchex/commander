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

### start packaging ###
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
if [ $nobuild == 0 ]; then
	$TOPDIR/tools/build_cmdr.sh
fi

### package ###
echo -n "Packaging......................."
mkdir -p $TOPDIR/deploy
fpm -s dir -t deb -n cmdr -v 0.2 -p $TOPDIR/deploy/ ./gui/build/web=/etc/updroid ./cmdr/bin/cmdr=/usr/bin/cmdr  > /dev/null
echo "OK"

### done ###
cd $TOPDIR
echo "##### Packaging complete. ########"
echo "You will find your debian package in deploy/."