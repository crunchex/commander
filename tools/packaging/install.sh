#!/usr/bin/env bash

# install.sh --version 0.1.0
# Copyright 2015 -- UpDroid, Inc.

usage() {
  echo "Usage: $0 [--options]"
  echo "Options:"
  echo "-version: display the version of this script"
  echo "--no-prompt: silently select standard options/defaults"
  echo "--quick-check: quickly try to determine if dependencies are installed"
  echo "               (this avoids interactive prompts and sudo commands,"
  echo "               so might not be 100% accurate)"
  echo "--unsupported: attempt installation even on unsupported systems"
  echo "Script will prompt interactively if options not given."
  exit 1
}

# If no args, display usage.
# if [[ -z "$var" ]] ; then
#   usage
# fi

while test "$1" != ""
do
  case "$1" in
  --version)				do_version=1;;
  --no-prompt)              do_default=1
                            do_quietly="-qq --assume-yes"
    ;;
  --quick-check)            do_quick_check=1;;
  --unsupported)            do_unsupported=1;;
  *) usage;;
  esac
  shift
done

if [ 1 -eq "${do_version-0}" ] ; then
  echo "0.1.0"
  exit 0
fi

# Check for lsb_release command in $PATH
if ! which lsb_release > /dev/null; then
  echo "ERROR: lsb_release not found in \$PATH" >&2
  exit 1
fi

lsb_release=$(lsb_release --codename --short)

# Ubuntu | Linux Mint | Elementary OS
trusty_codenames="(trusty|qiana|freya)"

if [ 0 -eq "${do_unsupported-0}" ] && [ 0 -eq "${do_quick_check-0}" ] ; then
  if [[ ! $lsb_release =~ $trusty_codenames ]]; then
    echo "ERROR: Only 14.04 (trusty) derivatives are"\
        "currently supported" >&2
    exit 1
  fi

  #if ! uname -m | egrep -q "i686|x86_64"; then
  if ! uname -m | egrep -q "x86_64"; then
    echo "Only x86_64 architectures are currently supported" >&2
    exit
  fi
fi

if [ "x$(id -u)" != x0 ] && [ 0 -eq "${do_quick_check-0}" ]; then
  echo "Running as non-root user."
  echo "You might have to enter your password one or more times for 'sudo'."
  echo
fi

echo -n "Setting up apt for HTTPS........"
sudo apt-get -qqy update
sudo apt-get install -qqy apt-transport-https
echo "DONE"

echo -n "Setting up Dart repo............"
# Get the Google Linux package signing key and set up the stable repository.
sudo sh -c 'curl –silent https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add -' >/dev/null 2>&1
sudo sh -c 'curl –silent https://storage.googleapis.com/download.dartlang.org/linux/debian/dart_stable.list > /etc/apt/sources.list.d/dart_stable.list' >/dev/null 2>&1
echo "DONE"

echo -n "Setting up ROS repo............."
# Get the ROS package signing key and set up the stable repository.
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116 >/dev/null 2>&1
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list' >/dev/null 2>&1
echo "DONE"

echo -n "Setting up UpDroid repo........."
# Get the UpDroid package signing key and set up the dev repository.
sudo sh -c 'curl -silent http://packages.updroid.com/packages.updroid.com-keyring.key | apt-key add -' >/dev/null 2>&1
sudo sh -c 'echo "deb http://packages.updroid.com/ubuntu trusty main" > /etc/apt/sources.list.d/updroid.list' >/dev/null 2>&1
echo "DONE"

echo -n "Installing cmdr and deps........"
sudo apt-get -qqy update
sudo apt-get -qqy install cmdr
echo "DONE"