# UpDroid Commander #

A simple web-based front end for interacting with the robot. Like an IDE, it includes basic implementations of a file manager/explorer, text editor, command console, and some user-configurable buttons. Additionally, it has a viewport for an embedded video stream such as a connected camera on the robot.

The git repository can be found at https://crunchex@bitbucket.org/updroid/commander.git

There are two parts in updroid-robot: server and client. The server handles filesystem interaction, receives user commands from the client, and runs as an http-server to serve the client to a connected user. The client is a graphical front end for all the features listed above that runs in the user's browser.

## Installation ##

1. [Install Dart.](https://www.dartlang.org/)
2. Clone this repo onto your local system.

## Usage ##

1. Run the server from the commander/server directory using: 
```
#!bash

./server.dart -d /path/to/workspace
```
2. Run an http-server from the commander/client/web directory. Alternatively, run the client via DartEditor.
3. Open a tab in Dartium (dart-enabled Chromium) and point it to the address where the http-server is running.
