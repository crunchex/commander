library updroid_server;

import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:args/args.dart';
import 'package:watcher/watcher.dart';
import 'package:args/command_runner.dart';
import 'package:http_server/http_server.dart';

import 'lib/client_responses.dart';
import 'lib/server_helper.dart' as help;

part 'pty.dart';
part 'camera.dart';
part 'commands.dart';
part 'editor.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class CmdrServer {
  static const String defaultWorkspacePath = '/home/user/uproot';
  static const String defaultGuiPath = '/opt/updroid/cmdr/web';
  static const bool defaultDebugFlag = false;

  CmdrServer (ArgResults results) {
    Directory dir = new Directory(results['workspace']);
    DirectoryWatcher watcher = new DirectoryWatcher(dir.path);

    VirtualDirectory virDir;
    if (!results['serveronly']) {
      virDir = _getVirDir(results);
    }

    _initServer(dir, virDir, watcher);
  }

  /// Returns a [VirtualDirectory] set up with a path from [results].
  VirtualDirectory _getVirDir (ArgResults results) {
    String guiPath = results['path'];
    VirtualDirectory virDir;
    virDir = new VirtualDirectory(Platform.script.resolve(guiPath).toFilePath())
        ..allowDirectoryListing = true
        ..followLinks = true
        ..directoryHandler = (dir, request) {
          // Redirects '/' to 'index.html'
          var indexUri = new Uri.file(dir.path).resolve('index.html');
          virDir.serveFile(new File(indexUri.toFilePath()), request);
        };

    return virDir;
  }

  /// Initializes and HTTP server to serve the gui and handle [WebSocket] requests.
  void _initServer(Directory dir, VirtualDirectory virDir, DirectoryWatcher watcher) {
    // Set up an HTTP webserver and listen for standard page requests or upgraded
    // [WebSocket] requests.
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12060).then((HttpServer server) {
      help.debug("HttpServer listening on port:${server.port}...", 0);
      server.listen((HttpRequest request) {
        // WebSocket requests are considered "upgraded" HTTP requests.
        if (WebSocketTransformer.isUpgradeRequest(request)) {
          WebSocketTransformer
            .upgrade(request)
            .then((WebSocket ws) => _handleWebSocket(ws, dir, watcher));
          return;
        }

        _handleRequest(request, virDir);
      });
    });
  }

  void _handleRequest(HttpRequest request, VirtualDirectory virDir) {
    help.debug("${request.method} request for: ${request.uri.path}", 0);

    if (virDir != null) {
      virDir.serveRequest(request);
    }
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void _handleWebSocket(WebSocket socket, Directory dir, DirectoryWatcher watcher) {
    help.debug('Client connected!', 0);
    StreamController<String> processInput = new StreamController<String>.broadcast();

    socket.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Incoming message: ' + s, 0);

      switch (um.header) {

        case "INITIAL_DIRECTORY_LIST":
          sendInitial(socket, dir);
          break;

        case 'EXPLORER_DIRECTORY_PATH':
          sendPath(socket, dir);
          break;

        case 'EXPLORER_DIRECTORY_LIST':
          sendDirectory(socket, dir);
          break;

        case 'EXPLORER_DIRECTORY_REFRESH':
          refreshDirectory(socket, dir);
          break;

        case 'EXPLORER_NEW_FILE':
          fsNewFile(um.body);
          break;

        case 'EXPLORER_NEW_FOLDER':
          fsNewFolder(um.body);
          // Empty folders don't trigger an incremental update, so we need to
          // refresh the entire workspace.
          sendDirectory(socket, dir);
          break;

        case 'EXPLORER_RENAME':
          fsRename(um.body);
          break;

        case 'EXPLORER_MOVE':
          // Currently implemented in the same way as RENAME as there is no
          // direct API for MOVE.
          fsRename(um.body);
          break;

        case 'EXPLORER_DELETE':
          fsDelete(um.body, socket);
          break;

        case 'EDITOR_REQUEST_LIST':
          sendEditorList(socket, dir);
          break;

        case 'EDITOR_OPEN':
          sendFileContents(socket, um.body);
          break;

        case 'EDITOR_SAVE':
          saveFile(um.body);
          break;

        case 'CLIENT_CONFIG':
          _initBackendClasses(um.body, dir, socket).then((value) {
            socket.add('[[CLIENT_SERVER_READY]]');
          });
          break;

        default:
          help.debug('Message received without updroid header.', 1);
      }
    });

    watcher.events.listen((e) => help.formattedFsUpdate(socket, e));
  }

  Future _initBackendClasses(String config, Directory dir, WebSocket ws) {
    var completer = new Completer();

    Map tabs = JSON.decode(config);

    for (String className in tabs['side']) {

    }

    for (String className in tabs['left']) {
      if (className == CmdrEditor.guiName) {
        CmdrEditor camera = new CmdrEditor(ws);
      } else if (className == CmdrCamera.guiName) {
        CmdrCamera camera = new CmdrCamera(1);
      }
    }

    int i = 1;
    for (String className in tabs['right']) {
      if (className == CmdrPty.guiName) {
        CmdrPty pty = new CmdrPty(i, dir.path);
      }
      i++;
    }

    completer.complete();
    return completer.future;
  }
}