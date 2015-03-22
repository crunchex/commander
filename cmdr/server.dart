library updroid_server;

import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:args/args.dart';
import 'package:watcher/watcher.dart';
import 'package:args/command_runner.dart';
import 'package:http_server/http_server.dart';
import 'package:path/path.dart' as pathLib;

import 'lib/server_helper.dart' as help;

part 'pty.dart';
part 'camera.dart';
part 'commands.dart';
part 'editor.dart';
part 'explorer.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class CmdrServer {
  static const String defaultWorkspacePath = '/home/user/uproot';
  static const String defaultGuiPath = '/opt/updroid/cmdr/web';
  static const bool defaultDebugFlag = false;

  List<CmdrExplorer> _explorers = [];
  List<CmdrEditor> _editors = [];
  List<CmdrPty> _ptys = [];
  List<CmdrCamera> _cameras = [];

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
      server.asBroadcastStream()
          .listen((HttpRequest request) => _routeRequest(request, dir, virDir, watcher))
          .asFuture()  // Automatically cancels on error.
          .catchError((_) => help.debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, Directory dir, VirtualDirectory virDir, DirectoryWatcher watcher) {
    // WebSocket requests are considered "upgraded" HTTP requests.
    if (!WebSocketTransformer.isUpgradeRequest(request)) {
      _handleStandardRequest(request, virDir);
      return;
    }

    // TODO: objectIDs start at 1, but List indexes start at 0 - fix this.
    int objectID = int.parse(request.uri.pathSegments[1]) - 1;
    switch (request.uri.pathSegments[0]) {
      case 'editor':
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _editors[objectID].handleWebSocket(ws));
        break;

      case 'explorer':
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _explorers[objectID].handleWebSocket(ws));
        break;

      default:
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _handleWebSocket(ws, dir, watcher));
    }
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    help.debug("${request.method} request for: ${request.uri.path}", 0);

    if (virDir != null) {
      virDir.serveRequest(request);
    }
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void _handleWebSocket(WebSocket socket, Directory dir, DirectoryWatcher watcher) {
    help.debug('Commander client connected.', 0);
    StreamController<String> processInput = new StreamController<String>.broadcast();

    socket.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Server incoming: ' + s, 0);

      switch (um.header) {
        case 'CLIENT_CONFIG':
          _initBackendClasses(um.body, dir, socket, watcher).then((value) {
            socket.add('[[CLIENT_SERVER_READY]]');
          });
          break;

        default:
          help.debug('Message received without updroid header.', 1);
      }
    }).onDone(() => _cleanUpBackend());
  }

  Future _initBackendClasses(String config, Directory dir, WebSocket ws, DirectoryWatcher watcher) {
    var completer = new Completer();

    Map tabs = JSON.decode(config);

    for (String column in tabs.keys) {
      for (String guiName in tabs[column]) {
        if (guiName == CmdrExplorer.guiName) {
          _explorers.add(new CmdrExplorer(dir, watcher));
        } else if (guiName == CmdrEditor.guiName) {
          _editors.add(new CmdrEditor(dir));
        } else if (guiName == CmdrCamera.guiName) {
          _cameras.add(new CmdrCamera(_cameras.length + 1));
        } else if (guiName == CmdrPty.guiName) {
          _ptys.add(new CmdrPty(_ptys.length + 1, dir.path));
        }
      }
    }

    completer.complete();
    return completer.future;
  }

  void _cleanUpBackend() {
    _explorers = [];
    _editors = [];
    _ptys = [];
    _cameras = [];
  }
}