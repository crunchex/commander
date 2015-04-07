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

import 'catkin.dart';
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

    _initServer(dir, _getVirDir(results));
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
  void _initServer(Directory dir, VirtualDirectory virDir) {
    // Set up an HTTP webserver and listen for standard page requests or upgraded
    // [WebSocket] requests.
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12060).then((HttpServer server) {
      help.debug("HttpServer listening on port:${server.port}...", 0);
      server.asBroadcastStream()
          .listen((HttpRequest request) => _routeRequest(request, dir, virDir))
          .asFuture()  // Automatically cancels on error.
          .catchError((_) => help.debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, Directory dir, VirtualDirectory virDir) {
    help.debug(request.uri.path, 0);

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

      case 'camera':
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _cameras[objectID].handleWebSocket(ws, request));
        break;

      default:
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _handleWebSocket(ws, dir));
    }
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    if (request.uri.pathSegments[0] == 'video') {
      int objectID = int.parse(request.uri.pathSegments[1]) - 1;
      _cameras[objectID].handleVideoFeed(request);
      return;
    }

    help.debug("${request.method} request for: ${request.uri.path}", 0);

    if (virDir != null) {
      virDir.serveRequest(request);
    }
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void _handleWebSocket(WebSocket socket, Directory dir) {
    help.debug('Commander client connected.', 0);
    StreamController<String> processInput = new StreamController<String>.broadcast();

    socket.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Server incoming: ' + s, 0);

      switch (um.header) {
        case 'CLIENT_CONFIG':
          _initBackendClasses(dir).then((value) {
            socket.add('[[CLIENT_SERVER_READY]]');
          });
          break;

        case 'WORKSPACE_BUILD':
          Catkin.buildWorkspace(dir.path).then((result) {
            socket.add('[[BUILD_RESULT]]' + result);
          });
          break;

        case 'CLOSE_TAB':
          _closeTab(um.body);
          break;

        case 'OPEN_TAB':
          _openTab(um.body, dir);
          break;

        default:
          help.debug('Message received without updroid header.', 1);
      }
    }).onDone(() => _cleanUpBackend());
  }

  Future _initBackendClasses(Directory dir) {
    var completer = new Completer();

    _explorers.add(new CmdrExplorer(dir));

    completer.complete();
    return completer.future;
  }

  void _openTab(String id, Directory dir) {
    List idList = id.split('-');
    int col = int.parse(idList[0]);
    int num = int.parse(idList[1]);
    String type = idList[2];

    switch (type) {
      case 'UpDroidEditor':
        _editors.add(new CmdrEditor(dir));
        break;
      case 'UpDroidCamera':
        _cameras.add(new CmdrCamera(num));
        break;

      case 'UpDroidConsole':
        _ptys.add(new CmdrPty(num, dir.path));
        break;
    }
  }

  void _closeTab(String id) {
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    switch (type) {
      case 'UpDroidEditor':
        _editors.removeAt(num - 1);
        break;

      case 'UpDroidCamera':
        _cameras.removeAt(num - 1);
        break;

      case 'UpDroidConsole':
        _ptys.removeAt(num - 1);
        break;
    }
  }

  void _cleanUpBackend() {
    _explorers = [];
    _editors = [];
    _ptys = [];
    _cameras = [];
  }
}