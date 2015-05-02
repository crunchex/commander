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

import 'ros/ros.dart';
import 'git.dart';
import 'server_helper.dart' as help;

part 'commands.dart';
part 'tab/pty.dart';
part 'tab/camera.dart';
part 'tab/editor.dart';
part 'tab/explorer.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class CmdrServer {
  static String defaultWorkspacePath = '/home/${Platform.environment['USER']}/uproot';
  static const String defaultGuiPath = '/opt/updroid/cmdr/web';
  static const bool defaultDebugFlag = false;

  List<CmdrExplorer> _explorers = [];
  List<CmdrEditor> _editors = [];
  List<CmdrPty> _ptys = [];
  List<CmdrCamera> _cameras = [];

  CmdrServer (ArgResults results) {
    Directory dir = new Directory(results['workspace']);
    Workspace ws = _setUpWorkspace(results['workspace']);
    _initServer(ws, dir, _getVirDir(results));
  }

  /// Ensure that the workspace exists and is in good order.
  Workspace _setUpWorkspace(String path) {
    Workspace ws = new Workspace(path);
//    ws.create(recursive: true).then((ws) {
//      ws.initSync();
//    });

    return ws;
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
  void _initServer(Workspace ws, Directory dir, VirtualDirectory virDir) {
    // Set up an HTTP webserver and listen for standard page requests or upgraded
    // [WebSocket] requests.
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12060).then((HttpServer server) {
      print('[UpDroid Commander serving on port 12060]');
      print('You can now enter "localhost:12060" in your browser.\nCtrl-C to exit.');
      help.debug("HttpServer listening on port:${server.port}...", 0);
      server.asBroadcastStream()
          .listen((HttpRequest request) => _routeRequest(request, ws, dir, virDir))
          .asFuture()  // Automatically cancels on error.
          .catchError((_) => help.debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, Workspace workspace, Directory dir, VirtualDirectory virDir) {
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
          .then((WebSocket ws) {
            _explorers[objectID].handleWebSocket(ws, request);
        });
        break;

      case 'camera':
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _cameras[objectID].handleWebSocket(ws, request));
        break;

      case 'pty':
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _ptys[objectID].handleWebSocket(ws));
        break;

      default:
        WebSocketTransformer
          .upgrade(request)
          .then((WebSocket ws) => _handleWebSocket(ws, workspace, dir));
    }
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    help.debug("${request.method} request for: ${request.uri.path}", 0);

    if (request.uri.pathSegments.length != 0 && request.uri.pathSegments[0] == 'video') {
      int objectID = int.parse(request.uri.pathSegments[1]) - 1;
      _cameras[objectID].handleVideoFeed(request);
      return;
    }

    if (virDir != null) {
      virDir.serveRequest(request);
    } else {
      help.debug('ERROR: no Virtual Directory to serve', 1);
    }
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void _handleWebSocket(WebSocket socket, Workspace workspace, Directory dir) {
    help.debug('Commander client connected.', 0);

    socket.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Server incoming: ' + s, 0);

      switch (um.header) {
        case 'CLIENT_CONFIG':
          _initBackendClasses(dir).then((value) {
            socket.add('[[CLIENT_SERVER_READY]]' + JSON.encode(value));
          });
          break;

        case 'WORKSPACE_CLEAN':
          workspace.clean().then((result) {
            socket.add('[[WORKSPACE_CLEAN_DONE]]');
          });
          break;

        case 'WORKSPACE_BUILD':
          workspace.build().then((ProcessResult result) {
            String resultString = result.exitCode == 0 ? '' : result.stderr;
            socket.add('[[BUILD_RESULT]]' + resultString);
          });
          break;

        case 'CATKIN_RUN':
          Ros.runNode(workspace, um.body);
          break;

		//TODO: Need to change to grab all directories
        case 'CATKIN_NODE_LIST':
          Ros.nodeList(workspace, socket);
          break;

        case 'GIT_PUSH':
          List runArgs = um.body.split('++');
          String dirPath = runArgs[0];
          String password = runArgs[1];
          //help.debug('dirPath: $dirPath, password: $password', 0);
          Git.push(dirPath, password);
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

    Directory srcDir = new Directory('${pathLib.normalize(dir.path)}');
    srcDir.list().toList().then((folderList) {
      var result = [];
      var names = [];
      for(FileSystemEntity item in folderList) {
        if(item.runtimeType.toString() == "_Directory"){
          result.add(item);
        }
      }
      folderList = result;

      int num = 1;
      for(var folder in folderList) {
        names.add(pathLib.basename(folder.path));
        _explorers.add(new CmdrExplorer(folder, num));
        num += 1;
      }

      completer.complete(names);
    });

    return completer.future;
  }

  void _openExplorer(String id, Directory dir) {

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
        String numRows = idList[3];
        String numCols = idList[4];
        _ptys.add(new CmdrPty(num, dir.path, numRows, numCols));
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