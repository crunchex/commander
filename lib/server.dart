library cmdr;

import 'dart:io';
import 'dart:async';
import 'dart:convert';

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:http_server/http_server.dart';
import 'package:path/path.dart' as pathLib;

import 'tab/pty.dart';
import 'tab/camera/camera.dart';
import 'tab/editor.dart';
import 'tab/explorer.dart';
import 'git.dart';
import 'server_mailbox.dart';
import 'server_helper.dart' as help;

part 'commands.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class CmdrServer {
  static String defaultUprootPath = '/home/${Platform.environment['USER']}/uproot';
  static const String defaultGuiPath = '/opt/updroid/cmdr/web';
  static const bool defaultDebugFlag = false;

  Map _explorers = {};
  Map _tabs = {};
  Map<int, CameraServer> _camServers = {};
  CmdrMailbox _mailbox;
  Directory dir;

  CmdrServer (ArgResults results) {
    dir = new Directory(results['workspace']);
    dir.create();
    _initServer(_getVirDir(results));

    _mailbox = new CmdrMailbox('UpDroidClient');
    _registerMailbox();
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
  void _initServer(VirtualDirectory virDir) {
    // Set up an HTTP webserver and listen for standard page requests or upgraded
    // [WebSocket] requests.
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12060).then((HttpServer server) {
      print('[UpDroid Commander serving on port 12060]');
      print('You can now enter "localhost:12060" in your browser.\nCtrl-C to exit.');
      help.debug("HttpServer listening on port:${server.port}...", 0);
      server.asBroadcastStream()
          .listen((HttpRequest request) => _routeRequest(request, virDir))
          .asFuture()  // Automatically cancels on error.
          .catchError((_) => help.debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, VirtualDirectory virDir) {
    // WebSocket requests are considered "upgraded" HTTP requests.
    if (!WebSocketTransformer.isUpgradeRequest(request)) {
      _handleStandardRequest(request, virDir);
      return;
    }

    help.debug('Upgraded request received: ${request.uri.path}', 0);

    // TODO: objectIDs start at 1, but List indexes start at 0 - fix this.
    int objectID = int.parse(request.uri.pathSegments[1]);
    String type = request.uri.pathSegments[0];

    if (type == 'updroidexplorer') {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _explorers[objectID].handleWebSocket(ws));
      return;
    } else if (type == 'updroidclient') {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _mailbox.handleWebSocket(ws, request));
      return;
    } else if (type == 'updroidteleop') {
      return;
    }

    WebSocketTransformer.upgrade(request)
    .then((WebSocket ws) => _tabs[type][objectID].mailbox.handleWebSocket(ws, request));
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    help.debug("${request.method} request for: ${request.uri.path}", 0);

    if (virDir != null) {
      virDir.serveRequest(request);
    } else {
      help.debug('ERROR: no Virtual Directory to serve', 1);
    }
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent('CLIENT_CONFIG', _clientConfig);
    _mailbox.registerWebSocketEvent('GIT_PUSH', _gitPush);
    _mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);
    _mailbox.registerWebSocketEvent('OPEN_TAB', _openTab);
    _mailbox.registerWebSocketEvent('ADD_EXPLORER', _newExplorerCmdr);
    _mailbox.registerWebSocketEvent('CLOSE_EXPLORER', _closeExplorerCmdr);
  }

  void _clientConfig(UpDroidMessage um) {
    _initBackendClasses(dir).then((value) {
      _mailbox.ws.add('[[CLIENT_SERVER_READY]]' + JSON.encode(value));
    });
  }

  void _gitPush(UpDroidMessage um) {
    List runArgs = um.body.split('++');
    String dirPath = runArgs[0];
    String password = runArgs[1];
    //help.debug('dirPath: $dirPath, password: $password', 0);
    Git.push(dirPath, password);
  }

  // TODO: foldername passed but not used
  Future _initBackendClasses(Directory dir) {
    var completer = new Completer();

    Directory srcDir = new Directory('${pathLib.normalize(dir.path)}');
    srcDir.list().toList().then((folderList) {
      var result = [];
      var names = [];
      bool workspace;
      for(FileSystemEntity item in folderList) {
        if(item.runtimeType.toString() == "_Directory") {
          result.add(item);
        }
      }
      folderList = result;

      int num = 1;
      for(var folder in folderList) {
        workspace = false;
        for (var subFolder in folder.listSync()) {
          if(pathLib.basename(subFolder.path) == 'src') workspace = true;
        }
        if (workspace == true) {
          names.add(pathLib.basename(folder.path));
          _explorers[num] = new CmdrExplorer(folder, num);
          num += 1;
        }
      }
      completer.complete(names);
    });

    return completer.future;
  }

  void _newExplorerCmdr(UpDroidMessage um) {
    List explorerInfo = JSON.decode(um.body);
    int expNum = int.parse(explorerInfo[0]);
    String name = explorerInfo[1];
    Directory newWorkspace = new Directory(pathLib.normalize(dir.path + "/" + name));
    Directory source = new Directory(pathLib.normalize(newWorkspace.path + "/src"));
    source.createSync(recursive: true);
    Process.runSync('bash', ['-c', '. /opt/ros/indigo/setup.bash && catkin_init_workspace'], workingDirectory: pathLib.normalize(newWorkspace.path + "/src"), runInShell: true);
    _explorers[expNum] = (new CmdrExplorer(newWorkspace, expNum));

  }

  void _closeExplorerCmdr(UpDroidMessage um) {
    int expNum = int.parse(um.body);
    var toRemove;

    toRemove = _explorers[expNum];
    Directory workspace = new Directory(toRemove.expPath);
    _explorers.remove(expNum);
    workspace.delete(recursive: true);
    toRemove.killExplorer();
  }

  void _openTab(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('-');
    //int col = int.parse(idList[0]);
    int num = int.parse(idList[1]);
    String type = idList[2].toLowerCase();

    help.debug('Open tab request received: $id', 0);

    if (!_tabs.containsKey(type)) _tabs[type] = {};

    switch (type) {
      case 'updroideditor':
        _tabs[type][num] = new CmdrEditor(dir);
        break;
      case 'updroidcamera':
        _tabs[type][num] = new CmdrCamera(num, _camServers);
        break;

      case 'updroidconsole':
        String numRows = idList[3];
        String numCols = idList[4];
        _tabs[type][num] = new CmdrPty(num, dir.path, numRows, numCols);
        break;
    }
  }

  void _closeTab(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('_');
    String type = idList[0].toLowerCase();
    int num = int.parse(idList[1]);

    help.debug('Close tab request received: $id', 0);

    if (_tabs[type][num] != null) {
      _tabs[type][num].cleanup();
      _tabs[type][num] = null;
    }
  }

  // TODO: fix this!
//  void _cleanUpBackend() {
//    _explorers = {};
//    _tabs = {};
//    _camServers = {};
//  }
}