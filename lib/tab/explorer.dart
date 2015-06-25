library cmdr_explorer;

import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'package:watcher/watcher.dart';
import 'package:path/path.dart' as pathLib;

import '../ros/ros.dart';
import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrExplorer {
  static const String guiName = 'UpDroidExplorer';

  int expNum;
  Directory uproot;

  Workspace _currentWorkspace;
  DirectoryWatcher _currentWatcher, _uprootWatcher;
  StreamSubscription _currentWatcherStream, _uprootWatcherStream;
  WebSocket _ws;

  //TODO: make asynchroneous
  CmdrExplorer(this.expNum, this.uproot) {
    if (_currentWorkspace != null) return;

    // TODO: retrieve saved data for the most recently opened workspace.
    // TODO: handle changes to uproot made on the server side.
//    _uprootWatcher = new DirectoryWatcher(uproot.path);
//    _uprootWatcherStream = _uprootWatcher.events.listen((WatchEvent w) {
//      _ws.add('[[WORKSPACE_NAME]]' + w.path.replaceFirst('${uproot.path}/', '').split('/').first);
//    });
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void handleWebSocket(WebSocket ws) {
    help.debug('Explorer client connected.', 0);
    _ws = ws;

    ws.listen((String s) {
      UpDroidMessage um = new UpDroidMessage.fromString(s);
      help.debug('Explorer incoming: ' + s, 0);

      switch (um.header) {
        case "REQUEST_WORKSPACE_CONTENTS":
          _sendWorkspaceSync(ws);
          break;

        case 'REQUEST_WORKSPACE_PATH':
          _sendPath(ws);
          break;

        case 'REQUEST_WORKSPACE_NAMES':
          _sendWorkspaceNames(ws);
          break;

        case 'NEW_WORKSPACE':
          _newWorkspace(um.body, ws);
          break;

        case 'SET_CURRENT_WORKSPACE':
          _setCurrentWorkspace(um.body, ws);
          break;

        case 'NEW_FILE':
          _fsNewFile(um.body);
          break;

        case 'NEW_FOLDER':
          _fsNewFolder(um.body);
          break;

        case 'RENAME':
          _fsRename(um.body, ws);
          break;

        case 'DELETE':
          _fsDelete(um.body, ws);
          break;

        case 'WORKSPACE_CLEAN':
          _workspaceClean(ws);
          break;

        case 'WORKSPACE_BUILD':
          _buildWorkspace(ws);
          break;

        case 'BUILD_PACKAGE':
          _buildPackage(um.body, ws);
          break;

        case 'BUILD_PACKAGES':
          _buildPackages(um.body, ws);
          break;

        case 'REQUEST_NODE_LIST':
          _nodeList(ws);
          break;

        case 'RUN_NODE':
          _runNode(um.body);
          break;

        default:
          help.debug('Explorer: message received without updroid header.', 1);
      }

    });
  }

  void killExplorer() {
    this.expNum = null;
    this._currentWatcher = null;
  }

  void _sendWorkspace(WebSocket s) {
    if (_currentWatcher == null) {
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcherStream = _currentWatcher.events.listen((e) => formattedFsUpdate(s, e));
    }

    _currentWorkspace.listContents().listen((String file) => s.add('[[ADD_UPDATE]]' + file));
  }

  void _sendWorkspaceSync(WebSocket s) {
    if (_currentWatcher == null) {
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcherStream = _currentWatcher.events.listen((e) => formattedFsUpdate(s, e));
    }

    List<String> files = _currentWorkspace.listContentsSync();
    files.forEach((String file) => s.add('[[ADD_UPDATE]]' + file));
  }

  void _sendPath(WebSocket s) {
    help.formattedMessage(s, 'EXPLORER_DIRECTORY_PATH', _currentWorkspace.path);
  }

  void _sendWorkspaceNames(WebSocket ws) {
    uproot.list()
      .where((Directory w) => w.path.split('/').length == uproot.path.split('/').length + 1)
      .listen((Directory w) => ws.add('[[WORKSPACE_NAME]]' + w.path.split('/').last));
  }

  void _setCurrentWorkspace(String newWorkspaceName, WebSocket ws) {
    if (_currentWatcherStream != null) _currentWatcherStream.cancel();

    _currentWorkspace = new Workspace('${uproot.path}/$newWorkspaceName');
    _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
    _currentWatcherStream = _currentWatcher.events.listen((e) => formattedFsUpdate(ws, e));
    _sendPath(ws);
  }

  void _newWorkspace(String data, WebSocket ws) {
    Workspace newWorkspace = new Workspace('${uproot.path}/$data');
    newWorkspace.create().then((Workspace workspace) {
      workspace.initSync();

      if (_currentWorkspace != null) return;

      if (_currentWatcherStream != null) _currentWatcherStream.cancel();
      _currentWorkspace = newWorkspace;
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcherStream = _currentWatcher.events.listen((e) => formattedFsUpdate(ws, e));
      _sendPath(ws);
    });
  }

  /// Convenience method for adding a formatted filesystem update to the socket
  /// stream.
  ///   ex. add /home/user/tmp => [[ADD]]/home/user/tmp
  Future formattedFsUpdate(WebSocket socket, WatchEvent e) async {
    List<String> split = e.toString().split(' ');
    String header = split[0].toUpperCase();
    String path = split[1];

    bool isFile = await FileSystemEntity.isFile(path);
    String fileString = isFile ? 'F:${path}' : 'D:${path}';

    var formatted = '[[${header}_UPDATE]]' + fileString;
    help.debug('Outgoing: ' + formatted, 0);
    if (header != 'MODIFY') socket.add(formatted);
  }

  void _fsNewFile(String path) {
    String fullPath = pathLib.join(path + '/untitled.py');
    File newFile = new File(fullPath);

    int untitledNum = 0;
    while (newFile.existsSync()) {
      untitledNum++;
      fullPath = path + '/untitled' + untitledNum.toString() + '.py';
      newFile = new File(fullPath);
    }

    newFile.create();
  }

  void _fsNewFolder(String path) {
    String fullPath = path;
    Directory newFolder = new Directory(fullPath);

    int untitledNum = 0;
    while(newFolder.existsSync()) {
      untitledNum++;
      fullPath = path + untitledNum.toString();
      newFolder = new Directory(fullPath);
    }

    newFolder.createSync();
  }

  void _fsRename(String data, WebSocket ws) {
    List<String> split = data.split(':');
    String oldPath = split[0];
    String newPath = split[1];

    FileSystemEntity.type(oldPath).then((FileSystemEntityType type) {
      if (type == FileSystemEntityType.NOT_FOUND) return;

      bool isDir = FileSystemEntity.isDirectorySync(oldPath);
      FileSystemEntity entity = isDir ? new Directory(oldPath) : new File(oldPath);
      entity.rename(newPath);

      // Force a remove update on the top level folder as
      // watcher issue workaround.
      if (isDir) ws.add('[[REMOVE_UPDATE]]D:$oldPath');
    });
  }

  void _fsDelete(String path, WebSocket socket) {
    FileSystemEntity entity;
    bool isDir = FileSystemEntity.isDirectorySync(path);

    entity = isDir ? new Directory(path) : new File(path);
    entity.delete(recursive: true);

    // Force a remove update on the top level folder as
    // watcher issue workaround.
    if (isDir) socket.add('[[REMOVE_UPDATE]]D:$path');
  }

  void _workspaceClean(WebSocket s) {
    _currentWorkspace.clean().then((result) {
      s.add('[[WORKSPACE_CLEAN]]');
    });
  }

  void _buildWorkspace(WebSocket s) {
    _currentWorkspace.buildWorkspace().then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      s.add('[[WORKSPACE_BUILD]]' + resultString);
      s.add('[[BUILD_COMPLETE]]' + JSON.encode([_currentWorkspace.path]));
    });
  }

  void _buildPackage(String packagePath, WebSocket s) {
    String packageName = packagePath.split('/').last;
    _currentWorkspace.buildPackage(packageName).then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      s.add('[[PACKAGE_BUILD_RESULTS]]' + resultString);
      s.add('[[BUILD_COMPLETE]]' + JSON.encode([packagePath]));
    });
  }

  void _buildPackages(String data, WebSocket s) {
    List<String> packagePaths = JSON.decode(data);

    List<String> packageNames = [];
    packagePaths.forEach((String packagePath) => packageNames.add(packagePath.split('/').last));

    _currentWorkspace.buildPackages(packageNames).then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      s.add('[[PACKAGE_BUILD_RESULTS]]' + resultString);
      s.add('[[BUILD_COMPLETE]]' + data);
    });
  }

  void _nodeList(WebSocket s) {
    _currentWorkspace.listNodes().listen((Map package) {
      String data = JSON.encode(package);
      s.add('[[LAUNCH]]' + data);
    });
  }

  void _runNode(String data) {
    List decodedData = JSON.decode(data);
    String packageName = decodedData[0];
    String nodeName = decodedData[1];
    List nodeArgs = decodedData.sublist(2);

    _currentWorkspace.runNode(packageName, nodeName, nodeArgs);
  }

  void cleanup() {
    _currentWatcherStream.cancel();
  }
}