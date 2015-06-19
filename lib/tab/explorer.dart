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
  DirectoryWatcher _currentWatcher;
  StreamSubscription _currentWatcherStream;
  WebSocket _ws;

  //TODO: make asynchroneous
  CmdrExplorer(this.expNum, this.uproot) {
    if (_currentWorkspace != null) return;

    // Just pick the first workspace unless there's a better choice.
    // TODO: retrieve saved data for the most recently opened workspace.
    uproot.list().first.then((Directory firstWorkspace) =>_currentWorkspace = new Workspace(firstWorkspace.path));
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void handleWebSocket(WebSocket ws) {
    help.debug('Explorer client connected.', 0);

    ws.listen((String s) {
      UpDroidMessage um = new UpDroidMessage.fromString(s);
      help.debug('Explorer incoming: ' + s, 0);

      switch (um.header) {
        case "INITIAL_DIRECTORY_LIST":
          _sendInitial(ws);
          break;

        case 'EXPLORER_DIRECTORY_PATH':
          _sendPath(ws);
          break;

        case 'EXPLORER_DIRECTORY_LIST':
          _sendDirectory(ws);
          break;

        case 'EXPLORER_DIRECTORY_REFRESH':
          _refreshDirectory(ws);
          break;

        case 'EXPLORER_NEW_FILE':
          _fsNewFile(um.body);
          break;

        case 'EXPLORER_NEW_FOLDER':
          _fsNewFolder(um.body);
          // Empty folders don't trigger an incremental update, so we need to
          // refresh the entire workspace.
          _sendDirectory(ws);
          break;

        case 'EXPLORER_RENAME':
          _fsRename(um.body);
          break;

        case 'EXPLORER_MOVE':
          // Currently implemented in the same way as RENAME as there is no
          // direct API for MOVE.
          _fsRename(um.body);
          break;

        case 'EXPLORER_DELETE':
          _fsDelete(um.body, ws);
          break;

        case 'EXPLORER_WORKSPACE_CLEAN':
          _workspaceClean(ws);
          break;

        case 'EXPLORER_WORKSPACE_BUILD':
          _workspaceBuild(ws);
          break;

        case 'CATKIN_NODE_LIST':
          _nodeList(ws);
          break;

        case 'CATKIN_RUN':
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

  void _sendInitial(WebSocket s) {
    if (_currentWatcher == null) {
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcher.events.listen((e) => formattedFsUpdate(s, e));
    }

    _currentWorkspace.listContents().listen((String file) => s.add('[[EXPLORER_ADD]]' + file));
  }

  void _sendDirectory(WebSocket s) {
    if (_currentWatcher == null) {
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcher.events.listen((e) => formattedFsUpdate(s, e));
    }

    _currentWorkspace.getContentsAsStrings().then((files) {
      s.add('[[EXPLORER_DIRECTORY_LIST]]' + JSON.encode(files));
    });
  }

  void _refreshDirectory(WebSocket s) {
    _currentWorkspace.getContentsAsStrings().then((files) {
      s.add('[[EXPLORER_DIRECTORY_REFRESH]]' + JSON.encode(files));
    });
  }

  void _sendPath(WebSocket s) {
    help.formattedMessage(s, 'EXPLORER_DIRECTORY_PATH', _currentWorkspace.path);
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

    var formatted = '[[EXPLORER_$header]]' + fileString;
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

  void _fsRename(String data) {
    List<String> split = data.split(':');
    String oldPath = split[0];
    String newPath = split[1];

    FileSystemEntity.type(oldPath).then((FileSystemEntityType type) {
      if (type == FileSystemEntityType.NOT_FOUND) return;

      bool isDir = FileSystemEntity.isDirectorySync(oldPath);
      FileSystemEntity entity = isDir ? new Directory(oldPath) : new File(oldPath);
      entity.rename(newPath);
    });
  }

  void _fsDelete(String path, WebSocket socket) {
    FileSystemEntity entity;
    bool isDir = FileSystemEntity.isDirectorySync(path);

    entity = isDir ? new Directory(path) : new File(path);
    entity.delete(recursive: true);

    // Force a remove update on the top level folder as
    // watcher issue workaround.
    if (isDir) socket.add('[[EXPLORER_REMOVE]]D:$path');
  }

  void _workspaceClean(WebSocket s) {
    _currentWorkspace.clean().then((result) {
      s.add('[[WORKSPACE_CLEAN]]');
    });
  }

  void _workspaceBuild(WebSocket s) {
    _currentWorkspace.build().then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      s.add('[[WORKSPACE_BUILD]]' + resultString);
    });
  }

  void _nodeList(WebSocket s) {
    Ros.nodeList(_currentWorkspace, s);
  }

  void _runNode(String runCommand) {
    Ros.runNode(_currentWorkspace, runCommand);
  }
}