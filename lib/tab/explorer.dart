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

  int id;
  CmdrMailbox mailbox;
  Directory uproot;

  Workspace _currentWorkspace;
  DirectoryWatcher _currentWatcher, _uprootWatcher;
  StreamSubscription _currentWatcherStream, _uprootWatcherStream;
  WebSocket _ws;

  //TODO: make asynchroneous
  CmdrExplorer(this.id, this.uproot) {
    if (_currentWorkspace != null) return;

    mailbox = new CmdrMailbox(guiName, id);
    _registerMailbox();

    // TODO: retrieve saved data for the most recently opened workspace.
    // TODO: handle changes to uproot made on the server side.
//    _uprootWatcher = new DirectoryWatcher(uproot.path);
//    _uprootWatcherStream = _uprootWatcher.events.listen((WatchEvent w) {
//      _ws.add('[[WORKSPACE_NAME]]' + w.path.replaceFirst('${uproot.path}/', '').split('/').first);
//    });
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('REQUEST_WORKSPACE_CONTENTS', _sendWorkspaceSync);
    mailbox.registerWebSocketEvent('REQUEST_WORKSPACE_PATH', _getPath);
    mailbox.registerWebSocketEvent('REQUEST_WORKSPACE_NAMES', _sendWorkspaceNames);
    mailbox.registerWebSocketEvent('NEW_WORKSPACE', _newWorkspace);
    mailbox.registerWebSocketEvent('SET_CURRENT_WORKSPACE', _setWorkspace);
    mailbox.registerWebSocketEvent('NEW_FILE', _fsNewFile);
    mailbox.registerWebSocketEvent('NEW_FOLDER', _fsNewFolder);
    mailbox.registerWebSocketEvent('RENAME', _fsRename);
    mailbox.registerWebSocketEvent('DELETE', _fsDelete);
    mailbox.registerWebSocketEvent('OPEN_FILE', _openFile);
    mailbox.registerWebSocketEvent('WORKSPACE_CLEAN', _workspaceClean);
    mailbox.registerWebSocketEvent('WORKSPACE_BUILD', _buildWorkspace);
    mailbox.registerWebSocketEvent('BUILD_PACKAGE', _buildPackage);
    mailbox.registerWebSocketEvent('BUILD_PACKAGES', _buildPackages);
    mailbox.registerWebSocketEvent('CREATE_PACKAGE', _createPackage);
    mailbox.registerWebSocketEvent('REQUEST_NODE_LIST', _launcherList);
    mailbox.registerWebSocketEvent('RUN_NODE', _runLauncher);
  }

  void _sendWorkspaceSync(UpDroidMessage um) {
    if (_currentWatcher == null) {
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcherStream = _currentWatcher.events.listen((e) => _formattedFsUpdate(e));
    }

    List<String> files = _currentWorkspace.listContentsSync();
    files.forEach((String file) => mailbox.ws.add('[[ADD_UPDATE]]' + file));
  }

  void _getPath(UpDroidMessage um) => _sendPath();

  void _sendWorkspaceNames(UpDroidMessage um) {
    List<String> names = [];
    uproot.list()
    .where((Directory w) => Workspace.isWorkspace(w.path))
    .listen((Directory w) => names.add(w.path.split('/').last))
    .onDone(() => mailbox.ws.add('[[WORKSPACE_NAMES]]' + JSON.encode(names)));
  }

  void _newWorkspace(UpDroidMessage um) {
    String data = um.body;
    Workspace newWorkspace = new Workspace('${uproot.path}/$data');
    newWorkspace.create().then((Workspace workspace) {
      workspace.initSync();
      _setCurrentWorkspace(data);
    });
  }

  void _setWorkspace(UpDroidMessage um) => _setCurrentWorkspace(um.body);

  void _fsNewFile(UpDroidMessage um) {
    String path = um.body;
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

  void _fsNewFolder(UpDroidMessage um) {
    String path = um.body;
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

  void _fsRename(UpDroidMessage um) {
    String data = um.body;
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
      if (isDir) mailbox.ws.add('[[REMOVE_UPDATE]]D:$oldPath');
    });
  }

  void _fsDelete(UpDroidMessage um) {
    String path = um.body;
    FileSystemEntity entity;
    bool isDir = FileSystemEntity.isDirectorySync(path);

    entity = isDir ? new Directory(path) : new File(path);
    entity.delete(recursive: true);

    // Force a remove update on the top level folder as
    // watcher issue workaround.
    if (isDir) mailbox.ws.add('[[REMOVE_UPDATE]]D:$path');
  }

  void _openFile(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidEditor', -1, um));
  }

  void _workspaceClean(UpDroidMessage um) {
    _currentWorkspace.clean().then((result) {
      mailbox.ws.add('[[WORKSPACE_CLEAN]]');
    });
  }

  void _buildWorkspace(UpDroidMessage um) {
    _currentWorkspace.buildWorkspace().then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      mailbox.ws.add('[[WORKSPACE_BUILD]]' + resultString);
      mailbox.ws.add('[[BUILD_COMPLETE]]' + JSON.encode([_currentWorkspace.path]));
    });
  }

  void _buildPackage(UpDroidMessage um) {
    String packagePath = um.body;
    String packageName = packagePath.split('/').last;
    _currentWorkspace.buildPackage(packageName).then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      mailbox.ws.add('[[PACKAGE_BUILD_RESULTS]]' + resultString);
      mailbox.ws.add('[[BUILD_COMPLETE]]' + JSON.encode([packagePath]));
    });
  }

  void _buildPackages(UpDroidMessage um) {
    String data = um.body;
    List<String> packagePaths = JSON.decode(data);

    List<String> packageNames = [];
    packagePaths.forEach((String packagePath) => packageNames.add(packagePath.split('/').last));

    _currentWorkspace.buildPackages(packageNames).then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      mailbox.ws.add('[[PACKAGE_BUILD_RESULTS]]' + resultString);
      mailbox.ws.add('[[BUILD_COMPLETE]]' + data);
    });
  }

  void _createPackage(UpDroidMessage um) {
    List<String> split = um.body.split(':');
    String name = split[0];

    List<String> dependencies = JSON.decode(split[1]);

    _currentWorkspace.createPackage(name, dependencies);
  }

  void _launcherList(UpDroidMessage um) {
    _currentWorkspace.listLaunchers().listen((Map launcher) {
      String data = JSON.encode(launcher);
      mailbox.ws.add('[[LAUNCH]]' + data);
    });
  }

  void _runLauncher(UpDroidMessage um) {
    String data = um.body;
    List decodedData = JSON.decode(data);
    String packageName = decodedData[0];
    String nodeName = decodedData[1];
    List nodeArgs = decodedData.sublist(2);

    _currentWorkspace.runNode(packageName, nodeName, nodeArgs);
  }

  /// Convenience method for adding a formatted filesystem update to the socket
  /// stream.
  ///   ex. add /home/user/tmp => [[ADD]]/home/user/tmp
  Future _formattedFsUpdate(WatchEvent e) async {
    List<String> split = e.toString().split(' ');
    String header = split[0].toUpperCase();
    String path = split[1];

    bool isFile = await FileSystemEntity.isFile(path);
    String fileString = isFile ? 'F:${path}' : 'D:${path}';

    var formatted = '[[${header}_UPDATE]]' + fileString;
    help.debug('Outgoing: ' + formatted, 0);
    if (header != 'MODIFY') mailbox.ws.add(formatted);
  }

  void _setCurrentWorkspace(String newWorkspaceName) {
    if (_currentWatcherStream != null) _currentWatcherStream.cancel();

    _currentWorkspace = new Workspace('${uproot.path}/$newWorkspaceName');
    _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
    _currentWatcherStream = _currentWatcher.events.listen((e) => _formattedFsUpdate(e));
    _sendPath();
  }

  void _sendPath() {
    mailbox.ws.add('[[EXPLORER_DIRECTORY_PATH]]' + _currentWorkspace.path);
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, id);
    _currentWatcherStream.cancel();
  }
}