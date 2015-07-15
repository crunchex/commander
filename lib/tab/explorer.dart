library cmdr_explorer;

import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'package:watcher/watcher.dart';
import 'package:path/path.dart' as pathLib;

import 'api/ros/ros.dart';
import 'api/updroid_message.dart';
import '../server_mailbox.dart';
import '../server_helper.dart' as help;
import '../post_office.dart';
import 'api/server_message.dart';

class CmdrExplorer {
  static const String guiName = 'UpDroidExplorer';

  int id;
  CmdrMailbox mailbox;
  Directory uproot;

  Workspace _currentWorkspace;
  DirectoryWatcher _currentWatcher;
  StreamSubscription _currentWatcherStream;

  //TODO: make asynchroneous
  CmdrExplorer(this.id, this.uproot) {
    if (_currentWorkspace != null) return;

    mailbox = new CmdrMailbox(guiName, id);
    _registerMailbox();

    // TODO: retrieve saved data for the most recently opened workspace.
    // TODO: handle changes to uproot made on the server side.
//    _uprootWatcher = new DirectoryWatcher(uproot.path);
//    _uprootWatcherStream = _uprootWatcher.events.listen((WatchEvent w) {
//      mailbox.send(new Msg('WORKSPACE_NAME', w.path.replaceFirst('${uproot.path}/', '').split('/').first);
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
    mailbox.registerWebSocketEvent('REQUEST_EDITOR_LIST', _requestEditorList);
    mailbox.registerWebSocketEvent('RETURN_SELECTED', _returnSelected);

    mailbox.registerServerMessageHandler('SEND_EDITOR_LIST', _sendEditorList);
    mailbox.registerServerMessageHandler('REQUEST_SELECTED', _getSelected);
  }

  void _sendWorkspaceSync(Msg um) {
    if (_currentWatcher == null) {
      _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
      _currentWatcherStream = _currentWatcher.events.listen((e) => _formattedFsUpdate(e));
    }

    List<String> files = _currentWorkspace.listContentsSync();
    files.forEach((String file) => mailbox.send(new Msg('ADD_UPDATE', file)));
  }

  void _getPath(Msg um) => _sendPath();

  void _sendWorkspaceNames(Msg um) {
    List<String> names = [];
    uproot.list()
    .where((Directory w) => Workspace.isWorkspace(w.path))
    .listen((Directory w) => names.add(w.path.split('/').last))
    .onDone(() => mailbox.send(new Msg('WORKSPACE_NAMES', JSON.encode(names))));
  }

  void _newWorkspace(Msg um) {
    String data = um.body;
    Workspace newWorkspace = new Workspace('${uproot.path}/$data');
    newWorkspace.create().then((Workspace workspace) {
      workspace.initSync();
      _setCurrentWorkspace(data);
    });
  }

  void _setWorkspace(Msg um) => _setCurrentWorkspace(um.body);

  void _fsNewFile(Msg um) {
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

  void _fsNewFolder(Msg um) {
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

  void _fsRename(Msg um) {
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
      if (isDir) mailbox.send(new Msg('REMOVE_UPDATE', 'D:$oldPath'));
    });
  }

  void _fsDelete(Msg um) {
    String path = um.body;
    FileSystemEntity entity;
    bool isDir = FileSystemEntity.isDirectorySync(path);

    entity = isDir ? new Directory(path) : new File(path);
    entity.delete(recursive: true);

    // Force a remove update on the top level folder as
    // watcher issue workaround.
    if (isDir) mailbox.send(new Msg('REMOVE_UPDATE', 'D:$path'));
  }

  void _openFile(Msg um) {
    int destinationId = -1;
    Msg newMessage = um;

    // Need to tell a specific UpDroidEditor to open the path.
    if (!um.body.startsWith('/')) {
      List<String> split = um.body.split(':');
      destinationId = int.parse(split[0]);
      newMessage = new Msg(um.header, split[1]);
    }

    CmdrPostOffice.send(new ServerMessage('UpDroidEditor', destinationId, newMessage));
  }

  void _workspaceClean(Msg um) {
    _currentWorkspace.clean().then((result) {
      mailbox.send(new Msg('WORKSPACE_CLEAN', ''));
    });
  }

  void _buildWorkspace(Msg um) {
    _currentWorkspace.buildWorkspace().then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      mailbox.send(new Msg('WORKSPACE_BUILD', resultString);
      mailbox.send(new Msg('BUILD_COMPLETE', JSON.encode([_currentWorkspace.path])));
    });
  }

  void _buildPackage(Msg um) {
    String packagePath = um.body;
    String packageName = packagePath.split('/').last;
    _currentWorkspace.buildPackage(packageName).then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      mailbox.send(new Msg('PACKAGE_BUILD_RESULTS', resultString);
      mailbox.send(new Msg('BUILD_COMPLETE', JSON.encode([packagePath])));
    });
  }

  void _buildPackages(Msg um) {
    String data = um.body;
    List<String> packagePaths = JSON.decode(data);

    List<String> packageNames = [];
    packagePaths.forEach((String packagePath) => packageNames.add(packagePath.split('/').last));

    _currentWorkspace.buildPackages(packageNames).then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      help.debug(resultString, 0);
//      mailbox.send(new Msg('PACKAGE_BUILD_RESULTS', resultString);
      mailbox.send(new Msg('BUILD_COMPLETE', data));
    });
  }

  void _createPackage(Msg um) {
    List<String> split = um.body.split(':');
    String name = split[0];

    List<String> dependencies = JSON.decode(split[1]);

    // A workspace that hasn't been built yet will cause problems.
    _currentWorkspace.createPackage(name, dependencies).then((ProcessResult result) {
      String stderr = result.stderr;
      if (stderr.contains('devel/setup.bash: No such file or directory')) {
        mailbox.send(new Msg('CREATE_PACKAGE_FAILED', stderr));
      }
    });
  }

  void _launcherList(Msg um) {
    _currentWorkspace.listLaunchers().listen((Map launcher) {
      String data = JSON.encode(launcher);
      mailbox.send(new Msg('LAUNCH', data));
    });
  }

  void _runLauncher(Msg um) {
    String data = um.body;
    List decodedData = JSON.decode(data);
    String packageName = decodedData[0];
    String nodeName = decodedData[1];
    List nodeArgs = decodedData.sublist(2);

    _currentWorkspace.runNode(packageName, nodeName, nodeArgs);
  }

  void _requestEditorList(Msg um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', 0, um));
  }

  void _returnSelected(Msg um) {
    List<String> split = um.body.split(':');
    int editorId = int.parse(split[0]);
    String selectedList = split[1];

    Msg newMessage = new Msg(um.header, selectedList);
    CmdrPostOffice.send(new ServerMessage('UpDroidEditor', editorId, newMessage));
  }

  void _sendEditorList(Msg um) => mailbox.send(um);
  void _getSelected(Msg um) => mailbox.send(um);

  /// Convenience method for adding a formatted filesystem update to the socket
  /// stream.
  ///   ex. add /home/user/tmp => [[ADD]]/home/user/tmp
  Future _formattedFsUpdate(WatchEvent e) async {
    List<String> split = e.toString().split(' ');
    String header = split[0].toUpperCase();
    String path = split[1];

    bool isFile = await FileSystemEntity.isFile(path);
    String fileString = isFile ? 'F:${path}' : 'D:${path}';

    Msg formatted = new Msg('${header}_UPDATE', fileString);
    help.debug('Outgoing: ' + formatted.toString(), 0);
    if (header != 'MODIFY') mailbox.send(formatted);
  }

  void _setCurrentWorkspace(String newWorkspaceName) {
    Msg um = new Msg('SET_CURRENT_WORKSPACE', newWorkspaceName);
    CmdrPostOffice.send(new ServerMessage('UpDroidEditor', 0, um));

    if (_currentWatcherStream != null) _currentWatcherStream.cancel();

    _currentWorkspace = new Workspace('${uproot.path}/$newWorkspaceName');
    _currentWatcher = new DirectoryWatcher(_currentWorkspace.src.path);
    _currentWatcherStream = _currentWatcher.events.listen((e) => _formattedFsUpdate(e));
    _sendPath();
  }

  void _sendPath() {
    mailbox.send(new Msg('EXPLORER_DIRECTORY_PATH', _currentWorkspace.path));
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, id);
    _currentWatcherStream.cancel();
  }
}