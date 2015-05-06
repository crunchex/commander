part of updroid_server;

class CmdrExplorer {
  static const String guiName = 'UpDroidExplorer';

  Directory _dir;
  DirectoryWatcher _watcher;
  Workspace _workspace;
  int expNum;
  var expPath;

  //TODO: make asynchroneous
  CmdrExplorer(Directory dir, num) {
    _workspace = new Workspace(dir.path);

    for (var item in dir.listSync()) {
      if(pathLib.basename(item.path) == 'src') _dir = item;
      expNum = num;

      _watcher = new DirectoryWatcher(dir.path + '/src');
    }
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void handleWebSocket(WebSocket ws) {
    help.debug('Explorer client connected.', 0);

    ws.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
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

        default:
          help.debug('Explorer: message received without updroid header.', 1);
      }

    });
    _watcher.events.listen((e) => help.formattedFsUpdate(ws, e));
  }

  void _sendInitial(WebSocket s) {
    help.getDirectory(_dir).then((files) {
      s.add('[[INITIAL_DIRECTORY_LIST]]' + files.toString());
    });
  }

  void _sendDirectory(WebSocket s) {
    help.getDirectory(_dir).then((files) {
      s.add('[[EXPLORER_DIRECTORY_LIST]]' + files.toString());
    });
  }

  void _refreshDirectory(WebSocket s) {
    help.getDirectory(_dir).then((files) {
      s.add('[[EXPLORER_DIRECTORY_REFRESH]]' + files.toString());
    });
  }

  void _sendPath(WebSocket s) {
    help.formattedMessage(s, 'EXPLORER_DIRECTORY_PATH', _dir.path);
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

  void _fsRename(String rename) {
    List<String> renameList = rename.split(':divider:');

    if (!FileSystemEntity.isDirectorySync(renameList[0])) {
      var fileToRename = new File(renameList[0]);
      fileToRename.rename(renameList[1]);
    } else {
      var dirToRename = new Directory(renameList[0]);
      dirToRename.rename(renameList[1]);
    }
  }

  void _fsDelete(String path, WebSocket socket) {
    // Can't simply just create a FileSystemEntity and delete it, since
    // it is an abstract class. This is a dumb way to create the proper
    // entity class.
    try {
      var dirToDelete = new Directory(path);
      dirToDelete.delete(recursive:true).then((path){
      //  sendDirectory(socket, dir);
      });
    } catch (e) {
      var fileToDelete = new File(path);
      fileToDelete.delete();
    }
  }

  void _workspaceClean(WebSocket s) {
    _workspace.clean().then((result) {
      s.add('[[WORKSPACE_CLEAN]]');
    });
  }

  void _workspaceBuild(WebSocket s) {
    _workspace.build().then((result) {
      String resultString = result.exitCode == 0 ? '' : result.stderr;
      s.add('[[WORKSPACE_BUILD]]' + resultString);
    });
  }

  void _nodeList(WebSocket s) {
    Ros.nodeList(_workspace, s);
  }
}