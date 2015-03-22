part of updroid_server;

class CmdrExplorer {
  static const String guiName = 'UpDroidExplorer';

  int editorNum = 1;

  Directory _dir;
  DirectoryWatcher _watcher;

  CmdrExplorer(Directory dir, DirectoryWatcher watcher) {
    _dir = dir;
    _watcher = watcher;
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void handleWebSocket(WebSocket ws) {
    help.debug('Client connected!', 0);
    StreamController<String> processInput = new StreamController<String>.broadcast();

    ws.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Incoming message: ' + s, 0);

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

        default:
          help.debug('Editor: message received without updroid header.', 1);
      }

      _watcher.events.listen((e) => help.formattedFsUpdate(ws, e));
    });
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
}