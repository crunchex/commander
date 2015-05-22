library cmdr_editor;

import 'dart:io';

import 'package:watcher/watcher.dart';

import '../server_helper.dart' as help;

class CmdrEditor {
  static const String guiName = 'UpDroidEditor';

  int editorNum = 1;
  Directory _dir;
  DirectoryWatcher _watcher;

  CmdrEditor(Directory dir) {
    _dir = dir;
  }

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void handleWebSocket(WebSocket ws) {
    help.debug('Editor client connected.', 0);

    ws.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Editor incoming: ' + s, 0);

      switch (um.header) {
        case 'EDITOR_DIRECTORY_PATH':
          _sendPath(ws);
          break;

        case 'EDITOR_REQUEST_LIST':
          _sendEditorList(ws);
          break;

        case 'EDITOR_OPEN':
          _sendFileContents(ws, um.body);
          break;

        case 'EDITOR_SAVE':
          _saveFile(um.body);
          break;

        default:
          help.debug('Editor: message received without updroid header.', 1);
      }
    });
  }

  void _sendPath(WebSocket s) {
    help.formattedMessage(s, 'EDITOR_DIRECTORY_PATH', _dir.path);
  }

  void _sendFileContents(WebSocket ws, String path) {
    var fileToOpen = new File(path);
    fileToOpen.readAsString().then((String contents) {
      ws.add('[[EDITOR_FILE_TEXT]]' + path + '[[CONTENTS]]' + contents);
    });
  }

  void _sendEditorList(WebSocket ws) {
    help.getDirectory(_dir).then((files) {
      ws.add('[[PATH_LIST]]' + files.toString());
    });
  }

  void _saveFile(String args) {
    // List[0] = data, List[1] = path.
    List<String> argsList = args.split('[[PATH]]');

    var fileToSave = new File(argsList[1]);
    fileToSave.writeAsString(argsList[0]);
  }

  void cleanup() {

  }
}