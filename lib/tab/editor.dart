library cmdr_editor;

import 'dart:io';
import 'dart:async';
import 'dart:convert';

import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrEditor {
  static const String guiName = 'UpDroidEditor';

  int editorNum;
  CmdrMailbox mailbox;
  Directory _dir;

  CmdrEditor(Directory dir, this.editorNum) {
    help.debug('Spawning $guiName ($editorNum)', 0);

    _dir = dir;

    mailbox = new CmdrMailbox(guiName, editorNum);
    _registerMailbox();
  }

  void _sendPath(UpDroidMessage um) {
    help.formattedMessage(mailbox.ws, 'EDITOR_DIRECTORY_PATH', _dir.path);
  }

  void _sendFileContents(UpDroidMessage um) {
    var fileToOpen = new File(um.body);
    fileToOpen.readAsString().then((String contents) {
      mailbox.ws.add('[[EDITOR_FILE_TEXT]]' + um.body + '[[CONTENTS]]' + contents);
    });
  }

  void _sendEditorList(UpDroidMessage um) {
    help.getDirectory(_dir).then((files) {
      mailbox.ws.add('[[PATH_LIST]]' + files.toString());
    });
  }

  void _saveFile(UpDroidMessage um) {
    List args = JSON.decode(um.body);
    // args[0] = data, args[1] = path. args[2] = executable option

    var fileToSave = new File(args[1]);

    fileToSave.writeAsString(args[0]);

    if (args[2] == true) {
      Process.run("chmod", ["u+x", fileToSave.path]).then((result) {
        if (result.exitCode != 0) throw new Exception(result.stderr);
      });
    }
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, editorNum);
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('EDITOR_DIRECTORY_PATH', _sendPath);
    mailbox.registerWebSocketEvent('EDITOR_REQUEST_LIST', _sendEditorList);
    mailbox.registerWebSocketEvent('EDITOR_OPEN', _sendFileContents);
    mailbox.registerWebSocketEvent('EDITOR_SAVE', _saveFile);

    mailbox.registerServerMessageHandler('OPEN_FILE', _sendFileContents);
  }
}