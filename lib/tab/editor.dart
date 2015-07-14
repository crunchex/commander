library cmdr_editor;

import 'dart:io';
import 'dart:convert';

import '../ros/ros.dart';
import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrEditor {
  static const String guiName = 'UpDroidEditor';

  int id;
  Directory uproot;
  CmdrMailbox mailbox;

  Workspace _currentWorkspace;

  CmdrEditor(this.id, this.uproot) {
    mailbox = new CmdrMailbox(guiName, id);
    _registerMailbox();
  }

  void _openFile(UpDroidMessage um) {
    var fileToOpen = new File(um.body);
    fileToOpen.readAsString().then((String contents) {
      mailbox.ws.add('[[OPEN_FILE]]' + um.body + '[[CONTENTS]]' + contents);
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

  void _requestSelected(UpDroidMessage um) {
    UpDroidMessage newMessage = new UpDroidMessage(um.header, id.toString());
    CmdrPostOffice.send(new ServerMessage('UpDroidExplorer', -1, newMessage));
  }

  void _closeTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _cloneTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _moveTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _setCurrentWorkspace(UpDroidMessage um) {
    _currentWorkspace = new Workspace('${uproot.path}/${um.body}');
  }

  void _returnSelected(UpDroidMessage um) {
    mailbox.ws.add('[[REQUEST_SELECTED]]' + um.body);
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, id);
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('SAVE_FILE', _saveFile);
    mailbox.registerWebSocketEvent('REQUEST_SELECTED', _requestSelected);
    mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);
    mailbox.registerWebSocketEvent('CLONE_TAB', _cloneTab);
    mailbox.registerWebSocketEvent('MOVE_TAB', _moveTab);

    mailbox.registerServerMessageHandler('OPEN_FILE', _openFile);
    mailbox.registerServerMessageHandler('SET_CURRENT_WORKSPACE', _setCurrentWorkspace);
    mailbox.registerServerMessageHandler('RETURN_SELECTED', _returnSelected);
  }
}