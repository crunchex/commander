library cmdr_editor;

import 'dart:io';
import 'dart:convert';

import 'api/ros/ros.dart';
import '../server_mailbox.dart';
import '../server_helper.dart' as help;
import 'api/tab.dart';
import 'api/updroid_message.dart';
import '../post_office.dart';
import 'api/server_message.dart';

class CmdrEditor extends Tab {
  Directory uproot;
  CmdrMailbox mailbox;

  Workspace _currentWorkspace;

  CmdrEditor(int id, this.uproot) :
  super(id, 'UpDroidEditor') {

  }

  void registerMailbox() {
    mailbox.registerWebSocketEvent('SAVE_FILE', _saveFile);
    mailbox.registerWebSocketEvent('REQUEST_SELECTED', _requestSelected);

    mailbox.registerServerMessageHandler('OPEN_FILE', _openFile);
    mailbox.registerServerMessageHandler('SET_CURRENT_WORKSPACE', _setCurrentWorkspace);
    mailbox.registerServerMessageHandler('RETURN_SELECTED', _returnSelected);
  }

  void _openFile(Msg um) {
    var fileToOpen = new File(um.body);
    fileToOpen.readAsString().then((String contents) {
      mailbox.ws.add('[[OPEN_FILE]]' + um.body + '[[CONTENTS]]' + contents);
    });
  }

  void _saveFile(Msg um) {
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

  void _requestSelected(Msg um) {
    Msg newMessage = new Msg(um.header, id.toString());
    CmdrPostOffice.send(new ServerMessage('UpDroidExplorer', -1, newMessage));
  }

  void _setCurrentWorkspace(Msg um) {
    _currentWorkspace = new Workspace('${uproot.path}/${um.body}');
  }

  void _returnSelected(Msg um) {
    mailbox.ws.add('[[REQUEST_SELECTED]]' + um.body);
  }

  void cleanup() {

  }
}