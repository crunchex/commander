library cmdr_teleop;

import 'dart:async';
import 'dart:io';
import 'dart:convert';

import '../ros/ros.dart';
import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrTeleop {
  static const String guiName = 'UpDroidTeleop';

  int id;
  CmdrMailbox mailbox;

  Process _shell;

  CmdrTeleop(this.id, String workspacePath) {
    mailbox = new CmdrMailbox(guiName, id);
    _registerMailbox();

    Workspace workspace = new Workspace(workspacePath);
//    Ros.runNode(workspace, 'ros_arduino_python joy_cmdr.launch');

    Process.start('bash', ['-c', '. ${workspace.path}/catkin_ws/devel/setup.bash && roslaunch ros_arduino_python joy_cmdr.launch'], runInShell: true).then((process) {
      _shell = process;
      stdout.addStream(process.stdout);
      stderr.addStream(process.stderr);
    });
  }

  void _closeTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _handleGamepadInput(HttpRequest request) {
    mailbox.ws.where((e) => request.uri.path == '/${guiName.toLowerCase()}/$id/controller/0')
    .listen((String s) {
      _shell.stdin.add(UTF8.encode(s));
    });
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, id);
    _shell.kill();
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);

    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/controller/0', _handleGamepadInput);
  }
}