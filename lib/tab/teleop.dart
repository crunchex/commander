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

  StreamController<UpDroidMessage> _serverStream;
  Process _shell;

  CmdrTeleop(this.id, String workspacePath, StreamController<UpDroidMessage> serverStream) {
    help.debug('Spawning UpDroidTeleop ($id)', 0);

    _serverStream = serverStream;

    mailbox = new CmdrMailbox(guiName);
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
    _serverStream.add(um);
  }

  void _handleGamepadInput(HttpRequest request) {
    mailbox.ws.where((e) => request.uri.path == '/${guiName.toLowerCase()}/$id/controller/0')
    .listen((String s) {
      _shell.stdin.add(UTF8.encode(s));
    });
  }

  void cleanup() {
    _shell.kill();
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);

    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/controller/0', _handleGamepadInput);
  }
}