library cmdr_teleop;

import 'dart:io';
import 'dart:convert';

import 'api/ros/ros.dart';
import 'api/tab.dart';

class CmdrTeleop extends Tab {
  Process _shell;

  CmdrTeleop(int id, Directory dir) :
  super(id, 'UpDroidTeleop') {
    Workspace workspace = new Workspace(dir.path);
//    Ros.runNode(workspace, 'ros_arduino_python joy_cmdr.launch');

    Process.start('bash', ['-c', '. ${workspace.path}/catkin_ws/devel/setup.bash && roslaunch ros_arduino_python joy_cmdr.launch'], runInShell: true).then((process) {
      _shell = process;
      stdout.addStream(process.stdout);
      stderr.addStream(process.stderr);
    });
  }

  void registerMailbox() {
    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/controller/0', _handleGamepadInput);
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
}