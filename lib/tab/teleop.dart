library cmdr_teleop;

import 'dart:io';

import '../ros/ros.dart';
import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrTeleop {
  static const String guiName = 'UpDroidTeleop';

  int id;
  CmdrMailbox mailbox;

  Process _shell;

  CmdrTeleop(this.id, String workspacePath) {
    help.debug('Spawning UpDroidTeleop ($id)', 0);

    mailbox = new CmdrMailbox(guiName);
    _registerMailbox();

//    Workspace workspace = new Workspace(workspacePath);
//    Ros.runNode(workspace, 'updroid_teleop updroid_teleop.launch');
  }

  void _handleGamepadInput(HttpRequest request) {
    mailbox.ws.where((e) => request.uri.path == '/${guiName.toLowerCase()}/$id/controller/0')
    .listen((e) {
      print('controller data: ${e.data}');
    });
  }

  void cleanup() {

  }

  void _registerMailbox() {
    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/controller/0', _handleGamepadInput);
  }
}