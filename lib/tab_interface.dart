library tab_interface;

import 'dart:async';
import 'dart:io';
import 'dart:isolate';

import 'package:upcom-api/server_message.dart';
import 'package:upcom-api/updroid_message.dart';

//import 'tab/api/tab.dart';
//import 'tab/camera/camera.dart';
//import 'tab/teleop.dart';
//import 'tab/editor.dart';
import 'console_mailbox.dart';

class TabInterface {
  String tabType;
  int id;
  Directory dir;
  List extra;

  Isolate tab;
  ConsoleMailbox mailbox;

  TabInterface(this.tabType, this.id, this.dir, [this.extra]) {
    mailbox = new ConsoleMailbox(tabType, id);

    // TODO: all this hardcoded stuff should be pulled from a tab registry file somewhere.
    Uri tabFile, packageRoot;
    switch (tabType) {
      case 'UpDroidConsole':
        tabFile = new Uri.file('/home/crunchex/work/upcom-console/lib/pty.dart');
        packageRoot = new Uri.file('/home/crunchex/work/upcom-console/packages/');
        break;
      case 'UpDroidEditor':
        tabFile = new Uri.file('/home/crunchex/work/upcom-editor/lib/editor.dart');
        packageRoot = new Uri.file('/home/crunchex/work/upcom-editor/packages/');
        break;
      case 'UpDroidTeleop':
        tabFile = new Uri.file('/home/crunchex/work/upcom-teleop/lib/teleop.dart');
        packageRoot = new Uri.file('/home/crunchex/work/upcom-teleop/packages/');
        break;
    }

    _spawnTab(tabFile, packageRoot);
  }

  Future _spawnTab(Uri tabFile, Uri packageRoot) async {
    SendPort initialSendPort = mailbox.receivePort.sendPort;

    // Prepare the args.
    List args = [id, dir.path];
    if (extra != null) args.addAll(extra);

    await Isolate.spawnUri(tabFile, args, initialSendPort, packageRoot: packageRoot);

    await for (var received in mailbox.receivePort) {
      if (mailbox.sendPort == null) {
        mailbox.sendPort = received;

        continue;
      }

      // At this point we are assuming messages are always strings.
      String message = received;
      if (message.startsWith('s:')) {
        mailbox.relay(new ServerMessage.fromString(message));
      } else if (message.startsWith('c:')) {
        mailbox.registerEndpoint(message);
      } else {
        mailbox.send(new Msg.fromString(received));
      }
    }
  }

  void close() {
//    tab.cleanup();
  }
}