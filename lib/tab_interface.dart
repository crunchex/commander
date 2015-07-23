library tab_interface;

import 'dart:async';
import 'dart:io';
import 'dart:isolate';

import 'package:upcom-api/tab_backend.dart';

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
    Uri tabFile;
    switch (tabType) {
      case 'UpDroidConsole':
        tabFile = new Uri.file('/home/crunchex/work/upcom-console/bin/main.dart');
        break;
      case 'UpDroidEditor':
        tabFile = new Uri.file('/home/crunchex/work/upcom-editor/bin/main.dart');
        break;
      case 'UpDroidTeleop':
        tabFile = new Uri.file('/home/crunchex/work/upcom-teleop/bin/main.dart');
        break;
      case 'UpDroidCamera':
        tabFile = new Uri.file('/home/crunchex/work/upcom-camera/bin/main.dart');
        break;
    }

    _spawnTab(tabFile);
  }

  Future _spawnTab(Uri tabFile) async {
    SendPort initialSendPort = mailbox.receivePort.sendPort;

    // Prepare the args.
    List args = [id, dir.path];
    if (extra != null) args.addAll(extra);

    await Isolate.spawnUri(tabFile, args, initialSendPort);

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
        Msg msg = new Msg.fromString(received);
        if (mailbox.endpointRegistry.contains(msg.header)) {
          mailbox.sendFromEndpoint(msg.body);
        } else {
          mailbox.send(new Msg.fromString(received));
        }
      }
    }
  }

  void close() {
//    tab.cleanup();
  }
}