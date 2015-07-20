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
    _spawnTab();
  }

  Future _spawnTab() async {
    SendPort initialSendPort = mailbox.receivePort.sendPort;
//    Isolate tab = await Isolate.spawn(CmdrPty.main, initialSendPort);
    await Isolate.spawnUri(new Uri.file('/home/crunchex/work/upcom-console/lib/pty.dart'), [], initialSendPort, packageRoot: new Uri.file('/home/crunchex/work/upcom-console/packages/'));

    await for (var received in mailbox.receivePort) {
      if (mailbox.sendPort == null) {
        mailbox.sendPort = received;

        // Send the args.
        mailbox.sendPort.send([id, dir.path, extra[0], extra[1]]);
        // Send a test message to try and get the message handler in the isolate called.
//        mailbox.sendPort.send(new Msg('START_PTY').toString());

        continue;
      }

      // At this point we are assuming messages are always strings.
      String message = received;
      if (message.startsWith('s:')) {
        mailbox.relay(new ServerMessage.fromString(message));
      } else {
        mailbox.send(new Msg.fromString(received));
      }
    }
  }

  void close() {
//    tab.cleanup();
  }
}