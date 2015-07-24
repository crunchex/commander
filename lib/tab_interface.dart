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

  TabInterface(String binPath, String fsName, this.tabType, this.id, this.dir, [this.extra]) {
    mailbox = new ConsoleMailbox(tabType, id);

    _spawnTab(new Uri.file('$binPath/tabs/$fsName/main.dart'));
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