library tab_interface;

import 'dart:async';
import 'dart:io';
import 'dart:isolate';

import 'package:upcom-api/tab_backend.dart';
import 'package:path/path.dart';

import 'isolate_mailbox.dart';

class TabInterface {
  String refName;
  int id;
  Directory dir;
  List extra;

  Isolate tab;
  IsolateMailbox mailbox;

  TabInterface(String binPath, this.refName, this.id, this.dir, [this.extra]) {
    mailbox = new IsolateMailbox(refName, id);

    String tabPath = '$binPath/tabs/$refName';
    _spawnTab(tabPath, new Uri.file(normalize('$tabPath/main.dart')));
  }

  Future _spawnTab(String tabPath, Uri tabFile) async {
    SendPort initialSendPort = mailbox.receivePort.sendPort;

    // Prepare the args.
    List args = [tabPath, id, dir.path];
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
    mailbox.close();
  }
}