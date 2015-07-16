library tab_interface;

import 'dart:async';
import 'dart:io';
import 'dart:isolate';

import 'tab/api/tab.dart';
import 'tab/pty.dart';
import 'tab/camera/camera.dart';
import 'tab/teleop.dart';
import 'tab/editor.dart';
import 'console_mailbox.dart';
import 'tab/api/updroid_message.dart';

class TabInterface {
  String tabType;
  int id;
  Directory dir;
  List extra;

  Isolate tab;
  ConsoleMailbox mailbox;
  Stream<String> input;
  Stream<String> output;

  StreamController<String> _inputController;
  StreamController<String> _outputController;

  TabInterface(this.tabType, this.id, this.dir, [this.extra]) {
    _inputController = new StreamController<String>();
    input = _inputController.stream;

    _outputController = new StreamController<String>();
    output = _outputController.stream;

    mailbox = new ConsoleMailbox(tabType, id);
    _spawnTab();
  }

  Future _spawnTab() async {
    SendPort initialSendPort = mailbox.receivePort.sendPort;
    Isolate tab = await Isolate.spawn(CmdrPty.main, initialSendPort);

    await for (var received in mailbox.receivePort) {
      if (mailbox.sendPort == null) {
        mailbox.sendPort = received;

        // Send the args.
        mailbox.sendPort.send([id, dir.path, extra[0], extra[1]]);
        // Send a test message to try and get the message handler in the isolate called.
        mailbox.sendPort.send(new Msg('TEST').toString());

        continue;
      }

      print('_spawnTab received: $received');
    }
  }

  void close() {
//    tab.cleanup();
  }
}