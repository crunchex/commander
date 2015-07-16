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
    Isolate tab = await Isolate.spawn(_main, mailbox.sendPort);
    SendPort sendPort = await mailbox.receivePort.first;
    for (int i = 0; i < 10; i++) {
      String msg = await sendReceive(sendPort, "message sent from main()");
      print("Main received: $msg");
    }
    sendReceive(sendPort, "done");
  }

  Future sendReceive(SendPort port, String msg) {
    ReceivePort receivePort = new ReceivePort();
    port.send([msg, receivePort.sendPort]);
    return receivePort.first;
  }

  void close() {
//    tab.cleanup();
  }
}

void _main(SendPort sender) async {
//  String idRows = extra[0];
//  String idCols = extra[1];
//  new CmdrPty(id, dir.path, idRows, idCols);
  print("echo() the Isolate has been spawned!");
  ReceivePort receivePort = new ReceivePort();
  sender.send(receivePort.sendPort);
  await for (List data in receivePort) {
    String msg = data[0];
    SendPort replyTo = data[1];
    print("Isolate received: $msg");
    replyTo.send("Greetings from echo() the Isolate!");
  };
}