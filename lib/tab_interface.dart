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

    SendPort sendPort;
    await for (var received in mailbox.receivePort) {
      if (sendPort == null) {
        print('_spawnTab got the port');
        sendPort = received;

        // Send the args.
        sendPort.send([id, dir.path, extra[0], extra[1]]);

        // Initiate a test with 10 outgoing messages.
        for (int i = 0; i < 10; i++) {
          sendPort.send('test $i');
        }

        sendPort.send('_spawnTab is done');

        continue;
      }

      print('_spawnTab received: $received');
    }
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

void _main(SendPort interfacesSendPort) async {
  print("_main() the Isolate has been spawned!");

  // Set up the isolate's port pair.
  ReceivePort isolatesReceivePort = new ReceivePort();
  interfacesSendPort.send(isolatesReceivePort.sendPort);

  List args;
  await for (var received in isolatesReceivePort) {
    if (args == null) {
      args = received;

      int id = received[0];
      String path = received[1];
      String idRows = received[2];
      String idCols = received[3];
      print('got args: ${id.toString()}, $path, $idRows, $idCols');

      continue;
    }

    print('received string: $received');
  }

//  new CmdrPty(id, path, idRows, idCols, isolatesReceivePort, interfacesSendPort);

  // Handle each message sent through its receive port.
//  await for (String msg in isolatesReceivePort) {
//    if (msg != '_spawnTab is done') {
//      print("_main() received: $msg");
//      continue;
//    }
//
//    interfacesSendPort.send('_main() thinks we\'re done');
//  }
}