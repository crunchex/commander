library console_mailbox;

import 'dart:io';
import 'dart:async';
import 'dart:isolate';

import 'server_helper.dart' as help;
import 'tab/api/updroid_message.dart';
import 'post_office.dart';

class ConsoleMailbox {
  String className;
  int id;
  WebSocket ws;
  Stream<Msg> inbox;
  ReceivePort receivePort;
  SendPort sendPort;

  ConsoleMailbox(this.className, this.id) {
    receivePort = new ReceivePort();

    inbox = CmdrPostOffice.registerClass(className, id);

    inbox.listen((Msg um) {
      help.debug('[${className}\'s Mailbox] UpDroid Message received with header: ${um.header}', 0);
      sendPort.send(um.toString());
    });
  }

  void send(Msg m) => ws.add(m.toString());

  void receive(WebSocket ws, HttpRequest request) {
    this.ws = ws;
    if (request.uri.pathSegments.length == 2 && request.uri.pathSegments.first == className.toLowerCase()) {
      ws.listen((String s) {
        Msg um = new Msg.fromString(s);
        help.debug('$className incoming: ' + um.header, 0);

        sendPort.send(um.toString());
      });
    } else {
      ws.where((e) => request.uri.path == '/${className}/$id/cmdr-pty')
      .listen((data) => sendPort.send('${request.uri.path}:um.'));
    }
  }
}