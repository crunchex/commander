library console_mailbox;

import 'dart:io';
import 'dart:async';
import 'dart:isolate';
import 'dart:convert';

import 'package:upcom-api/tab_backend.dart';

import 'server_helper.dart' as help;
import 'post_office.dart';

class ConsoleMailbox {
  String className;
  int id;
  WebSocket ws;
  Stream<Msg> inbox;
  ReceivePort receivePort;
  SendPort sendPort;
  Set endpointRegistry;

  ConsoleMailbox(this.className, this.id) {
    endpointRegistry = new Set<String>();
    receivePort = new ReceivePort();

    inbox = CmdrPostOffice.registerClass(className, id);

    inbox.listen((Msg um) {
      help.debug('[${className}\'s Mailbox] UpDroid Message received with header: ${um.header}', 0);
      sendPort.send(um.toString());
    });
  }

  void send(Msg m) => ws.add(m.toString());

  /// Use this function for special cases where raw data needs to be sent
  /// over Websocket, such as in the case of array buffered/binary data.
  void sendFromEndpoint(String s) => ws.add(JSON.decode(s));

  void receive(WebSocket ws, HttpRequest request) {
    this.ws = ws;

    if (endpointRegistry.contains(request.uri.path)) {
      help.debug('Sending endpoint connection confirmation to: ${request.uri.path}', 0);
      Msg um = new Msg(request.uri.path, '');
      sendPort.send(um.toString());
    }

    ws.listen((String s) {
      if (request.uri.pathSegments.length == 2 && request.uri.pathSegments.first == className) {
        Msg um = new Msg.fromString(s);
        help.debug('$className incoming: ' + um.header, 0);

        sendPort.send(um.toString());
      } else if (endpointRegistry.contains(request.uri.path)) {
        Msg um = new Msg(request.uri.path, s);
        sendPort.send(um.toString());
      } else {
        help.debug('Cannot deliver message to Isolate: ${request.uri.path}', 0);
      }
    });
  }

  void relay(ServerMessage sm) => CmdrPostOffice.send(sm);

  void registerEndpoint(String raw) {
    int endIndex = raw.indexOf(':/c');
    String serverHeader = raw.substring(2, endIndex);

    List<String> split = serverHeader.split(':');
    String header = split[0];

    if (header == 'REG_ENDPOINT') {
      String endpoint = raw.substring(endIndex + 4, raw.length);
      help.debug('Registering endpoint: $endpoint', 0);
      endpointRegistry.add(endpoint);
    }
  }
}