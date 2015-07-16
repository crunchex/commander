library tab_mailbox;

import 'dart:io';
import 'dart:isolate';

import 'updroid_message.dart';

/// Manages message passing for a tab.
class TabMailbox {
  Map _registry;
  Map _endpointRegistry;
  ReceivePort _receivePort;
  SendPort _sendPort;

  TabMailbox(ReceivePort receivePort, SendPort sendPort) {
    _receivePort = receivePort;
    _sendPort = sendPort;

    _registry = {};
    _endpointRegistry = {};

    _receivePort.transform(Msg.toMsg).listen((Msg m) {
      print('Received ${m.header}: "${m.body}"');
      _registry[m.header](m.body);
    });
  }

  /// Sends out a [Msg] through the [SendPort] associated with this [TabMailbox].
  void send(Msg m) => _sendPort.send(m.toString());

  /// Registers a [function] to be called when the Port receives a message that matches
  /// its associated header key.
  void registerMessageHandler(String header, function(String s)) {
    _registry[header] = function;
  }

  /// Registers a [function] to be called when a request is received at
  /// a given endpoint (excluding the main one used by [registerWebSocketEvent]).
  void registerEndpointHandler(String uri, function(HttpRequest request)) {
    _endpointRegistry[uri] = function;
  }
}