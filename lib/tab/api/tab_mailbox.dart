library tab_mailbox;

import 'dart:io';
import 'dart:async';
import 'dart:isolate';

import 'updroid_message.dart';

/// Manages message passing for a tab.
class TabMailbox {
//  Stream receivePort;

  Map _registry;
  Map _endpointRegistry;
  StreamController receivePort;
  SendPort _sendPort;

  TabMailbox(SendPort sendPort) {
    receivePort = new StreamController();
//    receivePort = _receivePort.stream;

    _sendPort = sendPort;

    _registry = {};
    _endpointRegistry = {};

    receivePort.stream.transform(Msg.toMsg).listen((Msg m) {
      print('Received ${m.header}: "${m.body}"');
      _registry[m.header](m.body);
    });
  }

//  void set receivePort(Stream receivePort) => _receivePort = receivePort;

  /// Sends out a [Msg] through the [SendPort] associated with this [TabMailbox].
  void send(Msg m) => _sendPort.send(m.toString());

  /// Registers a [function] to be called when the Port receives a message that matches
  /// its associated header key.
  void registerMessageHandler(String header, function(String s)) {
    _registry[header] = function;
  }

  /// Registers a [function] to be called when a request is received at
  /// a given endpoint (excluding the main one used by [registerWebSocketEvent]).
  void registerEndpointHandler(String uri, function(String s)) {
    _endpointRegistry[uri] = function;
  }
}