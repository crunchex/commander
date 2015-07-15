library tab_mailbox;

import 'dart:io';
import 'dart:async';
import 'dart:isolate';

import 'updroid_message.dart';

/// Manages message passing for a tab.
class TabMailbox {
  Map _registry;
  ReceivePort _receivePort;
  SendPort _sendPort;

  TabMailbox(SendPort replyTo) {
    _registry = {};

    _receivePort.transform(Msg.toMsg).listen((Msg m) {
      print('Received ${m.header}: "${m.body}"');
      _registry[m.header](m.body);
    });
  }

  /// Registers a [function] to be called when the Port receives a message that matches
  /// its associated header key.
  void registerMessageHandler(String header, function(String s)) {
    _registry[header] = function;
  }

  /// Sends out a [Msg] through the [SendPort] associated with this [TabMailbox].
  void send(Msg m) => _sendPort.send(m.toString());
}