library tab_mailbox;

import 'dart:io';
import 'dart:async';
import 'dart:isolate';

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

/// A class that defines the message structure for tab communication.
class Msg {
  String header, body;

  Msg(this.header, this.body) {
    // TODO: validate the Msg.
  }

  Msg.fromString(String s) {
    int dividerIndex = s.indexOf(':');
    header = s.substring(0, dividerIndex);
    body = s.substring(dividerIndex + 1, s.length);
  }

  String toString() {
    return '$header:$body';
  }

  /// Transformer to convert serialized [WebSocket] messages into the UpDroidMessage.
  static StreamTransformer toMsg = new StreamTransformer.fromHandlers(handleData: (event, sink) {
    sink.add(new Msg.fromString(event.data));
  });

  /// Transformer to convert UpDroidMessages into serialized [WebSocket] messages.
  static StreamTransformer fromMsg = new StreamTransformer.fromHandlers(handleData: (event, sink) {
    sink.add(event.data.s);
  });
}