library server_message;

import 'dart:async';

import 'updroid_message.dart';

class ServerMessage {
  String receiverClass;
  int id;
  Msg um;

  /// Constructs a new [ServerMessage] where [receiverClass] is a class type,
  /// such as 'UpDroidClient'. [id] can be -1 for the destination with the lowest
  /// registered id number, 0 for all destinations of type [receiverClass], or
  /// any positive integer for one specific destination.
  ServerMessage(this.receiverClass, this.id, this.um);

  ServerMessage.fromString(String raw) {
    int endIndex = raw.indexOf(':/s');
    String serverHeader = raw.substring(2, endIndex);

    List<String> split = serverHeader.split(':');
    receiverClass = split[0];
    id = int.parse(split[1]);
    um = new Msg.fromString(raw.substring(endIndex + 4, raw.length));
  }

  String toString() {
    return 's:$receiverClass:$id:/s:${um.toString()}';
  }

  /// Transformer to convert String messages into the ServerMessages.
  static StreamTransformer toServerMessage = new StreamTransformer.fromHandlers(handleData: (event, sink) {
    sink.add(new Msg.fromString(event.data));
  });

  /// Transformer to convert ServerMessages into Strings that could be sent over Websockets or ports.
  static StreamTransformer fromServerMessage = new StreamTransformer.fromHandlers(handleData: (event, sink) {
    sink.add(event.data.s);
  });
}