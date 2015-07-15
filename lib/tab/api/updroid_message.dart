library updroid_message;

import 'dart:async';

/// A class that defines the message structure for tab communication.
class Msg {
  String header, body;

  Msg(this.header, this.body);

  /// Returns a new instance of [Msg], given a formatted String.
  /// Throws an error if not in the format: [[HEADER]]body
  Msg.fromString(String s) {
    if (!s.contains('[[') || !s.contains(']]')) throw new MalformedMsgError(s);

    String minusFirstBrackets = s.replaceFirst('[[', '');
    int indexOfSecondBrackets = minusFirstBrackets.indexOf(']]');

    header = minusFirstBrackets.substring(0, indexOfSecondBrackets);
    body = minusFirstBrackets.substring(indexOfSecondBrackets + 2, minusFirstBrackets.length);
  }

  String toString() {
    return '[[$header]]$body';
  }

  /// Transformer to convert String messages into the Msg.
  static StreamTransformer toMsg = new StreamTransformer.fromHandlers(handleData: (event, sink) {
    sink.add(new Msg.fromString(event.data));
  });

  /// Transformer to convert Msg into Strings that could be sent over Websockets or ports.
  static StreamTransformer fromMsg = new StreamTransformer.fromHandlers(handleData: (event, sink) {
    sink.add(event.data.s);
  });
}

class MalformedMsgError extends StateError {
  MalformedMsgError(String msg) : super('Wrong format for Msg: $msg');
}