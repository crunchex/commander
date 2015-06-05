part of mailbox;

/// Container class that extracts the header (denoted with double brackets)
/// and body from the raw text of a formatted [WebSocket] message received
/// from the UpDroid server.
class UpDroidMessage {
  String s;

  UpDroidMessage(String header, String body) {
    s = '[[$header]]$body';
  }

  UpDroidMessage.fromString(this.s);

  String get header => createHeader();
  String get body => createBody();

  String createHeader() {
    var header = new RegExp(r'^\[\[[A-Z_]+\]\]').firstMatch(s)[0];
    return header.replaceAll(new RegExp(r'\[\[|\]\]'), '');
  }

  String createBody() => s.replaceFirst(new RegExp(r'^\[\[[A-Z_]+\]\]'), '');
}

/// Transformer to convert serialized [WebSocket] messages into the UpDroidMessage.
StreamTransformer toUpDroidMessage = new StreamTransformer.fromHandlers(handleData: (event, sink) {
  sink.add(new UpDroidMessage.fromString(event.data));
});

/// Transformer to convert UpDroidMessages into serialized [WebSocket] messages.
StreamTransformer fromUpDroidMessage = new StreamTransformer.fromHandlers(handleData: (event, sink) {
  sink.add(event.data.s);
});

/// A class for the intra-client message passing.
///   dest: the class that the message is meant for
///   type: the type of message (e.g. 'command')
///   body: the body of the message
class CommanderMessage {
  String dest, type;
  var body;
  CommanderMessage(this.dest, this.type, {this.body});
}