library updroid_message;

/// Container class that extracts the header (denoted with double brackets)
/// and body from the raw text of a formatted [WebSocket] message received
/// from the UpDroid client.
class Msg {
  String s;

  Msg(String header, String body) {
    s = '[[$header]]$body';
  }

  Msg.fromString(this.s);

  String get header => createHeader();
  String get body => createBody();

  String createHeader() {
    var header = new RegExp(r'^\[\[[A-Z_]+\]\]').firstMatch(s)[0];
    return header.replaceAll(new RegExp(r'\[\[|\]\]'), '');
  }

  String createBody() => s.replaceFirst(new RegExp(r'^\[\[[A-Z_]+\]\]'), '');

  String toString() => s;
}