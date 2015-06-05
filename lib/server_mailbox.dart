library server_mailbox;

import 'dart:io';

import 'server_helper.dart' as help;

class CmdrMailbox {
  String className;
  WebSocket ws;

  Map _wsRegistry;
  Map _endpointRegistry;

  CmdrMailbox(String className) {
    this.className = className;
    _wsRegistry = {};
    _endpointRegistry = {};
  }

  void handleWebSocket(WebSocket ws, HttpRequest request) {
    print(request.uri.path);
    this.ws = ws;
    if (request.uri.pathSegments.length == 2 && request.uri.pathSegments.first == className.toLowerCase()) {
      ws.listen((String s) {
        UpDroidMessage um = new UpDroidMessage.fromString(s);
        help.debug('$className incoming: ' + um.header, 0);

        _wsRegistry[um.header](um);
      });
    } else {
      _endpointRegistry[request.uri.path](request);
    }
  }

  /// Registers a [function] to be called on one of the main [WebSocket] requests.
  /// [msg] is required to know which function to call.
  void registerWebSocketEvent(String msg, function(UpDroidMessage um)) {
    _wsRegistry[msg] = function;
  }

  /// Registers a [function] to be called when a request is received at
  /// a given endpoint (excluding the main one used by [registerWebSocketEvent]).
  void registerEndpointHandler(String uri, function(HttpRequest request)) {
    _endpointRegistry[uri] = function;
  }
}

/// Container class that extracts the header (denoted with double brackets)
/// and body from the raw text of a formatted [WebSocket] message received
/// from the UpDroid client.
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