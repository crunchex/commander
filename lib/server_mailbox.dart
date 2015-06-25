library server_mailbox;

import 'dart:io';
import 'dart:async';

import 'server_helper.dart' as help;

class CmdrMailbox {
  String className;
  WebSocket ws;
  StreamController<ServerMessage> serverStream;

  Map _wsRegistry;
  List<Function> _wsCloseRegistry;
  Map _endpointRegistry;
  Map _serverStreamRegistry;

  CmdrMailbox(this.className,  this.serverStream) {
    _wsRegistry = {};
    _wsCloseRegistry = [];
    _endpointRegistry = {};
    _serverStreamRegistry = {};

    serverStream.stream.where((ServerMessage sm) => sm.receiver == className).listen((ServerMessage sm) {
      help.debug('[${className}\'s Server Mailbox] UpDroid Message received with header: ${sm.um.header}', 0);
      _serverStreamRegistry[sm.um.header](sm.um);
    });
  }

  void handleWebSocket(WebSocket ws, HttpRequest request) {
    this.ws = ws;
    if (request.uri.pathSegments.length == 2 && request.uri.pathSegments.first == className.toLowerCase()) {
      ws.listen((String s) {
        UpDroidMessage um = new UpDroidMessage.fromString(s);
        help.debug('$className incoming: ' + um.header, 0);

        _wsRegistry[um.header](um);
      }).onDone(() => _wsCloseRegistry.forEach((f()) => f()));
    } else {
      _endpointRegistry[request.uri.path](request);
    }
  }

  /// Registers a [function] to be called on one of the main [WebSocket] requests.
  /// [msg] is required to know which function to call.
  void registerWebSocketEvent(String msg, function(UpDroidMessage um)) {
    _wsRegistry[msg] = function;
  }

  /// Registers a [function] to be called at the end of the [WebSocket] request - onDone().
  /// The method will be executed in no particular order.
  void registerWebSocketCloseEvent(function()) {
    _wsCloseRegistry.add(function);
  }

  /// Registers a [function] to be called when a request is received at
  /// a given endpoint (excluding the main one used by [registerWebSocketEvent]).
  void registerEndpointHandler(String uri, function(HttpRequest request)) {
    _endpointRegistry[uri] = function;
  }

  /// Registers a [function] to be called on a received [UpDroidMessage] with [msg]
  /// as the header.
  void registerServerMessageHandler(String msg, function(UpDroidMessage um)) {
    _serverStreamRegistry[msg] = function;
  }
}

class ServerMessage {
  String receiver;
  UpDroidMessage um;

  ServerMessage(this.receiver, this.um);
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