library server_mailbox;

import 'dart:io';
import 'dart:async';

import 'package:upcom-api/tab_backend.dart';
import 'package:upcom-api/debug.dart';

import 'post_office.dart';

class CmdrMailbox {
  String refName;
  int id;
  WebSocket ws;
  Stream<Msg> inbox;

  Map _wsRegistry;
  List<Function> _wsCloseRegistry;
  Map _endpointRegistry;
  Map _serverStreamRegistry;

  CmdrMailbox(this.refName, this.id) {
    _wsRegistry = {};
    _wsCloseRegistry = [];
    _endpointRegistry = {};
    _serverStreamRegistry = {};

    inbox = CmdrPostOffice.registerClass(refName, id);

    inbox.listen((Msg um) {
      debug('[${refName}\'s Mailbox] UpDroid Message received with header: ${um.header}', 0);

      if (!_serverStreamRegistry.containsKey(um.header)) {
        debug('[${refName}\'s Mailbox] handler for header: ${um.header} not found', 0);
        return;
      }

      _serverStreamRegistry[um.header](um);
    });
  }

  void send(Msg m) => ws.add(m.toString());

  void handleWebSocket(WebSocket ws, HttpRequest request) {
    this.ws = ws;
    if (request.uri.pathSegments.length == 2 && request.uri.pathSegments.first == refName) {
      ws.listen((String s) {
        Msg um = new Msg.fromString(s);
        debug('$refName incoming: ' + um.header, 0);

        _wsRegistry[um.header](um);
      }).onDone(() => _wsCloseRegistry.forEach((f()) => f()));
    } else {
      _endpointRegistry[request.uri.path](request);
    }
  }

  /// Registers a [function] to be called on one of the main [WebSocket] requests.
  /// [msg] is required to know which function to call.
  void registerWebSocketEvent(String msg, function(Msg um)) {
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
  void registerServerMessageHandler(String msg, function(Msg um)) {
    _serverStreamRegistry[msg] = function;
  }
}