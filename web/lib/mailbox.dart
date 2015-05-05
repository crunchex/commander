library mailbox;

import 'dart:html';
import 'dart:async';
import 'updroid_message.dart';

enum EventType { ON_OPEN, ON_MESSAGE, ON_CLOSE }

/// A class to initialize its owning class' main [WebSocket] connection to the
/// server side. It also manages all incoming messages over [WebSocket] and
/// [CommanderMessage] stream.
class Mailbox {
  String _name;
  int _id;
  StreamController<CommanderMessage> _cs;
  WebSocket ws;

  Map _wsRegistry;
  Map _csRegistry;

  Mailbox(String name, int num, StreamController<CommanderMessage> cs) {
    _name = name;
    _id = num;
    _cs = cs;

    _wsRegistry = { EventType.ON_OPEN: [], EventType.ON_MESSAGE: {}, EventType.ON_CLOSE: [] };
    _csRegistry = {};

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.
    String url = window.location.host.split(':')[0];
    _initWebSocket(url);

    // Call the function registered to m.type.
    _cs.stream.where((m) => m.dest == _name.toUpperCase()).listen((CommanderMessage m) => _csRegistry[m.type](m));
  }

  /// Registers a [function] to be called on one of the [WebSocket] events.
  /// If registering for ON_MESSAGE, [msg] is required to know which function to call.
  void registerWebSocketEvent(EventType type, String msg, function(UpDroidMessage um)) {
    if (type == EventType.ON_MESSAGE) {
      _wsRegistry[type][msg] = function;
      return;
    }

    _wsRegistry[type].add(function);
  }

  /// Registers a [function] to be called on a received [CommanderMessage] with [msg]
  /// as the type.
  void registerCommanderEvent(String msg, function(CommanderMessage m)) {
    _csRegistry[msg] = function;
  }

  void _initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    ws = new WebSocket('ws://' + url + ':12060/${_name.toLowerCase()}/$_id');

    // Call all the functions registered to ON_OPEN.
    ws.onOpen.listen((e) => _wsRegistry[EventType.ON_OPEN].forEach((f(e)) => f(e)));

    // Call the function registered to ON_MESSAGE[um.header].
    ws.onMessage.transform(updroidTransformer).listen((um) => _wsRegistry[EventType.ON_MESSAGE][um.header](um));

    ws.onClose.listen((e) {
      _wsRegistry[EventType.ON_CLOSE].forEach((f(e)) => f(e));

      print('$_name-$_id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });

    ws.onError.listen((e) {
      print('$_name-$_id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }
}