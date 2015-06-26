library server_mailbox;

import 'dart:io';
import 'dart:async';

import 'server_helper.dart' as help;

CmdrPostOffice _postOffice;

/// Singleton class that facilitates intra-serverside communication.
class CmdrPostOffice {
  static CmdrPostOffice get postOffice {
    if (_postOffice == null) {
      _postOffice = new CmdrPostOffice();
    }

    return _postOffice;
  }

  static void send(ServerMessage sm) => postOffice.postOfficeStream.add(sm);

  static Stream registerClass(String receiverClass, int id) {
    // Set up the stream controller for the outgoing stream.
    if (!postOffice.outboxes.containsKey(receiverClass)) {
      postOffice.outboxes[receiverClass] = {};
    }

    postOffice.outboxes[receiverClass][id] = new StreamController<UpDroidMessage>();
    return postOffice.outboxes[receiverClass][id].stream;
  }

  static Future<bool> deregisterStream(String receiverClass, int id) {
    Completer c = new Completer();

    postOffice.outboxes[receiverClass][id].close().then((_) {
      postOffice.outboxes[receiverClass].remove(id);

      if (postOffice.outboxes[receiverClass].isEmpty) {
        postOffice.outboxes.remove(receiverClass);
      }

      c.complete(true);
    });

    return c.future;
  }

  StreamController<ServerMessage> postOfficeStream;
  Map<String, Map<int, StreamController<UpDroidMessage>>> outboxes = {};

  CmdrPostOffice() {
    postOfficeStream = new StreamController<ServerMessage>.broadcast();
    print('stream ready');
    postOfficeStream.stream.listen((ServerMessage sm) => _dispatch(sm));
  }

  void _dispatch(ServerMessage sm) {
    // TODO: set up some buffer or queue for currently undeliverable messages.
    if (!postOffice.outboxes.containsKey(sm.receiverClass) || !postOffice.outboxes[sm.receiverClass].containsKey(sm.id)) {
      help.debug('[CmdrPostOffice] Undeliverable message to ${sm.receiverClass}-${sm.id} with header ${sm.um.header}', 0);
      return;
    }

    // Dispatch message to registered receiver with lowest ID.
    if (sm.id < 0) {
      List<int> outboxIds = new List<int>.from(outboxes[sm.receiverClass].keys);
      outboxIds.sort();
      outboxes[sm.receiverClass][outboxIds.first].add(sm.um);
      return;
    }

    // Dispatch message to all registered receivers with matching class.
    if (sm.id == 0) {
      Map<int, StreamController<UpDroidMessage>> boxes = outboxes[sm.receiverClass];
      boxes.values.forEach((StreamController<UpDroidMessage> s) => s.add(sm.um));
      return;
    }

    // Dispatch message to registered receiver with matching class and specific ID.
    outboxes[sm.receiverClass][sm.id].add(sm.um);
  }
}

class CmdrMailbox {
  String className;
  int id;
  WebSocket ws;
  Stream<UpDroidMessage> inbox;

  Map _wsRegistry;
  List<Function> _wsCloseRegistry;
  Map _endpointRegistry;
  Map _serverStreamRegistry;

  CmdrMailbox(this.className, this.id) {
    _wsRegistry = {};
    _wsCloseRegistry = [];
    _endpointRegistry = {};
    _serverStreamRegistry = {};

    inbox = CmdrPostOffice.registerClass(className, id);

    inbox.listen((UpDroidMessage um) {
      help.debug('[${className}\'s Mailbox] UpDroid Message received with header: ${um.header}', 0);

      if (!_serverStreamRegistry.containsKey(um.header)) {
        help.debug('[${className}\'s Mailbox] handler for header: ${um.header} not found', 0);
        return;
      }

      _serverStreamRegistry[um.header](um);
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
  String receiverClass;
  int id;
  UpDroidMessage um;

  /// Constructs a new [ServerMessage] where [receiverClass] is a class type,
  /// such as 'UpDroidClient'. [id] can be -1 for the destination with the lowest
  /// registered id number, 0 for all destinations of type [receiverClass], or
  /// any positive integer for one specific destination.
  ServerMessage(this.receiverClass, this.id, this.um);
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