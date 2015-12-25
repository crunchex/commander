part of mailbox;

class IsolateMailbox extends Mailbox {
  ReceivePort receivePort;
  SendPort sendPort;
  Set endpointRegistry;

  IsolateMailbox(String refName, int id) : super(refName, id) {
    endpointRegistry = new Set<String>();
    receivePort = new ReceivePort();

    inbox.listen((Msg um) {
      debug('[${refName}\'s Mailbox] UpDroid Message received with header: ${um.header}', 0);
      sendPort.send(um.toString());
    });
  }

  /// Use this function for special cases where raw data needs to be sent
  /// over Websocket, such as in the case of array buffered/binary data.
  void sendFromEndpoint(String s) => ws.add(JSON.decode(s));

  void receive(WebSocket websocket, HttpRequest request) {
    ws = websocket;

    if (endpointRegistry.contains(request.uri.path)) {
      debug('Sending endpoint connection confirmation to: ${request.uri.path}', 0);
      sendPort.send(new Msg(request.uri.path, '').toString());
    }

    ws.listen((String s) {
      if (request.uri.pathSegments.length == 2 && request.uri.pathSegments.first == refName) {
        Msg um = new Msg.fromString(s);
        debug('$refName incoming: ' + um.header, 0);
        sendPort.send(um.toString());
      } else if (endpointRegistry.contains(request.uri.path)) {
        sendPort.send(new Msg(request.uri.path, s).toString());
      } else {
        debug('Cannot deliver message to Isolate: ${request.uri.path}', 0);
      }
    });
  }

  void relay(ServerMessage sm) => CmdrPostOffice.send(sm);

  void registerEndpoint(String raw) {
    int endIndex = raw.indexOf(':/c');
    String serverHeader = raw.substring(2, endIndex);

    List<String> split = serverHeader.split(':');
    String header = split[0];

    if (header == 'REG_ENDPOINT') {
      String endpoint = raw.substring(endIndex + 4, raw.length);
      debug('Registering endpoint: $endpoint', 0);
      endpointRegistry.add(endpoint);
    }
  }

  Future close() => CmdrPostOffice.deregisterStream(refName, id);
}