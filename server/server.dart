import 'terminal_commands.dart';

import 'dart:io';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';

final Logger log = new Logger('server');

void handleWebSocket(WebSocket socket) {
  log.info('Client connected!');
  socket.listen((String s) {
    log.info('Client sent: $s');
    List args = parseCommandInput(s);
    Process.run(args[0], args[1]).then((ProcessResult results) {
      socket.add(results.stdout);
    });
  }, onDone: () {
    log.info('Client disconnected');  
  });
}

void main() {
  Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
  
  HttpServer.bind(InternetAddress.ANY_IP_V4, 8080).then((HttpServer server) {
    log.info("HttpServer listening on port:${server.port}...");
    server.listen((HttpRequest request) {
      if (WebSocketTransformer.isUpgradeRequest(request)) {
        log.info("Upgraded ${request.method} request for: ${request.uri.path}");
        WebSocketTransformer.upgrade(request).then(handleWebSocket);
      } else {
        log.info("Regular ${request.method} request for: ${request.uri.path}");
      }
    });
  });
}