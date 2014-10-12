import 'dart:io';
import 'package:http_server/http_server.dart' show VirtualDirectory;
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';

VirtualDirectory virDir;
final Logger log = new Logger('server');

void directoryHandler(dir, request) {
  var indexUri = new Uri.file(dir.path).resolve('index.html');
  virDir.serveFile(new File(indexUri.toFilePath()), request);
}

void handleWebSocket(WebSocket socket) {
  log.info('Client connected!');
  socket.listen((String s) {
    log.info('Client sent: $s');
    Process.run('$s', []).then((ProcessResult results) {
      socket.add(results.stdout);
    });
  }, onDone: () {
    log.info('Client disconnected');  
  });
}

void main() {
  Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
  
  virDir = new VirtualDirectory(Platform.script.resolve('client').toFilePath())
    ..allowDirectoryListing = true
    ..directoryHandler = directoryHandler;

  HttpServer.bind(InternetAddress.ANY_IP_V4, 8080).then((HttpServer server) {
    log.info("HttpServer listening on port:${server.port}...");
    server.listen((HttpRequest request) {
      if (WebSocketTransformer.isUpgradeRequest(request)) {
        log.info("Upgraded ${request.method} request for: ${request.uri.path}");
        WebSocketTransformer.upgrade(request).then(handleWebSocket);
      } else {
        log.info("Regular ${request.method} request for: ${request.uri.path}");
        virDir.serveRequest(request);
      }
    });
  });
}