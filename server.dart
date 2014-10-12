import 'dart:io';
import 'package:http_server/http_server.dart' show VirtualDirectory;

VirtualDirectory virDir;

void directoryHandler(dir, request) {
  var indexUri = new Uri.file(dir.path).resolve('index.html');
  virDir.serveFile(new File(indexUri.toFilePath()), request);
}

void handleWebSocket(WebSocket socket) {
  print('Client connected!');
  socket.listen((String s) {
    print('Client sent: $s');
    Process.run('$s', []).then((ProcessResult results) {
      socket.add(results.stdout);
    });
  }, onDone: () {
    print('Client disconnected');  
  });
}

void main() {
  virDir = new VirtualDirectory(Platform.script.resolve('client').toFilePath())
    ..allowDirectoryListing = true
    ..directoryHandler = directoryHandler;

  HttpServer.bind(InternetAddress.ANY_IP_V4, 8080).then((HttpServer server) {
    print("HttpServer listening on port:${server.port}...");
    server.listen((HttpRequest request) {
      if (WebSocketTransformer.isUpgradeRequest(request)) {
        print("Upgraded ${request.method} request for: ${request.uri.path}");
        WebSocketTransformer.upgrade(request).then(handleWebSocket);
      } else {
        print("Regular ${request.method} request for: ${request.uri.path}");
        virDir.serveRequest(request);
      }
    });
  });
}