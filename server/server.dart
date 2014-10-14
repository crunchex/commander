import 'terminal_commands.dart';

import 'dart:io';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';
import 'package:args/args.dart';

final Logger log = new Logger('server');
var dir = Directory.current;

void handleWebSocket(WebSocket socket) {
  log.info('Client connected!');
  socket.listen((String s) {
    if (s == 'REQUEST FILESYSTEM UPDATES') {
      String contents = 'DIRECTORY_LIST:';

      // May want to use Directory.watch instead of having the user click Refresh
      var subscription = dir.list(recursive: true, followLinks: false).listen((FileSystemEntity entity) {
        var relativePath = entity.path.replaceFirst(new RegExp(dir.path), '');
        contents = contents + ' ' + relativePath;
      });
      subscription.onDone(() {
        socket.add(contents);
      });
      
    } else {
      log.info('Client sent: $s');
      List args = parseCommandInput(s);
      Process.run(args[0], args[1]).then((ProcessResult results) {
        socket.add(results.stdout);
      });
    }
  }, onDone: () {
    log.info('Client disconnected');  
  });
}

void main(List<String> args) {
  Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
  
  var parser = new ArgParser();
  parser.addOption('directory', abbr: 'd', defaultsTo: Directory.current.toString(), callback: (directory) {
    dir = new Directory(directory);
    print(directory);
  });
  var results = parser.parse(args);
  
  HttpServer.bind(InternetAddress.ANY_IP_V4, 8080).then((HttpServer server) {
    log.info("HttpServer listening on port:${server.port}...");
    server.listen((HttpRequest request) {
      // WebSocket requests are considered "upgraded" HTTP requests
      if (WebSocketTransformer.isUpgradeRequest(request)) {
        log.info("Upgraded ${request.method} request for: ${request.uri.path}");
        WebSocketTransformer.upgrade(request).then(handleWebSocket);
      } else {
        log.info("Regular ${request.method} request for: ${request.uri.path}");
        // TODO: serve regular HTTP requests such as GET pages, etc.
      }
    });
  });
}