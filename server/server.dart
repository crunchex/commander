#!/usr/bin/env dart

import 'dart:io';
import 'dart:async';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';
import 'package:args/args.dart';
import 'package:watcher/watcher.dart';

import 'terminal_commands.dart';

final Logger log = new Logger('server');

Future<List<FileSystemEntity>> getDirContents(Directory dir) {
  var files = <FileSystemEntity>[];
  var completer = new Completer();
  var lister = dir.list(recursive: true);
  lister.listen ( 
      (file) => files.add(file),
      // Should also register onError.
      onDone:   () => completer.complete(files)
      );
  return completer.future;
}

bool directoryPathRequest(String s) {
  RegExp exp = new RegExp('REQUEST_DIRECTORY_PATH');
  return exp.hasMatch(s);
}

bool deleteRequest(String s) {
  RegExp exp = new RegExp('REQUEST_DELETE');
  return exp.hasMatch(s);
}

bool openRequest(String s) {
  RegExp exp = new RegExp('REQUEST_FILE_TEXT');
  return exp.hasMatch(s);
}

void handleWebSocket(WebSocket socket, Directory dir) {
  log.info('Client connected!');
  
  socket.listen((String s) {
    if (directoryPathRequest(s)) {
      log.info('Request for DirectoryPath received');
      socket.add('RESPONSE_DIRECTORY_PATH' + dir.path);
      
      // Since it is assumed DirectoryPath is only requested on open,
      // Also send the initial directory list.
      getDirContents(dir).then((files) {
          socket.add('RESPONSE_DIRECTORY_LIST' + files.toString());
        });
    } else if (deleteRequest(s)) {
      var path = s.replaceFirst('REQUEST_DELETE', '');
      
      // Can't just create a FileSystemEntity and delete it.
      try {
        var dirToDelete = new Directory(path);
        dirToDelete.delete(recursive:true);
      } catch (e) {
        var fileToDelete = new File(path);
        fileToDelete.delete();
      }
    } else if (openRequest(s)) {
      var path = s.replaceFirst('REQUEST_FILE_TEXT', '');

      var fileToOpen = new File(path);
      fileToOpen.readAsString().then((String contents) {
        socket.add('RESPONSE_FILE_TEXT' + contents);
      });
    } else {
      // It's a Console command.
      log.info('Client sent: $s');
      List args = parseCommandInput(s);
      Process.run(args[0], args[1]).then((ProcessResult results) {
        socket.add(results.stdout);
      });
    }
  }, onDone: () {
    log.info('Client disconnected');  
  });

  var watcher = new DirectoryWatcher(dir.path);
  watcher.events.listen((WatchEvent e) => getDirContents(dir).then((files) {
    socket.add('RESPONSE_DIRECTORY_LIST' + files.toString());
  }));
}

void main(List<String> args) {
  var dir = Directory.current;
  Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
  
  var parser = new ArgParser();
  parser.addOption('directory', abbr: 'd', defaultsTo: Directory.current.toString(), callback: (directory) {
    dir = new Directory(directory);
  });
  var results = parser.parse(args);
  
  HttpServer.bind(InternetAddress.ANY_IP_V4, 8080).then((HttpServer server) {
    log.info("HttpServer listening on port:${server.port}...");
    server.listen((HttpRequest request) {
      // WebSocket requests are considered "upgraded" HTTP requests.
      if (WebSocketTransformer.isUpgradeRequest(request)) {
        log.info("Upgraded ${request.method} request for: ${request.uri.path}");
        WebSocketTransformer.upgrade(request).then((WebSocket ws) {
          handleWebSocket(ws, dir);
        });
      } else {
        log.info("Regular ${request.method} request for: ${request.uri.path}");
        // TODO: serve regular HTTP requests such as GET pages, etc.
      }
    });
  });
}