#!/usr/bin/env dart

import 'dart:io';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';
import 'package:args/args.dart';
import 'package:watcher/watcher.dart';

import 'lib/server_helper.dart' as help;
import 'lib/client_responses.dart';

Logger log;
DirectoryWatcher watcher;

/// Handler for the [WebSocket]. Performs various actions depending on requests
/// it receives or local events that it detects.
void handleWebSocket(WebSocket socket, Directory dir) {
  log.info('Client connected!');
  
  socket.listen((String s) {
    help.UpDroidMessage um = new help.UpDroidMessage(s);
    
    switch (um.header) {
      case 'EXPLORER_DIRECTORY_PATH':
        sendPath(socket, dir);
        break;
        
      case 'EXPLORER_DIRECTORY_LIST':
        sendDirectory(socket, dir);
        break;
        
      case 'EXPLORER_NEW_FILE':
        fsNewFile(um.body);
        break;
        
      case 'EXPLORER_NEW_FOLDER':
        fsNewFolder(um.body);
        // Empty folders don't trigger an incremental update, so we need to
        // refresh the entire workspace.
        sendDirectory(socket, dir);
        break;

      case 'EXPLORER_RENAME':
        fsRename(um.body);
        break;

      case 'EXPLORER_MOVE':
        // Currently implemented in the same way as RENAME as there is no
        // direct API for MOVE.
        fsRename(um.body);
        break;

      case 'EXPLORER_DELETE':
        fsDelete(um.body);
        break;
        
      case 'EDITOR_OPEN':
        sendFileContents(socket, um.body);
        break;
        
      case 'EDITOR_SAVE':
        saveFile(um.body);
        break;
        
      case 'CONSOLE_COMMAND':
        log.info('Client sent: $s');
        processCommand(socket, um.body);
        break;
        
      default:
        log.severe('Message received without updroid header.');
    }
  }, onDone: () {
    log.info('Client disconnected');  
  });

  watcher.events.listen((e) => help.formattedFsUpdate(socket, e));
}

void main(List<String> args) {
  // Set default dir as current working directory.
  Directory dir = Directory.current;
  
  // Set up logging.
  log = new Logger('server');
  Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
  
  // Create an args parser to override the workspace directory if one is supplied.
  var parser = new ArgParser();
  parser.addOption('directory', abbr: 'd', defaultsTo: Directory.current.toString(), callback: (directory) {
    dir = new Directory(directory);
  });
  parser.parse(args);

  // Initialize the DirectoryWatcher.
  watcher = new DirectoryWatcher(dir.path);
  
  // Set up an HTTP webserver and listen for standard page requests or upgraded
  // [WebSocket] requests.
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