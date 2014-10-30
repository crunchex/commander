#!/usr/bin/env dart

import 'dart:io';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';
import 'package:args/args.dart';
import 'package:watcher/watcher.dart';

import 'lib/terminal_commands.dart';
import 'lib/server_helper.dart' as help;

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
        socket.add('[[EXPLORER_DIRECTORY_PATH]]' + dir.path);
        
        // Since it is assumed DirectoryPath is only requested on open,
        // also send the initial directory list.
        help.getDirectory(dir).then((files) {
          socket.add('[[EXPLORER_DIRECTORY_LIST]]' + files.toString());
        });
        break;
        
      case 'EXPLORER_RENAME':
        List<String> renameArgs = um.body.split(' ');
        
        if (!FileSystemEntity.isDirectorySync(renameArgs[0])) {
          var fileToRename = new File(renameArgs[0]);
          fileToRename.rename(renameArgs[1]);
        } else {
          var dirToRename = new Directory(renameArgs[0]);
          dirToRename.rename(renameArgs[1]);
        }
        break;
      
      // Currently implemented in the same way as RENAME as there is no
      // direct API for MOVE.
      case 'EXPLORER_MOVE':
        List<String> renameArgs = um.body.split(' ');
        
        if (!FileSystemEntity.isDirectorySync(renameArgs[0])) {
          var fileToRename = new File(renameArgs[0]);
          fileToRename.rename(renameArgs[1]);
        } else {
          var dirToRename = new Directory(renameArgs[0]);
          dirToRename.rename(renameArgs[1]);
        }
        break;

      case 'EXPLORER_DELETE':
        var path = um.body;
              
        // Can't simply just create a FileSystemEntity and delete it, since
        // it is an abstract class. This is a dumb way to create the proper
        // entity class.
        try {
          var dirToDelete = new Directory(path);
          dirToDelete.delete(recursive:true);
        } catch (e) {
          var fileToDelete = new File(path);
          fileToDelete.delete();
        }
        break;
        
      case 'EDITOR_OPEN':
        var path = um.body;

        var fileToOpen = new File(path);
        fileToOpen.readAsString().then((String contents) {
          socket.add('[[EDITOR_FILE_TEXT]]' + contents);
        });
        break;
        
      case 'EDITOR_SAVE':
        // List[0] = data, List[1] = path.
        List<String> data = um.body.split('[[PATH]]');

        var fileToSave = new File(data[1]);
        fileToSave.writeAsString(data[0]);
        break;
        
      case 'CONSOLE_COMMAND':
        log.info('Client sent: $s');
        List args = parseCommandInput(um.body);
        Process.run(args[0], args[1]).then((ProcessResult results) {
          socket.add('[[CONSOLE_COMMAND]]' + results.stdout);
        });
        break;
        
      default:
        log.severe('Message received without updroid header');
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