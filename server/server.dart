#!/usr/bin/env dart

import 'dart:io';
import 'dart:async';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';
import 'package:args/args.dart';
import 'package:watcher/watcher.dart';

import 'terminal_commands.dart';

final Logger log = new Logger('server');

/// Recursively traverses the given directory path and asynchronously
/// returns a list of filesystem entities.
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

/// Container class that extracts the header (denoted with double brackets)
/// and body from the raw text of a [WebSocket] message.
class CommanderMessage {
  final String s;
  
  CommanderMessage(this.s);
  
  String header() {
    var header = new RegExp(r'^\[\[[A-Z_]+\]\]').firstMatch(s)[0];
    return header.replaceAll(new RegExp(r'\[\[|\]\]'), '');
  }
  
  String body() => s.replaceFirst(new RegExp(r'^\[\[[A-Z_]+\]\]'), '');
}

/// Handler for the [WebSocket]. Performs various actions depending on requests
/// it receives or local events that it detects.
void handleWebSocket(WebSocket socket, Directory dir) {
  log.info('Client connected!');
  
  socket.listen((String s) {
    CommanderMessage cm = new CommanderMessage(s);
    var switchConditional = cm.header();
    switch (switchConditional) {
      case 'EXPLORER_DIRECTORY_PATH':
        socket.add('[[EXPLORER_DIRECTORY_PATH]]' + dir.path);
        
        // Since it is assumed DirectoryPath is only requested on open,
        // also send the initial directory list.
        getDirContents(dir).then((files) {
          socket.add('[[EXPLORER_DIRECTORY_LIST]]' + files.toString());
        });
        break;
        
      case 'EXPLORER_DELETE':
        var path = cm.body();
              
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
        var path = cm.body();

        var fileToOpen = new File(path);
        fileToOpen.readAsString().then((String contents) {
          socket.add('[[EDITOR_FILE_TEXT]]' + contents);
        });
        break;
        
      case 'EDITOR_SAVE':
        // List[0] = data, List[1] = path.
        List<String> data = cm.body().split('[[PATH]]');

        var fileToSave = new File(data[1]);
        fileToSave.writeAsString(data[0]);
        break;
        
      case 'CONSOLE_COMMAND':
        // It's a Console command.
        log.info('Client sent: $s');
        List args = parseCommandInput(s);
        Process.run(args[0], args[1]).then((ProcessResult results) {
          socket.add('[[CONSOLE_COMMAND]]' + results.stdout);
        });
        break;
        
      default:
        log.severe('Message received without commander header');
    }
  }, onDone: () {
    log.info('Client disconnected');  
  });

  var watcher = new DirectoryWatcher(dir.path);
  watcher.events.listen((WatchEvent e) => getDirContents(dir).then((files) {
    socket.add('[[EXPLORER_DIRECTORY_LIST]]' + files.toString());
  }));
}

void main(List<String> args) {
  var dir = Directory.current;
  Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
  
  // Allow the user to specify what directory to use for the Commander's
  // server-side filesystem.
  var parser = new ArgParser();
  parser.addOption('directory', abbr: 'd', defaultsTo: Directory.current.toString(), callback: (directory) {
    dir = new Directory(directory);
  });
  var results = parser.parse(args);
  
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