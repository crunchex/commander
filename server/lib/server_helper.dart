library server_helper;

import 'dart:io';
import 'dart:async';
import 'package:watcher/watcher.dart';

/// Convenience method for adding a formatted filesystem update to the socket
/// stream.
///   ex. add /home/user/tmp => [[ADD]]/home/user/tmp
void formattedFsUpdate(WebSocket socket, WatchEvent e) {
  var split = e.toString().split(' ');
  var header = split[0].toUpperCase();
  var formatted = '[[$header]]' + split[1];
  print(formatted);
  socket.add(formatted);
}

/// Recursively traverses the given directory path and asynchronously
/// returns a list of filesystem entities.
Future<List<FileSystemEntity>> getDirectory(Directory dir) {
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
/// and body from the raw text of a formatted [WebSocket] message received
/// from the UpDroid client.
class CommanderMessage {
  final String s;
  
  CommanderMessage(this.s);
  
  String get header => createHeader();
  String get body => createBody();
  
  String createHeader() {
    var header = new RegExp(r'^\[\[[A-Z_]+\]\]').firstMatch(s)[0];
    return header.replaceAll(new RegExp(r'\[\[|\]\]'), '');
  }

  String createBody() => s.replaceFirst(new RegExp(r'^\[\[[A-Z_]+\]\]'), '');
}