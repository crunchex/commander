library server_helper;

import 'dart:io';
import 'dart:async';
import 'package:watcher/watcher.dart';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';

Logger log;
bool debugFlag;

void enableDebug(bool b) {
  if (b) {
    log = new Logger('server');
    Logger.root.onRecord.listen(new SyncFileLoggingHandler("server.log"));
    debugFlag = b;
  }
}


void debug(String logstring) {
  if (debugFlag) {
   log.info(logstring);
  }
}

/// Convenience method for a formatted socket message
void formattedMessage(WebSocket socket, String header, String body) {
  socket.add('[[$header]]$body');
}

/// Helper method to grab file name in case of spaces

String fNameGrabber(List<String> split){
  var fName = "";
  if(split.length > 2){
    for(var i = 1; i < split.length; i++){
      fName += split[i];
      if(i != (split.length - 1)){
        fName += " ";
      }
    }
  }
  else{
    fName = split[1];
  }
  return fName;
}

/// Convenience method for adding a formatted filesystem update to the socket
/// stream.
///   ex. add /home/user/tmp => [[ADD]]/home/user/tmp
void formattedFsUpdate(WebSocket socket, WatchEvent e) {
  var split = e.toString().split(' ');
  var header = split[0].toUpperCase();
  var formatted = '[[EXPLORER_$header]]' + fNameGrabber(split);
  socket.add(formatted);
  print(formatted);
  
  debug(formatted);  // Very helpful message for debugging explorer
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
class UpDroidMessage {
  final String s;
  
  UpDroidMessage(this.s);
  
  String get header => createHeader();
  String get body => createBody();
  
  String createHeader() {
    var header = new RegExp(r'^\[\[[A-Z_]+\]\]').firstMatch(s)[0];
    return header.replaceAll(new RegExp(r'\[\[|\]\]'), '');
  }

  String createBody() => s.replaceFirst(new RegExp(r'^\[\[[A-Z_]+\]\]'), '');
}
