library server_helper;

import 'dart:io';
import 'dart:async';
import 'package:watcher/watcher.dart';
import 'package:logging/logging.dart';
import 'package:logging_handlers/server_logging_handlers.dart';

Logger log;
bool debugFlag;
String logFileDir = '/var/log/updroid';

/// Enables/disables debug logging for the server_helper library.
void enableDebug(bool b) {
  if (b) {
    log = new Logger('server');
    File logFile = new File('$logFileDir/cmdr.log');
    try {
      logFile.createSync(recursive:true);
    } on FileSystemException {
      print('Debug mode (-d) requires write access to $logFileDir.');
      print('Here\'s one way to enable:');
      print('  \$ sudo groupadd var-updroid');
      print('  \$ sudo usermod -a -G var-updroid ${Platform.environment['USER']}');
      print('  \$ sudo mkdir -p $logFileDir');
      print('  \$ sudo chown -R root:var-updroid $logFileDir');
      print('  \$ sudo chmod 2775 $logFileDir');
      print('Log out and back in (or restart session) for changes to take effect.');
      exit(2);
    }

    Logger.root.onRecord.listen(new SyncFileLoggingHandler(logFile.path));
    debugFlag = b;
  }
}

/// Wrapper for varying log/debug levels. [logstring] is the debug message.
/// [level] is an int 0-1 from least severe to most.
void debug(String logstring, int level) {
  if (!debugFlag) {
    return;
  }

  switch (level) {
    case 0:
      log.info(logstring);
      break;

    case 1:
      log.severe(logstring);
      break;

    default:
      log.severe('Debug level not specified - fix this!');
      log.severe(logstring);
  }
}

/// Convenience method for a formatted socket message.
void formattedMessage(WebSocket socket, String header, String body) {
  socket.add('[[$header]]$body');
}

/// Helper method to grab file name in case of spaces.
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
  debug('Outgoing: ' + formatted, 0);
  if(header != 'MODIFY') socket.add(formatted);
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
