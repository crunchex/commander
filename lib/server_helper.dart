library server_helper;

import 'dart:io';
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
      print('Here\'s one way to enable (only need to do once):');
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
