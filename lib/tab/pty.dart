library cmdr_console;

import 'dart:async';
import 'dart:io';
import 'dart:convert';

import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrPty {
  static const String guiName = 'UpDroidConsole';

  int ptyNum = 1;
  CmdrMailbox mailbox;

  Process _shell;
  String _workspacePath;

  CmdrPty(this.ptyNum, String workspacePath, String numRows, String numCols) {
    help.debug('Spawning UpDroidPty ($ptyNum)', 0);

    _workspacePath = workspacePath;

    mailbox = new CmdrMailbox(guiName);
    _registerMailbox();
  }

  void _startPty(UpDroidMessage um) {
    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/updroid/cmdr-pty
    Process.start('cmdr-pty', ['-size', '${um.body}'], environment: {'TERM':'vt100'}, workingDirectory: _workspacePath).then((Process shell) {
      _shell = shell;

      Stream stdoutBroadcast = shell.stdout.asBroadcastStream();

      // Get the port returned by cmdr-pty and then close.
      StreamSubscription portListener;
      portListener = stdoutBroadcast.listen((data) {
        String dataString = UTF8.decode(data);
        if (dataString.contains('now listening on:  [::]:')) {
          String port = dataString.replaceFirst('now listening on:  [::]:', '');
          UpDroidMessage portMessage = new UpDroidMessage('PTY_READY', port);
          mailbox.ws.add(portMessage.s);
          portListener.cancel();
        }
      });

      // Log the rest of stdout/err for debug.
      stdoutBroadcast.listen((data) => help.debug('pty[$ptyNum] stdout: ${UTF8.decode(data)}', 0));
      shell.stderr.listen((data) => help.debug('pty[$ptyNum] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('cmdr-pty [$ptyNum]: run failed. Probably not installed', 1);
      return;
    });
  }

  void _resize(UpDroidMessage um) {
    _shell.stdin.writeln(um.body);
  }

  void cleanup() {
    _shell.kill();
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('START_PTY', _startPty);
    mailbox.registerWebSocketEvent('RESIZE', _resize);
  }
}