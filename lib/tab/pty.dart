library cmdr_console;

import 'dart:io';
import 'dart:convert';

import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrPty {
  static const String guiName = 'UpDroidConsole';

  int ptyNum = 1;
  CmdrMailbox mailbox;

  Process _shell;

  CmdrPty(this.ptyNum, String workspacePath, String numRows, String numCols) {
    help.debug('Spawning UpDroidPty ($ptyNum)', 0);

    mailbox = new CmdrMailbox(guiName);
    _registerMailbox();

    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/updroid/cmdr-pty
    Process.start('cmdr-pty', ['-addr', ':1206$ptyNum', '-size', '${numRows}x${numCols}'], environment: {'TERM':'vt100'}, workingDirectory: workspacePath).then((Process shell) {
      _shell = shell;
      shell.stdout.listen((data) => help.debug('pty[$ptyNum] stdout: ${UTF8.decode(data)}', 0));
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
    mailbox.registerWebSocketEvent('RESIZE', _resize);
  }
}