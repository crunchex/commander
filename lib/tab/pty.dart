part of updroid_server;

class CmdrPty {
  static const String guiName = 'UpDroidConsole';

  int ptyNum = 1;

  Process _shell;

  CmdrPty(this.ptyNum, String workspacePath, String numRows, String numCols) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidPty ($ptyNum)', 0);

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

  /// Handler for the [WebSocket]. Performs various actions depending on requests
  /// it receives or local events that it detects.
  void handleWebSocket(WebSocket ws) {
    help.debug('Console client connected.', 0);

    ws.listen((String s) {
      help.UpDroidMessage um = new help.UpDroidMessage(s);
      help.debug('Console incoming: ' + um.header, 0);

      switch (um.header) {
        case 'RESIZE':
          _resize(um.body);
          break;

        default:
          help.debug('Console: message received without updroid header.', 1);
      }
    });
  }

  void _resize(String newSize) {
    _shell.stdin.writeln(newSize);
  }

  void cleanup() {
    _shell.kill();
  }
}