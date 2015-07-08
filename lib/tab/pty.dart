library cmdr_console;

import 'dart:async';
import 'dart:io';
import 'dart:convert';

import '../server_mailbox.dart';
import '../server_helper.dart' as help;

class CmdrPty {
  static const String guiName = 'UpDroidConsole';

  int id;
  CmdrMailbox mailbox;

  Process _shell;
  String _workspacePath;
  Socket _ptySocket;

  CmdrPty(this.id, String workspacePath, String numRows, String numCols) {
    mailbox = new CmdrMailbox(guiName, id);
    _registerMailbox();

    _workspacePath = workspacePath;
  }

  void _startPty(UpDroidMessage um) {
    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/updroid/cmdr-pty
    Process.start('cmdr-pty', ['-p', 'tcp', '-s', '${um.body}'], environment: {'TERM':'vt100'}, workingDirectory: _workspacePath).then((Process shell) {
      _shell = shell;

      Stream stdoutBroadcast = shell.stdout.asBroadcastStream();

      // Get the port returned by cmdr-pty and then close.
      StreamSubscription portListener;
      portListener = stdoutBroadcast.listen((data) {
        String dataString = UTF8.decode(data);
        if (dataString.contains('listening on port: ')) {
          String port = dataString.replaceFirst('listening on port: ', '');
          UpDroidMessage portMessage = new UpDroidMessage('PTY_READY', '');
          mailbox.ws.add(portMessage.s);
          portListener.cancel();

          Socket.connect('127.0.0.1', int.parse(port)).then((socket) {
            _ptySocket = socket;
            help.debug('CmdrPty-$id connected to: ${socket.remoteAddress.address}:${socket.remotePort}', 0);
            socket.listen((data) => mailbox.ws.add((data)));
          });
        }
      });

      // Log the rest of stdout/err for debug.
      stdoutBroadcast.listen((data) => help.debug('pty[$id] stdout: ${UTF8.decode(data)}', 0));
      shell.stderr.listen((data) => help.debug('pty[$id] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('cmdr-pty [$id]: run failed. Probably not installed', 1);
      return;
    });
  }

  void _handleIOStream(HttpRequest request) {
    mailbox.ws
      .where((e) => request.uri.path == '/${guiName.toLowerCase()}/$id/cmdr-pty')
      .listen((data) => _ptySocket.add(data));
  }

  void _resizeRelay(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidConsole', 0, um));
  }

  void _closeTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _resizeHandle(UpDroidMessage um) {
    // Resize the shell.
    List newSize = um.body.split('x');
    int newRow = int.parse(newSize[0]);
    int newCol = int.parse(newSize[1]) - 1;
    _shell.stdin.writeln('${newRow}x${newCol}');

    // Send the new size to all UpDroidConsoles (including this one) to be relayed
    // back to their client side.
    mailbox.ws.add(um.toString());
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, id);
    if (_ptySocket != null) _ptySocket.destroy();
    if (_shell != null) _shell.kill();
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('START_PTY', _startPty);
    mailbox.registerWebSocketEvent('RESIZE', _resizeRelay);
    mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);

    mailbox.registerServerMessageHandler('RESIZE', _resizeHandle);

    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/cmdr-pty', _handleIOStream);
  }
}