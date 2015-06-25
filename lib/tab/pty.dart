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
  Socket _ptySocket;

  CmdrPty(this.ptyNum, String workspacePath, String numRows, String numCols, StreamController<ServerMessage> serverStream) {
    help.debug('Spawning UpDroidPty ($ptyNum)', 0);

    _workspacePath = workspacePath;

    mailbox = new CmdrMailbox(guiName, serverStream);
    _registerMailbox();
  }

  void _startPty(UpDroidMessage um) {
    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/updroid/cmdr-pty
    Process.start('cmdr-pty', ['-protocol', 'tcp', '-size', '${um.body}'], environment: {'TERM':'vt100'}, workingDirectory: _workspacePath).then((Process shell) {
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
            help.debug('CmdrPty-$ptyNum connected to: ${socket.remoteAddress.address}:${socket.remotePort}', 0);
            socket.listen((data) => mailbox.ws.add((data)));
          });
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

  void _handleIOStream(HttpRequest request) {
    mailbox.ws
      .where((e) => request.uri.path == '/${guiName.toLowerCase()}/$ptyNum/cmdr-pty')
      .listen((data) => _ptySocket.add(data));
  }

  void _resize(UpDroidMessage um) {
    _shell.stdin.writeln(um.body);
  }

  void cleanup() {
    if (_ptySocket != null) _ptySocket.destroy();
    if (_shell != null) _shell.kill();
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('START_PTY', _startPty);
    mailbox.registerWebSocketEvent('RESIZE', _resize);

    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$ptyNum/cmdr-pty', _handleIOStream);
  }
}