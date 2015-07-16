library cmdr_console;

import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'dart:isolate';

import 'api/tab.dart';
import '../post_office.dart';
import 'api/updroid_message.dart';
import 'api/server_message.dart';
import '../server_helper.dart' as help;
import 'api/tab_mailbox.dart';

class CmdrPty {
  int id;
  String guiName;

  Process _shell;
  String _workspacePath;
  Socket _ptySocket;
  TabMailbox mailbox;

  CmdrPty(int id, String workspacePath, String numRows, String numCols, SendPort sp) {
    _workspacePath = workspacePath;
    guiName = 'UpDroidConsole';
    mailbox = new TabMailbox(sp);

    registerMailbox();
  }

  void registerMailbox() {
    mailbox.registerMessageHandler('START_PTY', _startPty);
//    mailbox.registerMessageHandler('RESIZE', _resizeRelay);
//    mailbox.registerMessageHandler('RESIZE', _resizeHandle);
    mailbox.registerMessageHandler('TEST', _test);

    mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/cmdr-pty', _handleIOStream);
  }

  void _test(String s) => print('test successful');

  void _startPty(String um) {
    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/updroid/cmdr-pty
    Process.start('cmdr-pty', ['-p', 'tcp', '-s', '${um}'], environment: {'TERM':'vt100'}, workingDirectory: _workspacePath).then((Process shell) {
      _shell = shell;

      Stream stdoutBroadcast = shell.stdout.asBroadcastStream();

      // Get the port returned by cmdr-pty and then close.
      StreamSubscription portListener;
      portListener = stdoutBroadcast.listen((data) {
        String dataString = UTF8.decode(data);
        if (dataString.contains('listening on port: ')) {
          String port = dataString.replaceFirst('listening on port: ', '');
//          Msg portMessage = new Msg('PTY_READY', '');
//          mailbox.ws.add(portMessage.toString());
//          portListener.cancel();

          Socket.connect('127.0.0.1', int.parse(port)).then((socket) {
            _ptySocket = socket;
            help.debug('CmdrPty-$id connected to: ${socket.remoteAddress.address}:${socket.remotePort}', 0);
            socket.listen((data) => mailbox.send(new Msg('DATA', data)));
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

  void _handleIOStream(String data) => _ptySocket.add(UTF8.encode(data));

//  void _resizeRelay(Msg um) {
//    CmdrPostOffice.send(new ServerMessage('UpDroidConsole', 0, um));
//  }

//  void _resizeHandle(Msg um) {
//    // Resize the shell.
//    List newSize = um.body.split('x');
//    int newRow = int.parse(newSize[0]);
//    int newCol = int.parse(newSize[1]) - 1;
//    _shell.stdin.writeln('${newRow}x${newCol}');
//
//    // Send the new size to all UpDroidConsoles (including this one) to be relayed
//    // back to their client side.
////    mailbox.ws.add(um.toString());
//  }

  void cleanup() {
    if (_ptySocket != null) _ptySocket.destroy();
    if (_shell != null) _shell.kill();
  }
}