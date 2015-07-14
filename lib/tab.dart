library tab;

import 'dart:async';

import 'server_mailbox.dart';
import 'server_helper.dart' as help;

abstract class Tab {
  int id;
  String guiName;

  CmdrMailbox mailbox;

  Tab(this.id, this.guiName) {
    mailbox = new CmdrMailbox(guiName, id);

    // Register Tab's event handlers.
    mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);
    mailbox.registerWebSocketEvent('CLONE_TAB', _cloneTab);
    mailbox.registerWebSocketEvent('MOVE_TAB', _moveTab);

    // Register subclass' event handlers.
    registerMailbox();
  }

  void registerMailbox();
  void cleanup();

  Future close() {
    cleanup();
    CmdrPostOffice.deregisterStream(guiName, id);
  }

  void _closeTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _cloneTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _moveTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }
}