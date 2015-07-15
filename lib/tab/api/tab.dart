library tab;

import 'dart:async';

import '../../server_mailbox.dart';
import '../../post_office.dart';
import 'updroid_message.dart';
import 'server_message.dart';

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

  void close() {
    cleanup();
    CmdrPostOffice.deregisterStream(guiName, id);
  }

  void _closeTab(Msg um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _cloneTab(Msg um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _moveTab(Msg um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }
}