library tab;

import 'dart:async';
import 'dart:isolate';

import '../../server_mailbox.dart';
import '../../post_office.dart';
import 'updroid_message.dart';
import 'server_message.dart';
import 'tab_mailbox.dart';

abstract class Tab {
  int id;
  String guiName;

  TabMailbox mailbox;

  Tab(this.id, this.guiName, SendPort sendPort) {
    mailbox = new TabMailbox(sendPort);

    // Register Tab's event handlers.
    mailbox.registerMessageHandler('CLOSE_TAB', _closeTab);
    mailbox.registerMessageHandler('CLONE_TAB', _cloneTab);
    mailbox.registerMessageHandler('MOVE_TAB', _moveTab);

    // Register subclass' event handlers.
    registerMailbox();
  }

  void registerMailbox();
  void cleanup();

  void close() {
    cleanup();
    CmdrPostOffice.deregisterStream(guiName, id);
  }

  void _closeTab(String msg) {
    Msg m = new Msg('CLOSE_TAB', msg);
    mailbox.relay(new ServerMessage('UpDroidClient', -1, m));
  }

  void _cloneTab(String msg) {
    Msg m = new Msg('CLONE_TAB', msg);
    mailbox.relay(new ServerMessage('UpDroidClient', -1, m));
  }

  void _moveTab(String msg) {
    Msg m = new Msg('MOVE_TAB', msg);
    mailbox.relay(new ServerMessage('UpDroidClient', -1, m));
  }
}