library mailbox;

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:isolate';

import 'package:upcom-api/tab_backend.dart';
import 'package:upcom-api/debug.dart';

part 'cmdr_mailbox.dart';
part 'isolate_mailbox.dart';
part 'post_office.dart';

abstract class Mailbox {
  String refName;
  int id;
  WebSocket ws;
  Stream<Msg> inbox;

  Mailbox(this.refName, this.id) {
    inbox = CmdrPostOffice.registerClass(refName, id);
  }

  void send(Msg m) => ws.add(m.toString());
}