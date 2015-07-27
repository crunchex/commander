library post_office;

import 'dart:io';
import 'dart:async';

import 'package:upcom-api/tab_backend.dart';
import 'server_helper.dart' as help;

CmdrPostOffice _postOffice;

/// Singleton class that facilitates intra-serverside communication.
class CmdrPostOffice {
  static CmdrPostOffice get postOffice {
    if (_postOffice == null) {
      _postOffice = new CmdrPostOffice();
    }

    return _postOffice;
  }

  static void send(ServerMessage sm) => postOffice.postOfficeStream.add(sm);

  static Stream registerClass(String receiverClass, int id) {
    help.debug('[CmdrPostOffice] Registering $receiverClass-$id', 0);

    // Set up the stream controller for the outgoing stream.
    if (!postOffice.outboxes.containsKey(receiverClass)) {
      postOffice.outboxes[receiverClass] = {};
    }

    postOffice.outboxes[receiverClass][id] = new StreamController<Msg>();
    return postOffice.outboxes[receiverClass][id].stream;
  }

  static Future<bool> deregisterStream(String receiverClass, int id) {
    Completer c = new Completer();

    help.debug('[CmdrPostOffice] De-registering $receiverClass-$id', 0);

    if (!postOffice.outboxes.containsKey(receiverClass) || !postOffice.outboxes[receiverClass].containsKey(id)) {
      c.complete(false);
    }

    postOffice.outboxes[receiverClass][id].close().then((_) {
      postOffice.outboxes[receiverClass].remove(id);

      if (postOffice.outboxes[receiverClass].isEmpty) {
        postOffice.outboxes.remove(receiverClass);
      }

      c.complete(true);
    });

    return c.future;
  }

  StreamController<ServerMessage> postOfficeStream;
  Map<String, Map<int, StreamController<Msg>>> outboxes = {};

  CmdrPostOffice() {
    postOfficeStream = new StreamController<ServerMessage>.broadcast();
    postOfficeStream.stream.listen((ServerMessage sm) => _dispatch(sm));
  }

  void _dispatch(ServerMessage sm) {
    // TODO: set up some buffer or queue for currently undeliverable messages.
    if (!postOffice.outboxes.containsKey(sm.receiverClass)) {
      help.debug('[CmdrPostOffice] Undeliverable message to ${sm.receiverClass}-${sm.receiverId} with header ${sm.um.header}', 0);
      return;
    }

    // Dispatch message to registered receiver with lowest ID.
    if (sm.receiverId < 0) {
      List<int> outboxIds = new List<int>.from(outboxes[sm.receiverClass].keys);
      outboxIds.sort();
      outboxes[sm.receiverClass][outboxIds.first].add(sm.um);
      return;
    }

    // Dispatch message to all registered receivers with matching class.
    if (sm.receiverId == 0) {
      Map<int, StreamController<Msg>> boxes = outboxes[sm.receiverClass];
      boxes.values.forEach((StreamController<Msg> s) => s.add(sm.um));
      return;
    }

    // Dispatch message to registered receiver with matching class and specific ID.
    // TODO: set up some buffer or queue for currently undeliverable messages.
    if (!postOffice.outboxes[sm.receiverClass].containsKey(sm.receiverId)) {
      help.debug('[CmdrPostOffice] Undeliverable message to ${sm.receiverClass}-${sm.receiverId} with header ${sm.um.header}', 0);
      return;
    }
    outboxes[sm.receiverClass][sm.receiverId].add(sm.um);
  }
}

