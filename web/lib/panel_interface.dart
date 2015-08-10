library panel_interface;

import 'dart:async';
import 'dart:convert';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';

class PanelInterface {
  int id, col;
  PanelViewInterface view;
  Map panelInfo;
  String refName;
  Future setupComplete;

  Mailbox _mailbox;
  ScriptElement _panelJs;

  PanelInterface(this.id, this.col, this.panelInfo, Mailbox mailbox) {
    refName = panelInfo['refName'];
    _mailbox = mailbox;
    setupComplete = _setUp();
  }

  bool isActive() => view.isActive();
  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();
  void shutdownScript() => _panelJs.remove();

  Future _setUp() async {
    await _initiatePanelSetup();
    await _sendIdEvent();

    // Set up an interface to the new Panel's view.
    view = new PanelViewInterface(id, col, refName);

    return null;
  }

  Future _initiatePanelSetup() {
    // Wait for the Panel's frontend to be ready to receive the ID event.
    EventStreamProvider<CustomEvent> panelReadyStream = new EventStreamProvider<CustomEvent>('PanelReadyForId');

    // Launch the Panel's backend.
    _mailbox.ws.send('[[OPEN_PANEL]]' + '$col:$id:$refName');

    // Call the Panel's frontend (as a JS lib).
    _panelJs = new ScriptElement()
      ..id = '$refName-$id-script'
      ..type = 'text/javascript'
      ..src = 'panels/$refName/index.dart.js';

    document.body.children.add(_panelJs);

    return panelReadyStream.forTarget(window).where((CustomEvent e) => e.detail == refName).first;
  }

  Future _sendIdEvent() {
    // Wait for the Panel's frontend to be ready to receive the ID event.
    EventStreamProvider<CustomEvent> panelDoneStream = new EventStreamProvider<CustomEvent>('PanelSetupComplete');

    // Dispatch a custom event to pass ID info to the Panel's frontend.
    String detail = JSON.encode({ 'id': id, 'col': col, 'refName': refName });
    CustomEvent event = new CustomEvent('PanelIdEvent', canBubble: false, cancelable: false, detail: detail);
    window.dispatchEvent(event);

    return panelDoneStream.forTarget(window).where((e) => e.detail == refName).first;
  }
}

class PanelViewInterface {
  int id, col;
  String refName;
  DivElement tabHandle, tabContainer, tabContent;

  PanelViewInterface(this.id, this.col, this.refName) {
    tabHandle = querySelector('#tab-$refName-$id-handle');
    tabContainer = querySelector('#tab-$refName-$id-container');
    tabContent = querySelector('#tab-$refName-$id-content');
  }

  bool isActive() => tabHandle.classes.contains('active');

  /// Adds the CSS classes to make a tab 'active'.
  void makeActive() {
    tabHandle.classes.add('active');
    tabContainer.classes.add('active');
  }

  /// Removes the CSS classes to make a tab 'inactive'.
  void makeInactive() {
    tabHandle.classes.remove('active');
    tabContainer.classes.remove('active');
  }
}