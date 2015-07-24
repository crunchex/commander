library tab_interface;

import 'dart:async';
import 'dart:convert';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';

class TabInterface {
  // TODO: all this hardcoded stuff should be pulled from a tab registry file somewhere.
  static Map classNameToFsName = {
    'UpDroidEditor': 'upcom-editor',
    'UpDroidConsole': 'upcom-console',
    'UpDroidCamera': 'upcom-camera',
    'UpDroidTeleop': 'upcom-teleop',
  };

  int id, col;
  TabViewInterface view;
  String fullName;
  Future setupComplete;

  Mailbox _mailbox;
  ScriptElement _tabJs;

  TabInterface(this.id, this.col, this.fullName, Mailbox mailbox) {
    _mailbox = mailbox;
    setupComplete = _setUp();
  }

  bool isActive() => view.isActive();
  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();
  void shutdownScript() => _tabJs.remove();

  Future _setUp() async {
    await _initiateTabSetup();
    await _sendIdEvent();

    // Set up an interface to the new Tab's view.
    view = new TabViewInterface(id, col, fullName);

    return null;
  }

  Future _initiateTabSetup() {
    // Wait for the Tab's frontend to be ready to receive the ID event.
    EventStreamProvider<CustomEvent> tabReadyStream = new EventStreamProvider<CustomEvent>('TabReadyForId');

    // Launch the Tab's backend.
    _mailbox.ws.send('[[OPEN_TAB]]' + '$col-$id-$fullName');

    // Call the Tab's frontend (as a JS lib).
    String name = fullName.toLowerCase().replaceAll(' ', '-');

    _tabJs = new ScriptElement()
    ..id = '$name-$id-script'
    ..type = 'text/javascript'
    ..src = 'tabs/${classNameToFsName[fullName]}/index.dart.js';

    document.body.children.add(_tabJs);

    return tabReadyStream.forTarget(window).where((CustomEvent e) => e.detail == fullName).first;
  }

  Future _sendIdEvent() {
    // Wait for the Tab's frontend to be ready to receive the ID event.
    EventStreamProvider<CustomEvent> tabDoneStream = new EventStreamProvider<CustomEvent>('TabSetupComplete');

    // Dispatch a custom event to pass ID info to the Tab's frontend.
    String detail = JSON.encode({ 'id': id, 'col': col, 'className': fullName });
    CustomEvent event = new CustomEvent('TabIdEvent', canBubble: false, cancelable: false, detail: detail);
    window.dispatchEvent(event);

    return tabDoneStream.forTarget(window).where((e) => e.detail == fullName).first;
  }
}

class TabViewInterface {
  int id, col;
  String fullName;
  DivElement tabHandle, tabContainer, tabContent;

  TabViewInterface(this.id, this.col, this.fullName) {
    tabHandle = querySelector('#tab-$name-$id-handle');
    tabContainer = querySelector('#tab-$name-$id-container');
    tabContent = querySelector('#tab-$name-$id-content');
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

  String get name => fullName.toLowerCase().replaceAll(' ', '-');
}