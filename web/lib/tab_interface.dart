library tab_interface;

import 'dart:async';
import 'dart:convert';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';

class TabInterface {
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

  Future _setUp() async {
    // Launch the Tab's backend.
    _mailbox.ws.send('[[OPEN_TAB]]' + '$col-$id-$fullName');

    // Call the Tab's frontend (as a JS lib).
    _tabJs = new ScriptElement();
    _tabJs.type = 'text/javascript';

    if (fullName == 'UpDroidEditor') {
      _tabJs.src = 'tabs/upcom-editor/index.dart.js';
    } else if (fullName == 'UpDroidCamera') {
      _tabJs.src = 'tabs/upcom-camera/index.dart.js';
    } else if (fullName == 'UpDroidTeleop') {
      _tabJs.src = 'tabs/upcom-teleop/index.dart.js';
    } else if (fullName == 'UpDroidConsole') {
      _tabJs.src = 'tabs/upcom-console/index.dart.js';
    }

    document.body.children.add(_tabJs);

    // Wait for the Tab's frontend to be ready to receive the ID event.
    EventStreamProvider<CustomEvent> tabReadyStream = new EventStreamProvider<CustomEvent>('TabReadyForId');
    await tabReadyStream.forTarget(window).first;

    // Dispatch a custom event to pass ID info to the Tab's frontend.
    String detail = JSON.encode({ 'id': id, 'col': col });
    CustomEvent event = new CustomEvent('TabIdEvent', canBubble: false, cancelable: false, detail: detail);
    window.dispatchEvent(event);

    // Set up an interface to the new Tab's view.
    view = new TabViewInterface(id, col, fullName);
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