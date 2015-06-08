library updroid_teleop;

import 'dart:html';
import 'dart:async';
import 'dart:js' as js;

import '../tab_controller.dart';

class UpDroidTeleop extends TabController {
  static String className = 'UpDroidTeleop';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Controllers', 'items': []}
    ];
    return menu;
  }

  UpDroidTeleop(int id, int col) :
  super(id, col, className, 'Teleop', getMenuConfig()) {

  }

  void setUpController() {
    view.content.contentEdge.height = new Dimension.percent(100);
    view.content.style.backgroundColor = '#107C10';
    // TODO: compress this svg (use that OS X tool).
    ImageElement image = new ImageElement(src: 'lib/tabs/teleop/xbox.svg')
      ..style.position = 'absolute'
      ..style.top = '50%'
      ..style.left = '50%'
      ..style.transform = 'translate(-50%, -160px)';
    view.content.children.add(image);

    for (int i = 0; i < 4; i++) {
      ParagraphElement axis = new ParagraphElement()
        ..id = '${className.toLowerCase()}-$id-axis-$i'
        ..style.position = 'absolute'
        ..style.color = '#ffffff'
        ..style.fontSize = '16px'
        ..style.top = '50%'
        ..style.left = '50%'
        ..style.transform = 'translate(-50%, -${i * 20}px)'
        ..text = 'Axis $i: [disconnected]';

      view.content.children.add(axis);
    }

    ParagraphElement buttons = new ParagraphElement()
      ..id = '${className.toLowerCase()}-$id-buttons'
      ..style.position = 'absolute'
      ..style.color = '#ffffff'
      ..style.fontSize = '18px'
      ..style.top = '50%'
      ..style.left = '50%'
      ..style.transform = 'translate(-50%, 30px)'
      ..text = '[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';

    view.content.children.add(buttons);

    new js.JsObject(js.context['startScanning'], [id]);

    new Timer.periodic(new Duration(milliseconds: 100), (_) {
      var updateStatus = new js.JsObject(js.context['updateStatus'], []);
    });

    //_setGamepads();
  }

  void _setGamepads() {
    Map deviceIds = js.context['controllers'];
    //deviceIds.sort((a, b) => a.compareTo(b));
    for (int i = 0; i < deviceIds.keys.length; i++) {
      view.config.last['items'].add({'type': 'toggle', 'title': 'Gamepad$i'});
    }
    view.refreshMenus();
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {

  }

  void registerEventHandlers() {

  }

  void cleanUp() {}
}