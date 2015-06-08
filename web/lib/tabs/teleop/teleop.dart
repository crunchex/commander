library updroid_teleop;

import 'dart:html';
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
      ..style.transform = 'translate(-50%, -100px)';
    view.content.children.add(image);

    new js.JsObject(js.context['startScanning'], []);

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