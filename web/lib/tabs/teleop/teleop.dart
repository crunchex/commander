library updroid_teleop;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
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

  WebSocket _ws;
  ParagraphElement _buttons;

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
      SpanElement span = new SpanElement()
        ..style.position = 'absolute'
        ..style.top = '50%'
        ..style.left = '50%'
        ..style.transform = 'translate(-50%, -${i * 20 + 10}px)';
      view.content.children.add(span);

      ParagraphElement axisLabel = new ParagraphElement()
        ..id = '${className.toLowerCase()}-$id-axis-label-$i'
        ..style.display = 'inline'
        ..style.color = '#ffffff'
        ..style.fontSize = '16px'
        ..text = 'Axis $i: ';
      span.children.add(axisLabel);

      ParagraphElement axisData = new ParagraphElement()
        ..id = '${className.toLowerCase()}-$id-axis-data-$i'
        ..style.display = 'inline'
        ..style.color = '#ffffff'
        ..style.fontSize = '16px'
        ..text = 'disconnected';
      span.children.add(axisData);
    }

    for (int i = 0; i < 11; i++) {
      SpanElement span = new SpanElement()
        ..style.position = 'absolute'
        ..style.top = '50%'
        ..style.left = '50%'
        ..style.transform = 'translate(-50%, ${i * 20 + 10}px)';
      view.content.children.add(span);

      ParagraphElement buttonLabel = new ParagraphElement()
        ..id = '${className.toLowerCase()}-$id-button-label-$i'
        ..style.display = 'inline'
        ..style.color = '#ffffff'
        ..style.fontSize = '16px'
        ..text = 'Button $i: ';
      span.children.add(buttonLabel);

      ParagraphElement buttonData = new ParagraphElement()
        ..id = '${className.toLowerCase()}-$id-button-data-$i'
        ..style.display = 'inline'
        ..style.color = '#ffffff'
        ..style.fontSize = '16px'
        ..text = '0';
      span.children.add(buttonData);
    }

    new js.JsObject(js.context['startScanning'], [id]);

    String url = window.location.host;
    url = url.split(':')[0];
    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    _initWebSocket('ws://' + url + ':12060/${className.toLowerCase()}/$id/controller/0');

    //_setGamepads();
  }

  void _initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    _ws = new WebSocket(url);

    _ws.onOpen.listen((e) {
      new Timer.periodic(new Duration(milliseconds: 1000), (_) {
        var updateStatus = new js.JsObject(js.context['updateStatus'], []);
        String payloadString = '';
        for (int i = 1; i < 5; i++) {
          payloadString += view.content.children[i].text.replaceFirst(new RegExp(r'^Axis\ [0-9]+:'), '');
        }
        payloadString += _buttons.text;
//        print(payloadString);
//        _ws.send(payloadString);
      });
    });

    _ws.onError.listen((e) {
      print('Console-$id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  void _setGamepads() {
    Map deviceIds = js.context['controllers'];
    //deviceIds.sort((a, b) => a.compareTo(b));
    for (int i = 0; i < deviceIds.keys.length; i++) {
      view.addMenuItem({'type': 'toggle', 'title': 'Gamepad$i'}, '#${shortName.toLowerCase()}-$id-controllers');
    }
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {

  }

  void registerEventHandlers() {

  }

  void cleanUp() {}
}