library updroid_camera;

import 'dart:html';
import 'dart:js' as js;

import 'lib/updroid_message.dart';
import 'tab.dart';

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a websocket onto a canvas element.
class UpDroidCamera extends UpDroidTab {
  static const String className = 'UpDroidCamera';
  String type = 'UpDroidCamera';

  int num;
  int _col;
  WebSocket _ws;

  UpDroidCamera(this.num, int col, {bool active: false}) {
    _col = col;

    setUpTabHandle(num, _col, 'Camera', active);
    setUpTabContainer(num, _col, 'Camera', _getMenuConfig(), active).then((Map configRefs) {

      DivElement content = configRefs['content'];
      CanvasElement canvas = new CanvasElement()
          ..id = '#video-canvas'
          ..width = 447
          ..height = 450;
      content.children.add(canvas);

      _drawLoading(canvas);

      // Create the server <-> client [WebSocket].
      // Port 12060 is the default port that UpDroid uses.
      String url = window.location.host;
      url = url.split(':')[0];
      _ws = new WebSocket('ws://' + url + ':12060/camera/$num/cmdr');

      _registerEventHandlers(canvas);
    });
  }

  void _registerEventHandlers(CanvasElement canvas) {
    _ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'CAMERA_READY').listen((um) {
      _startPlayer(canvas);
    });
  }

  void _drawLoading(CanvasElement canvas) {
    CanvasRenderingContext2D context = canvas.context2D;
    context.fillStyle = 444;
    context.fillText('Loading...', canvas.width/2-30, canvas.height/3);
  }

  void _startPlayer(CanvasElement canvas) {
    String url = window.location.host;
    url = url.split(':')[0];
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':12060/camera/$num/input']);

    var options = new js.JsObject.jsify({'canvas': canvas});

    js.JsObject player = new js.JsObject(js.context['jsmpeg'], [client, options]);

  }

  List _getMenuConfig() {
    List menu = [
      {'title': 'Settings', 'items': [
        {'type': 'toggle', 'title': 'Quality'},
        {'type': 'toggle', 'title': 'Aspect Ratio'},
        {'type': 'toggle', 'title': 'Resolution'},
        {'type': 'toggle', 'title': 'CV Overlay'}]}
    ];
    return menu;
  }
}