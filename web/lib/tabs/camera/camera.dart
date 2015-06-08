library updroid_camera;

import 'dart:html';
import 'dart:convert';
import 'dart:js' as js;

import '../../mailbox.dart';
import '../tab_controller.dart';

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a [WebSocket] onto a [_canvasElement].
class UpDroidCamera extends TabController {
  static String className = 'UpDroidCamera';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Devices', 'items': []}
    ];
    return menu;
  }

  CanvasElement _canvas;
  int _width = 640;
  int _height = 480;

  UpDroidCamera(int id, int col) :
  super(id, col, className, 'Camera', getMenuConfig()) {

  }

  void setUpController() {
    _canvas = new CanvasElement();
    _canvas.classes.add('video-canvas');
    setDimensions();
    view.content.children.add(_canvas);

    _drawLoading();
  }

  void setDimensions() {
    // TODO: fix this fixed query selector.
    var con = querySelector('#col-1-tab-content');
    var width = (con.contentEdge.width - 13);
    var height = (con.contentEdge.height - 13);

    _canvas.width = width <= _width ? width : _width;
    _canvas.height = height <= _height ? height : _height;
  }

  void _drawLoading() {
    CanvasRenderingContext2D context = _canvas.context2D;
    context.fillStyle = '#ffffff';
    context.fillText('Loading...', _canvas.width / 2 - 30, _canvas.height / 2);
  }

  List<int> _setDevices(String devices) {
    List<int> deviceIds = JSON.decode(devices);
    deviceIds.sort((a, b) => a.compareTo(b));
    deviceIds.forEach((int i) {
      view.config.last['items'].add({'type': 'toggle', 'title': 'Video$i', 'handler': _startPlayer, 'args': i});
    });
    view.refreshMenus();

    // Returns the sorted list.
    return deviceIds;
  }

  void _startPlayer(int deviceId) {
    String deviceIdString = deviceId.toString();
    String url = window.location.host;
    url = url.split(':')[0];
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':12060/${className.toLowerCase()}/$id/input/$deviceIdString']);

    var options = new js.JsObject.jsify({'canvas': _canvas});

    new js.JsObject(js.context['jsmpeg'], [client, options]);
  }

  void _signalReady(UpDroidMessage um) {
    mailbox.ws.send('[[SIGNAL_READY]]');
  }

  //\/\/ Mailbox Handlers /\/\//

  void _postReadySetup(UpDroidMessage um) {
    List<int> sortedIds = _setDevices(um.body);
    _startPlayer(id % sortedIds.length);
  }

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'SIGNAL_READY', _signalReady);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CAMERA_READY', _postReadySetup);
  }

  void registerEventHandlers() {
    window.onResize.listen((e) {
      setDimensions();
    });
  }

  void cleanUp() {}
}