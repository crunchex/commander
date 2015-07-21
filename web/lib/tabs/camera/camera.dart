library updroid_camera;

import 'dart:async';
import 'dart:html';
import 'dart:convert';
import 'dart:js' as js;

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/tab/tab_controller.dart';

enum AspectType { FIXED, FULL }

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a [WebSocket] onto a [_canvasElement].
class UpDroidCamera extends TabController {
  static String className = 'UpDroidCamera';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Devices', 'items': []},
      {'title': 'Aspect', 'items': [
        {'type': 'toggle', 'title': 'Fixed'},
        {'type': 'toggle', 'title': 'Full-Stretched'}]}
    ];
    return menu;
  }

  AnchorElement _fixedButton;
  AnchorElement _fullStretchedButton;

  CanvasElement _canvas;
  int _width = 320;
  int _height = 240;
  AspectType _aspect;

  UpDroidCamera(int id, int col) :
  super(id, col, className, 'Camera', getMenuConfig(), true) {

  }

  void setUpController() {
    _aspect = AspectType.FULL;

    _canvas = new CanvasElement();
    _canvas.tabIndex = -1;
    _canvas.classes.add('video-canvas');
    setDimensions();
    view.content.children.add(_canvas);

    _drawLoading();

    _fixedButton = view.refMap['fixed'];
    _fullStretchedButton = view.refMap['full-stretched'];
  }

  void setDimensions() {
    var width = (view.content.contentEdge.width - 13);
    var height = (view.content.contentEdge.height - 13);

    _canvas.width = width <= _width ? width : _width;
    _canvas.height = height <= _height ? height : _height;

    if (_aspect == AspectType.FIXED) {
      _canvas.style.width = null;
      _canvas.style.height = null;
    } else if (_aspect == AspectType.FULL) {
      _canvas.style.width = '100%';
      _canvas.style.height = '100%';
    }
  }

  void _drawLoading() {
    CanvasRenderingContext2D context = _canvas.context2D;
    context.fillStyle = '#ffffff';
    context.fillText('Loading...', _canvas.width / 2 - 30, _canvas.height / 2);
  }

  List<int> _setDevices(String devices) {
    if (devices == '[]') return [];

    List<int> deviceIds = JSON.decode(devices);
    deviceIds.sort((a, b) => a.compareTo(b));
    deviceIds.forEach((int i) {
      view.addMenuItem({'type': 'toggle', 'title': 'Video$i', 'handler': _startPlayer, 'args': i}, '#${shortName.toLowerCase()}-$id-devices');
    });

    // Returns the sorted list.
    return deviceIds;
  }

  void _startPlayer(int deviceId) {
    String deviceIdString = deviceId.toString();
    String url = window.location.host;
    url = url.split(':')[0];
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':12060/$className/$id/input/$deviceIdString']);

    var options = new js.JsObject.jsify({'canvas': _canvas});

    new js.JsObject(js.context['jsmpeg'], [client, options]);
  }

  //\/\/ Mailbox Handlers /\/\//

  void _signalReady(UpDroidMessage um) {
    mailbox.ws.send('[[SIGNAL_READY]]');
  }

  void _throwAlert(UpDroidMessage um) {
    window.alert('Camera won\'t work without ffmpeg. Please install it!');
  }

  void _postReadySetup(UpDroidMessage um) {
    List<int> sortedIds = _setDevices(um.body);
    if (sortedIds.isEmpty) return;

    _startPlayer(id % sortedIds.length);
  }

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'TAB_READY', _signalReady);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'NO_FFMPEG', _throwAlert);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CAMERA_READY', _postReadySetup);
  }

  void registerEventHandlers() {
    _fixedButton.onClick.listen((e) {
      _aspect = AspectType.FIXED;
      setDimensions();
      e.preventDefault();
    });

    _fullStretchedButton.onClick.listen((e) {
      _aspect = AspectType.FULL;
      setDimensions();
      e.preventDefault();
    });

    window.onResize.listen((e) {
      setDimensions();
    });
  }

  Element get elementToFocus => view.content.children[0];

  Future<bool> preClose() {
    Completer c = new Completer();
    c.complete(true);
    return c.future;
  }

  void cleanUp() {}
}