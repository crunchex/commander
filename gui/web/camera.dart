library updroid_camera;

import 'dart:html';
import 'dart:async';
import 'dart:js' as js;
import 'package:dquery/dquery.dart' as dQuery;

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
  StreamController<CommanderMessage> _cs;

  AnchorElement _closeTabButton;

  UpDroidCamera(this.num, int col, StreamController<CommanderMessage> cs, {bool active: false}) {
    _col = col;
    _cs = cs;

    setUpTabHandle(num, _col, 'Camera', active);
    setUpTabContainer(num, _col, 'Camera', _getMenuConfig(), active).then((Map configRefs) {

      _closeTabButton = configRefs['close-tab'];

      DivElement content = configRefs['content'];
      CanvasElement canvas = new CanvasElement();
      canvas
          ..width = 447
          ..height = 450
          ..classes.add('video-canvas');
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

  List setDimensions (CanvasElement canvas) {
    var con = dQuery.$('.camera');
    var width = con.width;
    var height = con.height;
    return [width, height];
  }

  void resizeCanvas(CanvasElement canvas){
      var con = dQuery.$('.camera');
      var width = con.width;
      var height = con.height;

      canvas.width = width;
      canvas.height = width;
  }

  void _registerEventHandlers(CanvasElement canvas) {

    window.onResize.listen((e){
      resizeCanvas(canvas);
      print("resizing");
    });

    _ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'CAMERA_READY').listen((um) {
      _startPlayer(canvas);
    });

    tabHandleButton.onDoubleClick.listen((e) {
      e.preventDefault();
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_col}_UpDroidCamera'));
    });

    _closeTabButton.onClick.listen((e) {
      destroyTab();
      _cs.add(new CommanderMessage('CLIENT', 'CLOSE_TAB', body: '${type}_$num'));
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

    new js.JsObject(js.context['jsmpeg'], [client, options]);

  }

  List _getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Settings', 'items': [
        {'type': 'toggle', 'title': 'Quality'},
        {'type': 'toggle', 'title': 'Aspect Ratio'},
        {'type': 'toggle', 'title': 'Resolution'},
        {'type': 'toggle', 'title': 'CV Overlay'}]}
    ];
    return menu;
  }
}