library updroid_camera;

import 'dart:html';
import 'dart:async';
import 'dart:js' as js;

import '../../mailbox.dart';
import '../../updroid_message.dart';
import '../../tab_view.dart';
import '../../tab_controller.dart';

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a [WebSocket] onto a [_canvasElement].
class UpDroidCamera extends TabController {
  static String className = 'UpDroidCamera';

  CanvasElement _canvas;

  AnchorElement _closeTabButton;

  UpDroidCamera(int id, int col, StreamController<CommanderMessage> cs, {bool active: false}) : super(id, col, className, cs, active: active) {
    TabView.createTabView(id, col, className, active, _getMenuConfig()).then((tabView) {
      view = tabView;
      setUpController();
    });
  }

  void setUpController() {
    _closeTabButton = view.refMap['close-tab'];

    _registerMailbox();
    
    _canvas = new CanvasElement();
    _canvas.classes.add('video-canvas');
    setDimensions();
    view.content.children.add(_canvas);

    _drawLoading();

    _registerEventHandlers();
  }

  void setDimensions() {
    var con = querySelector('#col-$id-tab-content');
    var width = (con.borderEdge.width - 13);
    var height = (con.borderEdge.height - 13);

    width <= 640 ? _canvas.width = width : _canvas.width = 640;
    height <= 480 ? _canvas.height = height : _canvas.height = 482;
  }

  void resizeCanvas() {
    var con = querySelector('#col-$id-tab-content'),
    width = (con.borderEdge.width - 13),
    height = (con.borderEdge.height - 13);

    width <= 640 ? _canvas.width = width : _canvas.width = 640;
    height <= 480 ? _canvas.height = height : _canvas.height = 482;
  }

  void _drawLoading() {
    CanvasRenderingContext2D context = _canvas.context2D;
    context.fillStyle = 444;
    context.fillText('Loading...', _canvas.width/2-30, _canvas.height/3);
  }

  //\/\/ Mailbox Handlers /\/\//

  void _startPlayer(UpDroidMessage um) {
    String url = window.location.host;
    url = url.split(':')[0];
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':12060/${className.toLowerCase()}/$id/input']);

    var options = new js.JsObject.jsify({'canvas': _canvas});

    new js.JsObject(js.context['jsmpeg'], [client, options]);
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CAMERA_READY', _startPlayer);
  }

  void _registerEventHandlers() {
    window.onResize.listen((e) {
      resizeCanvas();
    });

    view.tabHandleButton.onDoubleClick.listen((e) {
      e.preventDefault();
      cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${col}_UpDroidCamera'));
    });

    _closeTabButton.onClick.listen((e) {
      view.destroy();
      cs.add(new CommanderMessage('CLIENT', 'CLOSE_TAB', body: '${className}_$id'));
    });
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