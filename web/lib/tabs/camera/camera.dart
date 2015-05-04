library updroid_camera;

import 'dart:html';
import 'dart:async';
import 'dart:js' as js;

import '../../updroid_message.dart';
import '../../tab_view.dart';

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a [WebSocket] onto a [CanvasElement].
class UpDroidCamera {
  String type = 'UpDroidCamera';

  int num;
  int _col;
  StreamController<CommanderMessage> _cs;

  TabView _view;
  WebSocket _ws;

  DivElement _content;
  AnchorElement _closeTabButton;

  UpDroidCamera(this.num, int col, StreamController<CommanderMessage> cs, {bool active: false}) {
    _col = col;
    _cs = cs;

    TabView.createTabView(num, _col, 'Camera', active, _getMenuConfig()).then((tabView) {
      _view = tabView;
      setUpController();
    });
  }

  void setUpController() {
    _closeTabButton = _view.refMap['close-tab'];

    _content = _view.content;
    CanvasElement canvas = new CanvasElement();
    canvas.classes.add('video-canvas');
    setDimensions(canvas);
    _content.children.add(canvas);

    _drawLoading(canvas);

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.
    String url = window.location.host;
    url = url.split(':')[0];
    _ws = new WebSocket('ws://' + url + ':12060/camera/$num/cmdr');

    _registerEventHandlers(canvas);
  }

  void setDimensions (CanvasElement canvas) {
    var con = querySelector('#col-$num-tab-content');
    var width = (con.borderEdge.width - 13);
    var height = (con.borderEdge.height - 13);

    width <= 640 ? canvas.width = width : canvas.width = 640;
    height <= 480 ? canvas.height = height : canvas.height = 482;
  }

  void resizeCanvas(CanvasElement canvas){
      var con = querySelector('#col-$num-tab-content'),
          width = (con.borderEdge.width - 13),
          height = (con.borderEdge.height - 13);

      width <= 640 ? canvas.width = width : canvas.width = 640;
      height <= 480 ? canvas.height = height : canvas.height = 482;

  }

  void _registerEventHandlers(CanvasElement canvas) {

    window.onResize.listen((e){
      resizeCanvas(canvas);
    });

    _ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'CAMERA_READY').listen((um) {
      _startPlayer(canvas);
    });

    _view.tabHandleButton.onDoubleClick.listen((e) {
      e.preventDefault();
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_col}_UpDroidCamera'));
    });

    _closeTabButton.onClick.listen((e) {
      _view.destroy();
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

  /// TODO: put these into super class.
  void makeActive() => _view.makeActive();
  void makeInactive() => _view.makeInactive();

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