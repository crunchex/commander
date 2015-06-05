library updroid_camera;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'dart:js' as js;

import '../../mailbox.dart';
import '../../updroid_message.dart';
import '../../tab_controller.dart';

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
  AnchorElement _closeTabButton;

  UpDroidCamera(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Camera', getMenuConfig(), cs) {

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
    // TODO: fix this fixed query selector.
    var con = querySelector('#col-1-tab-content');
    var width = (con.contentEdge.width - 13);
    var height = (con.contentEdge.height - 13);

    width <= 640 ? _canvas.width = width : _canvas.width = 640;
    height <= 480 ? _canvas.height = height : _canvas.height = 482;
  }

  void resizeCanvas() {
    // TODO: fix this fixed query selector.
    var con = querySelector('#col-1-tab-content');
    var width = (con.contentEdge.width - 13);
    var height = (con.contentEdge.height - 13);

    width <= 640 ? _canvas.width = width : _canvas.width = 640;
    height <= 480 ? _canvas.height = height : _canvas.height = 482;
  }

  void _drawLoading() {
    CanvasRenderingContext2D context = _canvas.context2D;
    context.fillStyle = 444;
    context.fillText('Loading...', _canvas.width/2-30, _canvas.height/3);
  }

  void _setDevices(String devices) {
    List<int> deviceIds = JSON.decode(devices);
    deviceIds.forEach((int i) {
      view.config.last['items'].add({'type': 'toggle', 'title': 'Video$i', 'handler': _startPlayer, 'args': [i]});
    });
    view.refreshMenus();
    _startPlayer(deviceIds);
  }

  void _startPlayer(List args) {
    String deviceId = '${args[0]}';
    String url = window.location.host;
    url = url.split(':')[0];
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':12060/${className.toLowerCase()}/$id/input/$deviceId']);

    var options = new js.JsObject.jsify({'canvas': _canvas});

    new js.JsObject(js.context['jsmpeg'], [client, options]);
  }

  //\/\/ Mailbox Handlers /\/\//

  void _postReadySetup(UpDroidMessage um) {
    _setDevices(um.body);
    //_startPlayer();
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CAMERA_READY', _postReadySetup);
  }

  void _registerEventHandlers() {
    window.onResize.listen((e) {
      resizeCanvas();
    });

    view.cloneControlHitbox.onClick.listen((e) {
      e.preventDefault();
      cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${col}_${className}'));
    });

    // TODO: this should be in tab_controller somehow.
    view.closeControlHitbox.onClick.listen((e) {
      view.destroy();
      cs.add(new CommanderMessage('UPDROIDCLIENT', 'CLOSE_TAB', body: '${className}_$id'));
    });

    _closeTabButton.onClick.listen((e) {
      view.destroy();
      cs.add(new CommanderMessage('UPDROIDCLIENT', 'CLOSE_TAB', body: '${className}_$id'));
    });
  }
}