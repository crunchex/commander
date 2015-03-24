library updroid_camera;

import 'dart:html';
import 'dart:js' as js;

import 'lib/updroid_message.dart';

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a websocket onto a canvas element.
class UpDroidCamera {
  static const String className = 'UpDroidCamera';
  int cameraNum;
  WebSocket _ws;

  UpDroidCamera(this.cameraNum) {
    CanvasElement canvas = querySelector('#video-canvas');
    _drawLoading(canvas);

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.
    String url = window.location.host;
    url = url.split(':')[0];
    _ws = new WebSocket('ws://' + url + ':12060/camera/$cameraNum/cmdr');

    _registerEventHandlers(canvas);
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
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':12060/camera/$cameraNum/input']);

    var options = new js.JsObject.jsify({'canvas': canvas});

    js.JsObject player = new js.JsObject(js.context['jsmpeg'], [client, options]);

  }
}