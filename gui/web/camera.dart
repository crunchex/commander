library updroid_camera;

import 'dart:html';
import 'dart:js' as js;

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a websocket onto a canvas element.
class UpDroidCamera {
  static const String className = 'UpDroidCamera';
  int cameraNum;

  UpDroidCamera(this.cameraNum) {
    CanvasElement canvas = querySelector('#video-canvas');
    _drawLoading(canvas);
    _startPlayer(canvas);
  }

  void _drawLoading(CanvasElement canvas) {
    CanvasRenderingContext2D context = canvas.context2D;
    context.fillStyle = 444;
    context.fillText('Loading...', canvas.width/2-30, canvas.height/3);
  }

  void _startPlayer(CanvasElement canvas) {
    String url = window.location.host;
    url = url.split(':')[0];
    js.JsObject client = new js.JsObject(js.context['WebSocket'], ['ws://' + url + ':1207$cameraNum/']);

    var options = new js.JsObject.jsify({'canvas': canvas});

    js.JsObject player = new js.JsObject(js.context['jsmpeg'], [client, options]);

  }
}