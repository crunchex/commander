part of updroid_client;

/// [UpDroidCamera] is a client-side class that uses the jsmpeg library
/// to render a video stream from a websocket onto a canvas element.
class UpDroidCamera {
  int cameraNum;

  UpDroidCamera(this.cameraNum) {
    String url = window.location.host;
    url = url.split(':')[0];

    // TODO: rewrite this init script in Dart.
    js.context.callMethod('initCanvas', ['ws://' + url + ':1207$cameraNum/']);
  }
}