part of updroid_server;

class CmdrCamera {
  static const String guiName = 'UpDroidCamera';

  int cameraNum;
  CameraServer server;

  WebSocket _ws;

  CmdrCamera(this.cameraNum, this.server) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidCamera ($cameraNum)', 0);
  }

  /// Route websocket connections to messages (cmdr) and video data (input).
  void handleWebSocket(WebSocket ws, HttpRequest request) {
    if (request.uri.pathSegments.length == 2) {
      ws.add('[[CAMERA_READY]]' + _getNumDevices().toString());
    } else {
      // request.uri is updroidcamera/id/input
      ws.add(server.streamHeader);
      server.transStream.stream.listen((data) {
        ws.add(data);
      });
    }
  }

  /// Set up a listener for incoming video data from ffmpeg.
  void handleVideoFeed(HttpRequest request) {
    request.listen((data) {
      server.transStream.add(data);
    });
  }

  int _getNumDevices() {
    Directory dev = new Directory('/dev');
    ProcessResult result = Process.runSync('bash', ['-c', 'find /dev -name "video*" | wc -l']);
    return int.parse(result.stdout);
  }
}