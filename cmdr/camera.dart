part of updroid_server;

class CmdrCamera {
  static const String guiName = 'UpDroidCamera';

  int cameraNum = 1;
  StreamController<List<int>> _transStream;

  Uint16List _streamHeader;

  CmdrCamera(this.cameraNum) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidCamera ($cameraNum)', 0);

    _streamHeader = [];
    _streamHeader.addAll(UTF8.encode('jsmp'));
    // TODO: replace hardcoding with actual UInt16BE conversion.
    //_streamHeader.add(width.g);
    //_streamHeader.add(height);
    //streamHeader.writeUInt16BE(width, 4);
    //streamHeader.writeUInt16BE(height, 6);
    _streamHeader.addAll([2, 128, 1, 224]);

    _transStream = new StreamController<List<int>>.broadcast();

    _runFFMpeg();
  }

  void handleWebSocket(WebSocket ws, HttpRequest request) {
    String type = request.uri.pathSegments[2];
    if (type == 'cmdr') {
      ws.add('[[CAMERA_READY]]');
    } else {
      ws.add(_streamHeader);
      _transStream.stream.listen((data) {
        ws.add(data);
      });
    }
  }

  void handleVideoFeed(HttpRequest request) {
    request.listen((data) {
      _transStream.add(data);
    });
  }

  void _runFFMpeg() {
    List<String> options = ['-s', '640x480', '-f', 'video4linux2', '-i', '/dev/video${cameraNum - 1}', '-f', 'mpeg1video', '-b', '800k', '-r', '20', 'http://127.0.0.1:12060/video/$cameraNum/640/480'];
    Process.run('ffmpeg', options).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('ffmpeg [cameraNum]: run failed. Probably not installed', 1);
      return;
    });
  }
}