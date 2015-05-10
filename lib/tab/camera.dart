part of updroid_server;

class CmdrCamera {
  static const String guiName = 'UpDroidCamera';

  int cameraNum = 1;
  StreamController<List<int>> _transStream;

  Process _shell;
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

  /// Route websocket connections to messages (cmdr) and video data (input).
  void handleWebSocket(WebSocket ws, HttpRequest request) {
    if (request.uri.pathSegments.length == 2) {
      ws.add('[[CAMERA_READY]]');
    } else {
      // request.uri is updroidcamera/id/input
      ws.add(_streamHeader);
      _transStream.stream.listen((data) {
        ws.add(data);
      });
    }
  }

  /// Set up a listener for incoming video data from ffmpeg.
  void handleVideoFeed(HttpRequest request) {
    request.listen((data) {
      _transStream.add(data);
    });
  }

  void _runFFMpeg() {
    // Only one camera per USB controller (check lsusb -> bus00x), or bump size down to 320x240 to avoid bus saturation.
    // ffmpeg -s 640x480 -f video4linux2 -input_format mjpeg -i /dev/video${cameraNum - 1} -f mpeg1video -b 800k -r 20 http://127.0.0.1:12060/video/$cameraNum/640/480
    List<String> options = ['-s', '640x480', '-f', 'video4linux2', '-input_format', 'mjpeg', '-i', '/dev/video${cameraNum - 1}', '-f', 'mpeg1video', '-b', '800k', '-r', '20', 'http://127.0.0.1:12060/video/$cameraNum/640/480'];
    Process.start('ffmpeg', options).then((shell) {
      _shell = shell;
      //shell.stdout.listen((data) => help.debug('camera [$cameraNum] stdout: ${UTF8.decode(data)}', 0));
      //shell.stderr.listen((data) => help.debug('camera [$cameraNum] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('ffmpeg [$cameraNum]: run failed. Probably not installed', 1);
      return;
    });
  }

  void cleanup() {
    bool success = _shell.kill();
  }
}