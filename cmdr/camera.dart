part of updroid_server;

class CmdrCamera {
  static const String guiName = 'UpDroidCamera';

  int cameraNum = 1;
  WebSocket _ws;
  StreamController<List<int>> _transStream;

  Uint16List _streamHeader;

  CmdrCamera(this.cameraNum) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidCamera ($cameraNum)', 0);

    _streamHeader = [];
    _transStream = new StreamController<List<int>>.broadcast();

    _setUpStreamReceiver();
    _runFFMpeg();
    _setUpWebSocket();
  }

  void _setUpWebSocket() {
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12070 + cameraNum).then((HttpServer server) {
      help.debug("WebSocket listening on port:${server.port}...", 0);
      server.listen((HttpRequest request) {
        if (WebSocketTransformer.isUpgradeRequest(request)) {
          WebSocketTransformer.upgrade(request).then((WebSocket ws) {
            _ws = ws;
            _ws.add(_streamHeader);
            _transStream.stream.listen((data) {
              _ws.add(data);
            });
          });
        }
      });
    });
  }

  void _setUpStreamReceiver() {
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12080 + cameraNum).then((HttpServer server) {
      help.debug("StreamReceiver listening on port:${server.port}...", 0);
      server.listen((HttpRequest request) {
        List<String> params = request.uri.path.split('/').sublist(1);
        if (params[0] == 'cheesecake') {
          int width = int.parse(params[1]);
          int height = int.parse(params[2]);

          _streamHeader.addAll(UTF8.encode('jsmp'));
          // TODO: replace hardcoding with actual UInt16BE conversion.
          //_streamHeader.add(width.g);
          //_streamHeader.add(height);
          //streamHeader.writeUInt16BE(width, 4);
          //streamHeader.writeUInt16BE(height, 6);
          _streamHeader.add(2);
          _streamHeader.add(128);
          _streamHeader.add(1);
          _streamHeader.add(224);

          request.listen((data) {
            _transStream.add(data);
          });
        } else {
          help.debug('Failed Stream Connection', 1);
        }
      });
    });
  }

  void _runFFMpeg() {
    List<String> options = ['-s', '640x480', '-f', 'video4linux2', '-i', '/dev/video0', '-f', 'mpeg1video', '-b', '800k', '-r', '20', 'http://127.0.0.1:1208$cameraNum/cheesecake/640/480'];
    Process.run('ffmpeg', options).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('ffmpeg [cameraNum]: run failed. Probably not installed', 1);
      return;
    });
  }
}