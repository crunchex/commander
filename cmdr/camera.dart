part of updroid_server;

class UpDroidCamera {
  int cameraNum = 1;
  WebSocket _ws;

  Uint16List _streamHeader;

  UpDroidCamera(this.cameraNum) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidCamera ($cameraNum)', 0);

    _streamHeader = [];

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
          // Need to send: 106, 115, 109, 2, 128, 1, 224

          _streamHeader.addAll(UTF8.encode('jsmp'));
          //_streamHeader.add(width);
          //_streamHeader.add(height);
          _streamHeader.add(2);
          _streamHeader.add(128);
          _streamHeader.add(1);
          _streamHeader.add(224);


        } else {
          help.debug('Failed Stream Connection', 1);
        }
      });
    });
  }

  void _runFFMpeg() {
    List<String> options = ['-s', '640x480', '-f', 'video4linux2', '-i', '/dev/video0', '-f', 'mpeg1video', '-b', '800k', '-r', '20', 'http://127.0.0.1:1208$cameraNum/cheesecake/640/480'];
    Process.start('ffmpeg', options).then((Process shell) {
      //shell.stdout.listen((data) => help.debug('ffmpeg[$cameraNum] stdout: ${UTF8.decode(data)}', 0));
      //shell.stderr.listen((data) => help.debug('ffmpeg[$cameraNum] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('ffmpeg [cameraNum]: run failed. Probably not installed', 1);
      return;
    });
  }
}