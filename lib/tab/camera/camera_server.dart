part of cmdr_camera;

Map<int, CameraServer> _camServers;

class CameraServer {
  static Map<int, CameraServer> get servers {
    if (_camServers == null) {
      _camServers = {};
    }

    return _camServers;
  }

  static Uint16List get streamHeader {
    Uint16List streamHeader = [];

    streamHeader.addAll(UTF8.encode('jsmp'));
    // TODO: replace hardcoding with actual UInt16BE conversion.
    //_streamHeader.add(width.g);
    //_streamHeader.add(height);
    //streamHeader.writeUInt16BE(width, 4);
    //streamHeader.writeUInt16BE(height, 6);

    // From jsmpg.js:
    //	this.width = (data[4] * 256 + data[5]);
    //  this.height = (data[6] * 256 + data[7]);
    streamHeader.addAll([1, 64, 0, 240]);

    return streamHeader;
  }

  int videoId;

  Process _shell;
  RawDatagramSocket _udpSocket;
  StreamController<List<int>> _transStream;
  int _listenerCount;

  CameraServer(this.videoId) {
    _listenerCount = 0;
    _transStream = new StreamController<List<int>>.broadcast();

    var addressesIListenFrom = InternetAddress.ANY_IP_V4;
    int portIListenOn = 13020 + videoId;
    RawDatagramSocket.bind(addressesIListenFrom, portIListenOn).then((RawDatagramSocket udpSocket) {
      _udpSocket = udpSocket;
      _udpSocket.listen((RawSocketEvent event) {
        if (event == RawSocketEvent.READ) {
          Datagram dg = _udpSocket.receive();
          _transStream.add(dg.data);
        }
      });
    });

    _runFFMpeg();
  }

  Stream subscribeToStream() {
    _listenerCount++;
    return _transStream.stream;
  }

  void unsubscribeToString(StreamSubscription sub) {
    _listenerCount--;
    sub.cancel();
  }

  void _runFFMpeg() {
    // Only one camera per USB controller (check lsusb -> bus00x), or bump size down to 320x240 to avoid bus saturation.
    // ffmpeg -s 320x240 -f video4linux2 -input_format mjpeg -i /dev/video0 -f mpeg1video -b 800k -r 20 udp://127.0.0.1:13020
    List<String> options = ['-s', '320x240', '-f', 'video4linux2', '-input_format', 'mjpeg', '-i', '/dev/video${videoId}', '-f', 'mpeg1video', '-b', '800k', '-r', '20', 'udp://127.0.0.1:1302${videoId}'];
    Process.start('ffmpeg', options).then((shell) {
      _shell = shell;
      //shell.stdout.listen((data) => help.debug('video [$videoId] stdout: ${UTF8.decode(data)}', 0));
      //shell.stderr.listen((data) => help.debug('video [$videoId] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('ffmpeg [$videoId]: run failed. Probably not installed', 1);
      return;
    });
  }

  Future<bool> cleanup() async {
    if (_listenerCount != 0) return false;

    _shell.kill();
    _udpSocket.close();
    await _transStream.close();

    return true;
  }
}