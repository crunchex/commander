part of cmdr_camera;

class CameraServer {
  int videoId;
  StreamController<List<int>> transStream;

  Process _shell;
  Uint16List streamHeader;

  CameraServer(this.videoId) {
    streamHeader = [];
    streamHeader.addAll(UTF8.encode('jsmp'));
    // TODO: replace hardcoding with actual UInt16BE conversion.
    //_streamHeader.add(width.g);
    //_streamHeader.add(height);
    //streamHeader.writeUInt16BE(width, 4);
    //streamHeader.writeUInt16BE(height, 6);
    streamHeader.addAll([2, 128, 1, 224]);

    transStream = new StreamController<List<int>>.broadcast();

    var addressesIListenFrom = InternetAddress.ANY_IP_V4;
    int portIListenOn = 13020 + videoId;
    RawDatagramSocket.bind(addressesIListenFrom, portIListenOn).then((RawDatagramSocket udpSocket) {
      udpSocket.listen((RawSocketEvent event) {
        if (event == RawSocketEvent.READ) {
          Datagram dg = udpSocket.receive();
          transStream.add(dg.data);
        }
      });
    });

    _runFFMpeg();
  }

  void _runFFMpeg() {
    // Only one camera per USB controller (check lsusb -> bus00x), or bump size down to 320x240 to avoid bus saturation.
    // ffmpeg -s 640x480 -f video4linux2 -input_format mjpeg -i /dev/video${cameraNum - 1} -f mpeg1video -b 800k -r 20 http://127.0.0.1:12060/video/$cameraNum/640/480
    List<String> options = ['-s', '640x480', '-f', 'video4linux2', '-input_format', 'mjpeg', '-i', '/dev/video${videoId}', '-f', 'mpeg1video', '-b', '800k', '-r', '20', 'udp://127.0.0.1:1302${videoId}'];
    Process.start('ffmpeg', options).then((shell) {
      _shell = shell;
      //shell.stdout.listen((data) => help.debug('camera [$cameraNum] stdout: ${UTF8.decode(data)}', 0));
      //shell.stderr.listen((data) => help.debug('camera [$cameraNum] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('ffmpeg [$videoId]: run failed. Probably not installed', 1);
      return;
    });
  }

  void cleanup() {
    bool success = _shell.kill();
  }
}