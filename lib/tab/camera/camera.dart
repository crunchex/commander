library cmdr_camera;

import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import '../../server_helper.dart' as help;

part 'camera_server.dart';

class CmdrCamera {
  static const String guiName = 'UpDroidCamera';

  int cameraNum;
  Map<int, CameraServer> servers;

  StreamSubscription _currentDeviceSub;


  CmdrCamera(this.cameraNum, this.servers) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidCamera ($cameraNum)', 0);
  }

  /// Route websocket connections to messages (cmdr) and video data (input).
  void handleWebSocket(WebSocket ws, HttpRequest request) {
    if (request.uri.pathSegments.length == 2) {
      ws.add('[[CAMERA_READY]]' + JSON.encode(_getDeviceIds()));
    } else {
      int deviceId = int.parse(request.uri.pathSegments.last);
      if (servers[deviceId] == null) {
        servers[deviceId] = new CameraServer(deviceId);
      }
      // request.uri is updroidcamera/id/input
      if (_currentDeviceSub != null) {
        _currentDeviceSub.cancel();
      }
      ws.add(servers[deviceId].streamHeader);
      _currentDeviceSub = servers[deviceId].transStream.stream.asBroadcastStream().listen((data) {
        ws.add(data);
      });
    }
  }

  List<int> _getDeviceIds() {
    ProcessResult result = Process.runSync('bash', ['-c', 'find /dev -name "video*"']);
    List<String> rawDevices = result.stdout.split(new RegExp('/dev/video|\n'));
    rawDevices.removeWhere((String s) => s == '');

    List<int> deviceIds = [];
    rawDevices.forEach((String id) => deviceIds.add(int.parse(id)));
    return deviceIds;
  }
}