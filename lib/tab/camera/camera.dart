library cmdr_camera;

import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import '../../server_mailbox.dart';
import '../../server_helper.dart' as help;

part 'camera_server.dart';

class CmdrCamera {
  static const String guiName = 'UpDroidCamera';

  int id;
  Map<int, CameraServer> servers;
  CmdrMailbox mailbox;

  StreamSubscription _currentDeviceSub;

  CmdrCamera(this.id, this.servers) {
    mailbox = new CmdrMailbox(guiName, id);
    _registerMailbox();
  }

  void _signalReady(UpDroidMessage) {
    ProcessResult result = Process.runSync('bash', ['-c', 'ffmpeg --help']);
    if (result.exitCode == 127) {
      mailbox.ws.add('[[NO_FFMPEG]]');
      return;
    }

    mailbox.ws.add('[[CAMERA_READY]]' + JSON.encode(_getDeviceIds()));
  }

  void _closeTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _cloneTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _moveTab(UpDroidMessage um) {
    CmdrPostOffice.send(new ServerMessage('UpDroidClient', -1, um));
  }

  void _handleInputStream(HttpRequest request) {
    int deviceId = int.parse(request.uri.pathSegments.last);
    if (servers[deviceId] == null) {
      servers[deviceId] = new CameraServer(deviceId);
    }
    // request.uri is updroidcamera/id/input
    if (_currentDeviceSub != null) {
      _currentDeviceSub.cancel();
    }
    mailbox.ws.add(servers[deviceId].streamHeader);
    _currentDeviceSub = servers[deviceId].transStream.stream.asBroadcastStream().listen((data) {
      mailbox.ws.add(data);
    });
  }

  List<int> _getDeviceIds() {
    ProcessResult result = Process.runSync('bash', ['-c', 'find /dev -name "video*"']);
    List<String> rawDevices = result.stdout.split(new RegExp('/dev/video|\n'));
    rawDevices.removeWhere((String s) => s == '');

    List<int> deviceIds = [];
    rawDevices.forEach((String id) => deviceIds.add(int.parse(id)));
    return deviceIds;
  }

  void _registerMailbox() {
    mailbox.registerWebSocketEvent('SIGNAL_READY', _signalReady);
    mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);
    mailbox.registerWebSocketEvent('CLONE_TAB', _cloneTab);
    mailbox.registerWebSocketEvent('MOVE_TAB', _moveTab);

    _getDeviceIds().forEach((int key) {
      mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/input/$key', _handleInputStream);
    });
  }

  void cleanup() {
    CmdrPostOffice.deregisterStream(guiName, id);
  }
}