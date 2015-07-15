library cmdr_camera;

import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import '../../server_helper.dart' as help;
import '../api/tab.dart';

part 'camera_server.dart';

class CmdrCamera extends Tab {
  StreamSubscription _currentDeviceSub;
  int _currentDeviceId;

  CmdrCamera(int id, this.servers) :
  super(id, 'UpDroidCamera') {

  }

  void registerMailbox() {
    mailbox.registerWebSocketEvent('SIGNAL_READY', _signalReady);

    _getDeviceIds().forEach((int key) {
      mailbox.registerEndpointHandler('/${guiName.toLowerCase()}/$id/input/$key', _handleInputStream);
    });
  }

  void _signalReady(UpDroidMessage) {
    ProcessResult result = Process.runSync('bash', ['-c', 'ffmpeg --help']);
    if (result.exitCode == 127) {
      mailbox.ws.add('[[NO_FFMPEG]]');
      return;
    }

    mailbox.ws.add('[[CAMERA_READY]]' + JSON.encode(_getDeviceIds()));
  }

  void _handleInputStream(HttpRequest request) {
    _currentDeviceId = int.parse(request.uri.pathSegments.last);
    if (CameraServer.servers[_currentDeviceId] == null) {
      CameraServer.servers[_currentDeviceId] = new CameraServer(_currentDeviceId);
    }
    // request.uri is updroidcamera/id/input
    if (_currentDeviceSub != null) {
      CameraServer.servers[_currentDeviceId].unsubscribeToString(_currentDeviceSub);
    }
    mailbox.ws.add(CameraServer.streamHeader);
    _currentDeviceSub = CameraServer.servers[_currentDeviceId].subscribeToStream().listen((data) {
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

  void cleanup() {
    CameraServer.servers[_currentDeviceId].unsubscribeToString(_currentDeviceSub);
  }
}