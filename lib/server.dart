library cmdr;

import 'dart:async';
import 'dart:io';
import 'dart:convert';

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:http_server/http_server.dart';
import 'package:path/path.dart' as pathLib;
import 'package:upcom-api/git.dart';
import 'package:upcom-api/tab_backend.dart';
import 'package:upcom-api/debug.dart';
import 'package:upcom-api/ros.dart';
import 'package:quiver/async.dart';

import 'server_mailbox.dart';
import 'tab_interface.dart';
import 'panel_interface.dart';
import 'post_office.dart';

part 'commands.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class CmdrServer {
  static final String defaultUprootPath = '/home/${Platform.environment['USER']}/uproot';
  static const String defaultInstallationPath = '/opt/updroid/cmdr';
  static const bool defaultDebugFlag = false;
  static const bool defaultQuiet = false;

  static const String explorerRefName = 'upcom-explorer';
  static const String speakRefName = 'upcom-speak';
  static const String editorRefName = 'upcom-editor';
  static const String consoleRefName = 'upcom-console';

  ArgResults _args;

  Map<String, Map<int, PanelInterface>> _panels = {};
  Map<String, Map<int, TabInterface>> _tabs = {};

  Map<String, List<String>> _idQueue = {};
  Map<String, List<String>> _pendingTabRequests = {};

  CmdrMailbox _mailbox;
  String _installationPath;
  Directory dir;

  CmdrServer (ArgResults results) {
    _args = results;
    _installationPath = _args['path'];

    dir = new Directory(_args['workspace']);
    dir.create();
    _initServer(_getVirDir());

    _mailbox = new CmdrMailbox(Tab.upcomName, 1);
    _registerMailbox();

    Ros.startRosCore();
  }

  /// Returns a [VirtualDirectory] set up with a path from [results].
  VirtualDirectory _getVirDir() {
    String guiPath = '${_args['path']}/web';
    VirtualDirectory virDir;
    virDir = new VirtualDirectory(Platform.script.resolve(guiPath).toFilePath())
        ..allowDirectoryListing = true
        ..followLinks = true
        // Uncomment to serve to Dartium for debugging.
        //..jailRoot = false
        ..directoryHandler = (dir, request) {
          // Redirects '/' to 'index.html'
          var indexUri = new Uri.file(dir.path).resolve('index.html');
          virDir.serveFile(new File(indexUri.toFilePath()), request);
        };

    return virDir;
  }

  /// Initializes and HTTP server to serve the gui and handle [WebSocket] requests.
  void _initServer(VirtualDirectory virDir) {
    // Set up an HTTP webserver and listen for standard page requests or upgraded
    // [WebSocket] requests.
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12060).then((HttpServer server) {
      _printStartMessage();

      debug("HttpServer listening on port:${server.port}...", 0);
      server.asBroadcastStream()
          .listen((HttpRequest request) => _routeRequest(request, virDir))
          .asFuture()  // Automatically cancels on error.
          .catchError((_) => debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, VirtualDirectory virDir) {
    // WebSocket requests are considered "upgraded" HTTP requests.
    if (!WebSocketTransformer.isUpgradeRequest(request)) {
      _handleStandardRequest(request, virDir);
      return;
    }

    debug('Upgraded request received: ${request.uri.path}', 0);

    // TODO: objectIDs start at 1, but List indexes start at 0 - fix this.
    int objectID = int.parse(request.uri.pathSegments[1]);
    String type = request.uri.pathSegments[0];

    if (type == Tab.upcomName) {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _mailbox.handleWebSocket(ws, request));
      return;
    }

    if (_tabs.containsKey(type)) {
      WebSocketTransformer.upgrade(request)
        .then((WebSocket ws) => _tabs[type][objectID].mailbox.receive(ws, request));
    } else if (_panels.containsKey(type)) {
      WebSocketTransformer.upgrade(request)
        .then((WebSocket ws) => _panels[type][objectID].mailbox.receive(ws, request));
    }
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    debug("${request.method} request for: ${request.uri.path}", 0);

    List<String> segs = request.uri.pathSegments;
    if (segs.length > 1 && segs[segs.length - 2] == 'requestId') {
      request.response
        ..headers.contentType = ContentType.JSON
        ..write(_idQueue[segs.last].first)
        ..close();

      // Pop the ID off the queue.
      _idQueue[segs.last].removeAt(0);
      return;
    }

    if (virDir != null) {
      virDir.serveRequest(request);
    } else {
      debug('ERROR: no Virtual Directory to serve', 1);
    }
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent('CLIENT_CONFIG', _clientConfig);
    _mailbox.registerWebSocketEvent('GIT_PUSH', _gitPush);
    _mailbox.registerWebSocketEvent('OPEN_TAB', _openTab);
    _mailbox.registerWebSocketEvent('OPEN_TAB_AS_REQUEST', _openTabAsRequest);
    _mailbox.registerWebSocketEvent('OPEN_PANEL', _openPanel);
    _mailbox.registerWebSocketEvent('UPDATE_COLUMN', _updateColumn);
    _mailbox.registerWebSocketEvent('REQUEST_PLUGINSINFO', _sendPluginInfo);
//    _mailbox.registerWebSocketEvent('ADD_EXPLORER', _newExplorerCmdr);
//    _mailbox.registerWebSocketEvent('CLOSE_EXPLORER', _closeExplorerCmdr);

    _mailbox.registerWebSocketCloseEvent(_cleanUpBackend);

    _mailbox.registerServerMessageHandler('GET_TABS_INFO', _sendTabsInfo);
    _mailbox.registerServerMessageHandler('REQUEST_TAB', _requestTabFromServer);
    _mailbox.registerServerMessageHandler('OPEN_TAB', _openTabFromServer);
    _mailbox.registerServerMessageHandler('CLOSE_TAB', _closeTabFromServer);
    _mailbox.registerServerMessageHandler('MOVE_TAB', _moveTabFromServer);
    _mailbox.registerServerMessageHandler('REQUEST_EDITOR_LIST', _sendEditorList);
    _mailbox.registerServerMessageHandler('ISSUE_ALERT', _relayAlert);
  }

  void _clientConfig(Msg um) {
    File configFile = new File('/home/${Platform.environment['USER']}/.config/updroid/.lastsession.json');
    configFile.exists().then((bool exists) {
      if (exists) {
        String strConfig = configFile.readAsStringSync();
        _mailbox.send(new Msg('SERVER_READY', strConfig));
      } else {
        List listConfig = [
          [],
          [{'id': 1, 'class': consoleRefName}],
          [{'id': 2, 'class': consoleRefName}]
        ];

        String strConfig = JSON.encode(listConfig);
        _mailbox.send(new Msg('SERVER_READY', strConfig));
      }
    });
  }

  void _gitPush(Msg um) {
    List runArgs = um.body.split('++');
    String dirPath = runArgs[0];
    String password = runArgs[1];
    //debug('dirPath: $dirPath, password: $password', 0);
    Git.push(dirPath, password);
  }

  void _openPanel(Msg um) {
	List idList = um.body.split(':');
    int id = int.parse(idList[1]);
    String refName = idList[2];

    String binPath = '$_installationPath/bin';

    debug('Open panel request received: ${um.body}', 0);

    if (!_panels.containsKey(refName)) _panels[refName] = {};

    _panels[refName][id] = new PanelInterface(binPath, refName, id, dir);
  }

  void _openTabAsRequest(Msg um) {
    String id = um.body;
    List idList = id.split(':');
    int num = int.parse(idList[1]);
    String refName = idList[2];

    String binPath = '$_installationPath/bin';

    debug('Open tab request received: $id', 0);

    if (!_tabs.containsKey(refName)) _tabs[refName] = {};

    if (idList.length <= 3) {
      _tabs[refName][num] = new TabInterface(binPath, refName, num, dir);
    } else {
      List extra = new List.from(idList.getRange(3, idList.length));
      _tabs[refName][num] = new TabInterface(binPath, refName, num, dir, extra);
    }

    // Send the ID of the new tab back to the original requester.
    if (_pendingTabRequests.containsKey(refName) && _pendingTabRequests[refName].isNotEmpty) {
      List<String> split = _pendingTabRequests[refName][0].split(':');
      String requesterRefName = split[0];
      int requesterId = int.parse(split[1]);

      Msg m = new Msg('REQUEST_FULFILLED', '$refName:$num');
      CmdrPostOffice.send(new ServerMessage(Tab.upcomName, 1, requesterRefName, requesterId, m));
      _pendingTabRequests[refName].removeAt(0);
    }
  }

  void _openTab(Msg um) {
    List idList = um.body.split(':');
    int col= int.parse(idList[0]);
    int id = int.parse(idList[1]);
    String refName = idList[2];

    // Add the ID to the queue so the Tab's frontend can pick it up.
    if (!_idQueue.containsKey(refName)) _idQueue[refName] = [];
    _idQueue[refName].add(JSON.encode([id, col]));

    String binPath = '$_installationPath/bin';

    debug('Open tab request received: ${um.body}', 0);

    if (!_tabs.containsKey(refName)) _tabs[refName] = {};

    if (idList.length <= 3) {
      _tabs[refName][id] = new TabInterface(binPath, refName, id, dir);
    } else {
      List extra = new List.from(idList.getRange(3, idList.length));
      _tabs[refName][id] = new TabInterface(binPath, refName, id, dir, extra);
    }
  }

  void _updateColumn(Msg um) {
    List idList = um.body.split(':');
    String type = idList[0];
    int id = int.parse(idList[1]);
    String newColumn = idList[2];

    Msg newMessage = new Msg(um.header, newColumn);
    CmdrPostOffice.send(new ServerMessage(Tab.upcomName, 0, type, id, newMessage));
  }

  void _sendTabsInfo(Msg um) {
    List idList = um.body.split(':');
    String type = idList[0];
    int id = int.parse(idList[1]);

    // Specialized transformer that takes a tab directory as input and extracts tab info
    // from the json file within.
    StreamTransformer extractTabInfo = new StreamTransformer.fromHandlers(handleData: (event, sink) {
      File tabInfoJson = new File(pathLib.normalize('${event.path}/tabinfo.json'));
      String tabInfoString = tabInfoJson.readAsStringSync();
      sink.add(JSON.decode(tabInfoString));
    });

    Map tabsInfo = {};

    new Directory('$_installationPath/bin/tabs')
      .list()
      .transform(extractTabInfo)
      .listen((Map tabInfoMap) => tabsInfo[tabInfoMap['refName']] = tabInfoMap)
      .onDone(() {
        Msg newMessage = new Msg('SEND_TABS_INFO', JSON.encode(tabsInfo));
        CmdrPostOffice.send(new ServerMessage(Tab.upcomName, 0, type, id, newMessage));
      });
  }

  void _sendPluginInfo(Msg um) {
    // Specialized transformer that takes a tab directory as input and extracts tab info
    // from the json file within.
    StreamTransformer extractTabInfo = new StreamTransformer.fromHandlers(handleData: (event, sink) {
      File tabInfoJson = new File(pathLib.normalize('${event.path}/tabinfo.json'));
      String tabInfoString = tabInfoJson.readAsStringSync();
      sink.add(JSON.decode(tabInfoString));
    });

    StreamTransformer extractPanelInfo = new StreamTransformer.fromHandlers(handleData: (event, sink) {
      File tabInfoJson = new File(pathLib.normalize('${event.path}/panelinfo.json'));
      String tabInfoString = tabInfoJson.readAsStringSync();
      sink.add(JSON.decode(tabInfoString));
    });

    FutureGroup group = new FutureGroup();
    Completer panelsDone = new Completer();
    Completer tabsDone = new Completer();
    group.add(panelsDone.future);
    group.add(tabsDone.future);

    Map pluginsInfo = {'panels': {}, 'tabs': {}};

    new Directory('$_installationPath/bin/panels')
      .list()
      .transform(extractPanelInfo)
      .listen((Map panelInfoMap) => pluginsInfo['panels'][panelInfoMap['refName']] = panelInfoMap)
      .onDone(() => panelsDone.complete());

    new Directory('$_installationPath/bin/tabs')
    .list()
    .transform(extractTabInfo)
    .listen((Map tabInfoMap) => pluginsInfo['tabs'][tabInfoMap['refName']] = tabInfoMap)
    .onDone(() => tabsDone.complete());

    group.future.then((_) => _mailbox.send(new Msg('PLUGINS_INFO', JSON.encode(pluginsInfo))));
  }

  void _openTabFromServer(Msg um) => _mailbox.send(new Msg('OPEN_TAB', um.body));

  void _requestTabFromServer(Msg um) {
    List<String> split = um.body.split(':');

    if (_pendingTabRequests[split[2]] == null) _pendingTabRequests[split[2]] = [];

    _pendingTabRequests[split[2]].add('${split[0]}:${split[1]}');
    _mailbox.send(new Msg('REQUEST_TAB', '${split[2]}:${split[3]}'));
  }

  void _closeTabFromServer(Msg um) {
    List idList = um.body.split(':');
    String type = idList[0];
    int id = int.parse(idList[1]);

    debug('Close tab request received: ${idList.toString()}', 0);

    // Send a message to Client to shut down the tab's script and remove the view.
    _mailbox.send(new Msg('CLOSE_TAB', um.body));

    // Shut down the tab's Server side.
    if (_tabs[type][id] != null) {
      _tabs[type][id].close();
      _tabs[type].remove(id);
    }
  }

  void _moveTabFromServer(Msg um) => _mailbox.send(new Msg('MOVE_TAB', um.body));

  void _sendEditorList(Msg um) {
    List<String> split = um.body.split(':');
    String senderClass = split[0];
    int senderId = int.parse(split[1]);
    String pathToOpen = split[2];

    List<String> editorList = [];
    _tabs[editorRefName].keys.forEach((int id) => editorList.add(id.toString()));
    Msg newMessage = new Msg('SEND_EDITOR_LIST', '$pathToOpen:$editorList');
    CmdrPostOffice.send(new ServerMessage(Tab.upcomName, 0, senderClass, senderId, newMessage));
  }

  void _relayAlert(Msg um) => _mailbox.send(um);

  void _cleanUpBackend() {
    debug('Client disconnected, cleaning up...', 0);

    _panels = {};

    _tabs.values.forEach((Map<int, dynamic> tabMap) {
      tabMap.values.forEach((dynamic tab) {
        tab.close();
      });
    });
    _tabs = {};

    debug('Clean up done.', 0);
  }

  void _printStartMessage() {
    if (_args['quiet'] != defaultQuiet) return;

    print('[UpDroid Commander serving on port 12060]');
    print('You can now enter "localhost:12060" in your browser on this machine,');
    print('  or "${Process.runSync('hostname', ['-I']).stdout.replaceAll('\n', '').trim()}:12060" on a machine in the same network.');

    if (!Platform.isMacOS) {
      ProcessResult pkgStatus = Process.runSync('dpkg' , ['-s', 'libnss-mdns', '|', 'grep', 'Status']);
      if (pkgStatus.stdout.contains('install ok installed')) {
        print('  or "${Platform.localHostname}.local:12060" on a Bonjour/libnss-mdns equipped machine.');
      }
    }

    print('Ctrl-C to exit.');
  }
}