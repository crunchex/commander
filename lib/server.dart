library cmdr;

import 'dart:io';

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:http_server/http_server.dart';
import 'package:upcom-api/git.dart';
import 'package:upcom-api/updroid_message.dart';
import 'package:upcom-api/server_message.dart';

import 'tab/explorer.dart';
import 'server_mailbox.dart';
import 'server_helper.dart' as help;
import 'tab_interface.dart';
import 'post_office.dart';

part 'commands.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class CmdrServer {
  static String defaultUprootPath = '/home/${Platform.environment['USER']}/uproot';
  static const String defaultGuiPath = '/opt/updroid/cmdr/web';
  static const bool defaultDebugFlag = false;
  static const bool defaultQuiet = false;

  ArgResults _args;

  Map _panels = {};
  Map<String, Map<int, dynamic>> _tabs = {};
  CmdrMailbox _mailbox;
  Directory dir;

  CmdrServer (ArgResults results) {
    _args = results;

    dir = new Directory(_args['workspace']);
    dir.create();
    _initServer(_getVirDir());

    _mailbox = new CmdrMailbox('UpDroidClient', 1);
    _registerMailbox();

    // A stream that pushes anything it receives onto the main websocket to the client.
  }

  /// Returns a [VirtualDirectory] set up with a path from [results].
  VirtualDirectory _getVirDir() {
    String guiPath = _args['path'];
    VirtualDirectory virDir;
    virDir = new VirtualDirectory(Platform.script.resolve(guiPath).toFilePath())
        ..allowDirectoryListing = true
        ..followLinks = true
        // Uncomment to serve to Dartium for debugging.
        //..jailroot = false
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

      help.debug("HttpServer listening on port:${server.port}...", 0);
      server.asBroadcastStream()
          .listen((HttpRequest request) => _routeRequest(request, virDir))
          .asFuture()  // Automatically cancels on error.
          .catchError((_) => help.debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, VirtualDirectory virDir) {
    // WebSocket requests are considered "upgraded" HTTP requests.
    if (!WebSocketTransformer.isUpgradeRequest(request)) {
      _handleStandardRequest(request, virDir);
      return;
    }

    help.debug('Upgraded request received: ${request.uri.path}', 0);

    // TODO: objectIDs start at 1, but List indexes start at 0 - fix this.
    int objectID = int.parse(request.uri.pathSegments[1]);
    String type = request.uri.pathSegments[0];

    if (type == 'updroidclient') {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _mailbox.handleWebSocket(ws, request));
      return;
    }

    if (type == 'updroidexplorer') {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _panels[type][objectID].mailbox.handleWebSocket(ws, request));
      return;
    }

    WebSocketTransformer.upgrade(request)
    .then((WebSocket ws) => _tabs[type][objectID].mailbox.receive(ws, request));
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    help.debug("${request.method} request for: ${request.uri.path}", 0);

    if (virDir != null) {
      virDir.serveRequest(request);
    } else {
      help.debug('ERROR: no Virtual Directory to serve', 1);
    }
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent('CLIENT_CONFIG', _clientConfig);
    _mailbox.registerWebSocketEvent('GIT_PUSH', _gitPush);
    _mailbox.registerWebSocketEvent('OPEN_TAB', _openTab);
    _mailbox.registerWebSocketEvent('CLOSE_TAB', _closeTab);
    _mailbox.registerWebSocketEvent('OPEN_PANEL', _openPanel);
//    _mailbox.registerWebSocketEvent('ADD_EXPLORER', _newExplorerCmdr);
//    _mailbox.registerWebSocketEvent('CLOSE_EXPLORER', _closeExplorerCmdr);

    _mailbox.registerWebSocketCloseEvent(_cleanUpBackend);

    _mailbox.registerServerMessageHandler('OPEN_TAB', _openTabFromServer);
    _mailbox.registerServerMessageHandler('CLOSE_TAB', _closeTabFromServer);
    _mailbox.registerServerMessageHandler('CLONE_TAB', _cloneTabFromServer);
    _mailbox.registerServerMessageHandler('MOVE_TAB', _moveTabFromServer);
    _mailbox.registerServerMessageHandler('REQUEST_EDITOR_LIST', _sendEditorList);
  }

  void _clientConfig(Msg um) {
    // TODO: send back some kind of saved config from the filesystem.
    _mailbox.send(new Msg('SERVER_READY', ''));
  }

  void _gitPush(Msg um) {
    List runArgs = um.body.split('++');
    String dirPath = runArgs[0];
    String password = runArgs[1];
    //help.debug('dirPath: $dirPath, password: $password', 0);
    Git.push(dirPath, password);
  }

  void _openPanel(Msg um) {
    String id = um.body;
    List idList = id.split('-');
    int num = int.parse(idList[1]);
    String type = idList[2].toLowerCase();

    help.debug('Open panel request received: $id', 0);

    if (!_panels.containsKey(type)) _panels[type] = {};

    switch (type) {
      case 'updroidexplorer':
        _panels[type][num] = new CmdrExplorer(num, dir);
        break;
    }
  }

  void _openTab(Msg um) {
    String id = um.body;
    List idList = id.split('-');
    int num = int.parse(idList[1]);
    String type = idList[2].toLowerCase();

    help.debug('Open tab request received: $id', 0);

    if (!_tabs.containsKey(type)) _tabs[type] = {};


    if (idList.length <= 3) {
      _tabs[type][num] = new TabInterface(type, num, dir);
    } else {
      List extra = new List.from(idList.getRange(3, idList.length));
      _tabs[type][num] = new TabInterface(type, num, dir, extra);
    }
  }

  void _openTabFromServer(Msg um) => _mailbox.send(new Msg('OPEN_TAB', um.body));

  void _closeTab(Msg um) {
    List idList = um.body.split('_');
    String type = idList[0].toLowerCase();
    int id = int.parse(idList[1]);

    help.debug('Close tab request received: ${idList.toString()}', 0);

    if (_tabs[type][id] != null) {
      _tabs[type][id].close();
      _tabs[type].remove(id);
    }
  }

  void _closeTabFromServer(Msg um) => _mailbox.send(new Msg('CLOSE_TAB', um.body));
  void _cloneTabFromServer(Msg um) => _mailbox.send(new Msg('CLONE_TAB', um.body));
  void _moveTabFromServer(Msg um) => _mailbox.send(new Msg('MOVE_TAB', um.body));

  void _sendEditorList(Msg um) {
    String pathToOpen = um.body;
    List<String> editorList = [];
    _tabs['updroideditor'].keys.forEach((int id) => editorList.add(id.toString()));
    Msg newMessage = new Msg('SEND_EDITOR_LIST', '$pathToOpen:$editorList');
    // TODO: need to able to reply back to exact sender in CmdrPostOffice.
    // This is a hacky way to reply back to the requesting explorer.
    CmdrPostOffice.send(new ServerMessage('UpDroidExplorer', 0, newMessage));
  }

  void _cleanUpBackend() {
    help.debug('Client disconnected, cleaning up...', 0);

    _panels = {};

    _tabs.values.forEach((Map<int, dynamic> tabMap) {
      tabMap.values.forEach((dynamic tab) {
        tab.close();
      });
    });
    _tabs = {};

    help.debug('Clean up done.', 0);
  }

  void _printStartMessage() {
    if (_args['quiet'] != defaultQuiet) return;

    print('[UpDroid Commander serving on port 12060]');
    print('You can now enter "localhost:12060" in your browser on this machine,');
    print('  or "<this machine\'s IP>:12060" on a machine in the same network.');

    ProcessResult pkgStatus = Process.runSync('dpkg' , ['-s', 'libnss-mdns', '|', 'grep', 'Status']);
    if (pkgStatus.stdout.contains('install ok installed')) {
      print('  or "${Platform.localHostname}.local:12060" on a Bonjour/libnss-mdns equipped machine.');
    }

    print('Ctrl-C to exit.');
  }
}