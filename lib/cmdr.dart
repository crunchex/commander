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

import 'mailbox.dart';
import 'tab_interface.dart';
import 'panel_interface.dart';
import 'post_office.dart';

part 'commands.dart';
part 'webserver.dart';

/// A class that serves the Commander frontend and handles [WebSocket] duties.
class Cmdr {
  static final String defaultUprootPath = '/home/${Platform.environment['USER']}/uproot';
  static const String defaultInstallationPath = '/opt/updroid/cmdr';
  static const bool defaultDebugFlag = false;
  static const bool defaultQuiet = false;

  static const String explorerRefName = 'upcom-explorer';
  static const String editorRefName = 'upcom-editor';
  static const String consoleRefName = 'upcom-console';

  ArgResults args;

  Map<String, Map<int, PanelInterface>> _panels;
  Map<String, Map<int, TabInterface>> _tabs;
  Map<String, List<String>> _idQueue, _pendingTabRequests;

  CmdrMailbox _mailbox;
  String _installationPath;
  Directory _uproot;

  Cmdr(this.args) {
    // Process args.
    _installationPath = args['path'];
    _uproot = new Directory(args['uproot'])..create();

    // Set up Cmdr's mailbox.
    _mailbox = new CmdrMailbox(Tab.upcomName, 1);
    _registerMailbox();

    // Initialize the maps.
    _panels = {};
    _tabs = {};
    _idQueue = {};
    _pendingTabRequests = {};

    // Create the webserver and start serving UpCom.
    new WebServer(args, _mailbox, _tabs, _panels, _idQueue)..init();

    // Start up ROS.
    Ros.startRosCore();
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent('CLIENT_CONFIG', _clientConfig);
    _mailbox.registerWebSocketEvent('GIT_PUSH', _gitPush);
    _mailbox.registerWebSocketEvent('OPEN_TAB', _openTab);
    _mailbox.registerWebSocketEvent('OPEN_TAB_AS_REQUEST', _openTabAsRequest);
    _mailbox.registerWebSocketEvent('OPEN_PANEL', _openPanel);
    _mailbox.registerWebSocketEvent('UPDATE_COLUMN', _updateColumn);
    _mailbox.registerWebSocketEvent('REQUEST_PLUGINSINFO', _sendPluginInfo);

    _mailbox.registerServerMessageHandler('GET_TABS_INFO', _sendTabsInfo);
    _mailbox.registerServerMessageHandler('REQUEST_TAB', _requestTabFromServer);
    _mailbox.registerServerMessageHandler('OPEN_TAB', _openTabFromServer);
    _mailbox.registerServerMessageHandler('CLOSE_TAB', _closeTabFromServer);
    _mailbox.registerServerMessageHandler('MOVE_TAB', _moveTabFromServer);
    _mailbox.registerServerMessageHandler('REQUEST_EDITOR_LIST', _sendEditorList);
    _mailbox.registerServerMessageHandler('ISSUE_ALERT', _relayAlert);

    _mailbox.registerWebSocketCloseEvent(_cleanUpBackend);
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

  void _openTab(Msg um) {
    debug('Open tab request received: ${um.body}', 0);

    List idList = um.body.split(':');
    int col= int.parse(idList[0]);
    int id = int.parse(idList[1]);
    String refName = idList[2];

    // Add the ID to the queue so the Tab's frontend can pick it up.
    if (!_idQueue.containsKey(refName)) _idQueue[refName] = [];
    _idQueue[refName].add(JSON.encode([id, col]));

    String binPath = '$_installationPath/bin';

    if (!_tabs.containsKey(refName)) _tabs[refName] = {};

    if (idList.length <= 3) {
      _tabs[refName][id] = new TabInterface(binPath, refName, id, _uproot);
    } else {
      List extra = new List.from(idList.getRange(3, idList.length));
      _tabs[refName][id] = new TabInterface(binPath, refName, id, _uproot, extra);
    }
  }

  void _openTabAsRequest(Msg um) {
    debug('Open tab request received: ${um.body}', 0);

    List idList = um.body.split(':');
    int num = int.parse(idList[1]);
    String refName = idList[2];

    String binPath = '$_installationPath/bin';

    if (!_tabs.containsKey(refName)) _tabs[refName] = {};

    if (idList.length <= 3) {
      _tabs[refName][num] = new TabInterface(binPath, refName, num, _uproot);
    } else {
      List extra = new List.from(idList.getRange(3, idList.length));
      _tabs[refName][num] = new TabInterface(binPath, refName, num, _uproot, extra);
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

  void _openPanel(Msg um) {
    debug('Open panel request received: ${um.body}', 0);

    List idList = um.body.split(':');
    int id = int.parse(idList[1]);
    String refName = idList[2];

    String binPath = '$_installationPath/bin';

    if (!_panels.containsKey(refName)) _panels[refName] = {};

    _panels[refName][id] = new PanelInterface(binPath, refName, id, _uproot);
  }

  void _updateColumn(Msg um) {
    List idList = um.body.split(':');
    String type = idList[0];
    int id = int.parse(idList[1]);
    String newColumn = idList[2];

    Msg newMessage = new Msg(um.header, newColumn);
    CmdrPostOffice.send(new ServerMessage(Tab.upcomName, 0, type, id, newMessage));
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

  void _openTabFromServer(Msg um) => _mailbox.send(new Msg('OPEN_TAB', um.body));

  void _requestTabFromServer(Msg um) {
    List<String> split = um.body.split(':');

    if (_pendingTabRequests[split[2]] == null) _pendingTabRequests[split[2]] = [];

    _pendingTabRequests[split[2]].add('${split[0]}:${split[1]}');
    _mailbox.send(new Msg('REQUEST_TAB', '${split[2]}:${split[3]}'));
  }

  void _closeTabFromServer(Msg um) {
    debug('Close tab request received: ${um.body}', 0);

    List idList = um.body.split(':');
    String type = idList[0];
    int id = int.parse(idList[1]);

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
      tabMap.values.forEach((dynamic tab) => tab.close());
    });

    _tabs = {};

    debug('Clean up done.', 0);
  }
}