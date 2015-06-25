library updroid_explorer;

import 'dart:html';
import 'dart:async';

import '../../mailbox.dart';
import '../panel_controller.dart';
import 'workspace/workspace_controller.dart';
import 'launchers/launchers_controller.dart';

part 'explorer_view.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket] into an UpDroid Commander tab.
class UpDroidExplorer extends PanelController {
  static const String className = 'UpDroidExplorer';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'submenu', 'title': 'Open Workspace', 'items': []}]},
//        {'type': 'toggle', 'title': 'Delete Workspace'},
//        {'type': 'toggle', 'title': 'Close Panel'}]},
      {'title': 'Actions', 'items': [
        {'type': 'divider', 'title': 'Workspace'},
        {'type': 'toggle', 'title': 'Build Packages'},
        {'type': 'toggle', 'title': 'Clean Workspace'},
//        {'type': 'toggle', 'title': 'Upload with Git'},
        {'type': 'divider', 'title': 'Launchers'},
        {'type': 'toggle', 'title': 'Run Nodes'}]},
      {'title': 'View', 'items': [
        {'type': 'toggle', 'title': 'Workspace'},
        {'type': 'toggle', 'title': 'Launchers'}]}
    ];
    return menu;
  }

  // Make dynamic
  bool closed;

  DivElement currentSelected;
  LIElement currentSelectedNode;
  String currentSelectedPath;
  InputElement nodeArgs;

  AnchorElement _dropdown;

  AnchorElement _fileDropdown;
  AnchorElement _openWorkspaceButton;
//  AnchorElement _deleteWorkspaceButton;
  AnchorElement _buildPackagesButton;
  AnchorElement _cleanWorkspaceButton;
//  AnchorElement _uploadButton;
  AnchorElement _runLaunchersButton;
  AnchorElement _workspaceButton;
  AnchorElement _launchersButton;

  StreamSubscription outsideClickListener;
  StreamSubscription controlLeave;
  String workspacePath;

  Map runParams = {};

  List recycleListeners = [];
  StreamSubscription _fileDropdownListener, _workspaceButtonListener, _launchersButtonListener;

  ExplorerController controller;
  StreamController<CommanderMessage> cs;

  UpDroidExplorer(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Explorer', getMenuConfig(), cs, true) {

  }

  Future setUpController() {
    _fileDropdown = view.refMap['file-dropdown'];

    _openWorkspaceButton = view.refMap['open-workspace'];
//    _deleteWorkspaceButton = view.refMap['delete-workspace'];

    _cleanWorkspaceButton = view.refMap['clean-workspace'];
    _buildPackagesButton = view.refMap['build-packages'];
//      _uploadButton = _view.refMap['upload-with-git'];
    _runLaunchersButton = view.refMap['run-nodes'];

    _workspaceButton = view.refMap['workspace'];
    _launchersButton = view.refMap['launchers'];
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'REQUEST_WORKSPACE_NAMES', _requestWorkspaceNames);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_PATH', _explorerDirPath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'WORKSPACE_NAME', _addWorkspaceToMenu);
  }

  /// Sets up the event handlers for the console.
  void registerEventHandlers() {
    _fileDropdownListener = _fileDropdown.onClick.listen((e) => _refreshWorkspaceNames());
  }

  //\/\/ UpDroidMessage Handlers /\/\//

  void _addWorkspaceToMenu(UpDroidMessage um) {
    view.addMenuItem({'type': 'toggle', 'title': um.body}, '#${shortName.toLowerCase()}-$id-open-workspace');
    AnchorElement item = view.refMap[um.body];
    item.onClick.listen((e) {
      _openExistingWorkspace(um.body);
    });
  }

  void _openExistingWorkspace(String name) {
    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    mailbox.ws.send('[[SET_CURRENT_WORKSPACE]]' + name);

    // TODO: make sure all the inner nodes (and listeners) get cleaned up properly.
    querySelector('#${shortName.toLowerCase()}-$id-open-workspace').innerHtml = '';
    mailbox.ws.send('[[REQUEST_WORKSPACE_NAMES]]');
  }

  void _requestWorkspacePath(UpDroidMessage um) => mailbox.ws.send('[[REQUEST_WORKSPACE_PATH]]');
  void _requestWorkspaceNames(UpDroidMessage um) => _refreshWorkspaceNames();

  void _refreshWorkspaceNames() {
    querySelector('#${shortName.toLowerCase()}-$id-open-workspace').innerHtml = '';
    mailbox.ws.send('[[REQUEST_WORKSPACE_NAMES]]');
  }

  void _explorerDirPath(UpDroidMessage um) {
    workspacePath = um.body;
    _showWorkspacesController();
  }

  void _showWorkspacesController() {
    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    // Disable buttons not applicable for Workspace View.
    _runLaunchersButton.classes.add('disabled');
    _workspaceButton.classes.add('disabled');
    if (_workspaceButtonListener != null) _workspaceButtonListener.cancel();

    // Re-enable buttons for Workspace View.
    _buildPackagesButton.classes.remove('disabled');
    _cleanWorkspaceButton.classes.remove('disabled');
    _launchersButton.classes.remove('disabled');
    _launchersButtonListener = _launchersButton.onClick.listen((e) => _showNodesController());

    List<AnchorElement> actionButtons = [_buildPackagesButton, _cleanWorkspaceButton];
    controller = new WorkspaceController(id, workspacePath, view, mailbox, actionButtons);
  }

  void _showNodesController() {
    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    // Disable buttons not applicable for Nodes View.
    _buildPackagesButton.classes.add('disabled');
    _cleanWorkspaceButton.classes.add('disabled');
    _launchersButton.classes.add('disabled');
    if (_launchersButtonListener != null) _launchersButtonListener.cancel();

    // Re-enable buttons for Workspace View.
    _runLaunchersButton.classes.remove('disabled');
    _workspaceButton.classes.remove('disabled');
    _workspaceButtonListener = _workspaceButton.onClick.listen((e) => _showWorkspacesController());

    List<AnchorElement> actionButtons = [_runLaunchersButton];
    controller = new LaunchersController(id, workspacePath, view, mailbox, actionButtons);
  }

  //\/\/ Handler Helpers /\/\//

  void cleanUp() {
    _fileDropdownListener.cancel();
    _workspaceButtonListener.cancel();
    _launchersButtonListener.cancel();
  }
}

abstract class ExplorerController {
  void registerMailbox();
  void registerEventHandlers();
  void cleanUp();
}