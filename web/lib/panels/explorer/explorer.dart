library updroid_explorer;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import '../../modal/modal.dart';
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
        {'type': 'toggle', 'title': 'New Workspace'},
        {'type': 'submenu', 'title': 'Open Workspace', 'items': []}]},
//        {'type': 'toggle', 'title': 'Close Workspace'}]},
//        {'type': 'toggle', 'title': 'Close Panel'}]},
      {'title': 'Actions', 'items': [
        {'type': 'divider', 'title': 'Workspace'},
        {'type': 'toggle', 'title': 'New Package'},
        {'type': 'toggle', 'title': 'Build Packages'},
        {'type': 'toggle', 'title': 'Clean Workspace'},
//        {'type': 'toggle', 'title': 'Upload with Git'},
        {'type': 'divider', 'title': 'Launchers'},
        {'type': 'toggle', 'title': 'Run Launchers'}]},
      {'title': 'View', 'items': [
        {'type': 'toggle', 'title': 'Workspace'},
        {'type': 'toggle', 'title': 'Launchers'}]}
    ];
    return menu;
  }

  AnchorElement _fileDropdown;
  AnchorElement _newWorkspaceButton;
//  AnchorElement _closeWorkspaceButton;
//  AnchorElement _deleteWorkspaceButton;
  AnchorElement _newPackageButton;
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
  StreamSubscription _fileDropdownListener, _newWorkspaceListener, _closeWorkspaceListener;
  StreamSubscription _workspaceButtonListener, _launchersButtonListener;

  ExplorerController controller;
  StreamController<CommanderMessage> cs;

  List<String> _workspaceNames;

  UpDroidExplorer(int id, int col) :
  super(id, col, className, 'Explorer', getMenuConfig(), true) {

  }

  void setUpController() {
    _fileDropdown = view.refMap['file-dropdown'];

    _newWorkspaceButton = view.refMap['new-workspace'];
//    _closeWorkspaceButton = view.refMap['close-workspace'];
//    _deleteWorkspaceButton = view.refMap['delete-workspace'];

    _newPackageButton = view.refMap['new-package'];
    _buildPackagesButton = view.refMap['build-packages'];
    _cleanWorkspaceButton = view.refMap['clean-workspace'];
//      _uploadButton = _view.refMap['upload-with-git'];
    _runLaunchersButton = view.refMap['run-launchers'];

    _workspaceButton = view.refMap['workspace'];
    _launchersButton = view.refMap['launchers'];
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'REQUEST_WORKSPACE_NAMES', _requestWorkspaceNames);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_PATH', _explorerDirPath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'WORKSPACE_NAMES', _refreshOpenMenu);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'REQUEST_SELECTED', _requestSelected);
  }

  void _requestWorkspaceNames(UpDroidMessage um) => _refreshWorkspaceNames();

  void _explorerDirPath(UpDroidMessage um) {
    workspacePath = um.body;
    showWorkspacesController();
  }

  void _refreshOpenMenu(UpDroidMessage um) {
    _workspaceNames = JSON.decode(um.body);
    _workspaceNames.forEach((String name) => _addWorkspaceToMenu(name));

    if (controller != null || view.content.children.isNotEmpty) return;

    ParagraphElement placeholderText = new ParagraphElement()
      ..classes.add('explorer-placeholder');

    if (_workspaceNames == null || _workspaceNames.isEmpty) {
      placeholderText.text = 'Create a new workspace from [File] menu.';
    } else {
      placeholderText.text = 'Create a new workspace or open an existing one from [File] menu.';
    }
    view.content.children.add(placeholderText);
  }

  /// Returns an empty list to the server to let it know that there is no [WorkspaceController]
  /// that consequently nothing is selected.
  void _requestSelected(UpDroidMessage um) {
    String selected = JSON.encode([]);

    if (controller != null && controller.type == 'workspace') {
      WorkspaceController workspaceController = controller;
      selected = workspaceController.returnSelected();
    }

    mailbox.ws.send('[[RETURN_SELECTED]]' + '${um.body}:' + selected);
  }

  void _addWorkspaceToMenu(String name) {
    view.addMenuItem({'type': 'toggle', 'title': name}, '#${shortName.toLowerCase()}-$id-open-workspace');
    AnchorElement item = view.refMap[name];
    item.onClick.listen((e) {
      _openExistingWorkspace(name);
    });
  }

  //\/\/ Event Handlers /\/\//

  void registerEventHandlers() {
    _fileDropdownListener = _fileDropdown.onClick.listen((e) => _refreshWorkspaceNames());
    _newWorkspaceListener = _newWorkspaceButton.onClick.listen((e) => _newWorkspace());
//    _closeWorkspaceListener = _closeWorkspaceButton.onClick.listen((e) => _closeWorkspace());
  }

  void _refreshWorkspaceNames() {
    _workspaceNames = [];

    querySelector('#${shortName.toLowerCase()}-$id-open-workspace').innerHtml = '';
    mailbox.ws.send('[[REQUEST_WORKSPACE_NAMES]]');
  }

  void _newWorkspace() {
    UpDroidWorkspaceModal modal;
    modal = new UpDroidWorkspaceModal(() {
      String newWorkspaceName = modal.input.value;
      if (newWorkspaceName != '') {
        mailbox.ws.send('[[NEW_WORKSPACE]]' + newWorkspaceName);
      }
    });
  }

  void _closeWorkspace() {
    _newPackageButton.classes.add('disabled');
    _buildPackagesButton.classes.add('disabled');
    _cleanWorkspaceButton.classes.add('disabled');
    _runLaunchersButton.classes.add('disabled');
    _launchersButton.classes.add('disabled');
    _workspaceButton.classes.add('disabled');

    if (_launchersButtonListener != null) _launchersButtonListener.cancel();
    if (_workspaceButtonListener != null) _workspaceButtonListener.cancel();

    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    _refreshWorkspaceNames();
  }

  //\/\/ Misc Methods /\/\//

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

  void showWorkspacesController() {
    view.content.innerHtml = '';

    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    // Disable buttons not applicable for Workspace View.
    _runLaunchersButton.classes.add('disabled');
    _workspaceButton.classes.add('disabled');
    if (_workspaceButtonListener != null) _workspaceButtonListener.cancel();

    // Re-enable buttons for Workspace View.
    _newPackageButton.classes.remove('disabled');
    _buildPackagesButton.classes.remove('disabled');
    _cleanWorkspaceButton.classes.remove('disabled');
    _launchersButton.classes.remove('disabled');
    _launchersButtonListener = _launchersButton.onClick.listen((e) => showLaunchersController());

    List<AnchorElement> actionButtons = [_newPackageButton, _buildPackagesButton, _cleanWorkspaceButton];
    controller = new WorkspaceController(id, workspacePath, view, mailbox, actionButtons, showLaunchersController);
  }

  void showLaunchersController() {
    view.content.innerHtml = '';

    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    // Disable buttons not applicable for Launchers View.
    _newPackageButton.classes.add('disabled');
    _buildPackagesButton.classes.add('disabled');
    _cleanWorkspaceButton.classes.add('disabled');
    _launchersButton.classes.add('disabled');
    if (_launchersButtonListener != null) _launchersButtonListener.cancel();

    // Re-enable buttons for Launchers View.
    _runLaunchersButton.classes.remove('disabled');
    _workspaceButton.classes.remove('disabled');
    _workspaceButtonListener = _workspaceButton.onClick.listen((e) => showWorkspacesController());

    List<AnchorElement> actionButtons = [_runLaunchersButton];
    controller = new LaunchersController(id, workspacePath, view, mailbox, actionButtons, showWorkspacesController);
  }

  //\/\/ Handler Helpers /\/\//

  void cleanUp() {
    _fileDropdownListener.cancel();
    _newWorkspaceListener.cancel();
//    _closeWorkspaceListener.cancel();
    _workspaceButtonListener.cancel();
    _launchersButtonListener.cancel();
  }
}

abstract class ExplorerController {
  String type;

  void registerMailbox();
  void registerEventHandlers();
  void cleanUp();
}