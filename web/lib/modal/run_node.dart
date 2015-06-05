part of updroid_modal;

class UpDroidRunNodeModal extends UpDroidModal {
  List<Map> _nodeList;
  StreamController<CommanderMessage> _cs;

  UpDroidRunNodeModal(List<Map> nodeList, StreamController<CommanderMessage> cs) {
    _nodeList = nodeList;
    _cs = cs;

    _setupHead('Available Nodes');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement selectorWrap = new DivElement()
      ..id = "selector-wrapper";
    _modalBody.children.add(selectorWrap);

    _nodeList.forEach((Map packageNode) {
      DivElement nodeWrap = new DivElement()
        ..classes.add('node-wrapper');
      selectorWrap.children.add(nodeWrap);
      InputElement nodeArgs = new InputElement()
        ..classes.add('node-args-input');

      if (packageNode.containsKey('args')) {
        String arguments = '';
        packageNode['args'].forEach((List arg) {
          if (arg.length == 1) {
            arguments += '${arg[0]}:=';
          } else {
            arguments += ' ${arg[0]}:=${arg[1]}';
          }
        });
        nodeArgs.placeholder = arguments;
      }

      String nodeName = packageNode['node'];
      String buttonText = nodeName.length <= 15 ? nodeName : nodeName.substring(0, 15) + ' ...';
      ButtonElement nodeButton = _createButton('default', buttonText, method: () {
        String runCommand;
        if (nodeArgs.value.isEmpty) {
          runCommand = JSON.encode([packageNode['package'], packageNode['package-path'], packageNode['node']]);
        } else {
          runCommand = JSON.encode([packageNode['package'], packageNode['package-path'], packageNode['node'], nodeArgs.value]);
        }
        //_ws.send('[[CATKIN_RUN]]' + runCommand);
        _cs.add(new CommanderMessage('EXPLORER', 'CATKIN_RUN', body: runCommand));
      });
      nodeButton
        ..dataset['toggle'] = 'tooltip'
        ..dataset['placement'] = 'bottom'
        ..title = nodeName;
      new Tooltip(nodeButton, showDelay: 700, container: selectorWrap);
      nodeWrap.children.add(nodeButton);
      nodeWrap.children.add(nodeArgs);
    });
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Cancel');
    _modalFooter.children.add(discard);
  }
}
