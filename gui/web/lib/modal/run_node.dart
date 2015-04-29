part of updroid_modal;

class UpDroidRunNodeModal extends UpDroidModal {
  List<Map> _nodeList;
  WebSocket _ws;

  UpDroidRunNodeModal(List<Map> nodeList, WebSocket ws) {
    _nodeList = nodeList;
    _ws = ws;

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

      ButtonElement nodeButton = _createButton('default', packageNode['node'], method: () {
        if (nodeArgs.value.isEmpty) {
          _ws.send('[[CATKIN_RUN]]' + JSON.encode([packageNode['package'], packageNode['node']]));
        } else {
          _ws.send('[[CATKIN_RUN]]' + JSON.encode([packageNode['package'], packageNode['node'], nodeArgs.value]));
        }
      });
      selectorWrap.children.add(nodeButton);
      selectorWrap.children.add(nodeArgs);
    });
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Cancel');
    _modalFooter.children.add(discard);
  }
}
