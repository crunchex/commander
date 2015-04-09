part of updroid_modal;

class UpDroidRunNodeModal extends UpDroidModal {
  List<Map> _nodeList;
  WebSocket _ws;

  UpDroidRunNodeModal(List<Map> nodeList, WebSocket ws) {
    _nodeList = nodeList;
    _ws = ws;

    _initModal('Available Nodes');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement selectorWrap = new DivElement()
      ..id = "selector-wrapper";
    _modalBody.children.add(selectorWrap);

    _nodeList.forEach((Map packageNode) {
      ButtonElement nodeButton = _createButton('default', packageNode['node'], method: () {
        _ws.send('[[CATKIN_RUN]]' + '${packageNode['package']}++${packageNode['node']}');
      });
      selectorWrap.children.add(nodeButton);
    });
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Discard');
    _modalFooter.children.add(discard);
  }
}
