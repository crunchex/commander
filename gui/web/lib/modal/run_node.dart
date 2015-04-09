part of updroid_modal;

class UpDroidRunNodeModal extends UpDroidModal {
  List<Map> _nodeList;
  WebSocket _ws;

  UpDroidRunNodeModal(List<Map> nodeList, WebSocket ws) {
    _nodeList = nodeList;
    _ws = ws;

    _createModal();
    _setupModal();

    _modal = new Modal(querySelector('.modal-base'));
    _modal.show();
  }

  void _setupModal() {
    _modalBase.id = "tab-selector";

    // Head
    Element closer = _createClose();

    Element h3 = new Element.tag('h3')..text = ('Available Nodes');
    _modalHead.children.insert(0, closer);
    _modalHead.children.insert(1, h3);

    // Body
    DivElement selectorWrap = new DivElement()..id = "selector-wrapper";

    _modalBody.children.add(selectorWrap);

    _nodeList.forEach((Map packageNode) {
      ButtonElement nodeButton = _createButton('default', packageNode['node'], method: () {
        _ws.send('[[CATKIN_RUN]]' + '${packageNode['package']}++${packageNode['node']}');
      });
      selectorWrap.children.add(nodeButton);
    });

    // Footer
    ButtonElement discard = _createButton('warning', 'Discard');
    _modalFooter.children.add(discard);
  }
}
