part of updroid_modal;

class UpDroidRunNodeModal extends UpDroidModal {
  List<Map> _nodeList;
  WebSocket _ws;

  UpDroidRunNodeModal(List<Map> nodeList, WebSocket ws) {
    _nodeList = nodeList;
    _ws = ws;
    _buttonListeners = [];

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
      ButtonElement nodeButton = new ButtonElement()
        ..text = packageNode['node']
        ..attributes['type'] = 'button'
        ..classes.addAll(['btn', 'btn-default']);

      _buttonListeners.add(nodeButton.onClick.listen((e) {
        _ws.send('[[CATKIN_RUN]]' + '${packageNode['package']}++${packageNode['node']}');
        _destroyModal();
      }));

      selectorWrap.children.add(nodeButton);
    });

    // Footer
    ButtonElement discard = _createButton('warning', 'Discard');
    _modalFooter.children.add(discard);

    _setUpListeners(closer, discard);
  }

  _setUpListeners(ButtonElement closer, ButtonElement discard) {
    _buttonListeners.add(closer.onClick.listen((e) {
      _destroyModal();
    }));

    _buttonListeners.add(discard.onClick.listen((e) {
      _destroyModal();
    }));
  }
}
