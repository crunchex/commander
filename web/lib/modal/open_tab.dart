part of updroid_modal;

class UpDroidOpenTabModal extends UpDroidModal {
  int _side;
  StreamController<CommanderMessage> _cs;

  UpDroidOpenTabModal(int side, StreamController<CommanderMessage> cs) {
    _side = side;
    _cs = cs;

    _setupHead('Select Tab: ');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement selectorWrap = new DivElement()
      ..id = "selector-wrapper";
    _modalBody.children.add(selectorWrap);

    ButtonElement sEditor = _createButton('default', 'Editor', method: () {
      _cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${_side}_UpDroidEditor'));
    });

    ButtonElement sConsole = _createButton('default', 'Console', method: () {
      _cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${_side}_UpDroidConsole'));
    });

    ButtonElement sCamera = _createButton('default', 'Camera', method: () {
      _cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${_side}_UpDroidCamera'));
    });

    ButtonElement sTeleop = _createButton('default', 'Teleop', method: () {
      _cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${_side}_UpDroidTeleop'));
    });

    selectorWrap.children.addAll([sEditor, sConsole, sCamera, sTeleop]);
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Cancel');
    _modalFooter.children.add(discard);
  }
}
