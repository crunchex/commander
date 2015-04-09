part of updroid_modal;

class UpDroidOpenTabModal extends UpDroidModal {
  int _side;
  StreamController<CommanderMessage> _cs;

  UpDroidOpenTabModal(int side, StreamController<CommanderMessage> cs) {
    _side = side;
    _cs = cs;

    _initModal('Select Tab: ');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement selectorWrap = new DivElement()
      ..id = "selector-wrapper";
    _modalBody.children.add(selectorWrap);

    ButtonElement sEditor = _createButton('default', 'Editor', method: () {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_side}_UpDroidEditor'));
    });

    ButtonElement sConsole = _createButton('default', 'Console', method: () {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_side}_UpDroidConsole'));
    });

    ButtonElement sCamera = _createButton('default', 'Camera', method: () {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_side}_UpDroidCamera'));
    });

    selectorWrap.children.addAll([sEditor, sConsole, sCamera]);
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Discard');
    _modalFooter.children.add(discard);
  }
}
