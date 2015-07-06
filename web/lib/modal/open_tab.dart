part of updroid_modal;

class UpDroidOpenTabModal extends UpDroidModal {
  int _side;
  Function _openTab;

  UpDroidOpenTabModal(int side, Function openTab) {
    _side = side;
    _openTab = openTab;

    _setupHead('Select Tab: ');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement selectorWrap = new DivElement()
      ..id = "selector-wrapper";
    _modalBody.children.add(selectorWrap);

    ButtonElement sEditor = _createButton('default', 'Editor')..onClick.listen((_) => _openTab(_side, 'UpDroidEditor'));
    ButtonElement sConsole = _createButton('default', 'Console')..onClick.listen((_) => _openTab(_side, 'UpDroidConsole'));
    ButtonElement sCamera = _createButton('default', 'Camera')..onClick.listen((_) => _openTab(_side, 'UpDroidCamera'));
    ButtonElement sTeleop = _createButton('default', 'Teleop')..onClick.listen((_) => _openTab(_side, 'UpDroidTeleop'));
    selectorWrap.children.addAll([sEditor, sConsole, sCamera, sTeleop]);
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Cancel');
    _modalFooter.children.add(discard);
  }
}
