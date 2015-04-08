part of updroid_modal;

class UpDroidOpenTabModal extends UpDroidModal {
  int _side;
  StreamController<CommanderMessage> _cs;

  UpDroidOpenTabModal(int side, StreamController<CommanderMessage> cs) {
    _side = side;
    _cs = cs;
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

    Element h3 = new Element.tag('h3')..text = ('Select Tab: ');
    _modalHead.children.insert(0, closer);
    _modalHead.children.insert(1, h3);

    // Body
    DivElement selectorWrap = new DivElement()..id = "selector-wrapper";

    _modalBody.children.add(selectorWrap);
    ButtonElement sEditor = new ButtonElement()
      ..text = 'Editor'
      ..id = 'select-editor'
      ..attributes['type'] = 'button'
      ..classes.addAll(['btn', 'btn-default']);

    ButtonElement sConsole = new ButtonElement()
      ..text = 'Console'
      ..id = 'select-console'
      ..attributes['type'] = 'button'
      ..classes.addAll(['btn', 'btn-default']);

    ButtonElement sVideo = new ButtonElement()
      ..text = 'Video'
      ..id = 'select-video'
      ..attributes['type'] = 'button'
      ..classes.addAll(['btn', 'btn-default']);

    selectorWrap.children.addAll([sEditor, sConsole, sVideo]);

    // Footer
    ButtonElement discard = _createButton('warning', 'Discard');
    _modalFooter.children.add(discard);

    _setUpListeners(closer, sEditor, sConsole, sVideo, discard);
  }

  _setUpListeners(ButtonElement closer, ButtonElement sEditor,
      ButtonElement sConsole, ButtonElement sVideo, ButtonElement discard) {
    _buttonListeners.add(closer.onClick.listen((e) {
      _destroyModal();
    }));

    _buttonListeners.add(sEditor.onClick.listen((e) {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB',
          body: '${_side}_UpDroidEditor'));
      _destroyModal();
    }));

    _buttonListeners.add(sConsole.onClick.listen((e) {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB',
          body: '${_side}_UpDroidConsole'));
      _destroyModal();
    }));

    _buttonListeners.add(sVideo.onClick.listen((e) {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB',
          body: '${_side}_UpDroidCamera'));
      _destroyModal();
    }));

    _buttonListeners.add(discard.onClick.listen((e) {
      _destroyModal();
    }));
  }
}
