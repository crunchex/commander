part of updroid_modal;

class UpDroidOpenTabModal extends UpDroidModal {
  int _side;
  StreamController<CommanderMessage> _cs;

  UpDroidOpenTabModal (int side, StreamController<CommanderMessage> cs) {
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
    var closer = _createClose();
    var h3 = new Element.tag('h3');
    h3.text = ('Select Tab: ');
    _modalHead.children.insert(0, closer);
    _modalHead.children.insert(1, h3);

    // Body
    var selectorWrap = new DivElement();
    selectorWrap.id = "selector-wrapper";

    _modalBody.children.add(selectorWrap);
    var sEditor = new ButtonElement();
    sEditor
      ..text = 'Editor'
      ..id = 'select-editor'
      ..attributes['type'] = 'button'
      ..classes.addAll(['btn', 'btn-default']);
    _buttonListeners.add(sEditor.onClick.listen((e) {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_side}_UpDroidEditor'));
      _destroyModal();
    }));

    var sConsole = new ButtonElement();
    sConsole
      ..text = 'Console'
      ..id = 'select-console'
      ..attributes['type'] = 'button'
      ..classes.addAll(['btn', 'btn-default']);
    _buttonListeners.add(sConsole.onClick.listen((e) {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_side}_UpDroidConsole'));
      _destroyModal();
    }));

    var sVideo = new ButtonElement();
    sVideo
      ..text = 'Video'
      ..id = 'select-video'
      ..attributes['type'] = 'button'
      ..classes.addAll(['btn', 'btn-default']);
    _buttonListeners.add(sVideo.onClick.listen((e) {
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_side}_UpDroidCamera'));
      _destroyModal();
    }));

    selectorWrap.children.addAll([sEditor, sConsole, sVideo]);

    // Footer
    var discard = _createButton('discard');
    _modalFooter.children.add(discard);
  }
}