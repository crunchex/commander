part of updroid_modal;

class UpDroidCreatePackageModal extends UpDroidModal {
  List _refs = [];
  var _doneHandler;

  UpDroidCreatePackageModal(doneHandler) {
    _doneHandler = doneHandler;

    _setupHead('Create Package');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement workspaceInput = new DivElement();
    HeadingElement promptName = new HeadingElement.h3();
    Input inputName = new InputElement();

//      ..attributes['type'] = 'text';

    workspaceInput.children.addAll([promptName, inputName]);
    _modalBody.children.add(workspaceInput);

    _buttonListeners.add(inputName.onKeyUp.listen((e) {
      if (e.keyCode == KeyCode.ENTER) {
        _doneHandler();
        _destroyModal();
      }
    }));
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Discard');
    discard.classes.add('modal-discard');
    discard.text = "Cancel";
    discard.onClick.listen((e) {
      _doneHandler();
      _destroyModal();
    });
    ButtonElement save = _createButton('primary', 'Save');
    save.classes.add('modal-save');
    save.text = "Create";
    save.onClick.listen((e) {
      _doneHandler();
      _destroyModal();
    });
    _refs.add(save);
    _modalFooter.children.addAll([save, discard]);
  }

  List passRefs() {
    return _refs;
  }
}