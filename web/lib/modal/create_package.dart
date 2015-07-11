part of updroid_modal;

class UpDroidCreatePackageModal extends UpDroidModal {
  InputElement inputName, inputDependencies;

  Function _doneHandler;

  UpDroidCreatePackageModal(Function doneHandler) {
    _doneHandler = doneHandler;

    _setupHead('Create Package');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement name = new DivElement();
    HeadingElement promptName = new HeadingElement.h3()
      ..text = 'Package Name:';
    inputName = new InputElement()
      ..attributes['type'] = 'text';
    name.children.addAll([promptName, inputName]);
    _modalBody.children.add(name);

    DivElement dependencies = new DivElement();
    HeadingElement promptDependencies = new HeadingElement.h3()
      ..text = 'Package Dependencies:';
    inputDependencies = new InputElement()
      ..placeholder = 'dep1, dep2, ...'
      ..attributes['type'] = 'text';
    dependencies.children.addAll([promptDependencies, inputDependencies]);
    _modalBody.children.add(dependencies);

    _buttonListeners.add(inputName.onKeyUp.listen((e) {
      if (e.keyCode == KeyCode.ENTER) {
        _doneHandler();
        _destroyModal();
      }
    }));

    _buttonListeners.add(inputDependencies.onKeyUp.listen((e) {
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
    _modalFooter.children.addAll([save, discard]);
  }
}