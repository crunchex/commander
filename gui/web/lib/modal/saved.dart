part of updroid_modal;

class UpDroidSavedModal extends UpDroidModal {
  UpDroidSavedModal() {
    _setupHead('Save Changes?');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement saveInput = new DivElement()
      ..id = 'save-input';

    // save input section
    HeadingElement askName = new HeadingElement.h3()
      ..text = "Enter Filename: ";
    InputElement input = new InputElement()
      ..id = "save-as-input"
      ..attributes['type'] = 'text';
    saveInput.children.addAll([askName, input]);
    _modalBody.children.add(saveInput);

    // overwrite warning
    DivElement warning = new DivElement()
      ..classes.add('hidden')
      ..id = 'warning';
    HeadingElement h4 = new HeadingElement.h4()
      ..text = 'Filename exists.';
    ButtonElement overwrite = new ButtonElement()
      ..text = "Overwrite?"
      ..attributes['type'] = 'button';
    warning.children.addAll([h4, overwrite]);
    _modalBody.children.add(warning);

    _buttonListeners.add(input.onKeyUp.listen((e) {
      var keyEvent = new KeyEvent.wrap(e);
      if (keyEvent.keyCode == KeyCode.ENTER) {
          _destroyModal();
        }
    }));
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Discard');
    discard.classes.add('modal-discard');
    ButtonElement save = _createButton('primary', 'Save', special: 'saveas');
    save
      ..classes.add('modal-save')
      ..id = 'save-as-commit';
    _modalFooter.children.addAll([save, discard]);
  }

  void hide() {
    this._destroyModal();
  }

}