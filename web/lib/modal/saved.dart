part of updroid_modal;

class UpDroidSavedModal extends UpDroidModal {
  InputElement input;
  CheckboxInputElement makeExec;
  ButtonElement discardButton;
  ButtonElement saveButton;

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
    input = new InputElement()
      ..id = "save-as-input"
      ..attributes['type'] = 'text';

    ParagraphElement note = new ParagraphElement()
      ..id = "note"
      ..text = "File will be saved to current selected path in Explorer";

    BRElement line = new BRElement();

    // executable option
    makeExec = new CheckboxInputElement()
      ..checked = false
      ..id = "make-exec";
    HeadingElement h5 = new HeadingElement.h5()
      ..id = 'exec-flag'
      ..text = "Make Executable";

    saveInput.children.addAll([askName, input, line, makeExec, h5, note]);

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
  }

  void _setupFooter() {
    discardButton = _createButton('warning', 'Discard');
    discardButton.classes.add('modal-discard');
    saveButton = _createButton('primary', 'Save', special: 'saveas');
    saveButton
      ..classes.add('modal-save')
      ..id = 'save-as-commit';
    _modalFooter.children.addAll([saveButton, discardButton]);
  }

  void hide() {
    this._destroyModal();
  }

}