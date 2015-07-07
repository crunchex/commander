part of updroid_modal;

class UpDroidUnsavedModal extends UpDroidModal {
  ButtonElement discardButton;
  ButtonElement saveButton;

  UpDroidUnsavedModal() {
    _setupHead('Save Changes?');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    ParagraphElement p = new ParagraphElement()
      ..text = "Unsaved changes detected.  Save these changes?";
    _modalBody.children.add(p);
  }

  void _setupFooter() {
    discardButton = _createButton('warning', 'Discard');
    discardButton.classes.add('modal-discard');
    saveButton = _createButton('primary', 'Save');
    saveButton.classes.add('modal-save');
    _modalFooter.children.addAll([saveButton, discardButton]);
  }

  void hide() {
    this._destroyModal();
  }
}