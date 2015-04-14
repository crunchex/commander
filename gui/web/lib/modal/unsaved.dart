part of updroid_modal;

class UpDroidUnsavedModal extends UpDroidModal {
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
    ButtonElement discard = _createButton('warning', 'Discard');
    discard.classes.add('modal-discard');
    ButtonElement save = _createButton('primary', 'Save');
    save.classes.add('modal-save');
    _modalFooter.children.addAll([save, discard]);
  }
}