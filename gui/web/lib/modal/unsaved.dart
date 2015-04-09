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
    var discard = _createButton('warning', 'Discard');
    var save = _createButton('primary', 'Save');
    _modalFooter.children.insertAll(0, [save, discard]);
  }
}