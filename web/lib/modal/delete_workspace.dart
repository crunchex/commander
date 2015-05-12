part of updroid_modal;

class UpDroidDeleteWorkspaceModal extends UpDroidModal {
  ButtonElement _deleteButton;

  UpDroidDeleteWorkspaceModal() {
    _setupHead('Delete Workspace?');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    ParagraphElement p = new ParagraphElement()
      ..text = "Workspace folder and all files will be deleted.";
    _modalBody.children.add(p);
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Discard');
    discard.classes.add('modal-discard');
    discard.text = "Cancel";
    ButtonElement save = _createButton('primary', 'Save');
    save.classes.add('modal-save');
    save.text = "Delete";
    _deleteButton = save;
    _modalFooter.children.addAll([save, discard]);
  }

  ButtonElement passRefs() {
    return _deleteButton;
  }
}