part of updroid_modal;

class UpDroidUnsavedModal extends UpDroidModal {
  UpDroidUnsavedModal () {
    _initModal('Save Changes?');
    _setupModal();
    _showModal();
  }

  void _setupModal() {
    _modalBase.id = "unsaved";

    var p = new Element.p();
    p.text = "Unsaved changes detected.  Save these changes?";
    _modalBody.children.add(p);

    var discard = _createButton('warning', 'Discard');
    var save = _createButton('primary', 'Save');
    _modalFooter.children.insertAll(0, [save, discard]);
  }
}