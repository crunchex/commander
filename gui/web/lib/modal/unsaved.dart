part of updroid_modal;

class UpDroidUnsavedModal extends UpDroidModal {
  UpDroidUnsavedModal () {
    _createModal();
    _setupModal();

    _modal = new Modal(querySelector('.modal-base'));
    _modal.show();
  }

  void _setupModal() {
    _modalBase.id = "unsaved";

    var closer = _createClose();
    var h3 = new Element.tag('h3');
    h3.text = ('Save Changes?');
    _modalHead.children.insert(0, closer);
    _modalHead.children.insert(1, h3);

    var p = new Element.p();
    p.text = "Unsaved changes detected.  Save these changes?";
    _modalBody.children.add(p);

    var discard = _createButton('warning', 'Discard');
    var save = _createButton('primary', 'Save');
    _modalFooter.children.insertAll(0, [save, discard]);
  }
}