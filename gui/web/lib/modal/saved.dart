part of updroid_modal;

class UpDroidSavedModal extends UpDroidModal {
  UpDroidSavedModal () {
    _createModal();
    _setupModal();

    _modal = new Modal(_modalBase);
    _modal.show();
  }

  void _setupModal() {
    _modalBase.id = "save-as";

    var closer = _createClose();
    var h3 = new Element.tag('h3');
    h3.text = ('Save Changes?');
    _modalHead.children.insert(0, closer);
    _modalHead.children.insert(1, h3);

    DivElement saveInput = new DivElement();
    saveInput.id = 'save-input';

    // save input section
    var askName = new Element.tag('h3');
    askName.text = "Enter Filename: ";
    var input = new InputElement();
    input.id = "save-as-input";
    input.attributes['type'] = 'text';
    saveInput.children.addAll([askName, input]);

    // overwrite warning
    DivElement warning = new DivElement()
      ..classes.add('hidden')
      ..id = 'warning';
    var h4 = new Element.tag('h4');
    h4.text = 'Filename exists.';
    var overwrite = new ButtonElement();
    overwrite.text = "Overwrite?";
    overwrite.attributes['type'] = 'button';
    warning.children.addAll([h4, overwrite]);

    _modalBody.children.add(saveInput);
    _modalBody.children.add(warning);

    // Footer

    var discard = _createButton('warning', 'Discard');
    var save = _createButton('primary', 'Save');
    _modalFooter.children.insertAll(0, [save, discard]);
  }
}