part of updroid_modal;

class UpDroidGitPassModal extends UpDroidModal {
  UpDroidGitPassModal(StreamController<CommanderMessage> cs) {
    _createModal();
    _setupModal(cs);

    _modal = new Modal(_modalBase);
    _modal.show();
  }

  void _setupModal(StreamController<CommanderMessage> cs) {
    _modalBase.id = "git-pass";

    var closer = _createClose();
    var h3 = new Element.tag('h3');
    h3.text = ('Git Push to Remote');
    _modalHead.children.insert(0, closer);
    _modalHead.children.insert(1, h3);

    DivElement passInput = new DivElement();
    passInput.id = 'git-pass-input';

    // password input section
    var askPassword = new Element.tag('h3');
    askPassword.text = "Git needs your password: ";
    PasswordInputElement input = new InputElement(type:'password');
    input.id = "pass-input";
    passInput.children.addAll([askPassword, input]);

    _modalBody.children.add(passInput);

    // Footer
    var submit = _createButton('primary', 'Submit', method: () {
      cs.add(new CommanderMessage('CLIENT', 'GIT_PASSWORD', body: input.value));
    });
    _modalFooter.children.insert(0, submit);
  }
}