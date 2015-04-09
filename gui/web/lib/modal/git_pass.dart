part of updroid_modal;

class UpDroidGitPassModal extends UpDroidModal {
  UpDroidGitPassModal(StreamController<CommanderMessage> cs) {
    _initModal('Git Push to Remote');
    _setupModal(cs);
    _showModal();
  }

  void _setupModal(StreamController<CommanderMessage> cs) {
    _modalBase.id = "git-pass";

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