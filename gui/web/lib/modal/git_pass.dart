part of updroid_modal;

class UpDroidGitPassModal extends UpDroidModal {
  StreamController<CommanderMessage> _cs;
  InputElement _input;

  UpDroidGitPassModal(StreamController<CommanderMessage> cs) {
    _cs = cs;

    _setupHead('Git Push to Remote');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement passInput = new DivElement();
    passInput.id = 'git-pass-input';

    // password input section
    ParagraphElement askPassword = new ParagraphElement();
    askPassword.text = "Git needs your password: ";
    _input = new InputElement(type:'password')
      ..id = "pass-input";
    passInput.children.addAll([askPassword, _input]);

    _modalBody.children.add(passInput);
  }

  void _setupFooter() {
    ButtonElement submit = _createButton('primary', 'Submit', method: () {
      _cs.add(new CommanderMessage('CLIENT', 'GIT_PASSWORD', body: _input.value));
    });
    _modalFooter.children.insert(0, submit);
  }
}