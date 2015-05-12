part of updroid_modal;

class UpDroidWorkspaceModal extends UpDroidModal {
  List _refs = [];

  UpDroidWorkspaceModal() {
    _setupHead('Enter Workspace Name');
    _setupBody();
    _setupFooter();

    _showModal();
  }

  void _setupBody() {
    DivElement workspaceInput = new DivElement()
      ..id = 'workspace-input';

    // save input section
    HeadingElement askName = new HeadingElement.h3()
      ..text = "WorkspacePath /";
    InputElement input = new InputElement()
      ..id = "workspace-input"
      ..attributes['type'] = 'text';
    _refs.add(input);
    workspaceInput.children.addAll([askName, input]);
    _modalBody.children.add(workspaceInput);

    _buttonListeners.add(input.onKeyUp.listen((e) {
      var keyEvent = new KeyEvent.wrap(e);
      if (keyEvent.keyCode == KeyCode.ENTER) {
        _destroyModal();
      }
    }));
  }

  void _setupFooter() {
    ButtonElement discard = _createButton('warning', 'Discard');
    discard.classes.add('modal-discard');
    discard.text = "Cancel";
    ButtonElement save = _createButton('primary', 'Save');
    save.classes.add('modal-save');
    save.text = "Create";
    _refs.add(save);
    _modalFooter.children.addAll([save, discard]);
  }

  List passRefs() {
    return _refs;
  }
}