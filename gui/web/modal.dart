library updroid_modal;

import 'dart:html';
// import 'dart:async';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
  DivElement _modalWrapper;
  DivElement _modalHead;
  DivElement _modalBody;
  DivElement _modalFooter;
  DivElement _modalBase;
  abstract class UpDroidModal {


  void createModal (String type) {
    // only checks one part of modal since all are created together
    if(_modalHead != null) {
      destroyModal();
    }

    _modalBase = querySelector('.modal-base');
    _modalWrapper = querySelector('.modal-content');
    _modalHead =  new DivElement();
    _modalHead.classes.add('modal-header');
    _modalBody = new DivElement();
    _modalBody.classes.add('modal-body');
    _modalFooter = new DivElement();
    _modalFooter.classes.add('modal-footer');
    _modalWrapper.children.insert(0, _modalHead);
    _modalWrapper.children.insert(1, _modalBody);
    _modalWrapper.children.insert(2, _modalFooter);

    if(type == "unsaved") {
      _modalBase.id = "unsaved";

      var closer = createClose();
      var h3 = new Element.tag('h3');
      h3.text = ('Save Changes?');
      _modalHead.children.insert(0, closer);
      _modalHead.children.insert(1, h3);

      var p = new Element.p();
      p.text = "Unsaved changes detected.  Save these changes?";
      _modalBody.children.add(p);

      var discard = createButton('discard');
      var save = createButton('save');
      _modalFooter.children.insertAll(0, [save, discard]);
    }

    else if(type == "saveAs") {
      _modalBase.id = "save-as";

      var closer = createClose();
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

      var discard = createButton('discard');
      var save = createButton('save');
      _modalFooter.children.insertAll(0, [save, discard]);

    }

    else if(type == "tabSelector") {
      _modalBase.id = "tab-selector";

      // Head
      var closer = createClose();
      var h3 = new Element.tag('h3');
      h3.text = ('Select Tab: ');
      _modalHead.children.insert(0, closer);
      _modalHead.children.insert(1, h3);

      // Body
      var selectorWrap = new DivElement();
      selectorWrap.id = "selector-wrapper";

      _modalBody.children.add(selectorWrap);
      var sEditor = new ButtonElement();
      sEditor
        ..text = 'Editor'
        ..id = 'select-editor'
        ..attributes['type'] = 'button'
        ..classes.addAll(['btn', 'btn-default']);
      var sConsole = new ButtonElement();
      sConsole
          ..text = 'Console'
        ..id = 'select-console'
        ..attributes['type'] = 'button'
        ..classes.addAll(['btn', 'btn-default']);
      var sVideo = new ButtonElement();
      sVideo
          ..text = 'Video'
        ..id = 'select-video'
        ..attributes['type'] = 'button'
        ..classes.addAll(['btn', 'btn-default']);
      selectorWrap.children.addAll([sEditor, sConsole, sVideo]);

      // Footer
      var discard = createButton('discard');
      _modalFooter.children.add(discard);
    }
  }

  void destroyModal() {
    _modalHead.remove();
    _modalBody.remove();
    _modalFooter.remove();
  }

  // Helper, creates 'X' close button

  Element createClose() {
    var button = new ButtonElement();
    button.attributes['type'] = 'button';
    button.attributes['data-dismiss'] = 'modal';
    button.classes.add('close');
    button.append(new DocumentFragment.html('&times'));
    return button;
  }

  ButtonElement createButton(String type) {
    var button = new ButtonElement();
    if(type == 'discard') {
      button.classes.addAll(['btn', 'btn-warning', 'modal-discard']);
      button.text = "Nope";
      button.attributes['data-dismiss'] = 'modal';
    }
    else if(type == 'save') {
      button.classes.addAll(['btn', 'btn-primary', 'modal-save']);
      button.text = "Save";
      button.attributes['data-dismis'] = 'modal';
    }
    return button;
  }
}