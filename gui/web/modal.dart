library updroid_modal;

import 'dart:html';
import 'dart:async';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
abstract class UpDroidModal {
  DivElement _modalWrapper = querySelector('.modal_content');
  DivElement _modalHead;
  DivElement _modalBody;
  DivElement _modalFooter;
  DivElement _modalBase = querySelector('.modal-base');


  void createModal (String type) {
    _modalHead =  new DivElement();
    _modalBody = new DivElement();
    _modalFooter = new DivElement();
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

    }

    else if(type == "tabSelector") {
      _modalBase.id = "tab-selector";

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