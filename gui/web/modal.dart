library updroid_modal;

import 'dart:html';
import 'dart:async';

import 'package:bootjack/bootjack.dart';

import 'lib/updroid_message.dart';

part 'lib/modal/unsaved.dart';
part 'lib/modal/saved.dart';
part 'lib/modal/open_tab.dart';
part 'lib/modal/build_results.dart';
part 'lib/modal/run_node.dart';

/// [UpDroidModal] contains methods to generate [Element]s that make up
/// a modal in the UpDroid Commander GUI.
abstract class UpDroidModal {
  DivElement _modalWrapper;
  DivElement _modalHead;
  DivElement _modalBody;
  DivElement _modalFooter;
  DivElement _modalBase;

  List<StreamSubscription<Event>> _buttonListeners;

  Modal _modal;

  void _createModal() {
    // only checks one part of modal since all are created together
    if (_modalHead != null) {
      _destroyModal();
    }

    _modalBase = querySelector('.modal-base');
    _modalWrapper = querySelector('.modal-content');
    _modalHead = new DivElement();
    _modalHead.classes.add('modal-header');
    _modalBody = new DivElement();
    _modalBody.classes.add('modal-body');
    _modalFooter = new DivElement();
    _modalFooter.classes.add('modal-footer');
    _modalWrapper.children.insert(0, _modalHead);
    _modalWrapper.children.insert(1, _modalBody);
    _modalWrapper.children.insert(2, _modalFooter);
  }

  void _destroyModal() {
    _buttonListeners.forEach((e) {
      e.cancel();
    });

    _modal.hide();

    _modalHead.remove();
    _modalBody.remove();
    _modalFooter.remove();
  }

  // Helper, creates 'X' close button

  Element _createClose() {
    var button = new ButtonElement();
    button.attributes['type'] = 'button';
    button.attributes['data-dismiss'] = 'modal';
    button.classes.add('close');
    button.append(new DocumentFragment.html('&times'));
    return button;
  }

  ButtonElement _createButton(String type) {
    var button = new ButtonElement();
    if (type == 'discard') {
      button.classes.addAll(['btn', 'btn-warning', 'modal-discard']);
      button.text = "Nope";
      button.attributes['data-dismiss'] = 'modal';
    } else if (type == 'save') {
      button.classes.addAll(['btn', 'btn-primary', 'modal-save']);
      button.text = "Save";
      button.attributes['data-dismiss'] = 'modal';
    } else if (type == 'okay') {
      button.classes.addAll(['btn', 'btn-primary', 'modal-save']);
      button.text = "Okay";
      button.attributes['data-dismiss'] = 'modal';
    }
    return button;
  }
}
