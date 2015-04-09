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
part 'lib/modal/git_pass.dart';

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
    _buttonListeners = [];

    _modalBase = querySelector('.modal-base');

    _modalWrapper = querySelector('.modal-content');

    _modalHead = new DivElement()
      ..classes.add('modal-header');
    _modalBody = new DivElement()
      ..classes.add('modal-body');
    _modalFooter = new DivElement()
      ..classes.add('modal-footer');

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

  /// Returns ButtonElement for an 'X' close button at the modal corner.
  /// Optionally, a [method] can be passed in for additional logic besides
  /// modal destruction.
  ButtonElement _createClose({dynamic method}) {
    ButtonElement button = new ButtonElement()
     ..attributes['type'] = 'button'
     ..attributes['data-dismiss'] = 'modal'
     ..classes.add('close')
     ..append(new DocumentFragment.html('&times'));

    _buttonListeners.add(button.onClick.listen((e) {
      method();
      _destroyModal();
    }));

    return button;
  }

  /// Returns ButtonElement for a button of [type] with [text]. Optionally,
  /// a [method] can be passed in for additional logic besides modal destruction.
  ButtonElement _createButton(String type, String text, {dynamic method}) {
    ButtonElement button = new ButtonElement()
      ..classes.addAll(['btn', 'btn-$type'])
      ..text = text
      ..attributes['data-dismiss'] = 'modal';

    _buttonListeners.add(button.onClick.listen((e) {
      if (method != null) method();
      _destroyModal();
    }));
    return button;
  }
}
