library updroid_modal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'package:bootjack/bootjack.dart';

import '../mailbox.dart';

part 'unsaved.dart';
part 'saved.dart';
part 'open_tab.dart';
part 'build_results.dart';
part 'run_node.dart';
part 'git_pass.dart';
part 'new_workspace.dart';
part 'delete_workspace.dart';
part 'create_package.dart';

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

  void _setupHead(String heading) {
    _buttonListeners = [];

    _modalBase = querySelector('.modal-base');
    _modalWrapper = querySelector('.modal-content');

    _modalHead = new DivElement()
      ..classes.add('modal-header');

    ButtonElement closer = _createClose();
    _modalHead.children.insert(0, closer);

    HeadingElement h3 = new HeadingElement.h3()
      ..text = heading;
    _modalHead.children.insert(1, h3);

    _modalWrapper.children.insert(0, _modalHead);

    _modalBody = new DivElement()
      ..classes.add('modal-body');
    _modalWrapper.children.insert(1, _modalBody);

    _modalFooter = new DivElement()
      ..classes.add('modal-footer');
    _modalWrapper.children.insert(2, _modalFooter);
  }

  void _showModal() {
    _modal = new Modal(_modalBase);
    _modal.show();
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
      if (method != null) method();
      _destroyModal();
    }));

    return button;
  }

  /// Returns ButtonElement for a button of [type] with [text]. Optionally,
  /// a [method] can be passed in for additional logic besides modal destruction.
  ButtonElement _createButton(String type, String text, {dynamic method, String special}) {
    ButtonElement button = new ButtonElement()
      ..classes.addAll(['btn', 'btn-$type'])
      ..text = text;
    if (text == 'discard') button.attributes['data-dismiss'] = 'modal';

    if (special == null) {
      _buttonListeners.add(button.onClick.listen((e) {
        if (method != null) method();
        _destroyModal();
      }));
    }

    return button;
  }
}
