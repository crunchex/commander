library tab_view;

import 'dart:html';

import 'container_view.dart';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
class TabView extends ContainerView {
  LIElement extra;
  DivElement closeControlHitbox;

  TabView(int id, int col, String refName, String fullName, String shortName, UListElement navTabs, DivElement columnContent) :
  super(id, col, refName, fullName, shortName, navTabs, columnContent) {
    loadExternalCss('tabs/$refName/${shortName.toLowerCase()}.css');

    tabHandle
      ..id = 'tab-$refName-$id-handle'
      ..classes.addAll(['tab-handle', 'active']);

    tabHandleButton
      ..id = 'button-$refName-$id'
      ..href = '#tab-$refName-$id-container'
      ..dataset['toggle'] = 'tab'
      ..text = '$shortName-$id';

//    extra = new LIElement()
//      ..id = 'extra-$id'
//      ..classes.add('extra-menubar');
//    menus.children.add(extra);

    closeControlHitbox = new DivElement()
      ..title = 'Close'
      ..classes.add('close-control-hitbox');
    tabHandle.children.insert(0, closeControlHitbox);

    DivElement closeControl = new DivElement()
      ..classes.addAll(['close-control', 'glyphicons', 'glyphicons-remove-2']);
    closeControlHitbox.children.add(closeControl);
  }
}