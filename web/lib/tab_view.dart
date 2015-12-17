library tab_view;

import 'dart:html';

import 'container_view.dart';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
class TabView extends ContainerView {
  LIElement extra;
  DivElement closeControlHitbox;

  TabView(int id, int col, String refName, String fullName, String shortName, UListElement navTabs, DivElement columnContent, {String icon: null}) :
  super(id, col, refName, fullName, shortName, navTabs, columnContent) {
    loadExternalCss('plugins/$refName/${shortName.toLowerCase()}.css');

    tabHandle
      ..id = 'tab-$refName-$id-handle';

    tabHandleButton
      ..id = 'button-$refName-$id'
      ..href = '#tab-$refName-$id-container'
      ..dataset['toggle'] = 'tab';

    if (icon != null) {
      tabHandle.classes.addAll(['tab-control', 'active']);

      SpanElement controlGlyph = new SpanElement()
        ..classes.addAll(['glyphicons', 'glyphicons-$icon']);
      tabHandleButton.children.add(controlGlyph);

      tabContent.classes.add('launcher-content');
    } else {
      tabHandle.classes.addAll(['tab-handle', 'active']);
      tabHandleButton.text = '$shortName-$id';

      closeControlHitbox = new DivElement()
        ..title = 'Close'
        ..classes.add('close-control-hitbox');
      tabHandle.children.insert(0, closeControlHitbox);

      DivElement closeControl = new DivElement()
        ..classes.addAll(['close-control', 'glyphicons', 'glyphicons-remove-2']);
      closeControlHitbox.children.add(closeControl);
    }
  }
}