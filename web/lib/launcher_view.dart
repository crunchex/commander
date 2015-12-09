library launcher_view;

import 'dart:html';

import 'container_view.dart';

/// [LauncherView] contains generic methods and fields that make up the visual
/// side of a Launcher.
class LauncherView extends ContainerView {
  LauncherView(int id, int col, String refName, String fullName, String shortName, UListElement navTabs, DivElement columnContent) :
  super(id, col, refName, fullName, shortName, navTabs, columnContent) {
    loadExternalCss('tabs/$refName/${shortName.toLowerCase()}.css');

    tabHandle
      ..id = 'tab-$refName-$id-handle'
      ..classes.addAll(['tab-control', 'active']);

    tabHandleButton
      ..id = 'button-$refName-$id'
      ..href = '#tab-$refName-$id-container'
      ..dataset['toggle'] = 'tab';

    SpanElement controlGlyph = new SpanElement()
      ..classes.addAll(['glyphicons', 'glyphicons-plus']);
    tabHandleButton.children.add(controlGlyph);

    tabContent.classes.add('launcher-content');
  }
}