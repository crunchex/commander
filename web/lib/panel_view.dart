library panel_view;

import 'dart:html';

import 'container_view.dart';

/// [PanelView] contains generic methods and fields that make up the visual
/// side of a Panel.
class PanelView extends ContainerView {
  PanelView(int id, int col, String refName, String fullName, String shortName, UListElement navTabs, DivElement columnContent, {String icon: null}) :
  super(id, col, refName, fullName, shortName, navTabs, columnContent) {
    loadExternalCss('plugins/$refName/${shortName.toLowerCase()}.css');

    tabHandle
      ..id = 'tab-$refName-$id-handle';

    tabHandleButton
      ..id = 'button-$refName-$id'
      ..href = '#tab-$refName-$id-container'
      ..dataset['toggle'] = 'tab';

    if (icon != null) {
      tabHandle.classes.addAll(['panel-control', 'active']);

      SpanElement controlGlyph = new SpanElement()
        ..classes.addAll(['glyphicons', 'glyphicons-$icon']);
      tabHandleButton.children.add(controlGlyph);

      tabContent.classes.add('launcher-content');
    } else {
      tabHandle.classes.addAll(['tab-handle', 'panel-handle', 'active']);
      tabHandleButton.text = '$shortName';
    }
  }
}