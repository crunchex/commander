library panel_view;

import 'dart:html';

import 'container_view.dart';

/// [PanelView] contains generic methods and fields that make up the visual
/// side of a Panel.
class PanelView extends ContainerView {
  PanelView(int id, int col, String refName, String fullName, String shortName, UListElement navTabs, DivElement columnContent) :
  super(id, col, refName, fullName, shortName, navTabs, columnContent) {
    loadExternalCss('plugins/$refName/${shortName.toLowerCase()}.css');

    tabHandle
      ..id = 'tab-$refName-$id-handle'
      ..classes.addAll(['tab-handle', 'panel-handle', 'active']);

    tabHandleButton
      ..id = 'button-$refName-$id'
      ..href = '#tab-$refName-$id-container'
      ..dataset['toggle'] = 'tab'
      ..text = '$shortName';
  }
}