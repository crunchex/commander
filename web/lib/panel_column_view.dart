library panel_column_view;

import 'dart:async';
import 'dart:html';

import 'column_view.dart';

class PanelColumnView extends ColumnView {
  static Future<PanelColumnView> createPanelColumnView(int id) {
    Completer c = new Completer();
    c.complete(new PanelColumnView(id));
    return c.future;
  }

  AnchorElement controlButton;

  PanelColumnView(int id) : super(id) {
    columnContent.classes.addAll(['col-xs-2', 'column-content']);

    LIElement controlLi = new LIElement()
      ..classes.add('panel-control');
    navTabs.children.add(controlLi);

    controlButton = new AnchorElement( )
      ..id = 'column-$id-new'
      ..classes.add('new-panel-button');
    controlLi.children.add(controlButton);

    SpanElement controlGlyph = new SpanElement()
      ..classes.addAll(['glyphicons', 'glyphicons-chevron-down']);
    controlButton.children.add(controlGlyph);
  }

  void cleanUp() {
    // Empty out nav-tabs.
    columnContent.children.first.innerHtml = '';

    // Empty the tab content.
    tabContent.innerHtml = '';
  }
}