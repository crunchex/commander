library panel_column_view;

import 'dart:async';

import 'column_view.dart';

class PanelColumnView extends ColumnView {
  static Future<PanelColumnView> createPanelColumnView(int id) {
    Completer c = new Completer();
    c.complete(new PanelColumnView(id));
    return c.future;
  }

  PanelColumnView(int id) : super(id) {
    columnContent.classes.addAll(['col-xs-2', 'column-content']);
  }

  void cleanUp() {
    // Empty out nav-tabs.
    columnContent.children.first.innerHtml = '';

    // Empty the tab content.
    tabContent.innerHtml = '';
  }
}