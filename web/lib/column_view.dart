library column_view;

import 'dart:html';
import 'dart:async';

class ColumnView {
  static Future<ColumnView> createColumnView(int id, int width) {
    Completer c = new Completer();
    c.complete(new ColumnView(id, width));
    return c.future;
  }

  int id, width;
  DivElement columnContent;
  AnchorElement controlButton, maximizeButton;

  DivElement _rowMain;

  ColumnView(this.id, this.width) {
    _rowMain = querySelector('#row-main');

    columnContent = new DivElement()
    ..id = 'column-$id'
    ..classes.addAll(['col-xs-$width', 'column-content']);
    _rowMain.children.add(columnContent);

    maximizeButton = new AnchorElement()
      ..id = 'column-$id-maximize'
      ..classes.add('maximize-button');
    columnContent.children.add(maximizeButton);

    SpanElement maximizeGlyph = new SpanElement();
    if (width == 5) {
      maximizeGlyph.classes.addAll(['glyphicons', 'glyphicons-resize-full']);
    } else if (width == 10) {
      maximizeGlyph.classes.addAll(['glyphicons', 'glyphicons-resize-small']);
    }
    maximizeButton.children.add(maximizeGlyph);

    UListElement navTabs = new UListElement()
    ..classes.addAll(['nav', 'nav-tabs'])
    ..attributes['role'] = 'tablist';
    columnContent.children.add(navTabs);

    LIElement controlLi = new LIElement();
    if (width == 5 || width == 10) navTabs.children.add(controlLi);

    controlButton = new AnchorElement()
    ..id = 'column-$id-new'
    ..classes.add('new-tab-button');
    controlLi.children.add(controlButton);

    SpanElement controlGlyph = new SpanElement();
    if (width == 2) {
      controlGlyph.classes.addAll(['glyphicons', 'glyphicons-chevron-down']);
    } else if (width == 5 || width == 10) {
      controlGlyph.classes.addAll(['glyphicons', 'glyphicons-plus']);
    }
    controlButton.children.add(controlGlyph);

    DivElement tabContent = new DivElement()
    ..id = 'col-$id-tab-content'
    ..classes.add('tab-content');
    columnContent.children.add(tabContent);
  }

  void cleanUp() {
    // TODO: make sure child nodes are all cleaned up.
    columnContent.innerHtml = '';
    columnContent = null;
  }
}