library column_view;

import 'dart:html';
import 'dart:async';

import 'column_controller.dart';

class ColumnView {
  static Future<ColumnView> createColumnView(int id, ColumnState state) {
    Completer c = new Completer();
    c.complete(new ColumnView(id, state));
    return c.future;
  }

  static final Map width = {
    ColumnState.MAXIMIZED: 9,
    ColumnState.NORMAL: 5,
    ColumnState.MINIMIZED: 1
  };

  int id;
  ColumnState state;
  DivElement columnContent;
  AnchorElement controlButton, maximizeButton;

  SpanElement _maximizeGlyph;
  DivElement _rowMain;

  ColumnView(this.id, this.state) {
    _rowMain = querySelector('#row-main');

    columnContent = new DivElement()
    ..id = 'column-$id'
    ..classes.addAll(['col-xs-${width[state]}', 'column-content']);
    _rowMain.children.add(columnContent);

    maximizeButton = new AnchorElement()
      ..id = 'column-$id-maximize'
      ..classes.add('maximize-button');
    columnContent.children.add(maximizeButton);

    _maximizeGlyph = new SpanElement()
    ..classes.add('glyphicons')
    ..classes.add((state == ColumnState.MAXIMIZED) ? 'glyphicons-resize-small' : 'glyphicons-resize-full');
    maximizeButton.children.add(_maximizeGlyph);

    UListElement navTabs = new UListElement()
    ..classes.addAll(['nav', 'nav-tabs'])
    ..attributes['role'] = 'tablist';
    columnContent.children.add(navTabs);

    LIElement controlLi = new LIElement();
    navTabs.children.add(controlLi);

    controlButton = new AnchorElement( )
    ..id = 'column-$id-new'
    ..classes.add('new-tab-button');
    controlLi.children.add(controlButton);

    SpanElement controlGlyph = new SpanElement()
    ..classes.addAll(['glyphicons', 'glyphicons-plus']);
    controlButton.children.add(controlGlyph);

    DivElement tabContent = new DivElement()
    ..id = 'col-$id-tab-content'
    ..classes.add('tab-content');
    columnContent.children.add(tabContent);
  }

  void maximize() {
    columnContent.classes.removeAll(['col-xs-1', 'col-xs-5', 'col-xs-10']);
    columnContent.classes.add('col-xs-9');

    columnContent.style.width = (querySelector('#column-0').clientWidth <= 200) ? 'calc(100% - 200px)' : '';
//    columnContent.style.visibility = 'visible';
    _maximizeGlyph.classes.remove('glyphicons-resize-full');
    _maximizeGlyph.classes.add('glyphicons-resize-small');
  }

  void normalize() {
    columnContent.classes.removeAll(['col-xs-1', 'col-xs-9', 'col-xs-10']);
    columnContent.classes.add('col-xs-5');

    columnContent.style.width = '';
//    columnContent.style.visibility = 'visible';
    _maximizeGlyph.classes.remove('glyphicons-resize-small');
    _maximizeGlyph.classes.add('glyphicons-resize-full');
  }

  void minimize() {
    print('minimizing: ${columnContent.id}');
    columnContent.classes.removeAll(['col-xs-5', 'col-xs-9', 'col-xs-10']);
    columnContent.classes.add('col-xs-1');

    columnContent.style.width = '';
//    columnContent.style.visibility = 'hidden';
    _maximizeGlyph.classes.remove('glyphicons-resize-small');
    _maximizeGlyph.classes.add('glyphicons-resize-full');
  }

  void cleanUp() {
    // TODO: make sure child nodes are all cleaned up.
    columnContent.innerHtml = '';
    columnContent = null;
  }
}