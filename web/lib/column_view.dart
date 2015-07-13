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
  UListElement _navTabs;
  DivElement _rowMain, _tabContent;

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

    _navTabs = new UListElement()
    ..classes.addAll(['nav', 'nav-tabs'])
    ..attributes['role'] = 'tablist';
    columnContent.children.add(_navTabs);

    LIElement controlLi = new LIElement();
    _navTabs.children.add(controlLi);

    controlButton = new AnchorElement( )
    ..id = 'column-$id-new'
    ..classes.add('new-tab-button');
    controlLi.children.add(controlButton);

    SpanElement controlGlyph = new SpanElement()
    ..classes.addAll(['glyphicons', 'glyphicons-plus']);
    controlButton.children.add(controlGlyph);

    _tabContent = new DivElement()
    ..id = 'col-$id-tab-content'
    ..classes.add('tab-content');
    columnContent.children.add(_tabContent);
  }

  void maximize() {
    columnContent.classes.removeAll(['col-xs-1', 'col-xs-5', 'col-xs-10']);
    columnContent.classes.add('col-xs-9');

    // Responsive width minus the other columns that are currently fixed;
    columnContent.style.width = 'calc(100% - 270px)';

    _maximizeGlyph.classes.remove('glyphicons-resize-full');
    _maximizeGlyph.classes.add('glyphicons-resize-small');

    _showTabsAndContent();
  }

  void normalize() {
    columnContent.classes.removeAll(['col-xs-1', 'col-xs-9', 'col-xs-10']);
    columnContent.classes.add('col-xs-5');

    // Responsive width minus half the panel column.
    columnContent.style.width = 'calc(50% - 110px)';

    _maximizeGlyph.classes.remove('glyphicons-resize-small');
    _maximizeGlyph.classes.add('glyphicons-resize-full');

    _showTabsAndContent();
  }

  void minimize() {
    columnContent.classes.removeAll(['col-xs-5', 'col-xs-9', 'col-xs-10']);
    columnContent.classes.add('col-xs-1');

    // Fixed-width.
    columnContent.style.width = '50px';

    _maximizeGlyph.classes.remove('glyphicons-resize-small');
    _maximizeGlyph.classes.add('glyphicons-resize-full');

    _hideTabsAndContent();
  }

  /// Restores content and removes all the "hidden" styling effects.
  void _showTabsAndContent() {
    // TODO: move these styles to CSS and use classes instead.
    _navTabs.style.visibility = 'visible';

    _navTabs.children.getRange(1, _navTabs.children.length)
    .forEach((e) => e.children
    .forEach((e) => e.style.display = ''));

    _tabContent.children.forEach((e) => e.style.display = '');
    _tabContent.style.backgroundColor = '';
  }

  /// Hides content and tacks on additional "hidden" styling effects.
  void _hideTabsAndContent() {
    // TODO: move these styles to CSS and use classes instead.
    _navTabs.style.visibility = 'hidden';

    _navTabs.children.getRange(1, _navTabs.children.length)
    .forEach((e) => e.children
    .forEach((e) => e.style.display = 'none'));

    _tabContent.children.forEach((e) => e.style.display = 'none');
    _tabContent.style.backgroundColor = '#eaecec';
  }

  void cleanUp() {
    // TODO: make sure child nodes are all cleaned up.
    columnContent.innerHtml = '';
    columnContent = null;
  }
}