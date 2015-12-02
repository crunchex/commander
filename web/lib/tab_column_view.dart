library tab_column_view;

import 'dart:html';
import 'dart:async';

import 'column_view.dart';
import 'column_controller.dart';

class TabColumnView extends ColumnView {
  static Future<TabColumnView> createTabColumnView(int id, ColumnState state) {
    Completer c = new Completer();
    c.complete(new TabColumnView(id, state));
    return c.future;
  }

  static final Map width = {
    ColumnState.MAXIMIZED: 9,
    ColumnState.NORMAL: 5,
    ColumnState.MINIMIZED: 1
  };

  ColumnState state;
  AnchorElement controlButton, maximizeButton;
  bool hideMinimizedCompletely;

  SpanElement _maximizeGlyph;

  TabColumnView(int id, this.state) : super(id) {
    // This controls whether or not a minimized column is hidden completely,
    // or stays visible as a col-xs-1 stub.
    // We may eventually decide to remove this option and a lot of code below.
    hideMinimizedCompletely = true;

    columnContent.classes.addAll(['col-xs-${width[state]}', 'column-content']);

    maximizeButton = new AnchorElement()
      ..id = 'column-$id-maximize'
      ..classes.add('maximize-button');
    columnContent.children.add(maximizeButton);

    _maximizeGlyph = new SpanElement()
    ..classes.add('glyphicons')
    ..classes.add((state == ColumnState.MAXIMIZED) ? 'glyphicons-resize-small' : 'glyphicons-resize-full');
    maximizeButton.children.add(_maximizeGlyph);

    LIElement controlLi = new LIElement()
      ..classes.add('tab-control');
    navTabs.children.add(controlLi);

    controlButton = new AnchorElement( )
      ..id = 'column-$id-new'
      ..classes.add('new-tab-button');
    controlLi.children.add(controlButton);

    SpanElement controlGlyph = new SpanElement()
      ..classes.addAll(['glyphicons', 'glyphicons-plus']);
    controlButton.children.add(controlGlyph);

    if (state == ColumnState.MAXIMIZED) {
      maximize();
    } else if (state == ColumnState.MINIMIZED) {
      minimize();
    }
  }

  void maximize() {
    if (hideMinimizedCompletely) {
      columnContent.style.width = 'calc(100% - 220px)';
      columnContent.style.display = '';

      _maximizeGlyph.classes.remove('glyphicons-resize-full');
      _maximizeGlyph.classes.add('glyphicons-resize-small');

      return;
    }

    columnContent.classes.removeAll(['col-xs-1', 'col-xs-5', 'col-xs-10']);
    columnContent.classes.add('col-xs-9');

    // Responsive width minus the other columns that are currently fixed;
    columnContent.style.width = 'calc(100% - 270px)';

    _maximizeGlyph.classes.remove('glyphicons-resize-full');
    _maximizeGlyph.classes.add('glyphicons-resize-small');

    _showTabsAndContent();
  }

  void normalize() {
    if (hideMinimizedCompletely) {
      columnContent.style.width = 'calc(50% - 110px)';
      columnContent.style.display = '';

      _maximizeGlyph.classes.remove('glyphicons-resize-small');
      _maximizeGlyph.classes.add('glyphicons-resize-full');

      return;
    }

    columnContent.classes.removeAll(['col-xs-1', 'col-xs-9', 'col-xs-10']);
    columnContent.classes.add('col-xs-5');

    // Responsive width minus half the panel column.
    columnContent.style.width = 'calc(50% - 110px)';

    _maximizeGlyph.classes.remove('glyphicons-resize-small');
    _maximizeGlyph.classes.add('glyphicons-resize-full');

    _showTabsAndContent();
  }

  void minimize() {
    if (hideMinimizedCompletely) {
      columnContent.style.width = '';
      columnContent.style.display = 'none';

      _maximizeGlyph.classes.remove('glyphicons-resize-small');
      _maximizeGlyph.classes.add('glyphicons-resize-full');

      return;
    }

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
    navTabs.style.visibility = 'visible';

    navTabs.children.getRange(1, navTabs.children.length)
    .forEach((e) => e.children
    .forEach((e) => e.style.display = ''));

    tabContent.children.forEach((e) => e.style.display = '');
    tabContent.style.backgroundColor = '';
  }

  /// Hides content and tacks on additional "hidden" styling effects.
  void _hideTabsAndContent() {
    // TODO: move these styles to CSS and use classes instead.
    navTabs.style.visibility = 'hidden';

    navTabs.children.getRange(1, navTabs.children.length)
    .forEach((e) => e.children
    .forEach((e) => e.style.display = 'none'));

    tabContent.children.forEach((e) => e.style.display = 'none');
    tabContent.style.backgroundColor = '#eaecec';
  }

  void cleanUp() {
    // Empty out nav-tabs.
    columnContent.children.first.innerHtml = '';

    // Remove the maximize button.
    columnContent.children.last.remove();

    // Empty the tab content.
    tabContent.innerHtml = '';
  }
}