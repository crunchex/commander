library tab_view;

import 'dart:async';
import 'dart:html';

import 'package:upcom-api/web/tab/container_view.dart';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
class LauncherView extends ContainerView {

  AnchorElement controlButton;

  /// Returns an initialized [LauncherView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<LauncherView> createLauncherView(int id, int col, String refName, String fullName, String shortName, List config, [String externalCssPath]) {
    Completer c = new Completer();
    c.complete(new LauncherView(id, col, refName, fullName, shortName, config));
    return c.future;
  }

  LIElement extra;
  DivElement closeControlHitbox;

  LauncherView(int id, int col, String refName, String fullName, String shortName, List config) :
  super(id, col, refName, fullName, shortName, config, querySelector('#column-$col').children[0], true) {
    LIElement controlLi = new LIElement()
    ..classes.add('tab-control');
    querySelector('#column-$col').children[0].children.add(controlLi);

    controlButton = new AnchorElement( )
    ..id = 'column-$id-new'
    ..classes.add('new-tab-button');
    controlLi.children.add(controlButton);

    SpanElement controlGlyph = new SpanElement()
    ..classes.addAll(['glyphicons', 'glyphicons-plus']);
    controlButton.children.add(controlGlyph);
  }
}