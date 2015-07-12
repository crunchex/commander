part of tab_controller;

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
class TabView extends ContainerView {

  /// Returns an initialized [TabView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<TabView> createTabView(int id, int col, String title, String shortName, List config, [bool externalCss=false]) {
    Completer c = new Completer();
    c.complete(new TabView(id, col, title, shortName, config, externalCss));
    return c.future;
  }

  LIElement extra;
  DivElement closeControlHitbox;

  TabView(int id, int col, String title, String shortName, List config, [bool externalCss=false]) :
  super(id, col, title, shortName, config, querySelector('#column-$col').children[1]) {
    if (externalCss) {
      String cssPath = 'lib/tabs/${shortName.toLowerCase()}/${shortName.toLowerCase()}.css';
      loadExternalCss(cssPath);
    }

    tabHandleButton.text = '$shortName-$id';

    extra = new LIElement();
    extra.id = 'extra-$id';
    extra.classes.add('extra-menubar');
    menus.children.add(extra);

    closeControlHitbox = new DivElement()
      ..title = 'Close'
      ..classes.add('close-control-hitbox');
    tabHandle.children.insert(0, closeControlHitbox);

    DivElement closeControl = new DivElement()
      ..classes.addAll(['close-control', 'glyphicons', 'glyphicons-remove-2']);
    closeControlHitbox.children.add(closeControl);
  }
}