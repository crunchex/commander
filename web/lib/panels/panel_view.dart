part of panel_controller;

/// [PanelView] contains generic methods and fields that make up the visual
/// side of a Panel.
class PanelView extends ContainerView {

  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<PanelView> createPanelView(int id, int col, String refName, String fullName, String shortName, List config, [bool externalCss=false]) {
    Completer c = new Completer();
    c.complete(new PanelView(id, col, refName, fullName, shortName, config, externalCss));
    return c.future;
  }

  PanelView(int id, int col, String refName, String fullName, String shortName, List config, [bool externalCss=false]) :
  super(id, col, refName, fullName, shortName, config, querySelector('#column-$col').children[1]) {
    if (externalCss) {
      String cssPath = 'lib/panels/${shortName.toLowerCase()}/${shortName.toLowerCase()}.css';
      loadExternalCss(cssPath);
    }

    tabHandleButton.text = '$shortName';
  }
}