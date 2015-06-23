part of panel_controller;

/// [PanelView] contains generic methods and fields that make up the visual
/// side of a Panel.
class PanelView extends ContainerView {

  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<PanelView> createPanelView(int id, int col, String title, String shortName, List config, [bool externalCss=false]) {
    Completer c = new Completer();
    c.complete(new PanelView(id, col, title, shortName, config, externalCss));
    return c.future;
  }

  PanelView(int id, int col, String title, String shortName, List config, [bool externalCss=false]) :
  super(id, col, title, shortName, config) {
    if (externalCss) {
      String cssPath = 'lib/panels/${shortName.toLowerCase()}/${shortName.toLowerCase()}.css';
      loadExternalCss(cssPath);
    }
  }
}