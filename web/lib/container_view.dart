library upcom_api.lib.web.tab.container_view;

import 'dart:html';
import 'dart:async';

/// [ContainerView] contains methods to generate [Element]s that make up a
/// tab/panel container with a menu bar in the UpDroid Commander GUI.
abstract class ContainerView {
  final int id;
  final String refName, fullName, shortName;

  // Column value can change when a container is moved.
  int col;

  LinkElement styleLink;
  AnchorElement tabHandleButton;
  DivElement columnContent, content, tabContainer, tabContent;
  LIElement tabHandle;
  UListElement navTabs, menus;

  bool _menuEnabled;

  ContainerView(this.id, this.col, this.refName, this.fullName, this.shortName, this.navTabs, this.columnContent) {
    _setUpTabHandle();
    _setUpTabContainer();
  }

  bool isActive() => tabHandle.classes.contains('active') && tabContainer.classes.contains('active');

  /// Adds the CSS classes to make a tab 'active'.
  void makeActive() {
    tabHandle.classes.add('active');
    tabContainer.classes.add('active');
  }

  /// Removes the CSS classes to make a tab 'inactive'.
  void makeInactive() {
    tabHandle.classes.remove('active');
    tabContainer.classes.remove('active');
  }

  /// Removes the tab elements from the DOM.
  void destroy() {
    tabHandle.remove();
    tabContainer.remove();
    if (styleLink != null) styleLink.remove();
  }

  void loadExternalCss(path) {
    // Inject the associated stylesheet if one exists.
    // TODO: somehow detect if it exists at runtime.
    styleLink = new LinkElement();
    styleLink.rel = 'stylesheet';
    styleLink.href = path;
    document.head.append(styleLink);
  }

  /// Takes a [num], [col], and [title] to add a new tab for the specified column.
  void _setUpTabHandle() {
    tabHandle = new LIElement();

    tabHandleButton = new AnchorElement();
    tabHandle.children.add(tabHandleButton);

    navTabs.children.add(tabHandle);
  }

  /// Takes a [num], [col], [title], [config], and [active] to generate the
  /// menu bar and menu items for a tab. Returns a [Map] of references to
  /// the new [Element]s as a [Future].
  void _setUpTabContainer() {
    tabContainer = new DivElement()
      ..id = 'tab-$refName-$id-container'
      ..classes.add('tab-pane')
      ..classes.add('active');

    tabContent = new DivElement()
      ..id = 'tab-$refName-$id-content'
      ..classes.add('tab-content')
      ..tabIndex = -1;
    tabContainer.children.add(tabContent);

    content = new DivElement()
      ..classes.add(refName);
    tabContent.children.add(content);

    columnContent.children.insert(0, tabContainer);
  }
}