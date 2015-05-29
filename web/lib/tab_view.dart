library updroid_tab;

import 'dart:html';
import 'dart:async';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
class TabView {

  /// Returns an initialized [TabView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<TabView> createTabView(int num, int col, String title, String shortName, bool active, List config) {
    Completer c = new Completer();
    c.complete(new TabView(num, col, title, shortName, active, config));
    return c.future;
  }

  int num, col;
  String title;
  String shortName;
  bool active;
  Map refMap;

  AnchorElement tabHandleButton;
  DivElement content;
  LIElement extra;

  LIElement _tabHandle;
  UListElement _menus;
  DivElement _tabContainer,_tabContent;
  List config;

  TabView(this.num, this.col, this.title, this.shortName, this.active, this.config) {
    refMap = {};

    _setUpTabHandle();
    _setUpTabContainer();
  }

  /// Adds the CSS classes to make a tab 'active'.
  void makeActive() {
    _tabHandle.classes.add('active');
    _tabContainer.classes.add('active');
    _tabContent.classes.add('active');
  }

  /// Removes the CSS classes to make a tab 'inactive'.
  void makeInactive() {
    _tabHandle.classes.remove('active');
    _tabContainer.classes.remove('active');
    _tabContent.classes.remove('active');
  }

  /// Removes the tab elements from the DOM.
  void destroy() {
    _tabHandle.remove();
    _tabContainer.remove();
  }

  refreshMenus() {
    _menus.children = new List<Element>();
    for (Map configItem in config) {
      _menus.children.add(_createDropdownMenu(configItem));
    }
  }

  /// Takes a [num], [col], and [title] to add a new tab for the specified column.
  void _setUpTabHandle() {
    _tabHandle = new LIElement()
      ..classes.add('tab-handle');
    if (active) _tabHandle.classes.add('active');

    String id = title.toLowerCase().replaceAll(' ', '-');

    tabHandleButton = new AnchorElement()
        ..id = 'button-$id-$num'
        ..href = '#tab-$id-$num-container'
        ..dataset['toggle'] = 'tab'
        ..text = '$shortName-$num';
    tabHandleButton.onClick.listen((e) {
      //e.preventDefault();
      //e.stopImmediatePropagation();
      _renameEventHandler();
    });
    _tabHandle.children.add(tabHandleButton);

    DivElement column = querySelector('#column-$col');
    column.children.first.children.add(_tabHandle);
  }

  /// Takes a [num], [col], [title], [config], and [active] to generate the menu bar and menu items
  /// for a tab. Returns a [Map] of references to the new [Element]s as a [Future].
  void _setUpTabContainer() {
    String id = title.toLowerCase().replaceAll(' ', '-');

    _tabContainer = new DivElement()
        ..id = 'tab-$id-$num-container'
        ..classes.add('tab-pane');
    if (active) _tabContainer.classes.add('active');

    _menus = new UListElement()
        ..classes.add('nav')
        ..classes.add('nav-tabs')
        ..classes.add('inner-tabs')
        ..attributes['role'] = 'tablist';
    _tabContainer.children.add(_menus);

    refreshMenus();

    extra = new LIElement();
    extra.id = 'extra-$num';
    extra.classes.add('extra-menubar');
    _menus.children.add(extra);

    _tabContent = new DivElement()
        ..classes.add('tab-content');
    if (active) _tabContent.classes.add('active');
    _tabContainer.children.add(_tabContent);

    content = new DivElement()
        ..classes.add(id);
    _tabContent.children.add(content);
    refMap['content'] = content;

    DivElement colOneTabContent = querySelector('#col-$col-tab-content');
    colOneTabContent.children.insert(0, _tabContainer);
  }

  /// Generates a dropdown menu and returns the new [LIElement].
  LIElement _createDropdownMenu(Map config) {
    String title = config['title'];
    List items = config['items'];

    LIElement dropdown = new LIElement();
    dropdown.classes.add('dropdown');

    AnchorElement dropdownToggle = new AnchorElement()
        ..href = '#'
        ..classes.add('dropdown-toggle')
        ..dataset['toggle'] = 'dropdown'
        ..text = title;
    dropdown.children.add(dropdownToggle);

    UListElement dropdownMenu = new UListElement()
        ..classes.add('dropdown-menu')
        ..attributes['role'] = 'menu';
    dropdown.children.add(dropdownMenu);

    LIElement item;
    for (Map i in items) {
      if (i['type'] == 'toggle') {
        if (i.containsKey('handler')) {
          if (i.containsKey('args')) {
            item = _createToggleItem(i['title'], i['handler'], i['args']);
          } else {
            item = _createToggleItem(i['title'], i['handler']);
          }
        } else {
          item = _createToggleItem(i['title']);
        }
      } else if (i['type'] == 'input') {
        item = _createInputItem(i['title']);
      } else if (i['type'] == 'submenu') {
        item = _createSubMenu("Templates", ['Publisher', 'Subscriber', 'Hello World Talker', 'Hello World Listener', 'Basic Launch File']);
      }
      dropdownMenu.children.add(item);
    }

    return dropdown;
  }

  ///Create a submenu within a dropdown
  LIElement _createSubMenu(String title, List<String> items) {
    LIElement item = new LIElement()
      ..classes.add('dropdown-submenu');
    AnchorElement button = new AnchorElement()
      ..tabIndex = -1
      ..href = '#'
      ..text = title;
    item.append(button);
    UListElement dropdown = new UListElement()
      ..classes.add('dropdown-menu');
    item.append(dropdown);

    for (String item in items) {
      LIElement menuItem = new LIElement();
      AnchorElement inner = new AnchorElement()
        ..tabIndex = -1
        ..href = "#"
        ..text = item
        ..id = "${item.toLowerCase().replaceAll(' ', '-')}-button";
      menuItem.append(inner);
      dropdown.append(menuItem);
      refMap[inner.id] = inner;
    }
    return item;
  }

  /// Generates an input item (label and input field) and returns
  /// the new [LIElement].
  LIElement _createInputItem(String title) {
    LIElement li = new LIElement();

    DivElement d = new DivElement()..style.display = 'inline-block';
    li.children.add(d);

    String id = title.toLowerCase().replaceAll(' ', '-');

    ParagraphElement p = new ParagraphElement()
        ..style.display = 'inline-block'
        ..text = title;
    d.children.add(p);

    InputElement i = new InputElement()
        ..id = '$id-input'
        ..type = 'text';
    d.children.add(i);
    refMap[id] = i;

    return li;
  }

  /// Generates a toggle item (button) and returns the new [LIElement].
  LIElement _createToggleItem(String title, [onClick, args]) {
    String id = title.toLowerCase().replaceAll(' ', '-');

    LIElement buttonList = new LIElement();
    AnchorElement button = new AnchorElement()
        ..id = 'button-$id'
        ..href = '#'
        ..attributes['role'] = 'button'
        ..text = title;
    if (onClick != null) {
      button.onClick.listen((e) {
        if (args != null) {
          onClick(args);
        } else {
          onClick();
        }
      });
    }
    buttonList.children.add(button);
    refMap[id] = button;

    return buttonList;
  }

  /// Handles tab renaming with a single-click event.
  void _renameEventHandler() {
    if (!_tabHandle.className.contains('editing') && _tabHandle.className.contains('active')) {
      bool editing = false;

      _tabHandle.classes.add('editing');

      String originalText = tabHandleButton.text;

      AnchorElement a = new AnchorElement();
      InputElement input = new InputElement();
      a.children.add(input);
      input.value = originalText;

      _tabHandle.children[0] = a;
      input.focus();
      input.select();

      editing = true;

      List<StreamSubscription> subs = [];

      subs.add(input.onKeyUp.listen((e) {
        if (e.keyCode == KeyCode.ENTER || e.keyCode == KeyCode.ESC) {
          if (e.keyCode == KeyCode.ENTER) {
            tabHandleButton.text = input.value;
          } else if (e.keyCode == KeyCode.ESC){
            tabHandleButton.text = originalText;
          }
          _tabHandle.children[0] = tabHandleButton;
          _tabHandle.classes.remove('editing');
          subs.forEach((sub) => sub.cancel());
        }
      }));

      subs.add(document.onClick.listen((e) {
        if (editing == true) {
          tabHandleButton.text = originalText;
          _tabHandle.children[0] = tabHandleButton;
          _tabHandle.classes.remove('editing');
          subs.forEach((sub) => sub.cancel());
        }
      }));
    }
  }
}