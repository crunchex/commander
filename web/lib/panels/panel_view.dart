part of panel_controller;

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
class PanelView {

  /// Returns an initialized [TabView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<PanelView> createPanelView(int num, int col, String title, String shortName, List config, [bool externalCss=false]) {
    Completer c = new Completer();
    c.complete(new PanelView(num, col, title, shortName, config, externalCss));
    return c.future;
  }

  int num, col;
  String title;
  String shortName;
  Map refMap;

  LinkElement styleLink;
  AnchorElement tabHandleButton;
  DivElement closeControlHitbox;
  DivElement cloneControlHitbox;
  DivElement content;
  LIElement extra;

  LIElement _tabHandle;
  UListElement _menus;
  DivElement _tabContainer,_tabContent;
  List config;

  PanelView(this.num, this.col, this.title, this.shortName, this.config, [bool externalCss=false]) {
    refMap = {};

    // Inject the associated stylesheet if one exists.
    // TODO: somehow detect if it exists at runtime.
    if (externalCss) {
      String cssPath = 'lib/panels/${shortName.toLowerCase()}/${shortName.toLowerCase()}.css';
      styleLink = new LinkElement();
      styleLink.rel = 'stylesheet';
      styleLink.href = cssPath;
      document.head.append(styleLink);
    }

    DivElement topLevel = querySelector('#column-$col');

    UListElement tabList = new UListElement()
      ..classes.addAll(['nav', 'nav-tabs'])
      ..setAttribute('role', 'tablist');
    topLevel.children.add(tabList);

//    LIElement newPanelLi = new LIElement();
//    tabList.children.add(newPanelLi);
//
//    AnchorElement panelButton = new AnchorElement()
//      ..id = 'column-$col-new'
//      ..classes.add('new-tab-button');
//    newPanelLi.children.add(panelButton);
//
//    SpanElement glyphiconNew = new SpanElement()
//      ..classes.addAll(['glyphicons', 'glyphicons-chevron-down']);
//    panelButton.children.add(glyphiconNew);

    DivElement topTabContent = new DivElement()
      ..id = 'col-$col-tab-content'
      ..classes.add('tab-content');
    topLevel.children.add(topTabContent);

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
    if (styleLink != null) styleLink.remove();
  }

  LIElement addMenuItem(Map itemConfig, [String dropdownMenuSelector]) {
    LIElement itemElement;
    if (itemConfig['type'] == 'toggle') {
      if (itemConfig.containsKey('handler')) {
        if (itemConfig.containsKey('args')) {
          itemElement = _createToggleItem(itemConfig['title'], itemConfig['handler'], itemConfig['args']);
        } else {
          itemElement = _createToggleItem(itemConfig['title'], itemConfig['handler']);
        }
      } else {
        itemElement = _createToggleItem(itemConfig['title']);
      }
    } else if (itemConfig['type'] == 'input') {
      itemElement = _createInputItem(itemConfig['title']);
    } else if (itemConfig['type'] == 'submenu') {
      itemElement = _createSubMenu("Templates", ['Publisher', 'Subscriber', 'Hello World Talker', 'Hello World Listener', 'Basic Launch File']);
    }

    if (dropdownMenuSelector != null) {
      UListElement dropdownMenu = querySelector(dropdownMenuSelector);
      dropdownMenu.children.add(itemElement);
    }

    return itemElement;
  }

  /// Takes a [num], [col], and [title] to add a new tab for the specified column.
  void _setUpTabHandle() {
    _tabHandle = new LIElement()
      ..classes.add('tab-handle')
      ..classes.add('active');

    String id = title.toLowerCase().replaceAll(' ', '-');

    closeControlHitbox = new DivElement()
      ..title = 'Close'
      ..classes.add('close-control-hitbox');
    _tabHandle.children.add(closeControlHitbox);

    DivElement closeControl = new DivElement()
      ..classes.add('close-control');
    closeControlHitbox.children.add(closeControl);

//    SpanElement closeSymbol = new SpanElement()
//      ..classes.add('close-control-symbol')
//      ..text = 'X';
//    closeControl.children.add(closeSymbol);

    cloneControlHitbox = new DivElement()
      ..title = 'Clone'
      ..classes.add('clone-control-hitbox');
    _tabHandle.children.add(cloneControlHitbox);

    DivElement cloneControl = new DivElement()
      ..classes.add('clone-control');
    cloneControlHitbox.children.add(cloneControl);

    tabHandleButton = new AnchorElement()
        ..id = 'button-$id-$num'
        ..href = '#tab-$id-$num-container'
        ..dataset['toggle'] = 'tab'
        ..text = '$shortName-$num';
    tabHandleButton.onClick.listen((e) {
      e.preventDefault();
      //e.stopImmediatePropagation();
      //_renameEventHandler();
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
        ..classes.add('tab-pane')
        ..classes.add('active');

    _menus = new UListElement()
        ..classes.add('nav')
        ..classes.add('nav-tabs')
        ..classes.add('inner-tabs')
        ..attributes['role'] = 'tablist';
    _tabContainer.children.add(_menus);

    _menus.children = new List<Element>();
    for (Map configItem in config) {
      _menus.children.add(_createDropdownMenu(configItem));
    }

    extra = new LIElement();
    extra.id = 'extra-$num';
    extra.classes.add('extra-menubar');
    _menus.children.add(extra);

    _tabContent = new DivElement()
        ..classes.add('tab-content')
        ..classes.add('active');
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
        ..id = '${shortName.toLowerCase()}-$num-${title.toLowerCase().replaceAll(' ', '-')}'
        ..classes.add('dropdown-menu')
        ..attributes['role'] = 'menu';
    dropdown.children.add(dropdownMenu);

    LIElement item;
    for (Map i in items) {
      item = addMenuItem(i);
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