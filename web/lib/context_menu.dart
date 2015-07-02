library context_menu;

import 'dart:html';
import 'dart:async';

ContextMenu _singleton;

class ContextMenu {
  static Future createContextMenu(Point origin, List<Map> config) {
    Completer c = new Completer();
    if (_singleton != null) {
      _singleton.cleanup();
      _singleton = null;
    }

    _singleton = new ContextMenu(origin, config);
    c.complete();
    return c.future;
  }

  static addItem(Map itemConfig) {
    _singleton.contextMenu.children.add(_singleton._addMenuItem(itemConfig));
  }

  List<Map> config;
  UListElement contextMenu;

  bool _clean;

  ContextMenu(Point origin, this.config) {
    contextMenu = new UListElement()
      ..style.position = 'absolute'
      ..style.left = origin.x.toString() + 'px'
      ..style.top = origin.y.toString() + 'px'
      ..classes.addAll(['dropdown-menu', 'context-menu'])
      ..attributes['role'] = 'menu';
    document.body.append(contextMenu);

    _clean = false;

    LIElement item;
    for (Map i in config) {
      item = _addMenuItem(i);
      contextMenu.children.add(item);
    }

    contextMenu.parent.classes.toggle('open');

    document.body.onClick.first.then((e) => cleanup());
  }

  LIElement _addMenuItem(Map itemConfig, [String dropdownMenuSelector]) {
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
    } else if (itemConfig['type'] == 'submenu') {
      itemElement = _createSubMenu(itemConfig['title'], itemConfig['items']);
    } else if (itemConfig['type'] == 'divider') {
      itemElement = _createDivider(itemConfig['title']);
    }

    if (dropdownMenuSelector != null) {
      UListElement dropdownMenu = querySelector(dropdownMenuSelector);
      dropdownMenu.children.add(itemElement);
    }

    return itemElement;
  }

  /// Generates a toggle item (button) and returns the new [LIElement].
  LIElement _createDivider([String title]) {

    LIElement dividerList = new LIElement();

    if (title != '') {
      ParagraphElement dividerTitle = new ParagraphElement()
        ..classes.add('menu-divider-title')
        ..text = title;
      dividerList.children.add(dividerTitle);
    }

    HRElement divider = new HRElement()
      ..classes.add('menu-divider');
    dividerList.children.add(divider);

    return dividerList;
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
    SpanElement dropdownIndicator = new SpanElement()
      ..classes.addAll(['glyphicons', 'glyphicons-chevron-right']);
    button.children.add(dropdownIndicator);
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
    }
    return item;
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
      button.onClick.first.then((MouseEvent e) {
        e.stopPropagation();
        args != null ? onClick(args) : onClick();
        cleanup();
      });
    }
    buttonList.children.add(button);
//    refMap[id] = button;

    return buttonList;
  }

  void cleanup() {
    if (_clean) return;
    contextMenu.parent.classes.toggle('open');
    contextMenu.remove();
    _clean = true;
  }
}


