library context_menu;

import 'dart:html';

class ContextMenu {
  List<Map> config;

  UListElement _contextMenu;
  bool _clean;

  ContextMenu(Point origin, this.config) {
    _contextMenu = new UListElement()
      ..style.position = 'absolute'
      ..style.left = origin.x.toString() + 'px'
      ..style.top = origin.y.toString() + 'px'
      ..classes.addAll(['dropdown-menu', 'context-menu'])
      ..attributes['role'] = 'menu';
    document.body.append(_contextMenu);

    _clean = false;

    LIElement item;
    for (Map i in config) {
      item = addMenuItem(i);
      _contextMenu.children.add(item);
    }

    _contextMenu.parent.classes.toggle('open');

    document.body.onClick.first.then((e) => cleanup());
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
    } else if (itemConfig['type'] == 'submenu') {
      itemElement = _createSubMenu("Templates", ['Publisher', 'Subscriber', 'Hello World Talker', 'Hello World Listener', 'Basic Launch File']);
    }

    if (dropdownMenuSelector != null) {
      UListElement dropdownMenu = querySelector(dropdownMenuSelector);
      dropdownMenu.children.add(itemElement);
    }

    return itemElement;
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
    _contextMenu.parent.classes.toggle('open');
    _contextMenu.remove();
    _clean = true;
  }
}


