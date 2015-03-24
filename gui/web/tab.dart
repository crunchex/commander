library updroid_tab;

import 'dart:html';
import 'dart:async';

abstract class UpDroidTab {

  void setUpTabHandle(int col, String title, {bool active: false}) {
    LIElement li = new LIElement();
    if (active) li.classes.add('active');

    String id = title.toLowerCase().replaceAll(' ', '-');

    AnchorElement a = new AnchorElement()
        ..id = 'button-$id'
        ..href = '#tab-$id-container'
        ..dataset['toggle'] = 'tab'
        ..text = title;
    li.children.add(a);

    DivElement column = querySelector('#column-$col');
    column.children.first.children.add(li);
  }

  Future setUpTabContainer(int col, String title, List config, {bool active: false}) {
    Completer completer = new Completer();

    Map configRefs = new Map();

    String id = title.toLowerCase().replaceAll(' ', '-');

    DivElement tabContainer = new DivElement()
        ..id = 'tab-$id-container'
        ..classes.add('tab-pane');

    if (active) tabContainer.classes.add('active');

    UListElement tabList = new UListElement()
        ..classes.add('nav')
        ..classes.add('nav-tabs')
        ..classes.add('inner-tabs')
        ..attributes['role'] = 'tablist';
    tabContainer.children.add(tabList);

    for (Map configItem in config) {
      tabList.children.add(_createDropdownMenu(configItem, configRefs));
    }

    LIElement filename = new LIElement();
    filename.id = 'filename';
    tabList.children.add(filename);
    configRefs['filename'] = filename;

    DivElement tabContent = new DivElement()
        ..classes.add('tab-content')
        ..classes.add('active');
    tabContainer.children.add(tabContent);

    DivElement content = new DivElement()
        ..id = id
        ..classes.add(id);
    tabContent.children.add(content);
    configRefs['content'] = content;

    DivElement colOneTabContent = querySelector('#col-$col-tab-content');
    colOneTabContent.children.insert(0, tabContainer);

    completer.complete(configRefs);
    return completer.future;
  }

  LIElement _createDropdownMenu(Map config, Map configRefs) {
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
        item = _createToggleItem(i['title'], configRefs);
      } else if (i['type'] == 'input') {
        item = _createInputItem(i['title'], configRefs);
      }
      dropdownMenu.children.add(item);
    }

    return dropdown;
  }

  LIElement _createInputItem(String title, Map configRefs) {
    LIElement li = new LIElement()..style.textAlign = 'center';

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
    configRefs[id] = i;

    return li;
  }

  LIElement _createToggleItem(String title, Map configRefs) {
    String id = title.toLowerCase().replaceAll(' ', '-');

    LIElement buttonList = new LIElement();
    AnchorElement button = new AnchorElement()
        ..id = 'button-$id'
        ..href = '#'
        ..attributes['role'] = 'button'
        ..text = title;
    buttonList.children.add(button);
    configRefs[id] = button;

    return buttonList;
  }
}