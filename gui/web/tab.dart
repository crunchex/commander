library updroid_tab;

import 'dart:html';
import 'dart:async';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
abstract class UpDroidTab {
  LIElement _tabHandle;
  DivElement _tabContainer;
  DivElement _tabContent;

  AnchorElement tabHandleButton;

  void makeTabActive() {
    _tabHandle.classes.add('active');
    _tabContainer.classes.add('active');
    _tabContent.classes.add('active');
  }

  void makeTabInactive() {
    _tabHandle.classes.remove('active');
    _tabContainer.classes.remove('active');
    _tabContent.classes.remove('active');
  }

  void destroyTab() {
    _tabHandle.remove();
    _tabContainer.remove();
  }

  /// Takes a [num], [col], and [title] to add a new tab for the specified column.
  void setUpTabHandle(int num, int col, String title, bool active) {
    _tabHandle = new LIElement();
    if (active) _tabHandle.classes.add('active');

    String id = title.toLowerCase().replaceAll(' ', '-');

    tabHandleButton = new AnchorElement()
        ..id = 'button-$id-$num'
        ..href = '#tab-$id-$num-container'
        ..dataset['toggle'] = 'tab'
        ..text = '$title-$num';
    _tabHandle.children.add(tabHandleButton);

    DivElement column = querySelector('#column-$col');
    column.children.first.children.add(_tabHandle);
  }

  /// Takes a [num], [col], [title], [config], and [active] to generate the menu bar and menu items
  /// for a tab. Returns a [Map] of references to the new [Element]s as a [Future].
  Future<Map> setUpTabContainer(int num, int col, String title, List config, bool active) {
    Completer completer = new Completer();

    Map configRefs = new Map();

    String id = title.toLowerCase().replaceAll(' ', '-');

    _tabContainer = new DivElement()
        ..id = 'tab-$id-$num-container'
        ..classes.add('tab-pane');
    if (active) _tabContainer.classes.add('active');

    UListElement tabList = new UListElement()
        ..classes.add('nav')
        ..classes.add('nav-tabs')
        ..classes.add('inner-tabs')
        ..attributes['role'] = 'tablist';
    _tabContainer.children.add(tabList);

    for (Map configItem in config) {
      tabList.children.add(_createDropdownMenu(configItem, configRefs));
    }

    LIElement extra = new LIElement();
    extra.id = 'filename';
    tabList.children.add(extra);
    configRefs['extra'] = extra;

    _tabContent = new DivElement()
        ..classes.add('tab-content');
    if (active) _tabContent.classes.add('active');
    _tabContainer.children.add(_tabContent);

    DivElement content = new DivElement()
        ..id = id
        ..classes.add(id);
    _tabContent.children.add(content);
    configRefs['content'] = content;

    DivElement colOneTabContent = querySelector('#col-$col-tab-content');
    colOneTabContent.children.insert(0, _tabContainer);

    completer.complete(configRefs);
    return completer.future;
  }

  /// Generates a dropdown menu and returns the new [LIElement].
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

  /// Generates an input item (label and input field) and returns
  /// the new [LIElement].
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

  /// Generates a toggle item (button) and returns the new [LIElement].
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