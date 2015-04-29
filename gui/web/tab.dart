library updroid_tab;

import 'dart:html';
import 'dart:async';

/// [UpDroidTab] contains methods to generate [Element]s that make up a tab
/// and menu bar in the UpDroid Commander GUI.
abstract class UpDroidTab {
  LIElement _tabHandle;
  DivElement _tabContainer;
  DivElement _tabContent;
  DivElement _explorersDiv = querySelector("#exp-container");
  UListElement _expList = querySelector("#side-menu ul");
  Element separator = querySelector('#side-menu-separator');
  DivElement _explorer;

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

  // Explorer related functions

  void createExplorer(int num) {
    ParagraphElement recycle = querySelector("#recycle");
    makeExpButton(num);
    _explorer = new DivElement()
      ..id = "exp-$num"
      ..dataset['num'] = num.toString();
    _explorersDiv.insertBefore(_explorer, recycle);
    DivElement explorerHead = new DivElement()
      ..classes.add('explorer-head');
    _explorer.append(explorerHead);
    LIElement newDnd = new LIElement()
        ..classes.add('new')
        ..text = "New";
    explorerHead.append(newDnd);
    SpanElement folder = new SpanElement()
      ..id = "folder-$num"
      ..classes.addAll(["glyphicon glyphicon-folder-close", 'folder']);
    SpanElement file = new SpanElement()
      ..id = "file-$num"
      ..classes.addAll(["glyphicon glyphicon-file", 'file']);
    newDnd.append(folder);
    newDnd.append(file);
    DivElement hrContainer = new DivElement()
      ..id = "file-explorer-hr-container-$num";
    explorerHead.append(hrContainer);
    DivElement drop = new DivElement()
      ..classes.add("new-file-drop")
      ..id = "new-file-drop-$num";
    hrContainer.append(drop);
    ParagraphElement p = new ParagraphElement();
    p.text = "Top Level";
    drop.append(p);
    DivElement body = new DivElement()
      ..classes.addAll(['well', 'well-sm', 'explorer-container'])
      ..id = "explorer-$num";
    _explorer.append(body);
    UListElement guts = new UListElement()
      ..classes.add("explorer-body")
      ..id = "explorer-body-$num";
    body.append(guts);
  }

  makeExpButton (int num) {
    LIElement item = new LIElement();
    AnchorElement link = new AnchorElement()
      ..id = "exp-button-$num"
      ..href = "#"
      ..text = "Workspace $num"
      ..attributes['role'] = 'button';
    item.append(link);
    _expList.insertBefore(item, separator);
    item.onClick.listen((e){
      for(var explorer in _explorersDiv.children) {
        if(!explorer.classes.contains('hidden') && int.parse(explorer.dataset['num']) != num) {
          explorer.classes.add('hidden');
        }
        if(int.parse(explorer.dataset['num']) == num) {
          explorer.classes.remove('hidden');
        }
      }
    });
  }

  void hideExplorer() {
    _explorer.classes.add('hidden');
  }

  void showExlorer() {
    _explorer.classes.remove('hidden');
  }

  // End of explorer functions

  /// Takes a [num], [col], and [title] to add a new tab for the specified column.
  void setUpTabHandle(int num, int col, String title, bool active) {
    _tabHandle = new LIElement()
      ..classes.add('tab-handle');
    if (active) _tabHandle.classes.add('active');

    String id = title.toLowerCase().replaceAll(' ', '-');

    tabHandleButton = new AnchorElement()
        ..id = 'button-$id-$num'
        ..href = '#tab-$id-$num-container'
        ..dataset['toggle'] = 'tab'
        ..text = '$title-$num';
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
    extra.id = 'filename-$num';
    extra.classes.add('editor-file-name');
    tabList.children.add(extra);
    configRefs['extra'] = extra;

    _tabContent = new DivElement()
        ..classes.add('tab-content');
    if (active) _tabContent.classes.add('active');
    _tabContainer.children.add(_tabContent);

    DivElement content = new DivElement()
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