part of updroid_client;

abstract class UpDroidTab {

  Future _setUpTabContainer({bool active: false}) {
    Completer completer = new Completer();

    DivElement tabContainer = new DivElement()
        ..id = 'tab-editor-container'
        ..classes.add('tab-pane');

    if (active) tabContainer.classes.add('active');

    UListElement tabList = new UListElement()
        ..classes.add('nav')
        ..classes.add('nav-tabs')
        ..classes.add('inner-tabs')
        ..attributes['role'] = 'tablist';
    tabContainer.children.add(tabList);

    tabList.children.add(_createDropdownMenu('File', ['New', 'Save', 'Save As']));
    tabList.children.add(_createDropdown2('Settings'));
    LIElement filename = new LIElement();
    filename.id = 'filename';
    tabList.children.add(filename);

    DivElement tabContent = new DivElement()
        ..classes.add('tab-content')
        ..classes.add('active');
    tabContainer.children.add(tabContent);

    DivElement content = new DivElement()
        ..id = 'editor'
        ..classes.add('editor');
    tabContent.children.add(content);


    completer.complete(tabContainer);
    return completer.future;
  }

  LIElement _createDropdownMenu(String title, List buttons) {
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

    for (String title in buttons) {
      dropdownMenu.children.add(_createToggleItem(title));
    }

    return dropdown;
  }

  /// Special method specific to editor.
  LIElement _createDropdown2(String title) {
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

    LIElement buttonTheme = _createToggleItem('Theme');
    buttonTheme.children[0].id = 'button-editor-theme';
    dropdownMenu.children.add(buttonTheme);

    LIElement li = _createInputItem();
    dropdownMenu.children.add(li);

    return dropdown;
  }

  LIElement _createInputItem() {
    LIElement li = new LIElement()..style.textAlign = 'center';

    DivElement d = new DivElement()..style.display = 'inline-block';
    li.children.add(d);

    ParagraphElement p = new ParagraphElement()
        ..style.display = 'inline-block'
        ..text = 'Font Size';
    d.children.add(p);

    InputElement i = new InputElement()
        ..id = 'font-size-input'
        ..type = 'text';
    d.children.add(i);

    return li;
  }

  LIElement _createToggleItem(String title) {
    String id = title.toLowerCase().replaceAll(' ', '-');

    LIElement buttonList = new LIElement();
    AnchorElement button = new AnchorElement()
        ..id = 'button-$id'
        ..href = '#'
        ..attributes['role'] = 'button'
        ..text = title;
    buttonList.children.add(button);

    return buttonList;
  }
}