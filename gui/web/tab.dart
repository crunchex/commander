part of updroid_client;

class UpDroidTab {

  static Future setUpTabContainer() {
    Completer completer = new Completer();

    DivElement tabContainer = new DivElement();
    tabContainer.id = 'tab-editor-container';
    tabContainer.classes.add('tab-pane');
    tabContainer.classes.add('active');

    UListElement tabList = new UListElement();
    tabList
        ..classes.add('nav')
        ..classes.add('nav-tabs')
        ..classes.add('inner-tabs')
        ..attributes['role'] = 'tablist';

    tabList.children.add(_createDropdown('File', ['New', 'Save', 'Save As']));
    tabList.children.add(_createDropdown2('Settings'));
    LIElement filename = new LIElement();
    filename.id = 'filename';
    tabList.children.add(filename);

    tabContainer.children.add(tabList);

    DivElement tabContent = new DivElement();
    tabContent.classes.add('tab-content');
    tabContent.classes.add('active');
    DivElement content = new DivElement();
    content
        ..id = 'editor'
        ..classes.add('editor');
    tabContent.children.add(content);
    tabContainer.children.add(tabContent);

    completer.complete(tabContainer);
    return completer.future;
  }

  static LIElement _createDropdown(String title, List buttons) {
    LIElement dropdown = new LIElement();
    dropdown.classes.add('dropdown');

    AnchorElement dropdownToggle = new AnchorElement();
    dropdownToggle
        ..href = '#'
        ..classes.add('dropdown-toggle')
        ..dataset['toggle'] = 'dropdown'
        ..text = title;
    dropdown.children.add(dropdownToggle);

    UListElement dropdownMenu = new UListElement();
    dropdownMenu
        ..classes.add('dropdown-menu')
        ..attributes['role'] = 'menu';

    for (String title in buttons) {
      dropdownMenu.children.add(_createButton(title));
    }

    dropdown.children.add(dropdownMenu);

    return dropdown;
  }

  static LIElement _createDropdown2(String title) {
    LIElement dropdown = new LIElement();
    dropdown.classes.add('dropdown');

    AnchorElement dropdownToggle = new AnchorElement();
    dropdownToggle
        ..href = '#'
        ..classes.add('dropdown-toggle')
        ..dataset['toggle'] = 'dropdown'
        ..text = title;
    dropdown.children.add(dropdownToggle);

    UListElement dropdownMenu = new UListElement();
    dropdownMenu
        ..classes.add('dropdown-menu')
        ..attributes['role'] = 'menu';

    LIElement buttonTheme = _createButton('Theme');
    buttonTheme.children[0].id = 'button-editor-theme';
    LIElement li = new LIElement();
    li.style.textAlign = 'center';
    DivElement d = new DivElement();
    d.style.display = 'inline-block';
    ParagraphElement p = new ParagraphElement();
    p.style.display = 'inline-block';
    p.text = 'Font Size';
    d.children.add(p);
    InputElement i = new InputElement();
    i.id = 'font-size-input';
    i.type = 'text';
    d.children.add(i);
    li.children.add(d);
    buttonTheme.children.add(li);

    dropdownMenu.children.add(buttonTheme);

    dropdown.children.add(dropdownMenu);

    return dropdown;
  }

  static LIElement _createButton(String title) {
    String id = title.toLowerCase().replaceAll(' ', '-');

    LIElement buttonList = new LIElement();
    AnchorElement button = new AnchorElement();
    button
        ..id = 'button-$id'
        ..href = '#'
        ..attributes['role'] = 'button'
        ..text = title;
    buttonList.children.add(button);

    return buttonList;
  }
}

//<li class="dropdown"><a href="#" class="dropdown-toggle" data-toggle="dropdown">Settings</a>
//  <ul class="dropdown-menu" role="menu">
//    <li><a href="#" id="button-editor-theme">Theme</a></li>
//      <li style="text-align: center">
//        <div style="display: inline-block;">
//          <p style="display: inline-block;">Font Size</p><input id="font-size-input" type="text">
//        </div>
//      </li>
//  </ul>
//</li>
//<li id="filename"></li>