import 'dart:html';

abstract class ColumnView {
  int id;
  DivElement columnContent, tabContent;
  UListElement navTabs;
  DivElement rowMain;

  ColumnView(this.id) {
    rowMain = querySelector('#row-main');

    columnContent = new DivElement()
      ..id = 'column-$id';
    rowMain.children.add(columnContent);

    navTabs = new UListElement()
      ..classes.addAll(['nav', 'nav-tabs'])
      ..attributes['role'] = 'tablist';
    columnContent.children.add(navTabs);

    tabContent = new DivElement()
      ..id = 'col-$id-tab-content'
      ..classes.add('tab-content');
    columnContent.children.add(tabContent);
  }

  void cleanUp();
}