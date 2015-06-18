part of updroid_explorer;

abstract class ExplorerView {
  DivElement content;
  DivElement explorersDiv;
  DivElement drop;

  ButtonElement _controlToggle;
  DivElement _explorer;
  DivElement _hrContainer;

  ExplorerView(int id, DivElement content) {
    this.content = content;

    explorersDiv = new DivElement()
      ..classes.add('exp-container');
    content.children.add(explorersDiv);
  }
}