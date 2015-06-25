part of updroid_explorer;

abstract class ExplorerView {
  DivElement content;
  DivElement explorersDiv;

  ExplorerView(int id, DivElement content) {
    this.content = content;

    explorersDiv = new DivElement()
      ..classes.add('exp-container');
    content.children.add(explorersDiv);
  }
}