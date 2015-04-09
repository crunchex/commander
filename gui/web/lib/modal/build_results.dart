part of updroid_modal;

class UpDroidBuildResultsModal extends UpDroidModal {
  UpDroidBuildResultsModal(String results) {
    _initModal('Build Results');
    _setupBody(results);
    _setupFooter();

    _showModal();
  }

  void _setupBody(String results) {
    _modalBase.id = "build-results";

    ParagraphElement p = new ParagraphElement();
    if (results == '') {
      p.text = 'Success!';
      _modalBody.children.add(p);
    } else {
      p.text = 'Your build was unsuccessful:\n\n';
      _modalBody.children.add(p);
      PreElement pre = new PreElement()
        ..text = results;
      _modalBody.children.add(pre);
    }
  }

  void _setupFooter() {
    var okay = _createButton('primary', 'Okay');
    _modalFooter.children.insert(0, okay);
  }
}