part of updroid_modal;

class UpDroidBuildResultsModal extends UpDroidModal {
  UpDroidBuildResultsModal (String results) {
    _initModal('Build Results');
    _setupModal(results);
    _showModal();
  }

  void _setupModal(String results) {
    _modalBase.id = "build-results";

    var p = new Element.p();
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

    var okay = _createButton('primary', 'Okay');
    _modalFooter.children.insert(0, okay);
  }
}