import 'dart:html';
import 'package:bootjack/bootjack.dart';

import 'client.dart';

void main() {
  setUpBootstrap();
  UpDroidClient client = new UpDroidClient();
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  //Modal.use();
  Transition.use();
  
  Popover.wire(querySelector('#console-help.dropdown-toggle'));
}