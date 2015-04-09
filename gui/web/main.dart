import 'package:bootjack/bootjack.dart';

import 'client.dart';

void main() {
  setUpBootstrap();
  new UpDroidClient();
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  //Modal.use();
  Transition.use();
}