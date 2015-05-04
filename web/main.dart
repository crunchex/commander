import 'package:bootjack/bootjack.dart';

import 'lib/client.dart';

void main() {
  setUpBootstrap();
  new UpDroidClient();
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  Transition.use();
}