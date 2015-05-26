import 'dart:html';
import 'dart:async';

import 'package:bootjack/bootjack.dart';

import 'lib/client.dart';

void main() {
  setUpBootstrap();

  // To enable login, set below bool to true
  // and comment out .login-overlay display: none in main.css.
  bool enableLogin = false;

  if (!enableLogin) {
    new UpDroidClient();
    return;
  }

  DivElement loginOverlay = querySelector('#login-overlay');
  DivElement loginLogo = querySelector('#login-logo');
  DivElement loginHeading = querySelector('#login-heading');
  InputElement username = querySelector('#login-username-input');
  InputElement password = querySelector('#login-password-input');

  Timer t = new Timer(new Duration(seconds: 1), () {
    loginLogo.classes.add('animate-end');
    loginHeading.classes.add('animate-end');
    username.classes.add('animate-end');
    password.classes.add('animate-end');

    List<StreamSubscription> subs = [];

    subs.add(username.onKeyUp.listen((e) {
      checkCredentials(e, username, password, loginOverlay, subs);
    }));

    subs.add(password.onKeyUp.listen((e) {
      checkCredentials(e, username, password, loginOverlay, subs);
    }));
  });
}

void checkCredentials(KeyboardEvent e, InputElement username, InputElement password, DivElement loginOverlay, List<StreamSubscription> subs) {
  var keyEvent = new KeyEvent.wrap(e);
  if (keyEvent.keyCode == KeyCode.ENTER) {
    if (username.value == 'updroid' && password.value == 'weareupdroid') {
      subs.forEach((sub) => sub.cancel());
      new UpDroidClient();
      loginOverlay.classes.add('granted');
      Timer t = new Timer(new Duration(milliseconds: 800), () {
        loginOverlay.style.display = 'none';
      });
    }
  }
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  Transition.use();
}