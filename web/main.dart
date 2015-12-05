import 'dart:html';
import 'dart:async';

import 'package:bootjack/bootjack.dart';

import 'lib/client.dart';

void main() {
  setUpBootstrap();

  // To enable login, set below bool to true
  // and comment out .login-overlay display: none in main.css.
  bool enableLogin = false;

  new UpDroidClient();

  String userAgent = window.navigator.userAgent;
  if (userAgent.contains('Mobile') || !enableLogin) {
    return;
//    loginHeading.classes.add('mobile');
//    window.scrollTo(0, 1);
  }

  DivElement loginOverlay = querySelector('#login-overlay');
  DivElement loginLogoDiv = querySelector('#login-logo-div');
  DivElement loginHeading = querySelector('#login-heading');
  InputElement username = querySelector('#login-username-input');
  InputElement password = querySelector('#login-password-input');

  loginOverlay.style.display = 'block';

  new Timer(new Duration(seconds: 1), () {
    loginLogoDiv.classes.add('animate-end');
    loginHeading.classes.add('animate-end');
    username.classes.add('animate-end');
    password.classes.add('animate-end');

    List<StreamSubscription> subs = [];

    subs.add(username.onKeyUp.listen((e) {
      checkCredentials(e, username, password, loginOverlay, loginHeading, subs);
    }));

    subs.add(password.onKeyUp.listen((e) {
      checkCredentials(e, username, password, loginOverlay, loginHeading, subs);
    }));
  });
}

void checkCredentials(KeyboardEvent e, InputElement username, InputElement password, DivElement loginOverlay, DivElement loginHeading, List<StreamSubscription> subs) {
  var keyEvent = new KeyEvent.wrap(e);
  if (keyEvent.keyCode == KeyCode.ENTER) {
    if (username.value != 'updroid' || password.value != 'updroid') return;

    loginHeading.text = 'Loading...';
    subs.forEach((sub) => sub.cancel());

    new Timer(new Duration(milliseconds: 1000), () {
      loginHeading.text = 'Loading complete.';
      loginOverlay.classes.add('granted');
      new Timer(new Duration(milliseconds: 800), () {
        loginOverlay.style.display = 'none';
      });
    });
  }
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  Transition.use();
}