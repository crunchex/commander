/*
 * Gamepad API Test
 * Written in 2013 by Ted Mielczarek <ted@mielczarek.org>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */
var haveEvents = 'GamepadEvent' in window;
var controllers = {};
var axes = [];
var buttons = [];

function connecthandler(e) {
    addgamepad(e.gamepad);
}

function addgamepad(gamepad) {
    controllers[gamepad.index] = gamepad;
    updateStatus();
}

function disconnecthandler(e) {
    removegamepad(e.gamepad);
}

function removegamepad(gamepad) {
    delete controllers[gamepad.index];
}

function updateStatus() {
    scangamepads();

    var controller = controllers[0];
    if (controller != null) {
        for (var k = 0; k < controller.axes.length; k++) {
            if (k == 0 && (controller.axes[1] == 1 || controller.axes[1] == -1)) {
                axes[k].innerHTML = 0.0;
            } else {
                axes[k].innerHTML = -controller.axes[k];
            }
        }

        for (var l = 0; l < controller.buttons.length; l++) {
            var val = controller.buttons[l];
            var pressed = val == 1.0;
            if (typeof(val) == "object") {
                pressed = val.pressed;
                val = val.value;
            }

            if (pressed) {
                buttons[l].innerHTML = 1;
            } else {
                buttons[l].innerHTML = 0;
            }
        }
    }
}

function scangamepads() {
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
            if (!(gamepads[i].index in controllers)) {
                addgamepad(gamepads[i]);
            } else {
                controllers[gamepads[i].index] = gamepads[i];
            }
        }
    }
}

function startScanning(idNum) {
    for (var i = 0; i < 4; i++) {
        axes[i] = document.getElementById('updroidteleop-' + idNum + '-axis-data-' + i);
    }

    for (var j = 0; j < 17; j++) {
        buttons[j] = document.getElementById('updroidteleop-' + idNum + '-button-data-' + j);
    }

    if (haveEvents) {
        window.addEventListener("gamepadconnected", connecthandler);
        window.addEventListener("gamepaddisconnected", disconnecthandler);
    } else {
        setInterval(scangamepads, 500);
    }
}