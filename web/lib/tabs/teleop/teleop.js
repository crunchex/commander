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
var axis0, axis1, axis2, axis3, buttons;

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

    var status = [];

    if (controller != null) {
        for (var i = 0; i < controller.axes.length; i++) {
            status[i] = controller.axes[i];
        }

        axis0.innerHTML = "Axis 0: " + status[0];
        axis1.innerHTML = "Axis 1: " + status[1];
        axis2.innerHTML = "Axis 2: " + status[2];
        axis3.innerHTML = "Axis 3: " + status[3];

        var buttonsString = "[";
        for (var j = 0; i < controller.buttons.length; i++) {
            var val = controller.buttons[i];
            var pressed = val == 1.0;
            if (typeof(val) == "object") {
                pressed = val.pressed;
                val = val.value;
            }

            if (pressed) {
                status[i+controller.axes.length] = 1;
            } else {
                status[i+controller.axes.length] = 0;
            }

            buttonsString += " " + status[i+controller.axes.length];
        }
        buttonsString += " ]";
        buttons.innerHTML = buttonsString;
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
    axis0 = document.getElementById('updroidteleop-' + idNum + '-axis-0');
    axis1 = document.getElementById('updroidteleop-' + idNum + '-axis-1');
    axis2 = document.getElementById('updroidteleop-' + idNum + '-axis-2');
    axis3 = document.getElementById('updroidteleop-' + idNum + '-axis-3');

    buttons = document.getElementById('updroidteleop-' + idNum + '-buttons');

    if (haveEvents) {
        window.addEventListener("gamepadconnected", connecthandler);
        window.addEventListener("gamepaddisconnected", disconnecthandler);
    } else {
        setInterval(scangamepads, 500);
    }
}