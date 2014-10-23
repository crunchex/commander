A repo for the user-interface to be installed on the UpDroid UP1. Contains two parts, a server and a client, both written in Dart.

The server runs commands on the system side, file operations or commands that user enters as input. It also serves the client side upon HTTP requests.

The client runs in the user's browser as UpDroid Commander, a graphical interface that includes a simple file explorer, file editor, and command console.

Once the client is loaded in the browser, it communicates with the server via a WebSocket connection.