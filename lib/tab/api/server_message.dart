library server_message;

import 'updroid_message.dart';

class ServerMessage {
  String receiverClass;
  int id;
  UpDroidMessage um;

  /// Constructs a new [ServerMessage] where [receiverClass] is a class type,
  /// such as 'UpDroidClient'. [id] can be -1 for the destination with the lowest
  /// registered id number, 0 for all destinations of type [receiverClass], or
  /// any positive integer for one specific destination.
  ServerMessage(this.receiverClass, this.id, this.um);
}