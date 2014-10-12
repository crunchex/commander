import 'package:tokenizr/tokenizr.dart';

class Command extends Token {
  get regexProto => '^[a-zA-Z_.-]+';
  get nextState => StdState;
}

class Flag extends Token {
  get regexProto => '^-[a-zA-Z_.-]+';
  get nextState => StdState;
}

class Whitespace extends IgnoreToken {
  get regexProto => " ";
  get nextState => StdState;
}

class StdState extends LexerState {
  get tokens => [Command, Flag, Whitespace];
}

class ExpLexer extends Lexer {
  get startState => StdState;
  get states => [StdState];
}

List parseCommandInput(String input) {
  var lexer = new ExpLexer();
  var tokens = lexer.run(input);
  List tokenValues = new List();
  if (tokens.length > 1) {
    for (var token in tokens.getRange(1, tokens.length)) {
      tokenValues.add(token.content);
    }
  }
  return [tokens[0].content, tokenValues];
}
