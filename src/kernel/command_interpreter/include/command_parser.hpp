#ifndef COMMAND_PARSER_HPP
#define COMMAND_PARSER_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include <core_msgs/CI_to_HLLAction.h>

/**
 * Class which describes exception thrown when parsing in Command Interpreter
 */
struct ParseError : public std::runtime_error {
    ParseError(const std::string & s) : std::runtime_error("ParseError: " + s) {}
};

/**
 * Parser for input commands
 */
struct command_parser {
    /**
     * String to print manual for this parser's input commands
     */
    static const std::string usage;

    /**
     * Allowed "commands", i.e. first words in the commands
     */
    static const std::string START;
    static const std::string STOP;
    static const std::string RESTART;

    /**
     * Method to parse users commands
     * @param command user command
     * @return parse_result structure
     * @throws ParseError in case string is incompatible with patterns
     */
    static core_msgs::CI_to_HLLGoal parse(const std::string& command);
};

#endif // COMMAND_PARSER_HPP
