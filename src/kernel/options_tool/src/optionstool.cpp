#include "options_tool/optionstool.hpp"

IncorrectOptionsSyntaxError::IncorrectOptionsSyntaxError(const std::string addInfo) 
    : std::runtime_error("Incorrect options syntax error. " + addInfo) {}

