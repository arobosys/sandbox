#ifndef OPTIONS_TOOL_H_
#define OPTIONS_TOOL_H_

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <stdexcept>

/**
 * @brief Exception class thrown when syntax error occured while parsing options
 */
struct IncorrectOptionsSyntaxError : public std::runtime_error {
    IncorrectOptionsSyntaxError(const std::string addInfo = "");
};

/**
 * @brief Interface for classes which implement options (de)serializing
 */
template <typename OptionsMapT>
struct OptionsTool {
    /**
     * @brief convert options string to given OptionsMapT container. IncorrectOptionsSyntaxError may throw. 
     */
    virtual OptionsMapT parseOptionsString(const std::string & options) = 0;

    /**
     * @brief checks if options can be converted to OptionsMapT container using parseOptionsString. Throws no exceptions.
     * @return true if syntax correct, false otherwise
     */
    virtual bool isSyntaxCorrect(const std::string & options) = 0;

    /**
     * @brief converts OptionsMapT container to string
     */
    virtual std::string getOptionsString(const OptionsMapT & optionsMap) = 0;
    virtual ~OptionsTool() {}
};

#endif // OPTIONS_TOOL_H_
