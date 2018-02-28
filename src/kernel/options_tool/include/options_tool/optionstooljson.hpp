#ifndef OPTIONS_TOOL_JSON_H_
#define OPTIONS_TOOL_JSON_H_

#include "optionstool.hpp"
#include "procptree.hpp"
#include <functional>
#include <sstream>
#include <utility>
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <options_tool/valijson/schema.hpp>
#include <options_tool/valijson/schema_parser.hpp>
#include <options_tool/valijson/validator.hpp>

/**
 * @brief Implementation of OptionsTool. Validates given options using schema given 
 * to constructor, transforms it to Property tree. (And backwards)
 */
class OptionsToolJSON : public OptionsTool<std::shared_ptr<ProcPropTree>> {
public:
    typedef std::shared_ptr<ProcPropTree> TreeTp;
private:
    std::hash<std::string> hashFn;
    std::pair<size_t, bool> hashCorrect;
    valijson::Validator validator;
    valijson::Schema schema;
    boost::property_tree::ptree successPtree;
    bool validate;
public:
    /**
     * @brief Ctor.
     * @param jsonSchemaPath FS path to schema used for option validating. (see http://json-schema.org)
     */
    OptionsToolJSON(const std::string & jsonSchemaPath, bool validate = true);
    
    /**
     * @brief Inherited from OptionsTool
     */
    virtual TreeTp parseOptionsString(const std::string & options);
    /**
     * @brief Inherited from OptionsTool
     */
    virtual bool isSyntaxCorrect(const std::string & options);
    /**
     * @brief Inherited from OptionsTool
     */
    virtual std::string getOptionsString(const TreeTp & optionsMap);
};

#endif // OPTIONS_TOOL_JSON_H_
