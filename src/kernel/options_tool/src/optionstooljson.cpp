#include "options_tool/optionstooljson.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <options_tool/valijson/adapters/property_tree_adapter.hpp>
#include <options_tool/valijson/utils/property_tree_utils.hpp>
#include "options_tool/procptree_boostptree.hpp"

OptionsToolJSON::OptionsToolJSON(const std::string & jsonSchemaPath, bool validate) 
    : hashCorrect(0, false), validate(validate)
{
    if (validate) { 
        boost::property_tree::ptree schemaDoc;
        if (!valijson::utils::loadDocument(jsonSchemaPath, schemaDoc)) {
            throw std::runtime_error("Failed to load schema document");
        }
    ProcPropTreeBoost pt(schemaDoc);
    //std::cout << "Jsonized: " << pt.JSONize() << std::endl;
        valijson::SchemaParser parser;
        valijson::adapters::PropertyTreeAdapter schemaAdapter(schemaDoc);
        parser.populateSchema(schemaAdapter, schema);
    }
}

OptionsToolJSON::TreeTp OptionsToolJSON::parseOptionsString(const std::string & options) {
    //std::cout << "parseOptionsString: " << options << std::endl; fflush(stdout);
    if (!isSyntaxCorrect(options) && validate) {
        throw IncorrectOptionsSyntaxError();
    } else {
        return std::dynamic_pointer_cast<ProcPropTree>(std::make_shared<ProcPropTreeBoost>(successPtree));
        //return successPtree;
    }
}

bool OptionsToolJSON::isSyntaxCorrect(const std::string & options) {
    const size_t hash = hashFn(options);
    if (hash == hashCorrect.first) {
        //std::cout << "returning hash" << std::endl;
        return hashCorrect.second;
    }
    hashCorrect.first = hash;

    bool & correct = hashCorrect.second;

    try {
        std::stringstream ss(options);        
        boost::property_tree::read_json(ss, successPtree);
    } catch (boost::property_tree::json_parser::json_parser_error & ex) {
        //std::cerr << "read json fault!" << std::endl;
        return correct = false;
    }
    ProcPropTreeBoost pt(successPtree);
    //std::cout << "Jsonized suc ptre: " << pt.JSONize() << std::endl;

    valijson::adapters::PropertyTreeAdapter ptAdapter(successPtree);
    correct = validator.validate(schema, ptAdapter, NULL);
    //std::cout << "validate result: " << correct << std::endl;
    return correct;
}

std::string OptionsToolJSON::getOptionsString(const OptionsToolJSON::TreeTp & optionsMap) {
    return optionsMap->JSONize();
}
