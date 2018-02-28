#ifndef PROCESS_OPTIONS_HANDLER_H_
#define PROCESS_OPTIONS_HANDLER_H_

#include "optionstooljson.hpp"
#include "procptree.hpp"
#include <utility>
#include <unordered_map>
#include <string>
#include <functional>
#include <memory>
#include <boost/optional.hpp>
#include <mutex>

template <typename ArgTp, typename CmdTp>
struct OptionCommandDecomposer {
    /**
     * @brief splits given command into its name and its arguments
     */
    virtual std::pair<std::string, boost::optional<ArgTp>> decomposeCommand(const CmdTp & cmd) const = 0;
    virtual ~OptionCommandDecomposer() {}
};

using PropertyTreeDecomposer = OptionCommandDecomposer<ProcPropTreePtr, ProcPropTreePtr>;

struct JsonOptionCommandDecomposer : public PropertyTreeDecomposer
{
    /**
     * @brief splits given command into its name and its arguments (uses command.name and command.args from json schema)
     */
    virtual std::pair<std::string, boost::optional<ProcPropTreePtr>> decomposeCommand(const ProcPropTreePtr & cmd) const;
};

template <typename HandlerTp>
struct ProcessOptionsHandler {
    /**
     * @brief Parse options string and call (blocking) handlers according to command name.
     * @param options serialized options string 
     */
    virtual void processOptions(const std::string & options) = 0;

    /**
     * @brief set handler for specific command.
     */
    virtual void setCommandHandler(const std::string & command, HandlerTp hdl) = 0;
    virtual ~ProcessOptionsHandler() {}
};

typedef std::function<void(boost::optional<ProcPropTreePtr>)> PropertyTreeHandler;
/**
 * @brief Class which maps options strings to some actions, and run this actions.
 */
class JsonProcessOptionsHandler : public ProcessOptionsHandler<PropertyTreeHandler> {
public:
private:
    std::mutex mxOptTool;
    OptionsToolJSON optTool;
    bool emptyHandlers;
    std::shared_ptr<const PropertyTreeDecomposer> cmdDecomposer;

protected:
    std::unordered_map<std::string, PropertyTreeHandler> commandHandlers;

public:
    /**
     * @brief Ctor
     * @param jsonSchemaPath see OptionToolJSON class
     * @param emptyHandlers if false, processOptions throws std::runtime_error in case of command serialized in options has no handler 
     * @param cmdDecomposer used for decomposing command into its name and its args
     */
    JsonProcessOptionsHandler(const std::string & jsonSchemaPath, bool emptyHandlers = false,
                              std::shared_ptr<const PropertyTreeDecomposer> cmdDecomposer = 
                                 std::dynamic_pointer_cast<const PropertyTreeDecomposer>(std::make_shared<JsonOptionCommandDecomposer>()));

    /**
     * @brief Parse options string and call (blocking) handlers according to command name. Can throw exception (see emptyHandlers param
     * in constructor)
     * @param options serialized options string according to jsonSchemaPath given to constructor
     */
    virtual void processOptions(const std::string & options); 

    /**
     * @brief set handler for command. Tree handler is function returning void, it gives ProcPropTreePtr as argument,
     * (that's command.args subtree)
     */
    virtual void setCommandHandler(const std::string & command, PropertyTreeHandler hdl) {
        commandHandlers[command] = hdl;
    }


};

/**
 * @brief this class supplies some default handlers (just for debugging)
 */
struct DefaultProcessOptionsHandler : public JsonProcessOptionsHandler {
    /**
     * @brief see superclass ctor
     */
    DefaultProcessOptionsHandler(const std::string & jsonSchemaPath, bool emptyHandlers = false);
};

#endif // PROCESS_OPTIONS_HANDLER_H_
