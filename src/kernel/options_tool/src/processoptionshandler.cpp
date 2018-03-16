#include "options_tool/processoptionshandler.hpp"
#include <iostream>

void defaultSignalHandler(boost::optional<ProcPropTreePtr> pt) {
    std::cout << "Default `signal` handler called. Signal code: " << (*pt)->getString("sigcode") << std::endl;
}

void defaultSetHandler(boost::optional<ProcPropTreePtr> pt) {
    std::cout << "Default `set` handler called." << std::endl;
}

void defaultRunHandler(boost::optional<ProcPropTreePtr> pt) {
    std::cout << "Default `run` handler called." << std::endl;
}

JsonProcessOptionsHandler::JsonProcessOptionsHandler(const std::string & jsonSchemaPath, bool emptyHandlers,
                              std::shared_ptr<const PropertyTreeDecomposer> cmdDecomposer)
    : optTool(jsonSchemaPath,false), emptyHandlers(emptyHandlers), cmdDecomposer(cmdDecomposer)
{
}

std::pair<std::string, boost::optional<ProcPropTreePtr>> 
JsonOptionCommandDecomposer::decomposeCommand(const ProcPropTreePtr & cmd) const {
    std::string cmdname = cmd->getString("command.name");
    auto args = cmd->getSubtreeOptional("command.args");
    return std::make_pair<std::string, boost::optional<ProcPropTreePtr>>(std::move(cmdname), std::move(args));
}

void JsonProcessOptionsHandler::processOptions(const std::string & options) {
    ProcPropTreePtr optTree;
    {
        std::lock_guard<std::mutex> lk(mxOptTool);
        optTree = std::move(optTool.parseOptionsString(options)); // this may throw
    } 
    auto cmd = cmdDecomposer->decomposeCommand(optTree);
    if (!commandHandlers.count(cmd.first)) { 
        if (emptyHandlers)
            commandHandlers[cmd.first] = [](boost::optional<ProcPropTreePtr>) {};
        else
            throw std::runtime_error("No command handler found for " + cmd.first);
    }
    commandHandlers[cmd.first](cmd.second);
}

DefaultProcessOptionsHandler::DefaultProcessOptionsHandler(const std::string & jsonSchemaPath, bool emptyHandlers) 
    : JsonProcessOptionsHandler(jsonSchemaPath, emptyHandlers)
{
    setCommandHandler("signal", defaultSignalHandler);
    setCommandHandler("set", defaultSetHandler);
    setCommandHandler("run", defaultRunHandler);
}
