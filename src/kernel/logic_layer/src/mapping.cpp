//
// Created by vladislav on 24.09.16.
//

#include "logic_layer/mapping.hpp"

DatabasePtr Mapping::DATABASE = nullptr;

const char* Mapping::HLLScenarioQuery = "SELECT fldIdSc, fldStartId FROM tbScenario WHERE fldName=?1";

const char* Mapping::HLLEdgesQuery = "SELECT fldIdEdge, fldIdMapFrom, fldIdMapTo, fldState, fldStatePriority "
        "FROM tbScenarioStructure AS ss JOIN tbState AS s ON ss.fldIdState=s.fldIdState WHERE fldIdSc=?1";

const char* Mapping::HLLNodesQuery = "SELECT t2.fldIdMap, fldIsOut, fldState FROM ((SELECT fldIdMapFrom "
        "FROM tbScenarioStructure WHERE fldIdSc=?1 UNION SELECT fldIdMapTo FROM tbScenarioStructure "
        "WHERE fldIdSc=?1) AS t1 JOIN tbMap AS t2 ON t1.fldIdMapFrom=t2.fldIdMap LEFT JOIN tbMapOutState AS t3 "
        "ON t2.fldIdMap=t3.fldIdMap LEFT JOIN tbState AS t4 ON t3.fldIdState=t4.fldIdState)";

const char* Mapping::HLLParamsQuery = "SELECT DISTINCT t3.fldIdProc, t1.fldIdTL, fldKey, fldDefValue, "
        "printf('%d/%d/%d', ?1, t1.fldIdTL, t3.fldIdProc) AS fldPath FROM (SELECT fldIdTLFrom AS fldIdTL "
        "FROM tbMapStructure WHERE fldIdMap=?1 UNION SELECT fldIdTLTo FROM tbMapStructure WHERE fldIdMap=?1) AS t1 "
        "JOIN tbTaskListStructure AS t2 ON t1.fldIdTL=t2.fldIdTL JOIN tbProcess AS t3 ON t2.fldIdProc=t3.fldIdProc "
        "JOIN tbOption AS t4 ON t3.fldIdProc=t4.fldIdProc AND t1.fldIdTL=t4.fldIdTL";

const char* Mapping::LLLStartIdQuery = "SELECT fldStartId FROM tbMap WHERE fldIdMap=?1";

const char* Mapping::LLLEdgesQuery = "SELECT fldIdEdge, fldIdTLFrom, fldIdTLTo, fldState, fldStatePriority "
        "FROM tbMapStructure AS ms JOIN tbState AS s ON ms.fldIdState=s.fldIdState WHERE fldIdMap=?1";

const char* Mapping::LLLNodesQuery = "SELECT t2.fldIdTL, fldIsOut, fldIdRequest, fldState FROM ((SELECT fldIdTLFrom "
        "FROM tbMapStructure WHERE fldIdMap=?1 UNION SELECT fldIdTLTo FROM tbMapStructure WHERE fldIdMap=?1) AS t1 "
        "JOIN tbTaskList AS t2 ON t1.fldIdTLFrom=t2.fldIdTL LEFT JOIN tbTaskListOutState AS t3 "
        "ON t2.fldIdTL=t3.fldIdTL LEFT JOIN tbState AS t4 ON t3.fldIdState=t4.fldIdState)";

const char* Mapping::LLLTLQuery = "SELECT t2.fldIdProc, fldName, fldPriority, fldCommand, fldFilepath FROM tbTaskListStructure "
        "AS t1 JOIN tbProcess AS t2 ON t1.fldIdProc=t2.fldIdProc WHERE fldIdTL=?1";

const char* Mapping::LLLParamsQuery = "SELECT DISTINCT t2.fldIdProc, t3.fldIdTL, fldKey, fldDefValue, "
        "printf('%d/%d', t3.fldIdTL, t3.fldIdProc) AS fldPath FROM tbTaskListStructure AS t1 JOIN tbProcess "
        "AS t2 ON t1.fldIdProc=t2.fldIdProc JOIN tbOption AS t3 ON t2.fldIdProc=t3.fldIdProc WHERE t3.fldIdTL=?1";

const char* Mapping::ChildProcessesParamsQuery = "WITH RECURSIVE tbProc(id, path) AS (SELECT fldIdChildProc, "
        "printf('/%d', fldIdChildProc) FROM tbParentToChildProcess WHERE fldIdParentProc=?1 UNION ALL "
        "SELECT fldIdChildProc, printf('%s/%d', path, fldIdChildProc) FROM tbProc, tbParentToChildProcess "
        "WHERE id=fldIdParentProc) SELECT DISTINCT id AS fldIdProc, fldIdTL, path AS fldPath, fldKey, fldDefValue "
        "FROM tbProc JOIN tbOption AS t1 ON tbProc.id=t1.fldIdProc WHERE fldIdTL=?2";

const char* Mapping::ChildProcessesQuery = "WITH RECURSIVE tbProc(id) AS (SELECT fldIdChildProc FROM "
        "tbParentToChildProcess WHERE fldIdParentProc=?1 UNION ALL SELECT fldIdChildProc FROM tbProc, "
        "tbParentToChildProcess WHERE id=fldIdParentProc) SELECT fldIdProc, fldName, fldFilepath FROM tbProc "
        "JOIN tbProcess ON id=fldIdProc";

const char* Mapping::ProcessGroupQuery = "SELECT pg.fldIdGroup, fldKey, fldValue, fldPriority, fldIsRequest, "
        "fldIdState, fldIdRequest FROM tbProcessGroup AS pg JOIN tbProcessParam AS p ON pg.fldIdGroup=p.fldIdGroup "
        "WHERE fldIdProc=?1 ORDER BY pg.fldIdGroup";

const char* Mapping::StateQuery = "SELECT fldState FROM tbState WHERE fldIdState=?1";

const char* Mapping::RequestQuery = "SELECT fldText, fldParam, fldIsVariable, fldName, fldState FROM tbRequest AS r "
        "LEFT JOIN tbRequestValue AS v ON r.fldIdRequest=v.fldIdRequest LEFT JOIN tbState AS s "
        "ON v.fldIdState=s.fldIdState WHERE r.fldIdRequest=?1";

void Mapping::setDatabase(const DatabasePtr& database) {
    DATABASE = database;
}

ScenarioExecutorPtr Mapping::get_scenario_by_name(const std::string& name) {
    int id, startId;
    DATABASE->execQuery(HLLScenarioQuery,
                        [&] (const ResultSet& resultSet) {
                            resultSet.next();
                            id = resultSet.getInteger("fldIdSc");
                            startId = resultSet.getInteger("fldStartId");
                        }
            , name);

    ScenarioExecutor::nodes_type nodes;
    DATABASE->execQuery(HLLNodesQuery,
                        [&] (const ResultSet& resultSet) {
                            while (resultSet.next()) {
                                int idMap = resultSet.getInteger("fldIdMap");
                                bool isOut = resultSet.getBoolean("fldIsOut");
                                std::string state = resultSet.isNull("fldState") ? "" : resultSet.getString("fldState");

                                NodePtr node = std::make_shared<HLL::ScenarioNode>(idMap, isOut, state);
                                nodes.push_back(node);
                            }
                        }
            , id);

    ScenarioExecutor::edges_type edges;
    DATABASE->execQuery(HLLEdgesQuery,
                        [&] (const ResultSet& resultSet) {
                            while (resultSet.next()) {
                                int idEdge = resultSet.getInteger("fldIdEdge");
                                int idFrom = resultSet.getInteger("fldIdMapFrom");
                                int idTo = resultSet.getInteger("fldIdMapTo");
                                std::string state = resultSet.getString("fldState");
                                int priority = resultSet.getInteger("fldStatePriority");

                                NodePtr from, to;
                                auto itFrom = std::find_if(nodes.begin(), nodes.end(),
                                                     [idFrom] (const NodePtr& nodePtr) {
                                                         return nodePtr->getID() == idFrom;
                                                     });
                                auto itTo = std::find_if(nodes.begin(), nodes.end(),
                                                   [idTo] (const NodePtr& nodePtr) {
                                                       return nodePtr->getID() == idTo;
                                                   });
                                if (itFrom != nodes.end()) from = *itFrom;
                                if (itTo != nodes.end()) to = *itTo;

                                EdgePtr edge = std::make_shared<Edge>(idEdge, from, to, state, priority);
                                edges.push_back(edge);
                            }
                        }
            , id);

    ScenarioExecutor::params_type paramsVec;
    for (const auto& n : nodes) {
        DATABASE->execQuery(HLLParamsQuery,
                            [&] (const ResultSet &resultSet) {
                                get_params(paramsVec, resultSet);
                            }, n->getID());
    }

    return std::make_shared<ScenarioExecutor>(id, nodes, edges, startId, paramsVec);
}

ScenarioExecutorPtr Mapping::get_tech_map_by_id(int id) {
    int startId;
    DATABASE->execQuery(LLLStartIdQuery,
                        [&] (const ResultSet& resultSet) {
                            resultSet.next();
                            startId = resultSet.getInteger("fldStartId");
                        }
            , id);

    ScenarioExecutor::nodes_type nodes;
    DATABASE->execQuery(LLLNodesQuery,
                        [&] (const ResultSet& resultSet) {
                            while (resultSet.next()) {
                                int idTL = resultSet.getInteger("fldIdTL");
                                bool isOut = resultSet.getBoolean("fldIsOut");
                                std::string state = resultSet.isNull("fldState") ? "" : resultSet.getString("fldState");

                                ProcessRequestPtr request = nullptr;
                                if (!resultSet.isNull("fldIdRequest")) {
                                    int requestId = resultSet.getInteger("fldIdRequest");
                                    request = std::make_shared<ProcessRequest>(get_request(requestId));
                                }

                                ProcessVec pData;
                                get_task_list_by_id(idTL, pData);
                                NodePtr node = std::make_shared<LLL::TechMapNode>(idTL, isOut, state, pData, request);

                                nodes.push_back(node);
                            }
                        }
            , id);

    ScenarioExecutor::edges_type edges;
    DATABASE->execQuery(LLLEdgesQuery,
                        [&] (const ResultSet& resultSet) {
                            while (resultSet.next()) {
                                int idEdge = resultSet.getInteger("fldIdEdge");
                                int idFrom = resultSet.getInteger("fldIdTLFrom");
                                int idTo = resultSet.getInteger("fldIdTLTo");
                                std::string state = resultSet.getString("fldState");
                                int priority = resultSet.getInteger("fldStatePriority");

                                NodePtr from, to;
                                auto itFrom = std::find_if(nodes.begin(), nodes.end(),
                                                           [idFrom] (const NodePtr& nodePtr) {
                                                               return nodePtr->getID() == idFrom;
                                                           });
                                auto itTo = std::find_if(nodes.begin(), nodes.end(),
                                                         [idTo] (const NodePtr& nodePtr) {
                                                             return nodePtr->getID() == idTo;
                                                         });
                                if (itFrom != nodes.end()) from = *itFrom;
                                if (itTo != nodes.end()) to = *itTo;

                                EdgePtr edge = std::make_shared<Edge>(idEdge, from, to, state, priority);
                                edges.push_back(edge);
                            }
                        }
            , id);

    ScenarioExecutor::params_type paramsVec;
    for (const auto& n : nodes) {
        DATABASE->execQuery(LLLParamsQuery,
                            [&] (const ResultSet &resultSet) {
                                get_params(paramsVec, resultSet);
                            }
                , n->getID());
    }

    return std::make_shared<ScenarioExecutor>(id, nodes, edges, startId, paramsVec);
}

void Mapping::get_task_list_by_id(int id, ProcessVec& pData) {
    pData.clear();

    DATABASE->execQuery(LLLTLQuery,
                        [&] (const ResultSet& resultSet) {
                            while (resultSet.next()) {
                                core_msgs::ProcessData processData;
                                processData.ID = resultSet.getInteger("fldIdProc");
                                processData.name = resultSet.getString("fldName");
                                processData.priority = resultSet.getInteger("fldPriority");
                                processData.filepath = resultSet.getString("fldFilepath");
                                processData.command = resultSet.getString("fldCommand");

                                pData.push_back(processData);
                            }
                        }
            , id);
}

void Mapping::get_params(params_type& paramsVec, const ResultSet& resultSet) {
    while (resultSet.next()) {
        int idProc = resultSet.getInteger("fldIdProc");
        int idTL = resultSet.getInteger("fldIdTL");

        core_msgs::KeyValue keyValue;
        keyValue.key = resultSet.getString("fldKey");
        keyValue.value = resultSet.getString("fldDefValue");

        bool push = true;
        core_msgs::LL_Params params;
        auto* paramPtr = &params;

        auto it = std::find_if(paramsVec.begin(), paramsVec.end(),
                               [&](const auto &p) {
                                   return (p.ID_Proc == idProc && p.ID_TL == idTL);
                               });
        if (it != paramsVec.end()) {
            paramPtr = &*it;
            push = false;
        }

        paramPtr->ID_Proc = idProc;
        paramPtr->ID_TL = idTL;
        paramPtr->key_value.push_back(keyValue);
        std::string path = resultSet.getString("fldPath");
        paramPtr->path = parse_path(path);

        if (push) {
            paramsVec.push_back(*paramPtr);

            DATABASE->execQuery(ChildProcessesParamsQuery,
                                [&paramsVec, &path](const ResultSet &resultSet1) {
                                    get_child_params(paramsVec, resultSet1, path);
                                }, idProc, idTL);
        }
    }
}

std::vector<core_msgs::ChildProcess> Mapping::get_child_processes_by_id(int id) {
    std::vector<core_msgs::ChildProcess> processes;

    DATABASE->execQuery(ChildProcessesQuery,
                        [&] (const ResultSet& resultSet) {
                            while (resultSet.next()) {
                                core_msgs::ChildProcess cp;
                                cp.id = resultSet.getInteger("fldIdProc");
                                cp.name = resultSet.getString("fldName");
                                cp.filepath = resultSet.getString("fldFilepath");
                                processes.push_back(cp);
                            }
                        }
            , id);

    return processes;
}

std::vector<ProcessGroup> Mapping::get_process_group(int id) {
    std::vector<ProcessGroup> sgVec;

    DATABASE->execQuery(ProcessGroupQuery,
                        [&](const ResultSet &resultSet) {
                            ProcessGroup sg;
                            sg.id = -1;

                            while (resultSet.next()) {
                                int gId = resultSet.getInteger("fldIdGroup");
                                if (sg.id != gId) {
                                    if (sg.id > 0) sgVec.push_back(sg);
                                    sg.group.clear();
                                    sg.id = gId;
                                    sg.isRequest = resultSet.getBoolean("fldIsRequest");

                                    if (sg.isRequest) {
                                        sg.requestId = resultSet.getInteger("fldIdRequest");
                                    } else {
                                        sg.stateId = resultSet.getInteger("fldIdState");
                                    }

                                    sg.priority = resultSet.getInteger("fldPriority");
                                }
                                sg.group.emplace_back(resultSet.getString("fldKey"), resultSet.getString("fldValue"));
                            }
                            if (sg.id > 0) sgVec.push_back(sg);
                        }
            , id);
    return sgVec;
}

std::string Mapping::get_state(int id) {
    std::string state;
    DATABASE->execQuery(StateQuery,
                        [&](const ResultSet &resultSet) {
                            resultSet.next();
                            state = resultSet.getString("fldState");
                        }
            , id);
    return state;
}

ProcessRequest Mapping::get_request(int id) {
    ProcessRequest processRequest;

    DATABASE->execQuery(RequestQuery,
                        [&](const ResultSet &resultSet) {
                            resultSet.next();

                            processRequest.text = resultSet.getString("fldText");
                            processRequest.param = resultSet.getString("fldParam");
                            processRequest.isVariable = resultSet.getInteger("fldIsVariable") == 1;

                            if (processRequest.isVariable) {
                                processRequest.value_to_state[""] = resultSet.getString("fldState");
                            } else {
                                do {
                                    processRequest.value_to_state[resultSet.getString("fldName")]
                                            = resultSet.getString("fldState");
                                } while (resultSet.next());
                            }
                        }
            , id);

    return processRequest;
}

void Mapping::get_child_params(params_type& paramsVec, const ResultSet& resultSet, const std::string& prePath) {
    while (resultSet.next()) {
        int idProc = resultSet.getInteger("fldIdProc");
        int idTL = resultSet.getInteger("fldIdTL");

        core_msgs::KeyValue keyValue;
        keyValue.key = resultSet.getString("fldKey");
        keyValue.value = resultSet.getString("fldDefValue");

        bool push = true;
        core_msgs::LL_Params params;
        auto* paramPtr = &params;

        auto it = std::find_if(paramsVec.begin(), paramsVec.end(),
                               [&](const auto &p) {
                                   return (p.ID_Proc == idProc && p.ID_TL == idTL);
                               });
        if (it != paramsVec.end()) {
            paramPtr = &*it;
            push = false;
        }

        paramPtr->ID_Proc = idProc;
        paramPtr->ID_TL = idTL;
        paramPtr->key_value.push_back(keyValue);
        paramPtr->path = parse_path(prePath + resultSet.getString("fldPath"));

        if (push) paramsVec.push_back(*paramPtr);
    }
}

std::vector<int> Mapping::parse_path(const std::string& path) {
    std::stringstream ss(path);
    std::vector<int> elems;
    std::string item;
    while (std::getline(ss, item, '/')) {
        elems.push_back(std::stoi(item));
    }
    return elems;
}