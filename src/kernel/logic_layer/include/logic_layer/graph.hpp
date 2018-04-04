//
// Created by vladislav on 13.06.16.
//

#ifndef PROJECT_GRAPH_HPP
#define PROJECT_GRAPH_HPP

#include <unordered_map>
#include <vector>
#include <queue>
#include <functional>
#include <condition_variable>
#include <mutex>
#include "edge.hpp"

class GraphNoOutError : public std::runtime_error {
public:
    GraphNoOutError(const std::string &addInfo = "")
        : std::runtime_error("Graph has no out vertex. " + addInfo) {}
};

class GraphUnexhaustedRuntimeError : public std::runtime_error {
public:
    GraphUnexhaustedRuntimeError(const std::string &addInfo = "")
            : std::runtime_error("Graph has unexhausted edges. " + addInfo) {}
};

class Graph {
public:
    typedef std::vector<core_msgs::LL_Params> params_type;

private:
    typedef std::function<void (const std::string&, const std::string&)> feedbackCb;

    std::unordered_map<int, NodePtr> nodes; /// nodeId -> NodePtr
    std::unordered_map<std::string, EdgePtr> edges; /// state -> output edge

    void init(const std::vector<NodePtr> &nodes, const std::vector<EdgePtr> &edges, int startNodeId);

    int startNodeId, currentNodeId;
    bool executing;

    feedbackCb fbCb;
    params_type params;

public:
    /**
     * @brief execute graph
     * @description Execution process is based on BFS. 
     *  BFS working node: at 1st step -- node which has ID=startNodeId, at other steps -- front of BFS queue.
     *  execute() method called for BFS working node; when execution is finished, edges which has possibility
     *  of jump is collected to set which adds to end of BFS queue. If this set is empty, then GraphUnexhaustedRuntimeError
     *  is set for returned std::future. In case of finished execution of node marked as  out (see Node abstract class) returned
     *  future is set succedeed with NodeResultPtrTp of that "out" node.
     */
    std::string execute();

    /***
     * @brief Graph ctor
     * @description 
     * @exception GraphNoOutError if graph has no vertex marked as "out" (see Node::isOut()) 
     * @param startNodeId read about execute() method 
     * @param wf function which called before execution of any node. can be used for logging
     */
    Graph(const std::vector<NodePtr> &nodes, const std::vector<EdgePtr> &edges, int startNodeId,
          const params_type& params);

    void registerFeedbackCallback(const feedbackCb& fb);

    inline params_type& getParameters() { return params; }

    inline void setParameters(const params_type& params) { this->params = params; }

};

#endif // PROJECT_GRAPH_HPP