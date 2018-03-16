//
// Created by vladislav on 13.06.16.
//

#ifndef PROJECT_SCENARIOEXECUTOR_HPP
#define PROJECT_SCENARIOEXECUTOR_HPP

#include "graph.hpp"
#include <ros/ros.h>
#include <core_msgs/LL_Params.h>
#include <boost/variant/get.hpp>

/***
* @brief Graph class wrapper
*/
class ScenarioExecutor {
public:
    typedef std::vector<NodePtr> nodes_type;
    typedef std::vector<EdgePtr> edges_type;
    typedef std::vector<core_msgs::LL_Params> params_type;

private:
    typedef std::function<void (const std::string&, const std::string&)> feedbackCb;

    int id;
    Graph graph;

public:
    /**
     * Execute given graph and return execution result
     * @return execution result
     * @throws GraphUnexhaustedRuntimeError if graph is unexhausteds
     * @throws std::runtime_error if more that 1 output edge from node
     */
    std::string execute() { return graph.execute(); }

    /**
     * Getter for scenario id
     * @return scenario id
     */
    inline int getId() const { return id; }

    inline auto parametersBegin() { return graph.getParameters().begin(); }
    inline auto parametersEnd() { return graph.getParameters().end(); }

    inline void registerFeedbackCallback(const feedbackCb& fb) { graph.registerFeedbackCallback(fb); }

    /**
     * Class constructor
     * @param id scenario id
     * @param nodes list of nodes used in graph
     * @param edges list of edges used in graph
     * @param start index of start node
     */
    ScenarioExecutor(int id, const nodes_type &nodes,
                     const edges_type &edges, int start,
                     const params_type& params)
            : id(id), graph(nodes, edges, start, params) {};
};

typedef std::shared_ptr<ScenarioExecutor> ScenarioExecutorPtr;

#endif //PROJECT_SCENARIOEXECUTOR_HPP
