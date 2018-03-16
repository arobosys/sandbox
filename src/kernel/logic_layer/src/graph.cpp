#include "logic_layer/graph.hpp"

void Graph::init(const std::vector<NodePtr> &nodes, const std::vector<EdgePtr> &edges, int startNodeId) {
    for (const auto &n : nodes)
        this->nodes[n->getID()] = n;

    for (const auto &e : edges)
        this->edges[e->getState()] = e;
}

std::string Graph::execute() {
    if (executing)
        throw GraphUnexhaustedRuntimeError("Execute have been already called");

    executing = true;

    NodePtr node;
    std::string state;
    for (node = nodes[currentNodeId]; !node->isOut(); node = nodes[currentNodeId]) {
        node->registerFeedbackCallback(
                [&](const std::string &fb) {
                    fbCb(state, fb);
                });
        node->setParameters(this->params);
        state = node->execute();
        fbCb(state, "");

        auto edge = edges[state];

        if (edge == nullptr) {
            throw GraphUnexhaustedRuntimeError(state);
        }

        if (edge->getFrom() != node)
            std::cout << "[WARNING] Edge (from) node not equals previous node" << std::endl;

        currentNodeId = edge->getTo()->getID();
    }

    return node->getState();
}

Graph::Graph(const std::vector<NodePtr> &nodes, const std::vector<EdgePtr> &edges,
             int startNodeId, const params_type &params)
        : startNodeId(startNodeId), currentNodeId(startNodeId), executing(false), params(params) {
    init(nodes, edges, startNodeId);
}

void Graph::registerFeedbackCallback(const feedbackCb &fb) {
    fbCb = fb;
}