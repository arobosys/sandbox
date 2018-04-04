//
// Created by vladislav on 13.06.16.
//

#ifndef PROJECT_EDGE_HPP
#define PROJECT_EDGE_HPP

#include <memory>
#include <functional>
#include "node.hpp"

/**
 * @brief Oriented edge 
 * @tparam NodeResultTp type for checking ability of processing subgraph pointed by edge.
 * @author Vasiliy
 */
class Edge {
protected:
    int id;
    std::string state;
    int state_priority;
    NodePtr from, to;

public:
    /**
     * @return UID of edge in storage (e.g database)
     */
    inline int getID() { return id; }

    /**
     * @return State of edge
     */
    inline std::string getState() { return state; }

    /**
     * @return "source" nodeptr of edge
     */
    inline NodePtr getFrom() const { return from; }

    /**
     * @return "sink" nodeptr of edge 
     */
    inline NodePtr getTo() const { return to; };

    Edge() = default;

    /**
     * @param from see getFrom()
     * @param to see getTo()
     */
    Edge(int id, const NodePtr &from, const NodePtr &to, const std::string& state, int statePriority)
            : id(id), from(from), to(to), state(state), state_priority(statePriority) {}
};

typedef std::shared_ptr<Edge> EdgePtr;

#endif //PROJECT_EDGE_HPP
