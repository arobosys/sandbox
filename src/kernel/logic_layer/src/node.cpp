//
// Created by vladislav on 30.10.16.
//

#include "logic_layer/node.hpp"

const std::string Node::SERVER_TIMEOUT_MESSAGE = "[timeout]";

Node::params_type Node::parseParameters(int id, const params_type &params) {
    params_type parsed;
    for (auto p : params) {
        int topId = p.path[0];
        if (id == topId) {
            p.path.erase(p.path.begin());
            parsed.push_back(p);
        }
    }
    return parsed;
}