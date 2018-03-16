//
// Created by vladislav on 21.10.16.
//

#include "logic_layer/process_group.hpp"
#include <algorithm>

bool ProcessGroup::match(const group_t &group) const {
    for (auto& g : group) {
        if (std::find(this->group.begin(), this->group.end(), g) == this->group.end())
            return false;
    }
    return true;
}
