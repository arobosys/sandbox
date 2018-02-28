#ifndef DEFAULTPARENTPROCESSINTERFACE_HPP
#define DEFAULTPARENTPROCESSINTERFACE_HPP

#include "process_interface.hpp"

namespace interface {

    class ParentProcessInterface {
    public:
        typedef std::function<bool(const ProcessInterface::Parameters &goal)> GoalPrehandler;
    private:
        bool _firstGoal = true;
        ProcessInterface _processInterface;
        GoalPrehandler _goalPrehandler;

        void feedbackCB(const std::vector<core_msgs::DataPL> &fb);

        void goalHandler(const ProcessInterface::Parameters &goal);

    public:
        const ProcessInterface &processInterface() const;

        /**
          * @param goalPrehandler this function executes before goal processing,
          *        and goal processing continues if this function returns true.
          */
        ParentProcessInterface(int argc, char **argv,
                               GoalPrehandler goalPrehandler = [](const auto &goal) { return true; });
    };

}

#endif // DEFAULTPARENTPROCESSINTERFACE_HPP
