//
// Created by vladislav on 13.06.16.
//

#ifndef PROJECT_NODEBASE_HPP
#define PROJECT_NODEBASE_HPP

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <core_msgs/LL_Params.h>

class Node {
protected:
    static const int SERVER_TIMEOUT = 5;
    static const std::string SERVER_TIMEOUT_MESSAGE;

    typedef std::function<void (const std::string&)> feedbackCb;
    typedef std::vector<core_msgs::LL_Params> params_type;

    int id;
    bool out;
    std::string outState;
    feedbackCb fbCb;
    params_type params;

    static params_type parseParameters(int id, const params_type& params);

public:

    /***
     * @brief get UID of node in storage
     */
    inline int getID() const { return id; }

    /***
     * @brief is node marked as "out" (used in graph execution)
     */
    inline bool isOut() const { return out; }

    /**
     * @brief returns outState if node marked as "out"
     */
    inline std::string getState() const { return outState; }

    /**
     * @brief registers callback function to get feedback from low level subsystem
     */
    inline void registerFeedbackCallback(const feedbackCb& fb) { fbCb = fb; }

    /**
     * @brief sets the part of parameters belonging only to this node
     * @param params Parameters of high level subsystem
     */
    inline void setParameters(const params_type& params) {
        this->params = parseParameters(this->id, params);
    }

    /**
     * @brief Executes current node
     * @return State to next node
     */
    virtual std::string execute() = 0;

    Node() = default;

    /**
     * @param id see getID()
     * @param nodeFunc can be empty
     * @param out see isOut()
     */
    Node(int id, bool out = false, const std::string& outState = std::string())
            : id(id), out(out), outState(outState) {}

    virtual ~Node() = default;
};

typedef std::shared_ptr<Node> NodePtr;

#endif //PROJECT_NODEBASE_HPP

