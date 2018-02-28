//
// Created by vladislav on 13.06.16.
//

#ifndef PROJECT_THREADPOOL_HPP
#define PROJECT_THREADPOOL_HPP

#include <queue>
#include <future>
#include <map>
#include <unordered_map>
#include <atomic>

template <typename T>
struct ExtendedFuture {
    int id;
    std::shared_future<T> future;
};

class ThreadPool {
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::map<int, std::atomic_bool> taskToFlag;

    std::mutex queue_mutex;
    std::condition_variable condition;
    bool isStopped;
    std::atomic_int maxId;

    std::thread addThreadToPool(size_t);

public:
    /**
     * Places task in queue
     * @param f function(task) to execute
     * @return ExtendedFuture that contains info about task
     */
    template<class F>
    auto enqueue(F &&f);

    /**
     * Cancels task with given ExtendedFuture
     */
    void cancelTask(const ExtendedFuture<void> &);

    /**
     * Class constructor
     * @return number of threads to run in pool
     */
    ThreadPool(size_t);


    /**
     * Class destructor
     */
    ~ThreadPool();
};

#include "threadpool.tcc"

#endif //PROJECT_THREADPOOL_HPP
