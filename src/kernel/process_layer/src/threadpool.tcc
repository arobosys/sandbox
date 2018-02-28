//
// Created by vladislav on 13.06.16.
//

#ifndef PROJECT_THREADPOOL_HPP
#error Include threadpool.hpp instead of this
#endif

#include "threadpool.hpp"

template<class F>
auto ThreadPool::enqueue(F &&f) {
    int curId = maxId++;
    taskToFlag[curId] = false;
    using return_type = decltype(f(taskToFlag[curId]));
    auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::ref(taskToFlag[curId]))
    );

    auto future = std::shared_future<return_type>(task->get_future());
    ExtendedFuture<return_type> result;
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        if (isStopped)
            throw std::runtime_error("enqueue on stopped ThreadPool");
        tasks.emplace([task]() { (*task)(); });
        result.id = curId;
        result.future = future;
    }

    condition.notify_one();
    return result;
}
