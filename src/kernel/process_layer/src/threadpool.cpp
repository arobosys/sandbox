//
// Created by vladislav on 13.06.16.
//
#include "threadpool.hpp"

std::thread ThreadPool::addThreadToPool(size_t index) {
    return std::thread([this, index] {
        for(;;) {
            std::function<void()> task;

            {
                std::unique_lock<std::mutex> lock(this->queue_mutex);
                this->condition.wait(lock,
                                     [this] { return this->isStopped || !this->tasks.empty(); });
                if (this->isStopped && this->tasks.empty())
                    return;
                task = std::move(this->tasks.front());
                this->tasks.pop();
            }
            task();
        }
    });
}

ThreadPool::ThreadPool(size_t threads)
        : isStopped(false) {
    maxId = 0;
    for (size_t i = 0; i < threads; ++i) {
        workers.push_back(addThreadToPool(i));
    }
}


void ThreadPool::cancelTask(const ExtendedFuture<void> &future) {
    taskToFlag[future.id].store(true);
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        isStopped = true;
    }
    condition.notify_all();
    for (auto &worker: workers)
        worker.join();
}