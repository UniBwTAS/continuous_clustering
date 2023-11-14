#ifndef CONTINUOUS_CLUSTERING_THREAD_POOL_H
#define CONTINUOUS_CLUSTERING_THREAD_POOL_H

#include <pthread.h>
#include <thread>
#include <utility>

#include "thread_save_queue.h"

namespace continuous_clustering
{

template<class T>
class ThreadPool
{
  public:
    explicit ThreadPool(std::string thread_name_prefix = "") : thread_name_prefix(std::move(thread_name_prefix))
    {
    }

    ~ThreadPool()
    {
        shutdown();
    }

    void init(const std::function<void(T)> f, int num_threads = 1)
    {
        terminated = false;
        for (int i = 0; i < num_threads; i++)
        {
            threads.template emplace_back(
                [this, f, i]()
                {
                    std::string thread_name = "continuous_" + thread_name_prefix + std::to_string(i);
                    pthread_setname_np(pthread_self(), thread_name.c_str());
                    while (!terminated)
                    {
                        T job = job_queue.dequeue();
                        if (terminated)
                            break;
                        f(std::move(job));
                    }
                });
        }
    }

    void enqueue(T&& t)
    {
        job_queue.enqueue(std::move(t));
    }

    void shutdown()
    {
        terminated = true;
        job_queue.shutdown();
        for (std::thread& t : threads)
            if (t.joinable())
                t.join();
        threads.clear();
        job_queue.clear();
    }

    size_t getNumberOfUnprocessedJobs()
    {
        return job_queue.size();
    }

    void clearJobs()
    {
        job_queue.shutdown();
        job_queue.clear();
    }

  private:
    std::list<std::thread> threads;
    ThreadSaveQueue<T> job_queue;
    bool terminated{true};
    std::string thread_name_prefix;
};

} // namespace continuous_clustering
#endif
