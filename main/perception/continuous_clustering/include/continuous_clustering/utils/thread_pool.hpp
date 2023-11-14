#ifndef CONTINUOUS_CLUSTERING_THREAD_POOL_HPP
#define CONTINUOUS_CLUSTERING_THREAD_POOL_HPP

#include <list>
// #include <pthread.h>
#include <thread>
#include <utility>

#include <continuous_clustering/utils/thread_save_queue.hpp>

namespace continuous_clustering
{

template<class T>
class ThreadPool
{
  public:
    explicit ThreadPool(std::string thread_name_prefix = "", bool stop_threads_when_job_queue_is_empty = false)
        : thread_name_prefix(std::move(thread_name_prefix)),
          stop_threads_when_job_queue_is_empty(stop_threads_when_job_queue_is_empty)
    {
    }

    ~ThreadPool()
    {
        shutdown();
    }

    void init(const std::function<void(T)> f, int num_threads = 1)
    {
        if (num_threads == 0)
        {
            sequential_execution = true;
            f_sequential = f;
        }

        terminated = false;
        for (int i = 0; i < num_threads; i++)
        {
            threads.template emplace_back(
                [this, f, i]()
                {
                    // std::string thread_name = "continuous_" + thread_name_prefix + std::to_string(i);
                    // pthread_setname_np(pthread_self(), thread_name.c_str());
                    while (!terminated)
                    {
                        T job = job_queue.dequeue();
                        if (terminated)
                            break;
                        f(std::move(job));
                        if (stop_threads_when_job_queue_is_empty && job_queue.size() == 0)
                            break;
                    }
                });
        }
    }

    void enqueue(T&& t)
    {
        if (sequential_execution)
        {
            f_sequential(std::move(t));
            return;
        }

        job_queue.enqueue(std::move(t));
    }

    void join()
    {
        for (std::thread& t : threads)
            if (t.joinable())
                t.join();
    }

    void shutdown()
    {
        terminated = true;
        job_queue.shutdown();
        join();
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
    bool stop_threads_when_job_queue_is_empty{false};

    // sequential execution when the number of threads is zero
    bool sequential_execution{false};
    std::function<void(T)> f_sequential;
};

} // namespace continuous_clustering
#endif
