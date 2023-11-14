#ifndef STREAMING_CLUSTERING_THREAD_SAVE_QUEUE_H
#define STREAMING_CLUSTERING_THREAD_SAVE_QUEUE_H

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>

namespace streaming_clustering
{

template<class T>
class ThreadSaveQueue
{
  public:
    ThreadSaveQueue() : q(), m(), c()
    {
    }

    void enqueue(T&& t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(std::move(t));
        c.notify_one();
    }

    T dequeue()
    {
        std::unique_lock<std::mutex> lock(m);
        c.wait(lock, [&] { return !q.empty() || terminated; });
        if (terminated)
            return T{};
        T val = std::move(q.front());
        q.pop();
        return val;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(m);
        terminated = false;
        q = std::queue<T>();
    }

    void shutdown()
    {
        terminated = true;
        c.notify_all();
    }

    size_t size()
    {
        std::lock_guard<std::mutex> lock(m);
        return q.size();
    }

  private:
    std::queue<T> q;
    mutable std::mutex m;
    std::condition_variable c;
    bool terminated{false};
};

} // namespace streaming_clustering
#endif
