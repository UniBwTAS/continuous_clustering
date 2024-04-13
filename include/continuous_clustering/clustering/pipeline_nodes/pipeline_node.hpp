#ifndef CONTINUOUS_CLUSTERING_PIPELINE_NODE_HPP
#define CONTINUOUS_CLUSTERING_PIPELINE_NODE_HPP

#include <functional>

#include <continuous_clustering/clustering/continuous_range_image.hpp>
#include <continuous_clustering/utils/thread_pool.hpp>
#include <utility>

namespace continuous_clustering
{

template<class InputType, class OutputType>
class PipelineNode
{
  public:
    PipelineNode(uint8_t num_treads = 1, std::string thread_name_prefix = "")
        : num_treads(num_treads), thread_pool(thread_name_prefix)
    {
    }

    void stop()
    {
        thread_pool.shutdown();
    }

    virtual void reset()
    {
    }
    virtual void processJob(InputType&& job) = 0;

    void start()
    {
        thread_pool.init([this](InputType&& job) { processJob(std::forward<InputType>(job)); }, num_treads);
    }

    void setJobFinishedCallback(std::function<void(const OutputType&)> f)
    {
        job_finished_callback = std::move(f);
    }

    void setContinuousRangeImage(const std::shared_ptr<ContinuousRangeImage>& img)
    {
        range_image = img;
    }

    void enqueueJob(InputType&& job)
    {
        thread_pool.enqueue(std::move(job));
    }

    template<class T>
    void setConsumerNode(PipelineNode<OutputType, T>& consumer)
    {
        next_thread_pool = &consumer.thread_pool;
    }

  protected:
    void passJobToNextNode(OutputType job)
    {
        if (job_finished_callback)
            job_finished_callback(job);

        if (next_thread_pool)
            next_thread_pool->enqueue(std::move(job));
    }

  public:
    ThreadPool<InputType> thread_pool;
    ThreadPool<OutputType>* next_thread_pool{nullptr}; // TODO
    std::function<void(const OutputType&)> job_finished_callback;
    uint8_t num_treads;
    std::shared_ptr<ContinuousRangeImage> range_image;
};

} // namespace continuous_clustering
#endif // CONTINUOUS_CLUSTERING_PIPELINE_NODE_HPP
