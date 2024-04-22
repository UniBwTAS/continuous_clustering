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
    explicit PipelineNode(uint8_t num_threads = 1, std::string thread_name_prefix = "")
        : num_threads(num_threads), thread_pool(thread_name_prefix)
    {
    }

    void stop()
    {
        thread_pool.shutdown();
        if (successor_stop_fn)
            successor_stop_fn();
    }

    virtual void reset()
    {
    }
    virtual void processJob(InputType&& job) = 0;

    void start()
    {
        thread_pool.init([this](InputType&& job) { processJob(std::forward<InputType>(job)); }, num_threads);
        if (successor_start_fn)
            successor_start_fn();
    }

    void setJobFinishedCallback(std::function<void(const OutputType&)> f)
    {
        job_finished_callback = std::move(f);
    }

    void setContinuousRangeImage(const std::shared_ptr<ContinuousRangeImage>& img)
    {
        range_image = img;
        if (successor_set_range_image_fn)
            successor_set_range_image_fn(img);
    }

    void enqueueJob(const InputType& job)
    {
        InputType copy = job;
        thread_pool.enqueue(std::move(copy));
    }

    void enqueueJob(InputType&& job)
    {
        thread_pool.enqueue(std::move(job));
    }

    template<class T>
    void connectToSuccessorThreadPool(PipelineNode<OutputType, T>& consumer)
    {
        successor_thread_pool = &consumer.thread_pool;
    }

    template<template<class> class NextPipelineNodeType>
    std::shared_ptr<NextPipelineNodeType<OutputType>> createSuccessor()
    {
        auto node = std::make_shared<NextPipelineNodeType<OutputType>>();
        connectToSuccessorThreadPool(*node);
        successor_start_fn = [&]() { node->start(); };
        successor_stop_fn = [&]() { node->stop(); };
        successor_reset_fn = [&]() { node->reset(); };
        successor_set_range_image_fn = [&](const std::shared_ptr<ContinuousRangeImage>& img)
        { node->setContinuousRangeImage(img); };
        return node;
    }

  protected:
    void passJobToNextNode(OutputType job)
    {
        if (job_finished_callback)
            job_finished_callback(job);

        if (successor_thread_pool)
            successor_thread_pool->enqueue(std::move(job));
    }

  public:
    ThreadPool<InputType> thread_pool;
    std::function<void(const OutputType&)> job_finished_callback;
    uint8_t num_threads;
    std::shared_ptr<ContinuousRangeImage> range_image;

    // access successor members
    ThreadPool<OutputType>* successor_thread_pool{nullptr}; // TODO
    std::function<void()> successor_reset_fn;
    std::function<void()> successor_start_fn;
    std::function<void()> successor_stop_fn;
    std::function<void(const std::shared_ptr<ContinuousRangeImage>&)> successor_set_range_image_fn;
};

} // namespace continuous_clustering
#endif // CONTINUOUS_CLUSTERING_PIPELINE_NODE_HPP
