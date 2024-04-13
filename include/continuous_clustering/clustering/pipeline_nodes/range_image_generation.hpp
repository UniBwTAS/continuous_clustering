#ifndef CONTINUOUS_CLUSTERING_RANGE_IMAGE_GENERATION_HPP
#define CONTINUOUS_CLUSTERING_RANGE_IMAGE_GENERATION_HPP

#include <continuous_clustering/clustering/continuous_range_image.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

struct RangeImageGenerationResult
{
    int64_t column_index;
};

template<class InputType>
class RangeImageGeneration : public PipelineNode<InputType, RangeImageGenerationResult>
{
  public:
    explicit RangeImageGeneration(uint8_t num_treads = 1)
        : PipelineNode<InputType, RangeImageGenerationResult>(num_treads, "R"){};

    void processJob(InputType&& job)
    {
        int64_t first, last;
        this->range_image->insertFiring(job.firing, first, last);

        // iterate over finished but unfinished columns and publish them
        for (int64_t global_column_index = first; global_column_index <= last; global_column_index++)
            this->passJobToNextNode({global_column_index});
    }
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_RANGE_IMAGE_GENERATION_HPP
