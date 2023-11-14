#ifndef CONTINUOUS_CLUSTERING_POINT_TYPES_HPP
#define CONTINUOUS_CLUSTERING_POINT_TYPES_HPP

#include <memory>
#include <vector>

namespace continuous_clustering
{

struct RawPoint
{
    float x{};
    float y{};
    float z{};
    uint64_t firing_index{};
    uint8_t intensity{};
    uint64_t stamp{};
    uint64_t globally_unique_point_index{}; // used for evaluation in order to be able to obtain corresponding GT label
};

struct RawPoints
{
    uint64_t stamp;
    std::vector<RawPoint> points;

    typedef std::shared_ptr<RawPoints> Ptr;
    typedef std::shared_ptr<RawPoints const> ConstPtr;
};

} // namespace continuous_clustering

#endif