#include "voxel_operations.h"

namespace sbpl {
namespace collision {

std::vector<int> ConvertToVertexIndices(
    const std::vector<shape_msgs::MeshTriangle>& triangles)
{
    std::vector<int> triangle_indices(3 * triangles.size());
    for (int j = 0; j < triangles.size(); ++j) {
        triangle_indices[3 * j + 0] = triangles[j].vertex_indices[0];
        triangle_indices[3 * j + 1] = triangles[j].vertex_indices[1];
        triangle_indices[3 * j + 2] = triangles[j].vertex_indices[2];
    }
    return triangle_indices;
}

} // namespace collision
} // namespace sbpl
