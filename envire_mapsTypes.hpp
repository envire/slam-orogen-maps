#ifndef envire_maps_TYPES_HPP
#define envire_maps_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include "base/Pose.hpp"
#include "base/Eigen.hpp"
#include "base/samples/RigidBodyState.hpp"

namespace envire {
namespace maps {

struct PointCloudAggregatorConfig {

    base::Vector3d scanner_position;
    base::Orientation scanner_orientation;

};


}  // namespace maps
}  // namespace envire

#endif

