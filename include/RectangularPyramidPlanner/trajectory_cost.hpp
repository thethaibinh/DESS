//! Class used to find trajectory that moves the fastest in the desired exploration direction
#include "CommonMath/Vec3.hpp"

class ExplorationCost {
  public:
    //! Constructor. Defines desired exploration direction as written in the camera-fixed frame. For example,
    //! a value of (0, 0, 1) would give trajectories that travel the fastest in the direction
    //! that the camera is pointing the lowest cost.
    ExplorationCost(CommonMath::Vec3 explorationDirection)
        : _explorationDirection(explorationDirection) {
  }
    //! Returns the cost of a given trajectory. Note that because each candidate trajectory is written in
    //! the camera-fixed frame, the initial position of each candidate trajectory is always (0, 0, 0) because
    //! we fix the trajectories to originate at the focal point of the camera. Thus, we only need to evaluate
    //! the end point of the trajectory.
    double get_cost(CommonMath:: Vec3& endpoint_vector) {
    CommonMath::Vec3 endpoint_unit_vector = endpoint_vector.GetUnitVector();
    return -_explorationDirection.Dot(endpoint_unit_vector);
  }

    double get_distance_to_goal_cost(CommonMath:: Vec3 endpoint_vector) {
    return (_explorationDirection - endpoint_vector).GetNorm2();
  }

  //! We pass this wrapper function to the planner (see FindLowestCostTrajectory), and this function calls the
  //! related GetCost function to compute the cost of the given trajectory. We structure the code this way so
  //! that other custom cost functions can be used in a similar fashion in the future.
  static double direction_cost_wrapper(void* ptr2obj,
    CommonMath:: Vec3& endpoint_vector) {
    ExplorationCost * explorationCost = (ExplorationCost *)ptr2obj;
    return explorationCost -> get_cost(endpoint_vector);
  }

  static double distance_cost_wrapper(void* ptr2obj,
    CommonMath:: Vec3& endpoint_vector) {
    ExplorationCost * explorationCost = (ExplorationCost *)ptr2obj;
    return explorationCost -> get_distance_to_goal_cost(endpoint_vector);
  }

  private:
    CommonMath::Vec3 _explorationDirection;
};
