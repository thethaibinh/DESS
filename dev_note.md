check_step_for_position_extremum()
    check_position_extremum()


SteeringTrajectoryGenerator traj(
        Vec3(0,0,0),
        vel,
        accel,
        // Vec3(0,0,0),
        // Vec3(0,0,0),
        Vec3(0, 9.8066f, 0));

  RandomTrajectoryGenerator trajGenObj(this, sampling_mode);
  return FindLowestCostTrajectory(
      trajectory, allocatedComputationTime, costFunctionDefinitionObject,
      costFunctionWrapper, (void*) &trajGenObj,
      &RandomTrajectoryGenerator::GetNextCandidateTrajectoryWrapper);

    static int GetNextCandidateTrajectoryWrapper(
        void* ptr2obj,
        RapidQuadrocopterTrajectoryGenerator::SteeringTrajectoryGenerator& nextTraj) {
      RandomTrajectoryGenerator* obj = (RandomTrajectoryGenerator*) ptr2obj;
      return obj->GetNextCandidateTrajectory(nextTraj);
    }

RandomTrajectoryGenerator
profiles[0][dof] --> first index is section between waypoints, second index is the DOF