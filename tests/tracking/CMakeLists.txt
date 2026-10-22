# Checks of the motion tracking path against golden CSVs of mjlab's scripts/hrp5p/export_deploy.py. The CSVs are not
# in the repository (they are made from a trained policy, about 19 MB each): pass one with -DTRACKING_GOLDEN_CSV=<path>
# to register the tests.
find_package(Eigen3 REQUIRED NO_MODULE)

# The observation (obs_format 9) rebuilt from the golden inputs
add_executable(test_tracking_observation test_tracking_observation.cpp ${PROJECT_SOURCE_DIR}/src/TrackingObservation.cpp)
target_include_directories(test_tracking_observation PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(test_tracking_observation PRIVATE Eigen3::Eigen)
set(TRACKING_IMU_SITE_POS "-0.09;0.0;0.125" CACHE STRING "IMU site position in the base frame (the tracking section's imu_site_pos)")
if(TRACKING_GOLDEN_CSV)
  add_test(NAME tracking_observation COMMAND test_tracking_observation ${TRACKING_GOLDEN_CSV} ${TRACKING_IMU_SITE_POS})
endif()

# The policy run by RLPolicyInterface on the golden observations, against the golden actions
add_executable(test_tracking_policy test_tracking_policy.cpp)
target_link_libraries(test_tracking_policy PRIVATE ${PROJECT_NAME})
set(TRACKING_POLICY_ONNX "${PROJECT_SOURCE_DIR}/policy/tracking/P6_final/policy.onnx" CACHE STRING "Policy of the golden CSV")
if(TRACKING_GOLDEN_CSV)
  add_test(NAME tracking_policy COMMAND test_tracking_policy ${TRACKING_GOLDEN_CSV} ${TRACKING_POLICY_ONNX})
endif()
