add_executable(Benchmarker Benchmarker.cpp)
target_link_libraries(Benchmarker
					  RAPPIDS
					  RapidQuadcopterTrajectories
					  RootFinder
					  ${OpenCV_LIBS}
                      Boost::program_options)

include_directories(${RAPPIDS_project_SOURCE_DIR}/include)