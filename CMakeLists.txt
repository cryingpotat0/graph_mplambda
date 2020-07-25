# 3.10 introduces C++ 17 support
cmake_minimum_required (VERSION 3.10)
project (mplambda)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable extensions to keep compatible with standards
set(CMAKE_CXX_EXTENSIONS OFF)

#find_package(FCL REQUIRED)
######## Find CCD the hard way
find_package(PkgConfig)
if (PKGCONFIG_FOUND)
  pkg_check_modules(FCL fcl>=0.6)
  pkg_check_modules(CCD ccd>=2.0)
  pkg_check_modules(PNG libpng)
  pkg_check_modules(ASSIMP assimp)
else()
    find_package(FCL REQUIRED)
    find_package(CCD CONFIG)
endif()
#########

find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(Assimp REQUIRED)
find_package(Threads)
#find_package(PNG REQUIRED)

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()




if (${APPLE})
	add_executable(mpl_coordinator src/coordinator.cpp src/comm.cpp src/write_queue.cpp )
	target_link_libraries(mpl_coordinator PUBLIC ${FCL_LIBRARIES} ${ASSIMP_LIBRARIES} ${PNG_LIBRARIES})

	add_executable(mpl_test src/test.cpp src/comm.cpp src/write_queue.cpp )
	target_link_libraries(mpl_test PUBLIC ${FCL_LIBRARIES} ${ASSIMP_LIBRARIES} ${PNG_LIBRARIES})
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAS_AWS_SDK")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_AWS_SDK")
    
    find_package(aws-lambda-runtime REQUIRED)
    find_package(AWSSDK COMPONENTS s3 lambda)
    add_executable(mpl_fixed_graph_aws src/mpl_lambda_aws.cpp  src/comm.cpp src/write_queue.cpp)
    target_link_libraries(mpl_fixed_graph_aws PUBLIC Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES} AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES} ${PNG_LIBRARIES} )
    #target_link_libraries(mpl_fixed_graph_aws PUBLIC Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES} AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES} ${PNG_LIBRARIES} /usr/local/lib/libIrrXML.a)
    #target_link_libraries(mpl_fixed_graph_aws PUBLIC Eigen3::Eigen Threads::Threads ${FCL_LIBRARIES} ${CCD_LIBRARIES} AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES} ${PNG_LIBRARIES})
    
    # the following line adds "aws-lambda-package-mpl_lambda_aws" as a
    # target which creates a .zip file with everything we need to
    # upload a function to aws.
    aws_lambda_package_target(mpl_fixed_graph_aws)

    # this is the command to run in order to create the full lambda zip:
    add_custom_target(mpl_fixed_graph_aws_zip
    	COMMAND zip -ur9 mpl_fixed_graph_aws.zip resources
    	DEPENDS aws-lambda-package-mpl_fixed_graph_aws)

    # After creating the lambda zip, we can upload it using:
    add_custom_target(update-function-code
        COMMAND aws lambda update-function-code --function-name mpl_fixed_graph_aws --zip-file fileb://mpl_fixed_graph_aws.zip)

    add_executable(mpl_lambda_invoke src/mpl_lambda_invoke.cpp)
    target_link_libraries(mpl_lambda_invoke AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES})

    add_executable(mpl_coordinator src/coordinator.cpp src/comm.cpp src/write_queue.cpp)
    target_link_libraries(mpl_coordinator Threads::Threads AWS::aws-lambda-runtime ${AWSSDK_LINK_LIBRARIES} ${PNG_LIBRARIES} ${FCL_LIBRARIES} ${ASSIMP_LIBRARIES} ${CCD_LIBRARIES})

endif()

add_executable(mpl_lambda_fixed_graph src/demo/lambda_fixed_graph.cpp src/comm.cpp src/write_queue.cpp)
target_link_libraries(mpl_lambda_fixed_graph PUBLIC Eigen3::Eigen ${FCL_LIBRARIES} ${ASSIMP_LIBRARIES} ${PNG_LIBRARIES})

link_directories(${FCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})
include_directories(include ../nigh/src ${FCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${PNG_INCLUDE_DIRS})

#link_directories(${FCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})
#include_directories(include ../../nigh/src ${FCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS})
#
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/png_planning_input.png ${CMAKE_CURRENT_SOURCE_DIR}/resources/house_layout.png 
		${CMAKE_CURRENT_SOURCE_DIR}/resources/AUTOLAB.dae
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/resources)
 add_custom_target(run_script_symlink ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_experiments.py ${CMAKE_CURRENT_BINARY_DIR}/run_experiments.py)
#file(GLOB SE3RSRC
#     "resources/se3/Twistycool*.dae" "resources/se3/alpha*.dae"
#     "resources/se3/Apartment*.dae" "resources/se3/cubicles*.dae"
#     "resources/se3/Home*.dae" "resources/se3/Easy*.dae")
#file(COPY ${SE3RSRC} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/resources/se3)
#
#add_executable(mpl_lambda_pseudo src/mpl_lambda_pseudo.cpp src/mpl/demo/lambda_common.cpp src/mpl/comm.cpp src/mpl/write_queue.cpp src/mpl/demo/app_options.cpp)
#target_link_libraries(mpl_lambda_pseudo Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})

#add_executable(mpl_lambda_pseudo src/mpl_lambda_pseudo.cpp src/mpl/demo/lambda_common.cpp src/mpl/comm.cpp src/mpl/write_queue.cpp src/mpl/demo/app_options.cpp)
#target_link_libraries(mpl_lambda_pseudo Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})
#
#add_executable(mpl_robot src/mpl_robot.cpp src/mpl/write_queue.cpp src/mpl/demo/app_options.cpp)
#target_link_libraries(mpl_robot Eigen3::Eigen Threads::Threads ${ASSIMP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})
#
#add_executable(mpl_fetch src/mpl_fetch.cpp)
#target_link_libraries(mpl_fetch Eigen3::Eigen ${FCL_LIBRARIES} ${CCD_LIBRARIES})


