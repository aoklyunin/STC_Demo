cmake_minimum_required(VERSION 3.26)
project(STC_Demo)

set(CMAKE_CXX_STANDARD 17)


enable_testing()

add_compile_options(
        -O3
        -g3
        #-Wall
)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(DemoForwardTask
        demo_ft.cpp
        ofunctions/Functor.h
        ofunctions/ForwardTaskFunctor.h
)

target_link_libraries(DemoForwardTask Eigen3::Eigen)

add_executable(TestForwardTask
        test/testForwardTask.cpp
        ofunctions/Functor.h
        ofunctions/ForwardTaskFunctor.h
)

target_link_libraries(TestForwardTask Eigen3::Eigen)


add_executable(DemoInverseTask
        demo_it.cpp
        ofunctions/Functor.h
        ofunctions/InverseTaskFunctor.h
)

target_link_libraries(DemoInverseTask Eigen3::Eigen)

add_executable(TestInverseTask
        test/testInverseTask.cpp
        ofunctions/Functor.h
        ofunctions/InverseTaskFunctor.h
)

target_link_libraries(TestInverseTask Eigen3::Eigen)


add_test(NAME TestForwardTask COMMAND TestForwardTask)
add_test(NAME TestInverseTask COMMAND TestInverseTask)
