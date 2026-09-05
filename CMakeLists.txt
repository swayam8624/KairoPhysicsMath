cmake_minimum_required(VERSION 3.28)
project(KairoPhysicsMath LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(KAIRO_PHYSICS_MATH_BUILD_TESTS "Build KairoPhysicsMath tests" ${PROJECT_IS_TOP_LEVEL})

# Used only when neither an explicit local source directory nor the sibling
# workspace checkout is available. Keep this revision aligned with the
# umbrella's validated KairoMath submodule pin.
set(KAIRO_PHYSICS_MATH_MATH_REVISION e12778734a8e8ffe060255a9c03683ad6c4b1a92)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
elseif(MSVC)
    add_compile_options(/W4)
endif()

if(NOT TARGET KairoMath)
    set(KAIRO_MATH_SOURCE_DIR "" CACHE PATH "Path to a local KairoMath checkout")

    if(KAIRO_MATH_SOURCE_DIR AND EXISTS "${KAIRO_MATH_SOURCE_DIR}/CMakeLists.txt")
        add_subdirectory(
            "${KAIRO_MATH_SOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoMath"
        )
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../KairoMath/CMakeLists.txt")
        add_subdirectory(
            "${CMAKE_CURRENT_SOURCE_DIR}/../KairoMath"
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoMath"
        )
    else()
        include(FetchContent)
        FetchContent_Declare(
            KairoMath
            GIT_REPOSITORY https://github.com/swayam8624/KairoMath.git
            GIT_TAG ${KAIRO_PHYSICS_MATH_MATH_REVISION}
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(KairoMath)
    endif()
endif()

add_library(KairoPhysicsMath)

target_sources(KairoPhysicsMath
    PUBLIC
        FILE_SET CXX_MODULES
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            PhysicsTypes.cppm
            InertiaTensor.cppm
            MassProperties.cppm
            Force.cppm
            RigidBodyMath.cppm
            Impulse.cppm
            Integrators.cppm
            Contact.cppm
            ConstraintMath.cppm
            KairoPhysicsMath.cppm
)

target_link_libraries(KairoPhysicsMath PUBLIC KairoMath)

if(KAIRO_PHYSICS_MATH_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(KairoPhysicsMathTests tests/PhysicsMathTests.cpp)
    target_link_libraries(KairoPhysicsMathTests PRIVATE KairoPhysicsMath Catch2::Catch2WithMain)

    enable_testing()
    add_test(NAME KairoPhysicsMathTests COMMAND KairoPhysicsMathTests)
endif()
