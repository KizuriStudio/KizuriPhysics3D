# KizuriPhysics tests
add_executable(kizuri_tests
    Main.cpp
    TestMath.cpp
    TestShapes.cpp
    TestCollision.cpp
    TestWorld.cpp
    TestFeatures.cpp)

target_link_libraries(kizuri_tests PRIVATE Kizuri::Physics)
target_include_directories(kizuri_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME kizuri_tests COMMAND kizuri_tests)
