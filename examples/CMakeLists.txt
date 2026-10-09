# KizuriPhysics - examples/CMakeLists.txt
set(KZ_EXAMPLES
    HelloWorld
    Stack
    Joints
    Raycast
    DebugDraw
    Benchmark
    Character
    Vehicle
    SoftBody
    CCD
)

foreach(example ${KZ_EXAMPLES})
    add_executable(kizuri_${example} ${example}.cpp)
    target_link_libraries(kizuri_${example} PRIVATE KizuriPhysics)
    set_target_properties(kizuri_${example} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
endforeach()

add_custom_target(kizuri_examples DEPENDS
    ${KZ_EXAMPLES}
)
