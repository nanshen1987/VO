add_library( VO SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    visualodometry.cpp
)

target_link_libraries( VO
    ${THIRD_PARTY_LIBS}
)
