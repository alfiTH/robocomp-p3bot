# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/TargetSelector.ui

)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)
