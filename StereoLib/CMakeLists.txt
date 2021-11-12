set(CPP_SOURCE StereoCapture.cpp Calibrator.cpp Disparity.cpp Filters.cpp)
set(HEADERS StereoCapture.h Calibrator.h Disparity.h Filters.h)

add_library(StereoLib ${CPP_SOURCE} )