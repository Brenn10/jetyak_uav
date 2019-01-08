#ifndef MODE_H
#define MODE_H
#include <string>
enum class Mode : char { TAKEOFF, FOLLOW, LEAVE, RETURN, LAND, RIDE, HOVER };

static std::string nameFromMode[] = {"TAKEOFF", "FOLLOW", "LEAVE", "RETURN",
                                     "LAND",    "RIDE",   "HOVER"};

#endif