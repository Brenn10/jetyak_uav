#ifndef JETYAK_UAV_H_
#define JETYAK_UAV_H_

#include <string>

namespace JETYAK_UAV
{
// Flag constants. Use these flags internally
enum Flag : uint8_t
{
	BODY_FRAME = 0b10,
	WORLD_FRAME = 0b00,
	VELOCITY_CMD = 0b00,
	POSITION_CMD = 0b01
};

// Enumerate the modes
enum Mode : char
{
	TAKEOFF,
	FOLLOW,
	LEAVE,
	RETURN,
	LAND,
	RIDE,
	HOVER
};

// Save the names for human readable display
static std::string nameFromMode[] = { "TAKEOFF", "FOLLOW", "LEAVE", "RETURN", "LAND", "RIDE", "HOVER" };
};	// namespace JETYAK_UAV

#endif	// JETYAK_FLAG_H_