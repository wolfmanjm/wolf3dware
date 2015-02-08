#pragma once

#include <vector>

#include "GCode.h"

class GCodeProcessor
{
public:
	GCodeProcessor();
	~GCodeProcessor();

	using GCodes_t = std::vector<GCode>;

	GCodes_t parse(const char *line);

private:
	// modal settings
	GCode group0, group1;

};
