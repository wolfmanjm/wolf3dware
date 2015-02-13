#include "Dispatcher.h"
#include "GCode.h"
#include "GCodeProcessor.h"

#include <ctype.h>
#include <cmath>

using namespace std;

//#define LOG_WARNING printf
#define LOG_WARNING(...)

bool Dispatcher::dispatch(GCode& gc)
{
	auto& handler= gc.hasG() ? gcode_handlers : mcode_handlers;
	const auto& f= handler.equal_range(gc.getCode());
	bool ret= false;
	output_stream.clear();
	for (auto it=f.first; it!=f.second; ++it) {
		if(it->second(gc)) {
			ret= true;
		}else{
			LOG_WARNING("handler did not handle %c%d\n", gc.hasG() ? 'G':'M', gc.getCode());
		}
	}

	if(ret) {
		// get any output the command(s) returned
		result= output_stream.str();

		if(output_stream.isAppendNL()) {
			// append newline
			result.append("\r\n");
		}

		if(output_stream.isPrependOK()) {
			// output the result after the ok
			result.insert(0, "ok ").append("\r\n");
		}else{
			result.append("ok\r\n");
		}
	}

	return ret;
}

Dispatcher::Handlers_t::iterator Dispatcher::addHandler(HANDLER_NAME gcode, uint16_t code, Handler_t fnc)
{
	Handlers_t::iterator ret;
	switch(gcode) {
		case GCODE_HANDLER: ret= gcode_handlers.insert( Handlers_t::value_type(code, fnc) ); break;
		case MCODE_HANDLER: ret= mcode_handlers.insert( Handlers_t::value_type(code, fnc) ); break;
	}
	return ret;
}

void Dispatcher::removeHandler(HANDLER_NAME gcode, Handlers_t::iterator i)
{
	switch(gcode) {
		case GCODE_HANDLER: gcode_handlers.erase(i); break;
		case MCODE_HANDLER: mcode_handlers.erase(i); break;
	}
}

// mainly used for testing
void Dispatcher::clearHandlers()
{
	gcode_handlers.clear();
	mcode_handlers.clear();
}
