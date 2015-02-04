#include "Dispatcher.h"
#include "GCode.h"
#include "GCodeProcessor.h"

#include <ctype.h>
#include <cmath>

using namespace std;

bool Dispatcher::dispatch(GCode& gc)
{
	auto& handler= gc.has_g() ? gcode_handlers : mcode_handlers;
	const auto& f= handler.equal_range(gc.get_code());
	bool ret= false;
	for (auto it=f.first; it!=f.second; ++it) {
		it->second(gc);
		ret= true;
	}

	return ret;
}

Dispatcher::Handlers_t::iterator Dispatcher::add_handler(HANDLER_NAME gcode, uint16_t code, Handler_t fnc)
{
	Handlers_t::iterator ret;
	switch(gcode) {
		case GCODE_HANDLER: ret= gcode_handlers.insert( Handlers_t::value_type(code, fnc) ); break;
		case MCODE_HANDLER: ret= mcode_handlers.insert( Handlers_t::value_type(code, fnc) ); break;
	}
	return ret;
}

void Dispatcher::remove_handler(HANDLER_NAME gcode, Handlers_t::iterator i)
{
	switch(gcode) {
		case GCODE_HANDLER: gcode_handlers.erase(i); break;
		case MCODE_HANDLER: mcode_handlers.erase(i); break;
	}
}

// mainly used for testing
void Dispatcher::clear_handlers()
{
	gcode_handlers.clear();
	mcode_handlers.clear();
}
