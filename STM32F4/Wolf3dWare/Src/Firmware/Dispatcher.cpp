#include "Dispatcher.h"
#include "Kernel.h"
#include "GCode.h"
#include "GCodeProcessor.h"

#include <ctype.h>
#include <cmath>
#include <string.h>

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

	// special case is M500 - M503
	if(gc.hasM() && gc.getCode() >= 500 && gc.getCode() <= 503) {
		ret= handleConfigurationCommands(gc);
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
	output_stream.clear();

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

bool Dispatcher::handleConfigurationCommands(GCode& gc)
{
	if(gc.getCode() == 500) {
		if(gc.getSubcode() == 3) {
			// just print it
			return true;

		}else if(gc.getSubcode() == 0){
			// write stream to non volatile memory
			// prepend magic number so we know there is a valid configuration saved
			std::string str= "CONF";
			str.append(output_stream.str());
			str.append(4, 0); // terminate with 4 nuls
			output_stream.clear(); // clear to save some memory
			size_t n= str.size();
			size_t r= THEKERNEL.nonVolatileWrite((void *)str.data(), n, 0);
			if(r == n) {
				output_stream.printf("Configuration saved\n");
			}else{
				output_stream.printf("Failed to save configuration, needed to write %d, wrote %d\n", n, r);
			}
			return true;
		}

	}else if(gc.getCode() == 501) {
		// load configuration from non volatile memory
		char buf[64];
		size_t r= THEKERNEL.nonVolatileRead(buf, 4, 0);
		if(r == 4) {
			if(strncmp(buf, "CONF", 4) == 0) {
				size_t off= 4;
				// read everything until we get some nulls
				std::string str;
				do {
					r= THEKERNEL.nonVolatileRead(buf, sizeof(buf), off);
					if(r != sizeof(buf)) {
						output_stream.printf("Read failed\n");
						return true;
					}
					str.append(buf, r);
					off += r;
				}while(str.find('\0') == string::npos);

				// foreach line dispatch it
				std::stringstream ss(str);
				std::string line;
				std::vector<string> lines;
				GCodeProcessor& gp= THEKERNEL.getGCodeProcessor();
			    while(std::getline(ss, line, '\n')){
      				if(line.find('\0') != string::npos) break; // hit the end
      				lines.push_back(line);
      				// Parse the Gcode
					GCodeProcessor::GCodes_t gcodes= gp.parse(line.c_str());
					// dispatch it
					for(auto& i : gcodes) {
						if(i.getCode() >= 500 && i.getCode() <= 503) continue; // avoid recursion death
						dispatch(i);
    				}
    			}
    			for(auto& s : lines) {
    				output_stream.printf("Loaded %s\n", s.c_str());
    			}

			}else{
				output_stream.printf("No saved configuration\n");
			}

		}else{
			output_stream.printf("Failed to read saved configuration\n");
		}
		return true;

	}else if(gc.getCode() == 502) {
		// delete the saved configuration
		uint32_t zero= 0xFFFFFFFFUL;
		if(THEKERNEL.nonVolatileWrite(&zero, 4, 0) == 4) {
			output_stream.printf("Saved configuration deleted - reset to restore defaults\n");
		}else{
			output_stream.printf("Failed to delete saved configuration\n");
		}
		return true;
	}

	return false;
}
