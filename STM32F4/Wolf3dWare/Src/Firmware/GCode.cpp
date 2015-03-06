#include "GCode.h"

GCode::GCode()
{
	clear();
}

void GCode::clear()
{
	is_g= false;
	is_m= false;
	is_t= false;
	is_modal= false;
	is_immediate= false;
	argbitmap= 0;
	args.clear();
	code= subcode= 0;
	os.clear();
}

void GCode::dump(std::ostream& o) const
{
	o << (is_g?"G":"") << (is_m?"M":"") << code;
	if(subcode != 0) {
		o << "." << subcode;
	}
	o << " ";
	for(auto& i : args) {
		o << i.first << ":" << i.second << " ";
	}
	o << std::endl;
}
