#include "GCode.h"

GCode::GCode()
{
	clear();
}

GCode::GCode(const GCode &to_copy)
{
	is_g= to_copy.is_g;
	is_m= to_copy.is_m;
	is_t= to_copy.is_t;
	is_modal= to_copy.is_modal;
	is_immediate= to_copy.is_immediate;
	argbitmap= to_copy.argbitmap;
	args= to_copy.args;
	code= to_copy.code;
	subcode= to_copy.subcode;
	os.clear();
}

GCode &GCode::operator= (const GCode &to_copy)
{
    if( this != &to_copy ) {
	 	is_g= to_copy.is_g;
		is_m= to_copy.is_m;
		is_t= to_copy.is_t;
		is_modal= to_copy.is_modal;
		is_immediate= to_copy.is_immediate;
		argbitmap= to_copy.argbitmap;
		args= to_copy.args;
		code= to_copy.code;
		subcode= to_copy.subcode;
		os.clear();
	}
    return *this;
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
