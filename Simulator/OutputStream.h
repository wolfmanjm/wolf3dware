#pragma once

#include <sstream>

/**
	Handles an output stream from gcode/mcode handlers
	can be told to append a NL at end, and also to prepend or postpend the ok
*/
class OutputStream
{
public:
	OutputStream() : append_nl(false), prepend_ok(false) {};
	~OutputStream(){};
	OutputStream(const OutputStream &to_copy);
	OutputStream& operator= (const OutputStream &to_copy);

	void clear() { oss.str(""); append_nl= false; prepend_ok= false; }
	int printf(const char *format, ...);
	void setAppendNL() { append_nl= true; }
	void setPrependOK() { prepend_ok= true; }
	bool isAppendNL() const { return append_nl; }
	bool isPrependOK() const { return prepend_ok; }
	std::string str() const { return oss.str(); }
	std::ostringstream& os() { return oss; }

private:
	std::ostringstream oss;

	struct {
		bool append_nl:1;
		bool prepend_ok:1;
	};
};
