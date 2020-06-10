/*
 * GMaker - C/C++ Library to enable Game Maker functionality for DLLs.
 * Copyright © 2009 Preston N. Smith
 * (this is a modified version of the original library - Maarten Baert)
 */

#ifndef GMAKER_H
#define GMAKER_H

#include <cstddef>
#include <exception>

/*
String support is rather limited. It only works for arguments, not return values.
And I don't think it will work if the function stores the value of the string internally,
so don't use it with functions like ds_grid_set.
*/

class GMProc;

class GMVariable {
	
	friend class GMProc;
	
	private:
	int type;
	double real;
	char* string;
	int padding;
	
	public:
	inline GMVariable() {
		type = 0;
		real = 0.0;
		string = NULL;
		padding = 0;
	}
	inline GMVariable(double a) {
		type = 0;
		real = a;
		string = NULL;
		padding = 0;
	}
	inline GMVariable(const char* a) {
		unsigned int len = strlen(a);
		char *data = (char*)(malloc(len+13));
		if(data==NULL) {
			throw std::bad_alloc();
		}
		// Apparently the first two bytes are the code page (0xfde9 = UTF8)
		// and the next two bytes are the number of bytes per character (1).
		// But it also works if you just set it to 0, apparently.
		// This is little-endian, so the two first bytes actually go last.
		*(unsigned int*)(data) = 0x0001fde9;
		// This is the reference count. I just set it to a high value
		// so GM doesn't try to free the memory.
		*(unsigned int*)(data+4) = 1000;
		// Finally, the length of the string.
		*(unsigned int*)(data+8) = len;
		memcpy(data+12, a, len+1);
		type = 1;
		real = 0.0;
		string = data+12;
		padding = 0;
	}
	inline ~GMVariable() {
		if(string!=NULL) {
			free(string-12);
		}
	}
	
};

class GMProc {
	
	private:
	const char* name;
	void *procaddr;
	
	public:
	GMProc(const char* functionname);
	bool Find();
	
	static void SetLookupFuncAddress(unsigned int address);
	
	private:
#ifdef __GNUC__
	double ExternalCall(int argcount, GMVariable* args) __attribute__((noinline));
#else
	double ExternalCall(int argcount, GMVariable* args);
#endif
	
	public:
	inline double operator()() {
		GMVariable args[] = {0.0}; // don't change this
		return ExternalCall(0, args);
	}
	template<typename T1>
	inline double operator()(T1 arg1) {
		GMVariable args[] = {arg1};
		return ExternalCall(1, args);
	}
	template<typename T1, typename T2>
	inline double operator()(T1 arg1, T2 arg2) {
		GMVariable args[] = {arg1, arg2};
		return ExternalCall(2, args);
	}
	template<typename T1, typename T2, typename T3>
	inline double operator()(T1 arg1, T2 arg2, T3 arg3) {
		GMVariable args[] = {arg1, arg2, arg3};
		return ExternalCall(3, args);
	}
	template<typename T1, typename T2, typename T3, typename T4>
	inline double operator()(T1 arg1, T2 arg2, T3 arg3, T4 arg4) {
		GMVariable args[] = {arg1, arg2, arg3, arg4};
		return ExternalCall(4, args);
	}
	template<typename T1, typename T2, typename T3, typename T4, typename T5>
	inline double operator()(T1 arg1, T2 arg2, T3 arg3, T4 arg4, T5 arg5) {
		GMVariable args[] = {arg1, arg2, arg3, arg4, arg5};
		return ExternalCall(5, args);
	}
	template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
	inline double operator()(T1 arg1, T2 arg2, T3 arg3, T4 arg4, T5 arg5, T6 arg6) {
		GMVariable args[] = {arg1, arg2, arg3, arg4, arg5, arg6};
		return ExternalCall(6, args);
	}
	template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
	inline double operator()(T1 arg1, T2 arg2, T3 arg3, T4 arg4, T5 arg5, T6 arg6, T7 arg7) {
		GMVariable args[] = {arg1, arg2, arg3, arg4, arg5, arg6, arg7};
		return ExternalCall(7, args);
	}
	template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
	inline double operator()(T1 arg1, T2 arg2, T3 arg3, T4 arg4, T5 arg5, T6 arg6, T7 arg7, T8 arg8) {
		GMVariable args[] = {arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8};
		return ExternalCall(8, args);
	}
};

#endif // GMAKER_H

