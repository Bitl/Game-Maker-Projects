/*
 * GMaker - C/C++ Library to enable Game Maker functionality for DLLs.
 * Copyright © 2009 Preston N. Smith
 * (this is a modified version of the original library - Maarten Baert)
 */

#include "GMaker.h"

#include <cstring>

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define WINVER          0x0500
#define _WIN32_WINNT    0x0500
#define _WIN32_WINDOWS  0x0500
#define _WIN32_IE       0x0500
#include <windows.h>

GMProc gmproc_lookupfunc(NULL);

GMProc::GMProc(const char* functionname) {
	name = functionname;
}

bool GMProc::Find() {
	/*
	Notes:
	* The address of the proctable is NOT fixed (the last four hex digits seem to be fixed though).
	  The address of the pointer to the proctable is fixed so this is used instead.
	* Apparently GetModuleHandleA(NULL) always returns 00400000
	  on Windows XP and Vista 32-bit. I'm not sure about 64-bit or Windows 7,
	  the return value seems to be different sometimes (invalid in this case).
	* The easiest way to find the new proctable address is to scan for 'action_path_old',
	  subtract 1 (to get the first entry), then do a reverse pointer scan. The pointer
	  should point exactly to the first byte. I used Cheat Engine 5.5 to do this.
	- Maarten Baert
	*/
	/*
	Update: 21/04/2010
	Since YYG is now updating GM8.1 about every week, it's impossible to keep the list of proctable addresses up-to-date.
	I'm now simply scanning the entire memory space to find the proctable - let's hope this will work better.
	*/
	/*
	Update: 30/07/2011
	YYG has added a new function (get_function_address) to get the address of a build-in function,
	so this function isn't needed anymore if you use GM8.1.
	*/
	
	if(procaddr!=NULL) {
		return true;
	}
	
	// for GM8.1
	if(gmproc_lookupfunc.procaddr!=NULL) {
		double addr = gmproc_lookupfunc(name);
		if(addr<=0.0) {
			return false;
		} else {
			procaddr = (void*)(unsigned int)(addr);
			return true;
		}
	}
	
	// for older versions
	static char* proctableaddress = NULL;
	if(proctableaddress==NULL) {
		
		unsigned int module = 0x00400000;
		unsigned int ret = *((int*)(module+0x0003a926));
		switch(ret) {
			/*
			16-04-2011 : added values for GM8.1 (EDIT: didn't work for later revisions, removed)
			23-12-2009 : added values for GM8
			*/
			case 0xda8b5557: proctableaddress = *((char**)(module+0x0018f148)); break; // GM8
			case 0x000002ff: proctableaddress = *((char**)(module+0x00189744)); break; // GM7
			case 0x00000259: proctableaddress = *((char**)(module+0x00138550)); break; // GM6
			case 0x5a592404: proctableaddress = *((char**)(module+0x00123470)); break; // ?
			case 0x8b01b0e4: proctableaddress = *((char**)(module+0x0011CA24)); break; // ?
			default: return false;
		}
		
	}
	for(char *address = proctableaddress; *address!=0; address += 0x50) {
		if(strcmp(address+1, name)==0) {
			procaddr = (void*)(*(unsigned int**)(address+0x44));
			return true;
		}
	}
	
	return false;
	
}

void GMProc::SetLookupFuncAddress(unsigned int address) {
	gmproc_lookupfunc.procaddr = (void*)(address);
}

double GMProc::ExternalCall(int argcount, GMVariable* args) {
	if(procaddr==NULL) return 0.0;
	int argsused = 0; // fixed number of arguments
	GMVariable retval;
	void *addr = procaddr;
#ifdef _MSC_VER
	__asm {
		push args;
		push argcount;
		lea ecx, retval;  // ecx = &retval
		push ecx;         // push &retval
		mov ecx, argsused;
		call addr;
	}
#else
	asm(
		".intel_syntax noprefix\n"
		"push %4\n"
		"push %3\n"
		"push %2\n"
		"mov ecx, %1\n"
		"call %0\n"
		".att_syntax\n"
		:: "a" (addr), "b" (argsused), "c" (&retval), "d" (argcount), "r" (args)
	);
#endif
	return retval.real;
}

