/*
 * Copyright 2009-2011 Maarten Baert
 * maarten-baert@hotmail.com
 * http://www.maartenbaert.be/
 * 
 * This file is part of ExtremePhysics.
 * 
 * ExtremePhysics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ExtremePhysics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ExtremePhysics. If not, see <http://www.gnu.org/licenses/>.
 * 
 * File: gm_gmfunctions.cpp
 * Game Maker related functions.
 */

#include "gm.h"

bool gmfunctions_initialized = false;
GMProc GM_draw_set_color("draw_set_color");
GMProc GM_draw_circle("draw_circle");
GMProc GM_draw_line("draw_line");
GMProc GM_draw_line_width("draw_line_width");
GMProc GM_draw_arrow("draw_arrow");
GMProc GM_draw_rectangle("draw_rectangle");

bool ep_gmfuncinit() {
	if(!GM_draw_set_color.Find()) return false;
	if(!GM_draw_circle.Find()) return false;
	if(!GM_draw_line.Find()) return false;
	if(!GM_draw_line_width.Find()) return false;
	if(!GM_draw_arrow.Find()) return false;
	if(!GM_draw_rectangle.Find()) return false;
	return true;
}

gmexport double ep_gm_functions_init() {
	if(gmfunctions_initialized) return 1;
	if(!ep_gmfuncinit()) return 0;
	gmfunctions_initialized = true;
	return 1;
}

gmexport double ep_gm_functions_init_address(double address) {
	if(gmfunctions_initialized) return 1;
	GMProc::SetLookupFuncAddress(gm_cast<unsigned int>(address));
	if(!ep_gmfuncinit()) return 0;
	gmfunctions_initialized = true;
	return 1;
}

