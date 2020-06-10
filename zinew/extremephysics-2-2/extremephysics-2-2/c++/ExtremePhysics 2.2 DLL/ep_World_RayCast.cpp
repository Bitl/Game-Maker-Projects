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
 * File: ep_World_RayCast.cpp
 * Calculates collisions between rays and shapes.
 */

#include "ExtremePhysics.h"

#include "ep_World_RayCastBox.h"
#include "ep_World_RayCastCircle.h"
#include "ep_World_RayCastPolygon.h"

double ep_World::RayCast(ep_Shape* shape, double x, double y, double vx, double vy) {
	
	switch(shape->shapetype) {
		case EP_SHAPETYPE_BOX: return RayCastBox(shape, x, y, vx, vy);
		case EP_SHAPETYPE_CIRCLE: return RayCastCircle(shape, x, y, vx, vy);
		case EP_SHAPETYPE_POLYGON: return RayCastPolygon(shape, x, y, vx, vy);
	}
	return -1.0; // "not all possible execution paths return a value"
	
}

