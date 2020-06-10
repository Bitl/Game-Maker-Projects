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
 * File: ep_World_RayCastCircle.h
 * Calculates collisions between rays and circle shapes.
 * Included in ep_World_RayCast.cpp.
 */

#ifndef EP_WORLD_RAYCASTCIRCLE_H
#define EP_WORLD_RAYCASTCIRCLE_H

#include "ExtremePhysics.h"

EP_FORCEINLINE double ep_World::RayCastCircle(ep_Shape* shape, double x, double y, double vx, double vy) {
	
	if(shape->shapedata.circle.r==0.0) return -1.0;
	
	double q = fabs((shape->x_t-x)*vy-(shape->y_t-y)*vx);
	if(q>shape->shapedata.circle.r) return -1.0;
	double p = (shape->x_t-x)*vx+(shape->y_t-y)*vy;
	double offset = sqrt(ep_sqr(shape->shapedata.circle.r)-ep_sqr(q));
	if(p<-offset) return -1.0;
	return ep_max(0.0, p-offset);
	
}

#endif // EP_WORLD_RAYCASTCIRCLE_H

