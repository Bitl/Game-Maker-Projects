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
 * File: ep_World_CollisionCircleCircle.h
 * Calculates collisions between circle shapes.
 * Included in ep_World_Collision.cpp.
 */

#ifndef EP_WORLD_COLLISIONCIRCLECIRCLE_H
#define EP_WORLD_COLLISIONCIRCLECIRCLE_H

#include "ExtremePhysics.h"

EP_FORCEINLINE bool ep_World::CollisionCircleCircle(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
	if(shape1->shapedata.circle.r==0.0 && shape2->shapedata.circle.r==0.0) return false;
	
	// get normal vector in world coordinates
	double x, y, d;
	x = shape2->x_t-shape1->x_t;
	y = shape2->y_t-shape1->y_t;
	d = ep_sqr(x)+ep_sqr(y);
	
	if(d>=ep_sqr(shape1->shapedata.circle.r+shape2->shapedata.circle.r+contact_threshold)) return false;
	if(data==NULL) return true; // no more data needed
	
	// normalize
	d = sqrt(d);
	if(d<EP_EPSILON_MINNORMALVECTOR) {
		x = 1.0;
		y = 0.0;
	} else {
		x /= d;
		y /= d;
	}
	
	data->nx = -x;
	data->ny = -y;
	data->cp_active[0] = true;
	data->cp_x[0][0] = shape1->x_t+shape1->shapedata.circle.r*x;
	data->cp_y[0][0] = shape1->y_t+shape1->shapedata.circle.r*y;
	data->cp_x[0][1] = shape2->x_t-shape2->shapedata.circle.r*x;
	data->cp_y[0][1] = shape2->y_t-shape2->shapedata.circle.r*y;
	data->cp_active[1] = false;
	
	return true;
	
}

#endif // EP_WORLD_COLLISIONCIRCLECIRCLE_H

