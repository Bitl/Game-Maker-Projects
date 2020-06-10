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
 * File: ep_World_CollisionBoxCircle.h
 * Calculates collisions between box and circle shapes.
 * Included in ep_World_Collision.cpp.
 */

#ifndef EP_WORLD_COLLISIONBOXCIRCLE_H
#define EP_WORLD_COLLISIONBOXCIRCLE_H

#include "ExtremePhysics.h"

EP_FORCEINLINE bool ep_World::CollisionBoxCircle(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
	// choose reference shape (box)
	bool flipped;
	if(shape1->shapetype!=EP_SHAPETYPE_BOX) {
		flipped = true;
		{ ep_Shape *t = shape1; shape1 = shape2; shape2 = t; }
	} else { // shape 2 is box
		flipped = false;
	}
	// now shape1 is the box
	
	if((shape1->shapedata.box.w==0.0 || shape1->shapedata.box.h==0.0) && shape2->shapedata.circle.r==0.0) return false;
	
	double best;
	unsigned long best_i;
	{
		// round 1
		best = -DBL_MAX;
		double xx, yy;
		xx = ep_invtransform_x(shape1->sin_t, shape1->cos_t, shape2->x_t-shape1->x_t, shape2->y_t-shape1->y_t);
		yy = ep_invtransform_y(shape1->sin_t, shape1->cos_t, shape2->x_t-shape1->x_t, shape2->y_t-shape1->y_t);
		// find maximum separation
		if(fabs(yy)-shape1->shapedata.box.h>fabs(xx)-shape1->shapedata.box.w) {
			best = fabs(yy)-shape1->shapedata.box.h-shape2->shapedata.circle.r-contact_threshold;
			if(best>=0.0) return false;
			best_i = (yy>0.0)? 1 : 3;
		} else {
			best = fabs(xx)-shape2->shapedata.circle.r-shape1->shapedata.box.w-contact_threshold;
			if(best>=0.0) return false;
			best_i = (xx>0.0)? 0 : 2;
		}
	}
	
	if(data==NULL && shape2->shapedata.circle.r==0.0) return true; // no more data needed
	
	// calculate the normal of shape1 in world coordinates
	double x, y;
	{
		double vx, vy;
		vx = ((best_i==0)? 1.0 : 0.0)-((best_i==2)? 1.0 : 0.0);
		vy = ((best_i==1)? 1.0 : 0.0)-((best_i==3)? 1.0 : 0.0);
		x = ep_transform_x(shape1->sin_t, shape1->cos_t, vx, vy);
		y = ep_transform_y(shape1->sin_t, shape1->cos_t, vx, vy);
	}
	
	// copy some variables for quick access
	double best_dist, best_pos, best_len;
	best_dist = (best_i==0 || best_i==2)? shape1->shapedata.box.w : shape1->shapedata.box.h;
	best_pos = (best_i==0 || best_i==2)? shape1->shapedata.box.h : shape1->shapedata.box.w;
	best_len = 2.0*best_pos;
	
	// get distance and position of circle center
	double d1, pos1;
	{
		double xx, yy;
		xx = shape2->x_t-shape1->x_t;
		yy = shape2->y_t-shape1->y_t;
		d1 = xx*x+yy*y-best_dist;
		pos1 = -(xx*y-yy*x-best_pos);
	}
	
	if((pos1<0.0 || pos1>best_len || best_pos==0.0) && d1>EP_EPSILON_MINCIRCLEVERTEX && shape2->shapedata.circle.r!=0.0) { // circle-vertex collision
		
		// clip point to reference edge
		if(pos1<0.0) pos1 = 0.0;
		if(pos1>best_len) pos1 = best_len;
		
		// calculate contact point on reference edge
		double xx, yy;
		xx = best_dist*x+(best_pos-pos1)*y;
		yy = best_dist*y-(best_pos-pos1)*x;
		double rx, ry;
		rx = shape2->x_t-shape1->x_t-xx;
		ry = shape2->y_t-shape1->y_t-yy;
		double d;
		d = ep_sqr(rx)+ep_sqr(ry);
		
		// check distance
		if(d>=ep_sqr(shape2->shapedata.circle.r+contact_threshold)) {
			return false;
		}
		
		if(data==NULL) return true; // no more data needed
		
		d = sqrt(d);
		if(d<EP_EPSILON_MINNORMALVECTOR) {
			rx = x;
			ry = y;
		} else {
			rx /= d;
			ry /= d;
		}
		
		data->nx = (flipped)? rx : -rx;
		data->ny = (flipped)? ry : -ry;
		data->cp_active[0] = true;
		data->cp_x[0][(flipped)? 1 : 0] = shape1->x_t+xx;
		data->cp_y[0][(flipped)? 1 : 0] = shape1->y_t+yy;
		data->cp_x[0][(flipped)? 0 : 1] = shape2->x_t-shape2->shapedata.circle.r*rx;
		data->cp_y[0][(flipped)? 0 : 1] = shape2->y_t-shape2->shapedata.circle.r*ry;
		data->cp_active[1] = false;
		
	} else { // circle-edge collision
		
		if(data==NULL) return true; // no more data needed
		
		data->nx = (flipped)? x : -x;
		data->ny = (flipped)? y : -y;
		data->cp_active[0] = true;
		data->cp_x[0][(flipped)? 1 : 0] = shape1->x_t+best_dist*x+(best_pos-pos1)*y;
		data->cp_y[0][(flipped)? 1 : 0] = shape1->y_t+best_dist*y-(best_pos-pos1)*x;
		data->cp_x[0][(flipped)? 0 : 1] = shape2->x_t-shape2->shapedata.circle.r*x;
		data->cp_y[0][(flipped)? 0 : 1] = shape2->y_t-shape2->shapedata.circle.r*y;
		data->cp_active[1] = false;
		
	}
	
	return true;
	
}

#endif // EP_WORLD_COLLISIONBOXCIRCLE_H

