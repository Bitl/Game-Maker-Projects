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
 * File: ep_World_CollisionBoxBox.h
 * Calculates collisions between box shapes.
 * Included in ep_World_Collision.cpp.
 */

#ifndef EP_WORLD_COLLISIONBOXBOX_H
#define EP_WORLD_COLLISIONBOXBOX_H

#include "ExtremePhysics.h"

EP_FORCEINLINE bool ep_World::CollisionBoxBox(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
	// copy some variables for quick access
	double total_sin_abs, total_cos_abs, w1, h1, w2, h2;
	total_sin_abs = fabs(ep_totalinvtransform_sin(shape1->sin_t, shape1->cos_t, shape2->sin_t, shape2->cos_t));
	total_cos_abs = fabs(ep_totalinvtransform_cos(shape1->sin_t, shape1->cos_t, shape2->sin_t, shape2->cos_t));
	w1 = shape1->shapedata.box.w;
	h1 = shape1->shapedata.box.h;
	w2 = shape2->shapedata.box.w;
	h2 = shape2->shapedata.box.h;
	
	double best1, best2;
	unsigned long best1_i, best2_i;
	{
		// round 1
		// calculate AABB of shape2 relative to shape1
		double vx, vy;
		vx = total_cos_abs*w2+total_sin_abs*h2;
		vy = total_sin_abs*w2+total_cos_abs*h2;
		// calculate vector from shape1 to shape2 relative to shape1
		double xx, yy;
		xx = ep_invtransform_x(shape1->sin_t, shape1->cos_t, shape2->x_t-shape1->x_t, shape2->y_t-shape1->y_t);
		yy = ep_invtransform_y(shape1->sin_t, shape1->cos_t, shape2->x_t-shape1->x_t, shape2->y_t-shape1->y_t);
		// find maximum separation
		if(fabs(yy)-vy-h1>fabs(xx)-vx-w1) {
			best1 = fabs(yy)-vy-h1-contact_threshold;
			if(best1>=0.0) return false;
			best1_i = (yy>0.0)? 1 : 3;
		} else {
			best1 = fabs(xx)-vx-w1-contact_threshold;
			if(best1>=0.0) return false;
			best1_i = (xx>0.0)? 0 : 2;
		}
	}
	{
		// round 2
		// calculate AABB of shape1 relative to shape2
		double vx, vy;
		vx = total_cos_abs*w1+total_sin_abs*h1;
		vy = total_sin_abs*w1+total_cos_abs*h1;
		// calculate vector from shape2 to shape1 relative to shape2
		double xx, yy;
		xx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, shape1->x_t-shape2->x_t, shape1->y_t-shape2->y_t);
		yy = ep_invtransform_y(shape2->sin_t, shape2->cos_t, shape1->x_t-shape2->x_t, shape1->y_t-shape2->y_t);
		// find maximum separation
		if(fabs(yy)-vy-h2>fabs(xx)-vx-w2) {
			best2 = fabs(yy)-vy-h2-contact_threshold;
			if(best2>=0.0) return false;
			best2_i = (yy>0.0)? 1 : 3;
		} else {
			best2 = fabs(xx)-vx-w2-contact_threshold;
			if(best2>=0.0) return false;
			best2_i = (xx>0.0)? 0 : 2;
		}
	}
	
	if(data==NULL) return true; // no more data needed
	
	// choose reference shape
	bool flipped;
	unsigned long best_i;
	if(best2>best1) {
		flipped = true;
		best_i = best2_i;
		{ ep_Shape *t = shape1; shape1 = shape2; shape2 = t; }
		w1 = shape1->shapedata.box.w;
		h1 = shape1->shapedata.box.h;
		w2 = shape2->shapedata.box.w;
		h2 = shape2->shapedata.box.h;
	} else {
		flipped = false;
		best_i = best1_i;
	}
	
	// calculate the normal of shape1 in world coordinates
	double x, y;
	{
		double vx, vy;
		vx = ((best_i==0)? 1.0 : 0.0)-((best_i==2)? 1.0 : 0.0);
		vy = ((best_i==1)? 1.0 : 0.0)-((best_i==3)? 1.0 : 0.0);
		x = ep_transform_x(shape1->sin_t, shape1->cos_t, vx, vy);
		y = ep_transform_y(shape1->sin_t, shape1->cos_t, vx, vy);
	}
	// calculate the normal of shape1 relative to shape2
	double nx, ny;
	nx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, x, y);
	ny = ep_invtransform_y(shape2->sin_t, shape2->cos_t, x, y);
	
	// copy some variables for quick access
	double best_dist, best_pos, best_len;
	best_dist = (best_i==0 || best_i==2)? w1 : h1;
	best_pos = (best_i==0 || best_i==2)? h1 : w1;
	best_len = 2.0*best_pos;
	
	// find best edge on shape2
	unsigned long min_j1, min_j2;
	if(fabs(nx)>fabs(ny)) {
		min_j1 = (nx<0.0)? 0 : 2;
		min_j2 = (nx<0.0)? 1 : 3;
	} else {
		min_j1 = (ny<0.0)? 1 : 3;
		min_j2 = (ny<0.0)? 2 : 0;
	}
	
	// calculate vector from shape1 to shape2 relative to the reference edge
	double e, f;
	{
		double xx, yy;
		xx = shape2->x_t-shape1->x_t;
		yy = shape2->y_t-shape1->y_t;
		e = xx*x+yy*y-best_dist;
		f = xx*y-yy*x-best_pos;
	}
	
	// get distance and position of edge points
	double d1, d2, pos1, pos2;
	{
		double vx, vy;
		vx = w2*((min_j1==0 || min_j1==1)? +1.0 : -1.0);
		vy = h2*((min_j1==1 || min_j1==2)? +1.0 : -1.0);
		d1 = e+vx*nx+vy*ny;
		pos1 = -(f+vx*ny-vy*nx);
		vx = w2*((min_j2==0 || min_j2==1)? +1.0 : -1.0);
		vy = h2*((min_j2==1 || min_j2==2)? +1.0 : -1.0);
		d2 = e+vx*nx+vy*ny;
		pos2 = -(f+vx*ny-vy*nx);
	}
	
	// check if the segments overlap (assuming pos1>pos2)
	if(pos1<0.0-contact_threshold || pos2>best_len+contact_threshold) {
		return false;
	}
	
	data->nx = (flipped)? x : -x;
	data->ny = (flipped)? y : -y;
	
	// clip edge points to reference edge
	double newd1, newd2, newpos1, newpos2;
	
	// is linear interpolation possible?
	if(pos1-pos2<EP_EPSILON_MINCONTACTPOINTS) {
		newd1 = d1;
		newpos1 = pos1;
		newd2 = d2;
		newpos2 = pos2;
	} else {
		if(pos1>best_len) {
			double d;
			d = (pos1-best_len)/(pos1-pos2);
			d = ep_clamp(0.0, 1.0, d);
			newd1 = d*d2+(1.0-d)*d1;
			newpos1 = best_len;
		} else {
			newd1 = d1;
			newpos1 = pos1;
		}
		if(pos2<0.0) {
			double d;
			d = (0.0-pos2)/(pos1-pos2);
			d = ep_clamp(0.0, 1.0, d);
			newd2 = d*d1+(1.0-d)*d2;
			newpos2 = 0.0;
		} else {
			newd2 = d2;
			newpos2 = pos2;
		}
	}
	
	// check the distance between the points
	if(newpos1-newpos2<EP_EPSILON_MINCONTACTPOINTS) {
		
		if(newd2<newd1) { // contact point 2 is more important
			newd1 = newd2;
			newpos1 = newpos2;
		}
		newpos1 = ep_clamp(0.0, best_len, newpos1);
		
		if(newd1<contact_threshold) {
			data->cp_active[0] = true;
			data->cp_x[0][(flipped)? 1 : 0] = shape1->x_t+best_dist*x+(best_pos-newpos1)*y;
			data->cp_y[0][(flipped)? 1 : 0] = shape1->y_t+best_dist*y-(best_pos-newpos1)*x;
			data->cp_x[0][(flipped)? 0 : 1] = shape1->x_t+(best_dist+newd1)*x+(best_pos-newpos1)*y;
			data->cp_y[0][(flipped)? 0 : 1] = shape1->y_t+(best_dist+newd1)*y-(best_pos-newpos1)*x;
		} else {
			return false; // rounding error
		}
		data->cp_active[1] = false;
		
	} else {
		
		if(newd1<contact_threshold) {
			data->cp_active[(flipped)? 1 : 0] = true;
			data->cp_x[(flipped)? 1 : 0][(flipped)? 1 : 0] = shape1->x_t+best_dist*x+(best_pos-newpos1)*y;
			data->cp_y[(flipped)? 1 : 0][(flipped)? 1 : 0] = shape1->y_t+best_dist*y-(best_pos-newpos1)*x;
			data->cp_x[(flipped)? 1 : 0][(flipped)? 0 : 1] = shape1->x_t+(best_dist+newd1)*x+(best_pos-newpos1)*y;
			data->cp_y[(flipped)? 1 : 0][(flipped)? 0 : 1] = shape1->y_t+(best_dist+newd1)*y-(best_pos-newpos1)*x;
		} else {
			data->cp_active[(flipped)? 1 : 0] = false;
		}
		if(newd2<contact_threshold) {
			data->cp_active[(flipped)? 0 : 1] = true;
			data->cp_x[(flipped)? 0 : 1][(flipped)? 1 : 0] = shape1->x_t+best_dist*x+(best_pos-newpos2)*y;
			data->cp_y[(flipped)? 0 : 1][(flipped)? 1 : 0] = shape1->y_t+best_dist*y-(best_pos-newpos2)*x;
			data->cp_x[(flipped)? 0 : 1][(flipped)? 0 : 1] = shape1->x_t+(best_dist+newd2)*x+(best_pos-newpos2)*y;
			data->cp_y[(flipped)? 0 : 1][(flipped)? 0 : 1] = shape1->y_t+(best_dist+newd2)*y-(best_pos-newpos2)*x;
		} else {
			data->cp_active[(flipped)? 0 : 1] = false;
		}
		if(!data->cp_active[0] && !data->cp_active[1]) return false; // rounding error
		
	}
	
	return true;
	
}

#endif // EP_WORLD_COLLISIONBOXBOX_H

