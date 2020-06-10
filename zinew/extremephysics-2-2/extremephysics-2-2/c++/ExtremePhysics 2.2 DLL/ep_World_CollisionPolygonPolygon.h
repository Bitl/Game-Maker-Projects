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
 * File: ep_World_CollisionPolygonPolygon.h
 * Calculates collisions between polygon shapes.
 * Included in ep_World_Collision.cpp.
 */

#ifndef EP_WORLD_COLLISIONPOLYGONPOLYGON_H
#define EP_WORLD_COLLISIONPOLYGONPOLYGON_H

#include "ExtremePhysics.h"

/*EP_FORCEINLINE*/ bool ep_World::CollisionPolygonPolygon(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
	// copy some variables for quick access
	ep_PolygonVertex *vertices1, *vertices2;
	unsigned long vc1, vc2;
	vertices1 = shape1->shapedata.polygon.polygon->vertices;
	vertices2 = shape2->shapedata.polygon.polygon->vertices;
	vc1 = shape1->shapedata.polygon.polygon->vertexcount;
	vc2 = shape2->shapedata.polygon.polygon->vertexcount;
	
	// find maximum separation
	double best1, best2;
	unsigned long best1_i = 0, best2_i = 0;
	{
		// round 1
		double xx, yy;
		xx = shape2->x_t-shape1->x_t;
		yy = shape2->y_t-shape1->y_t;
		best1 = -DBL_MAX;
		unsigned long n = ep_shuffle_getn(vc1);
		for(unsigned long k = 0; k<vc1; ++k) {
			unsigned long i = ep_shuffle(k, n);
			double x, y;
			x = ep_transform_x(shape1->sin_t, shape1->cos_t, vertices1[i].nx, vertices1[i].ny);
			y = ep_transform_y(shape1->sin_t, shape1->cos_t, vertices1[i].nx, vertices1[i].ny);
			double nx, ny;
			nx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, x, y);
			ny = ep_invtransform_y(shape2->sin_t, shape2->cos_t, x, y);
			// find maximum separation
			double min_dot = vertices2[0].x*nx+vertices2[0].y*ny;
			for(unsigned long j = 1; j<vc2; ++j) {
				double d = vertices2[j].x*nx+vertices2[j].y*ny;
				if(d<min_dot) {
					min_dot = d;
				}
			}
			min_dot += xx*x+yy*y-vertices1[i].dist-contact_threshold;
			if(min_dot>=0.0) return false;
			if(min_dot>best1) {
				best1 = min_dot;
				best1_i = i;
			}
		}
	}
	{
		// round 2
		double xx, yy;
		xx = shape1->x_t-shape2->x_t;
		yy = shape1->y_t-shape2->y_t;
		best2 = -DBL_MAX;
		unsigned long n = ep_shuffle_getn(vc2);
		for(unsigned long k = 0; k<vc2; ++k) {
			unsigned long i = ep_shuffle(k, n);
			double x, y;
			x = ep_transform_x(shape2->sin_t, shape2->cos_t, vertices2[i].nx, vertices2[i].ny);
			y = ep_transform_y(shape2->sin_t, shape2->cos_t, vertices2[i].nx, vertices2[i].ny);
			double nx, ny;
			nx = ep_invtransform_x(shape1->sin_t, shape1->cos_t, x, y);
			ny = ep_invtransform_y(shape1->sin_t, shape1->cos_t, x, y);
			// find maximum separation
			double min_dot = vertices1[0].x*nx+vertices1[0].y*ny;
			for(unsigned long j = 1; j<vc1; ++j) {
				double d = vertices1[j].x*nx+vertices1[j].y*ny;
				if(d<min_dot) {
					min_dot = d;
				}
			}
			min_dot += xx*x+yy*y-vertices2[i].dist-contact_threshold;
			if(min_dot>=0.0) return false;
			if(min_dot>best2) {
				best2 = min_dot;
				best2_i = i;
			}
		}
	}
	
	if(data==NULL) return true; // no more data needed
	
	// choose reference object
	bool flipped;
	unsigned long best_i;
	if(best2>best1) {
		flipped = true;
		best_i = best2_i;
		{ ep_Shape *t = shape1; shape1 = shape2; shape2 = t; }
		{ ep_PolygonVertex *t = vertices1; vertices1 = vertices2; vertices2 = t; }
		{ unsigned long t = vc1; vc1 = vc2; vc2 = t; }
	} else {
		flipped = false;
		best_i = best1_i;
	}
	
	// calculate the normal of shape1 in world coordinates
	double x, y;
	x = ep_transform_x(shape1->sin_t, shape1->cos_t, vertices1[best_i].nx, vertices1[best_i].ny);
	y = ep_transform_y(shape1->sin_t, shape1->cos_t, vertices1[best_i].nx, vertices1[best_i].ny);
	// calculate the normal of shape1 relative to shape2
	double nx, ny;
	nx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, x, y);
	ny = ep_invtransform_y(shape2->sin_t, shape2->cos_t, x, y);
	
	// copy some variables for quick access
	double best_dist, best_pos, best_len;
	best_dist = vertices1[best_i].dist;
	best_pos = vertices1[best_i].pos;
	best_len = vertices1[best_i].len;
	
	// find best edge on shape2
	unsigned long min_j1, min_j2;
	double min_dot = vertices2[0].nx*nx+vertices2[0].ny*ny;
	min_j1 = 0;
	for(unsigned long j=1;j<vc2;++j) {
		double d = vertices2[j].nx*nx+vertices2[j].ny*ny;
		if(d<min_dot) {
			min_dot = d;
			min_j1 = j;
		}
	}
	min_j2 = (min_j1+1)%vc2;
	
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
	d1 = e+vertices2[min_j1].x*nx+vertices2[min_j1].y*ny;
	pos1 = -(f+vertices2[min_j1].x*ny-vertices2[min_j1].y*nx);
	d2 = e+vertices2[min_j2].x*nx+vertices2[min_j2].y*ny;
	pos2 = -(f+vertices2[min_j2].x*ny-vertices2[min_j2].y*nx);
	
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

#endif // EP_WORLD_COLLISIONPOLYGONPOLYGON_H

