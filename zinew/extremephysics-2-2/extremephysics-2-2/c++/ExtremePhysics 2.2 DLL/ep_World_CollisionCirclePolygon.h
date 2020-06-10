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
 * File: ep_World_CollisionCirclePolygon.h
 * Calculates collisions between circle and polygon shapes.
 * Included in ep_World_Collision.cpp.
 */

#ifndef EP_WORLD_COLLISIONCIRCLEPOLYGON_H
#define EP_WORLD_COLLISIONCIRCLEPOLYGON_H

#include "ExtremePhysics.h"

EP_FORCEINLINE bool ep_World::CollisionCirclePolygon(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
	// choose reference shape (polygon)
	bool flipped;
	if(shape1->shapetype!=EP_SHAPETYPE_POLYGON) {
		flipped = true;
		{ ep_Shape *t = shape1; shape1 = shape2; shape2 = t; }
	} else {
		flipped = false;
	}
	// now shape1 is the polygon
	
	// copy some variables for quick access
	ep_PolygonVertex *vertices1;
	unsigned long vc;
	vertices1 = shape1->shapedata.polygon.polygon->vertices;
	vc = shape1->shapedata.polygon.polygon->vertexcount;
	
	double best;
	unsigned long best_i = 0;
	{
		// round1
		double xx, yy;
		xx = shape2->x_t-shape1->x_t;
		yy = shape2->y_t-shape1->y_t;
		best = -DBL_MAX;
		double rr = contact_threshold+shape2->shapedata.circle.r;
		unsigned long n = ep_shuffle_getn(vc);
		for(unsigned long k = 0; k<vc; ++k) {
			unsigned long i = ep_shuffle(k,n);
			double min_dot =
			  xx*ep_transform_x(shape1->sin_t, shape1->cos_t, vertices1[i].nx, vertices1[i].ny)+
			  yy*ep_transform_y(shape1->sin_t, shape1->cos_t, vertices1[i].nx, vertices1[i].ny)
			  -vertices1[i].dist-rr;
			// find maximum separation
			if(min_dot>=0.0) return false;
			if(min_dot>best) {
				best = min_dot;
				best_i = i;
			}
		}
	}
	
	// calculate the normal of shape1 in world coordinates
	double x, y;
	x = ep_transform_x(shape1->sin_t, shape1->cos_t, vertices1[best_i].nx, vertices1[best_i].ny);
	y = ep_transform_y(shape1->sin_t, shape1->cos_t, vertices1[best_i].nx, vertices1[best_i].ny);
	
	// copy some variables for quick access
	double best_dist, best_pos, best_len;
	best_dist = vertices1[best_i].dist;
	best_pos = vertices1[best_i].pos;
	best_len = vertices1[best_i].len;
	
	// get distance and position of circle center
	double d1, pos1;
	{
		double xx, yy;
		xx = shape2->x_t-shape1->x_t;
		yy = shape2->y_t-shape1->y_t;
		d1 = xx*x+yy*y-best_dist;
		pos1 = -(xx*y-yy*x-best_pos);
	}
	
	if((pos1<0.0 || pos1>best_len) && d1>EP_EPSILON_MINCIRCLEVERTEX && shape2->shapedata.circle.r!=0.0) { // circle-vertex collision
		
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

#endif // EP_WORLD_COLLISIONCIRCLEPOLYGON_H

