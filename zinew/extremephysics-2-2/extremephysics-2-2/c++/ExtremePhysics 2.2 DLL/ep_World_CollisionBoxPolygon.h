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
 * File: ep_World_CollisionBoxPolygon.h
 * Calculates collisions between box and polygon shapes.
 * Included in ep_World_Collision.cpp.
 */

#ifndef EP_WORLD_COLLISIONBOXPOLYGON_H
#define EP_WORLD_COLLISIONBOXPOLYGON_H

#include "ExtremePhysics.h"

EP_FORCEINLINE bool ep_World::CollisionBoxPolygon(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
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
	double total_sin, total_cos, w, h;
	vertices1 = shape1->shapedata.polygon.polygon->vertices; // vertex2 is never used
	vc = shape1->shapedata.polygon.polygon->vertexcount;
	total_sin = ep_totalinvtransform_sin(shape1->sin_t, shape1->cos_t, shape2->sin_t, shape2->cos_t);
	total_cos = ep_totalinvtransform_cos(shape1->sin_t, shape1->cos_t, shape2->sin_t, shape2->cos_t);
	w = shape2->shapedata.box.w;
	h = shape2->shapedata.box.h;
	
	double best1, best2;
	unsigned long best1_i = 0, best2_i = 0;
	{
		// round 1
		double xx, yy;
		// store vector from shape1 to shape2
		xx = shape2->x_t-shape1->x_t;
		yy = shape2->y_t-shape1->y_t;
		best1 = -DBL_MAX;
		unsigned long n = ep_shuffle_getn(vc);
		for(unsigned long k = 0; k<vc; ++k) {
			unsigned long i = ep_shuffle(k,n);
			double x, y;
			x = ep_transform_x(shape1->sin_t, shape1->cos_t, vertices1[i].nx, vertices1[i].ny);
			y = ep_transform_y(shape1->sin_t, shape1->cos_t, vertices1[i].nx, vertices1[i].ny);
			double nx, ny;
			nx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, x, y);
			ny = ep_invtransform_y(shape2->sin_t, shape2->cos_t, x, y);
			// find maximum separation
			double min_dot = -w*fabs(nx)-h*fabs(ny)+xx*x+yy*y-vertices1[i].dist-contact_threshold;
			if(min_dot>=0.0) return false;
			if(min_dot>best1) {
				best1 = min_dot;
				best1_i = i;
			}
		}
	}
	{
		// round 2
		double vx1, vy1, vx2, vy2, vx, vy;
		vx = ep_transform_x(total_sin, total_cos, vertices1[0].x, vertices1[0].y);
		vy = ep_transform_y(total_sin, total_cos, vertices1[0].x, vertices1[0].y);
		vx1 = vx;
		vy1 = vy;
		vx2 = vx;
		vy2 = vy;
		for(unsigned long j = 1; j<vc; ++j) {
			vx = ep_transform_x(total_sin, total_cos, vertices1[j].x, vertices1[j].y);
			vy = ep_transform_y(total_sin, total_cos, vertices1[j].x, vertices1[j].y);
			if(vx<vx1) vx1 = vx;
			if(vy<vy1) vy1 = vy;
			if(vx>vx2) vx2 = vx;
			if(vy>vy2) vy2 = vy;
		}
		{
			double xx, yy;
			xx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, shape1->x_t-shape2->x_t, shape1->y_t-shape2->y_t);
			yy = ep_invtransform_y(shape2->sin_t, shape2->cos_t, shape1->x_t-shape2->x_t, shape1->y_t-shape2->y_t);
			vx1 += xx;
			vy1 += yy;
			vx2 += xx;
			vy2 += yy;
		}
		// find maximum separation
		if(ep_max(vy1, -vy2)-h>ep_max(vx1, -vx2)-w) {
			best2 = ep_max(vy1, -vy2)-h-contact_threshold;
			if(best2>=0.0) return false;
			best2_i = (vy1>-vy2)? 1 : 3;
		} else {
			best2 = ep_max(vx1, -vx2)-w-contact_threshold;
			if(best2>=0.0) return false;
			best2_i = (vx1>-vx2)? 0 : 2;
		}
	}
	
	if(data==NULL) return true; // no more data needed
	
	// choose reference object
	if(best2>best1) {
		flipped = !flipped;
		{ ep_Shape *t = shape1; shape1 = shape2; shape2 = t; }
	}
	// best1_i = polygon
	// best2_i = box
	
	// calculate the normal of shape1 in world coordinates
	double x, y;
	if(shape1->shapetype==EP_SHAPETYPE_POLYGON) {
		x = ep_transform_x(shape1->sin_t, shape1->cos_t, vertices1[best1_i].nx, vertices1[best1_i].ny);
		y = ep_transform_y(shape1->sin_t, shape1->cos_t, vertices1[best1_i].nx, vertices1[best1_i].ny);
	} else {
		double vx, vy;
		vx = ((best2_i==0)? 1.0 : 0.0)-((best2_i==2)? 1.0 : 0.0);
		vy = ((best2_i==1)? 1.0 : 0.0)-((best2_i==3)? 1.0 : 0.0);
		x = ep_transform_x(shape1->sin_t, shape1->cos_t, vx, vy);
		y = ep_transform_y(shape1->sin_t, shape1->cos_t, vx, vy);
	}
	// calculate the normal of shape1 relative to shape2
	double nx, ny;
	nx = ep_invtransform_x(shape2->sin_t, shape2->cos_t, x, y);
	ny = ep_invtransform_y(shape2->sin_t, shape2->cos_t, x, y);
	
	// copy some variables for quick access
	double best_dist, best_pos, best_len;
	if(shape1->shapetype==EP_SHAPETYPE_POLYGON) {
		best_dist = vertices1[best1_i].dist;
		best_pos = vertices1[best1_i].pos;
		best_len = vertices1[best1_i].len;
	} else {
		best_dist = (best2_i==0 || best2_i==2)? w : h;
		best_pos = (best2_i==0 || best2_i==2)? h : w;
		best_len = 2.0*best_pos;
	}
	
	// find best edge on shape2
	unsigned long min_j1, min_j2;
	if(shape2->shapetype==EP_SHAPETYPE_POLYGON) {
		double min_dot = vertices1[0].nx*nx+vertices1[0].ny*ny;
		min_j1 = 0;
		for(unsigned long j=1;j<vc;++j) {
			double d = vertices1[j].nx*nx+vertices1[j].ny*ny;
			if(d<min_dot) {
				min_dot = d;
				min_j1 = j;
			}
		}
		min_j2 = (min_j1+1)%vc;
	} else {
		if(fabs(nx)>fabs(ny)) {
			min_j1 = (nx<0.0)? 0 : 2;
			min_j2 = (nx<0.0)? 1 : 3;
		} else {
			min_j1 = (ny<0.0)? 1 : 3;
			min_j2 = (ny<0.0)? 2 : 0;
		}
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
	if(shape2->shapetype==EP_SHAPETYPE_POLYGON) {
		d1 = e+vertices1[min_j1].x*nx+vertices1[min_j1].y*ny;
		pos1 = -(f+vertices1[min_j1].x*ny-vertices1[min_j1].y*nx);
		d2 = e+vertices1[min_j2].x*nx+vertices1[min_j2].y*ny;
		pos2 = -(f+vertices1[min_j2].x*ny-vertices1[min_j2].y*nx);
	} else {
		double vx, vy;
		vx = (min_j1==0 || min_j1==1)? +w : -w;
		vy = (min_j1==1 || min_j1==2)? +h : -h;
		d1 = e+vx*nx+vy*ny;
		pos1 = -(f+vx*ny-vy*nx);
		vx = (min_j2==0 || min_j2==1)? +w : -w;
		vy = (min_j2==1 || min_j2==2)? +h : -h;
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

#endif // EP_WORLD_COLLISIONBOXPOLYGON_H

