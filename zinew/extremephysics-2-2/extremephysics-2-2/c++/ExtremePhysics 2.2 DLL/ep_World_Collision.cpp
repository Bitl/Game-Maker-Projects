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
 * File: ep_World_Collision.cpp
 * Calculates collisions between shapes.
 */

#include "ExtremePhysics.h"

#include "ep_World_CollisionBoxBox.h"
#include "ep_World_CollisionBoxCircle.h"
#include "ep_World_CollisionBoxPolygon.h"
#include "ep_World_CollisionCircleCircle.h"
#include "ep_World_CollisionCirclePolygon.h"
#include "ep_World_CollisionPolygonPolygon.h"

bool ep_World::CollisionAABB(ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	if(shape1->aabb_x1>shape2->aabb_x2+contact_threshold
	|| shape1->aabb_y1>shape2->aabb_y2+contact_threshold
	|| shape2->aabb_x1>shape1->aabb_x2+contact_threshold
	|| shape2->aabb_y1>shape1->aabb_y2+contact_threshold) return false;
	return Collision(NULL, shape1, shape2, contact_threshold);
}

bool ep_World::Collision(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold) {
	
	if(shape1->shapetype==EP_SHAPETYPE_BOX && shape2->shapetype==EP_SHAPETYPE_BOX) {
		return CollisionBoxBox(data, shape1, shape2, contact_threshold);
	}
	if((shape1->shapetype==EP_SHAPETYPE_BOX && shape2->shapetype==EP_SHAPETYPE_CIRCLE) || (shape1->shapetype==EP_SHAPETYPE_CIRCLE && shape2->shapetype==EP_SHAPETYPE_BOX)) {
		return CollisionBoxCircle(data, shape1, shape2, contact_threshold);
	}
	if((shape1->shapetype==EP_SHAPETYPE_BOX && shape2->shapetype==EP_SHAPETYPE_POLYGON) || (shape1->shapetype==EP_SHAPETYPE_POLYGON && shape2->shapetype==EP_SHAPETYPE_BOX)) {
		return CollisionBoxPolygon(data, shape1, shape2, contact_threshold);
	}
	if(shape1->shapetype==EP_SHAPETYPE_CIRCLE && shape2->shapetype==EP_SHAPETYPE_CIRCLE) {
		return CollisionCircleCircle(data, shape1, shape2, contact_threshold);
	}
	if((shape1->shapetype==EP_SHAPETYPE_CIRCLE && shape2->shapetype==EP_SHAPETYPE_POLYGON) || (shape1->shapetype==EP_SHAPETYPE_POLYGON && shape2->shapetype==EP_SHAPETYPE_CIRCLE)) {
		return CollisionCirclePolygon(data, shape1, shape2, contact_threshold);
	}
	if(shape1->shapetype==EP_SHAPETYPE_POLYGON && shape2->shapetype==EP_SHAPETYPE_POLYGON) {
		return CollisionPolygonPolygon(data, shape1, shape2, contact_threshold);
	}
	
	return false; // "not all possible execution paths return a value"
}

