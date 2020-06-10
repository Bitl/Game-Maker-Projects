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
 * File: ep_Shape.cpp
 * Implementation of ep_Shape.
 */

#include "ExtremePhysics.h"

void ep_Shape::Init(ep_Body* _body, int _shapetype, double _x, double _y, double _rot, double _density, double arg1, double arg2, ep_Polygon* arg3, unsigned long _id) {
	
	// add to linked list
	main = _body->main;
	world = _body->world;
	body = _body;
	prev = body->last_shape;
	next = NULL;
	if(prev==NULL) body->first_shape = this;
	else prev->next = this;
	body->last_shape = this;
	
	// initialize variables
	id = _id;
	
	shapetype = _shapetype;
	switch(shapetype) {
		case EP_SHAPETYPE_BOX: {
			shapedata.box.w = arg1;
			shapedata.box.h = arg2;
			break;
		}
		case EP_SHAPETYPE_CIRCLE: {
			shapedata.circle.r = arg1;
			break;
		}
		case EP_SHAPETYPE_POLYGON: {
			shapedata.polygon.polygon = arg3;
			break;
		}
	}
	
	x = _x;
	y = _y;
	rot = _rot;
	rot_sin = sin(_rot);
	rot_cos = cos(_rot);
	density = _density;
	
	restitution = 0.0;
	friction = 0.0;
	normalvelocity = 0.0;
	tangentvelocity = 0.0;
	collidemask1 = 0;
	collidemask2 = 0;
	group = 0;
	
	updateaabb = true;
	updatecontacts = true;
	
	first_contactlink = NULL;
	last_contactlink = NULL;
	
#if EP_COMPILE_DEBUGCHECKS
	if(shapetype==EP_SHAPETYPE_POLYGON) {
		++(shapedata.polygon.polygon->referencecount);
	}
#endif
	
	++world->shapecount;
	++body->shapecount;
	
}

void ep_Shape::DeInit() {
	
	// destroy contacts
	while(first_contactlink!=NULL) world->DestroyContact(first_contactlink->contact);
	
	// remove from linked list
	if(prev==NULL) body->first_shape = next;
	else prev->next = next;
	if(next==NULL) body->last_shape = prev;
	else next->prev = prev;
	if(body->current_shape==this) body->current_shape = NULL;
	
#if EP_COMPILE_DEBUGCHECKS
	if(shapetype==EP_SHAPETYPE_POLYGON) {
		--(shapedata.polygon.polygon->referencecount);
	}
#endif
	
	--world->shapecount;
	--body->shapecount;
	
}

void ep_Shape::UpdateTransformedCoordinates() {
	
	// calculate world coordinates of shape origin
	x_t = body->x+ep_transform_x(body->rot_sin, body->rot_cos, x-body->xoff, y-body->yoff);
	y_t = body->y+ep_transform_y(body->rot_sin, body->rot_cos, x-body->xoff, y-body->yoff);
	
	// calculate total transformation
	sin_t = ep_totaltransform_sin(body->rot_sin, body->rot_cos, rot_sin, rot_cos);
	cos_t = ep_totaltransform_cos(body->rot_sin, body->rot_cos, rot_sin, rot_cos);
	
}

void ep_Shape::UpdateAABB() {
	// calculates the AABB
	// call UpdateTransformedCoordinates() first
	
	// set up AABB (Axis Aligned Bounding Box)
	if(shapetype==EP_SHAPETYPE_BOX) {
		double xx = fabs(cos_t)*shapedata.box.w+fabs(sin_t)*shapedata.box.h;
		double yy = fabs(sin_t)*shapedata.box.w+fabs(cos_t)*shapedata.box.h;
		aabb_x1 = x_t-xx;
		aabb_y1 = y_t-yy;
		aabb_x2 = x_t+xx;
		aabb_y2 = y_t+yy;
	}
	if(shapetype==EP_SHAPETYPE_CIRCLE) {
		aabb_x1 = x_t-shapedata.circle.r;
		aabb_y1 = y_t-shapedata.circle.r;
		aabb_x2 = x_t+shapedata.circle.r;
		aabb_y2 = y_t+shapedata.circle.r;
	}
	if(shapetype==EP_SHAPETYPE_POLYGON) {
		ep_PolygonVertex *vertex = shapedata.polygon.polygon->vertices;
		unsigned long vc = shapedata.polygon.polygon->vertexcount;
		double xx = x_t+ep_transform_x(sin_t, cos_t, vertex[0].x, vertex[0].y);
		double yy = y_t+ep_transform_y(sin_t, cos_t, vertex[0].x, vertex[0].y);
		aabb_x1 = xx;
		aabb_y1 = yy;
		aabb_x2 = xx;
		aabb_y2 = yy;
		for(unsigned long i = 1; i<vc; ++i) {
			xx = x_t+ep_transform_x(sin_t, cos_t, vertex[i].x, vertex[i].y);
			yy = y_t+ep_transform_y(sin_t, cos_t, vertex[i].x, vertex[i].y);
			if(xx<aabb_x1) aabb_x1 = xx;
			if(yy<aabb_y1) aabb_y1 = yy;
			if(xx>aabb_x2) aabb_x2 = xx;
			if(yy>aabb_y2) aabb_y2 = yy;
		}
	}
	
	updateaabb = false;
	
}

void ep_Shape::InitVirtualBox(double w, double h, double _x, double _y, double _rot) {
	if(w<EP_EPSILON_MINNORMALVECTOR) w = 0.0f;
	if(h<EP_EPSILON_MINNORMALVECTOR) h = 0.0f;
	if(w==0.0 && h==0.0) {
		shapetype = EP_SHAPETYPE_CIRCLE;
		shapedata.circle.r = 0.0;
		x_t = _x;
		y_t = _y;
		sin_t = 0.0;
		cos_t = 1.0;
	} else {
		shapetype = EP_SHAPETYPE_BOX;
		shapedata.box.w = ep_max(0.0, w*0.5);
		shapedata.box.h = ep_max(0.0, h*0.5);
		x_t = _x;
		y_t = _y;
		sin_t = sin(_rot);
		cos_t = cos(_rot);
	}
}

void ep_Shape::InitVirtualLine(double x1, double y1, double x2, double y2) {
	double vx, vy, length;
	vx = x2-x1;
	vy = y2-y1;
	length = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(length<EP_EPSILON_MINNORMALVECTOR) {
		shapetype = EP_SHAPETYPE_CIRCLE;
		shapedata.circle.r = 0.0;
		x_t = (x1+x2)*0.5;
		y_t = (y1+y2)*0.5;
		sin_t = 0.0;
		cos_t = 1.0;
	} else {
		shapetype = EP_SHAPETYPE_BOX;
		shapedata.box.w = length*0.5;
		shapedata.box.h = 0.0;
		x_t = (x1+x2)*0.5;
		y_t = (y1+y2)*0.5;
		sin_t = -vy/length;
		cos_t = vx/length;
	}
}

void ep_Shape::InitVirtualCircle(double r, double _x, double _y) {
	shapetype = EP_SHAPETYPE_CIRCLE;
	shapedata.circle.r = (r<EP_EPSILON_MINNORMALVECTOR)? 0.0 : r;
	x_t = _x;
	y_t = _y;
	sin_t = 0.0;
	cos_t = 1.0;
}

bool ep_Shape::InitVirtualPolygon(ep_Polygon* polygon, double _x, double _y, double _rot) {
#if EP_COMPILE_DEBUGCHECKS
	// does the polygon belong to this world?
	if(polygon->world!=world) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not do collision test in world %lu, the polygon does not belong to this world.", world->id);
#endif
		return false;
	}
	// has the polygon been initialized?
	if(!polygon->initialized) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not do collision test in world %lu, polygon %lu has not been initialized.", world->id, polygon->id);
#endif
		return false;
	}
#endif
	shapetype = EP_SHAPETYPE_POLYGON;
	shapedata.polygon.polygon = polygon;
	x_t = _x;
	y_t = _y;
	sin_t = sin(_rot);
	cos_t = cos(_rot);
	return true;
}

// contacts

ep_Contact* ep_Shape::GetFirstContact() {
	if(first_contactlink==NULL) return NULL;
	else return first_contactlink->contact;
}

ep_Contact* ep_Shape::GetLastContact() {
	if(last_contactlink==NULL) return NULL;
	else return last_contactlink->contact;
}

ep_Contact* ep_Shape::GetPreviousContact(ep_Contact* contact) {
	if(this==contact->shape1) {
		if(contact->link1.prev==NULL) return NULL;
		else return contact->link1.prev->contact;
	}
	if(this==contact->shape2) {
		if(contact->link2.prev==NULL) return NULL;
		else return contact->link2.prev->contact;
	}
	return NULL;
}

ep_Contact* ep_Shape::GetNextContact(ep_Contact* contact) {
	if(this==contact->shape1) {
		if(contact->link1.next==NULL) return NULL;
		else return contact->link1.next->contact;
	}
	if(this==contact->shape2) {
		if(contact->link2.next==NULL) return NULL;
		else return contact->link2.next->contact;
	}
	return NULL;
}

// other

void ep_Shape::SetMaterial(float _restitution, float _friction, double _normalvelocity, double _tangentvelocity) {
	if(restitution!=_restitution || friction!=_friction || normalvelocity!=_normalvelocity || tangentvelocity!=_tangentvelocity) {
		restitution = _restitution;
		friction = _friction;
		normalvelocity = _normalvelocity;
		tangentvelocity = _tangentvelocity;
		body->Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed material of shape %lu of body %lu in world %lu.", id, body->id, world->id);
#endif
}

void ep_Shape::SetCollision(unsigned long _collidemask1, unsigned long _collidemask2, unsigned long _group) {
	if(collidemask1!=_collidemask1 || collidemask2!=_collidemask2 || group!=_group) {
		collidemask1 = _collidemask1;
		collidemask2 = _collidemask2;
		group = _group;
		updatecontacts = true;
		// Awake is not needed
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed collision settings of shape %lu of body %lu in world %lu.", id, body->id, world->id);
#endif
}

bool ep_Shape::CollisionTestBox(double w, double h, double _x, double _y, double _rot, double contact_threshold) {
	ep_Shape dummy;
	dummy.InitVirtualBox(w, h, _x, _y, _rot);
	if(updateaabb) {
		UpdateTransformedCoordinates();
		UpdateAABB();
	}
	dummy.UpdateAABB();
	return world->CollisionAABB(this, &dummy, contact_threshold);
}

bool ep_Shape::CollisionTestLine(double x1, double y1, double x2, double y2, double contact_threshold) {
	ep_Shape dummy;
	dummy.InitVirtualLine(x1, y1, x2, y2);
	if(updateaabb) {
		UpdateTransformedCoordinates();
		UpdateAABB();
	}
	dummy.UpdateAABB();
	return world->CollisionAABB(this, &dummy, contact_threshold);
}

bool ep_Shape::CollisionTestCircle(double r, double _x, double _y, double contact_threshold) {
	ep_Shape dummy;
	dummy.InitVirtualCircle(r, _x, _y);
	if(updateaabb) {
		UpdateTransformedCoordinates();
		UpdateAABB();
	}
	dummy.UpdateAABB();
	return world->CollisionAABB(this, &dummy, contact_threshold);
}

bool ep_Shape::CollisionTestPolygon(ep_Polygon* polygon, double _x, double _y, double _rot, double contact_threshold) {
	ep_Shape dummy;
	if(!dummy.InitVirtualPolygon(polygon, _x, _y, _rot)) return false;
	if(updateaabb) {
		UpdateTransformedCoordinates();
		UpdateAABB();
	}
	dummy.UpdateAABB();
	return world->CollisionAABB(this, &dummy, contact_threshold);
}

double ep_Shape::RayCast(double _x, double _y, double vx, double vy) {
	double d = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	UpdateTransformedCoordinates();
	return world->RayCast(this, _x, _y, vx/d, vy/d);
}

