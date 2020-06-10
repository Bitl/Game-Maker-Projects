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
 * File: ep_Body.cpp
 * Implementation of ep_Body.
 */

#include "ExtremePhysics.h"

void ep_Body::Init(ep_World* _world, bool _isstatic, bool _norotation, unsigned long _id) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_body;
	next = NULL;
	if(prev==NULL) world->first_body = this;
	else prev->next = this;
	world->last_body = this;
	
	// initialize variables
	id = _id;
#if EP_USE_IDHASHTABLE
	world->idhashtable_bodies.Insert(this);
#endif
	
	idcounter_shapes = 0;
	shapecount = 0;
	first_shape = NULL;
	last_shape = NULL;
	current_shape = NULL;
	
	idcounter_forces = 0;
	forcecount = 0;
	first_force = NULL;
	last_force = NULL;
	current_force = NULL;
	
	isstatic = _isstatic;
	norotation = _norotation;
	
	xoff = 0.0;
	yoff = 0.0;
	mass = (isstatic)? 0.0 : 1.0;
	invmass = (isstatic)? 0.0 : 1.0/mass;
	inertia = (norotation)? 0.0 : 1.0;
	invinertia = (norotation)? 0.0 : 1.0/inertia;
	
	x = 0.0;
	y = 0.0;
	rot = 0.0;
	rot_sin = sin(rot);
	rot_cos = cos(rot);
	xvel = 0.0;
	yvel = 0.0;
	rotvel = 0.0;
	
	maxvel = 0.0;
	maxrotvel = 0.0;
	gravity_x = 0.0;
	gravity_y = 0.0;
	damping = 0.0;
	rotdamping = 0.0;
	damping_factor = 1.0;
	rotdamping_factor = 1.0;
	
	storecontactimpulses = true;
	storejointimpulses = true;
	sleepstable = true;
	sleepoutofview = true;
	issleeping = false;
	stabletimer = 0.0;
	outofviewtimer = 0.0;
	
	nextislandbody = NULL;
	
	first_hingejointlink = NULL;
	last_hingejointlink = NULL;
	first_distancejointlink = NULL;
	last_distancejointlink = NULL;
	first_railjointlink = NULL;
	last_railjointlink = NULL;
	first_sliderjointlink = NULL;
	last_sliderjointlink = NULL;
	//JOINTS//
	
#if EP_COMPILE_BOXCHAIN
	boxchain_vertexcount = 0;
	boxchain_vertices = NULL;
	boxchain_first_shape = NULL;
	boxchain_last_shape = NULL;
#endif // EP_COMPILE_BOXCHAIN
	++world->bodycount;
	
}

void ep_Body::DeInit() {
	
#if EP_COMPILE_BOXCHAIN
	ep_free(boxchain_vertices);
#endif // EP_COMPILE_BOXCHAIN
	
	// destroy joints
	while(first_hingejointlink!=NULL) world->DestroyHingeJoint(first_hingejointlink->hingejoint);
	while(first_distancejointlink!=NULL) world->DestroyDistanceJoint(first_distancejointlink->distancejoint);
	while(first_railjointlink!=NULL) world->DestroyRailJoint(first_railjointlink->railjoint);
	while(first_sliderjointlink!=NULL) world->DestroySliderJoint(first_sliderjointlink->sliderjoint);
	//JOINTS//
	
	// destroy everything (in reverse order)
	while(first_force!=NULL) DestroyForce(first_force);
	while(first_shape!=NULL) DestroyShape(first_shape);
	
	// remove from linked list
	if(prev==NULL) world->first_body = next;
	else prev->next = next;
	if(next==NULL) world->last_body = prev;
	else next->prev = prev;
	if(world->current_body==this) world->current_body = NULL;
	world->idhashtable_bodies.Remove(this);
	
	--world->bodycount;
	
}

/*
Sleeping system notes
These may sound a bit strange, but it actually works :).
* Bodies can go to sleep if sleeping is enabled, and the bodies are:
	- stable (very low velocity for a specified amount of time after last change) AND sleepstable==true AND world->time_stable!=0.0, OR
	- out of view (out of view for a specified amount of time) AND sleepoutofview==true AND world->time_outofview!=0.0
* When a change occurs, the stable timer is set to zero.
* A linkage change means a new link was created or destroyed:
	- a contact was created/destroyed -> Awake(true, false)
	- a joint was created/destroyed -> Awake(true, false)
	- ...
* A property change means some property of the body has changed which might influence other
  connected bodies even if the current body is static:
	- the position, velocity, mass, friction, ... was changed -> Awake(false, true)
	- ...
* Other changes:
	- a force was created/destroyed/changed (with ignoremass = false) -> Awake(false, false)
	- changes made to contacts, joints, ... -> Awake(false, false)
	- ...
* Static bodies can not sleep. They act like different bodies for every body they are connected with,
  copying the sleeping state of that body. This means a static body can be in contact with both sleeping
  bodies and awake bodies at the same time. When a property change occurs to a static body, the stable
  timer of all linked bodies is set to zero. This only applies to property changes, as linkage changes will
  only influence the 'copy' for the changed body, not the copies for the other bodies.
* Bodies can only sleep if all connected bodies are sleeping too. Groups of connected bodies are called islands.
  Sleeping islands are stored (for performance) but the other islands are recreated every step. When a linkage
  change occurs, a sleeping island will wake up.
* Sleeping is updated when ep_World::Step is called. The world will check the state of all bodies and
  awake them or put them to sleep. The world will also reset the stable timer if needed.
*/

void ep_Body::Awake(bool linkagechange, bool propertychange) {
	stabletimer = 0.0;
	if(linkagechange && issleeping) {
		issleeping = false;
		for(ep_Body *body2 = nextislandbody; body2!=this && body2!=NULL; body2 = body2->nextislandbody) {
			body2->issleeping = false;
		}
	}
	if(propertychange && isstatic) {
		// static bodies can not sleep, so when the properties of the static
		// body change we awake all linked bodies instead
		for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
			for(ep_ContactLink *contactlink = shape->first_contactlink; contactlink!=NULL; contactlink = contactlink->next) {
				contactlink->other_shape->body->stabletimer = 0.0;
			}
		}
		for(ep_HingeJointLink *hingejointlink = first_hingejointlink; hingejointlink!=NULL; hingejointlink = hingejointlink->next) {
			hingejointlink->other_body->stabletimer = 0.0;
		}
		for(ep_DistanceJointLink *distancejointlink = first_distancejointlink; distancejointlink!=NULL; distancejointlink = distancejointlink->next) {
			distancejointlink->other_body->stabletimer = 0.0;
		}
		for(ep_RailJointLink *railjointlink = first_railjointlink; railjointlink!=NULL; railjointlink = railjointlink->next) {
			railjointlink->other_body->stabletimer = 0.0;
		}
		for(ep_SliderJointLink *sliderjointlink = first_sliderjointlink; sliderjointlink!=NULL; sliderjointlink = sliderjointlink->next) {
			sliderjointlink->other_body->stabletimer = 0.0;
		}
		//JOINTS//
	}
}

void ep_Body::Moved() {
	for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
		shape->updateaabb = true;
		shape->updatecontacts = true;
	}
}

// shape

ep_Shape* ep_Body::CreateBoxShape(double _w, double _h, double _x, double _y, double _rot, double _density) {
	
	ep_Shape *shape = world->AllocateShape();
	if(shape==NULL) return NULL;
	
	if(_w<EP_EPSILON_MINNORMALVECTOR) _w = 0.0f;
	if(_h<EP_EPSILON_MINNORMALVECTOR) _h = 0.0f;
	if(_w==0.0 && _h==0.0) {
		shape->Init(this, EP_SHAPETYPE_CIRCLE, _x, _y, 0.0, 0.0, 0.0, 0.0, NULL, ++idcounter_shapes);
	} else {
		shape->Init(this, EP_SHAPETYPE_BOX, _x, _y, _rot, _density, _w*0.5, _h*0.5, NULL, ++idcounter_shapes);
	}
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created shape %lu for body %lu in world %lu.", shape->id, id, world->id);
#endif
	
	return shape;
}

ep_Shape* ep_Body::CreateLineShape(double x1, double y1, double x2, double y2, double _density) {
	
	ep_Shape *shape = world->AllocateShape();
	if(shape==NULL) return NULL;
	
	double vx, vy, length;
	vx = x2-x1;
	vy = y2-y1;
	length = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(length<EP_EPSILON_MINNORMALVECTOR) {
		shape->Init(this, EP_SHAPETYPE_CIRCLE, (x1+x2)*0.5, (y1+y2)*0.5, 0.0, 0.0, 0.0, 0.0, NULL, ++idcounter_shapes);
	} else {
		shape->Init(this, EP_SHAPETYPE_BOX, (x1+x2)*0.5, (y1+y2)*0.5, atan2(-vy, vx), _density, length*0.5, 0.0, NULL, ++idcounter_shapes);
	}
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created shape %lu for body %lu in world %lu.", shape->id, id, world->id);
#endif
	
	return shape;
}

ep_Shape* ep_Body::CreateCircleShape(double _r, double _x, double _y, double _rot, double _density) {
	// I know rotation is rather useless here,
	// but I keep it for consistency.
	
	ep_Shape *shape = world->AllocateShape();
	if(shape==NULL) return NULL;
	shape->Init(this, EP_SHAPETYPE_CIRCLE, _x, _y, _rot, _density, (_r<EP_EPSILON_MINNORMALVECTOR)? 0.0 : _r, 0.0, NULL, ++idcounter_shapes);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created shape %lu for body %lu in world %lu.", shape->id, id, world->id);
#endif
	
	return shape;
}

ep_Shape* ep_Body::CreatePolygonShape(ep_Polygon* polygon, double _x, double _y, double _rot, double _density) {
	
#if EP_COMPILE_DEBUGCHECKS
	// does the polygon belong to this world?
	if(polygon->world!=world) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create shape for body %lu in world %lu, the polygon does not belong to this world.", id, world->id);
#endif
		return NULL;
	}
	// has the polygon been initialized?
	if(!polygon->initialized) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create shape for body %lu in world %lu, polygon %lu has not been initialized.", id, world->id, polygon->id);
#endif
		return NULL;
	}
#endif
	
	ep_Shape *shape = world->AllocateShape();
	if(shape==NULL) return NULL;
	shape->Init(this, EP_SHAPETYPE_POLYGON, _x, _y , _rot, _density, 0.0, 0.0, polygon, ++idcounter_shapes);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created shape %lu for body %lu in world %lu.", shape->id, id, world->id);
#endif
	
	return shape;
}

void ep_Body::DestroyShape(ep_Shape* shape) {
	
	world->ClearCollisionList();
	
#if EP_COMPILE_BOXCHAIN
	ep_free(boxchain_vertices);
	boxchain_vertexcount = 0;
	boxchain_vertices = NULL;
	boxchain_first_shape = NULL;
	boxchain_last_shape = NULL;
#endif // EP_COMPILE_BOXCHAIN
	
	shape->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = shape->id;
#endif
	ep_free(shape);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed shape %lu of body %lu in world %lu.", _id, id, world->id);
#endif
	
}

ep_Shape* ep_Body::FindShape(unsigned long _id) {
	if(current_shape!=NULL) {
		if(current_shape->id==_id) return current_shape;
	}
	for(current_shape = first_shape; current_shape!=NULL; current_shape = current_shape->next) {
		if(current_shape->id==_id) return current_shape;
	}
	return NULL;
}

// force

ep_Force* ep_Body::CreateForce(double _x, double _y, bool _local, bool _ignoremass) {
	
	ep_Force *force = world->AllocateForce();
	if(force==NULL) return NULL;
	force->Init(this, _x, _y, _local, _ignoremass, ++idcounter_forces);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created force %lu for body %lu in world %lu.", force->id, id, world->id);
#endif
	
	return force;
}

void ep_Body::DestroyForce(ep_Force* force) {
	
	force->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = force->id;
#endif
	ep_free(force);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed force %lu of body %lu in world %lu.", _id, id, world->id);
#endif
	
}

ep_Force* ep_Body::FindForce(unsigned long _id) {
	if(current_force!=NULL) {
		if(current_force->id==_id) return current_force;
	}
	for(current_force = first_force; current_force!=NULL; current_force = current_force->next) {
		if(current_force->id==_id) return current_force;
	}
	return NULL;
}

// hinge joint links

ep_HingeJoint* ep_Body::GetFirstHingeJoint() {
	if(first_hingejointlink==NULL) return NULL;
	else return first_hingejointlink->hingejoint;
}

ep_HingeJoint* ep_Body::GetLastHingeJoint() {
	if(last_hingejointlink==NULL) return NULL;
	else return last_hingejointlink->hingejoint;
}

ep_HingeJoint* ep_Body::GetPreviousHingeJoint(ep_HingeJoint* hingejoint) {
	if(this==hingejoint->body1) {
		if(hingejoint->link1.prev==NULL) return NULL;
		else return hingejoint->link1.prev->hingejoint;
	}
	if(this==hingejoint->body2) {
		if(hingejoint->link2.prev==NULL) return NULL;
		else return hingejoint->link2.prev->hingejoint;
	}
	return NULL;
}

ep_HingeJoint* ep_Body::GetNextHingeJoint(ep_HingeJoint* hingejoint) {
	if(this==hingejoint->body1) {
		if(hingejoint->link1.next==NULL) return NULL;
		else return hingejoint->link1.next->hingejoint;
	}
	if(this==hingejoint->body2) {
		if(hingejoint->link2.next==NULL) return NULL;
		else return hingejoint->link2.next->hingejoint;
	}
	return NULL;
}

// distance joint links

ep_DistanceJoint* ep_Body::GetFirstDistanceJoint() {
	if(first_distancejointlink==NULL) return NULL;
	else return first_distancejointlink->distancejoint;
}

ep_DistanceJoint* ep_Body::GetLastDistanceJoint() {
	if(last_distancejointlink==NULL) return NULL;
	else return last_distancejointlink->distancejoint;
}

ep_DistanceJoint* ep_Body::GetPreviousDistanceJoint(ep_DistanceJoint* distancejoint) {
	if(this==distancejoint->body1) {
		if(distancejoint->link1.prev==NULL) return NULL;
		else return distancejoint->link1.prev->distancejoint;
	}
	if(this==distancejoint->body2) {
		if(distancejoint->link2.prev==NULL) return NULL;
		else return distancejoint->link2.prev->distancejoint;
	}
	return NULL;
}

ep_DistanceJoint* ep_Body::GetNextDistanceJoint(ep_DistanceJoint* distancejoint) {
	if(this==distancejoint->body1) {
		if(distancejoint->link1.next==NULL) return NULL;
		else return distancejoint->link1.next->distancejoint;
	}
	if(this==distancejoint->body2) {
		if(distancejoint->link2.next==NULL) return NULL;
		else return distancejoint->link2.next->distancejoint;
	}
	return NULL;
}

// rail joint links

ep_RailJoint* ep_Body::GetFirstRailJoint() {
	if(first_railjointlink==NULL) return NULL;
	else return first_railjointlink->railjoint;
}

ep_RailJoint* ep_Body::GetLastRailJoint() {
	if(last_railjointlink==NULL) return NULL;
	else return last_railjointlink->railjoint;
}

ep_RailJoint* ep_Body::GetPreviousRailJoint(ep_RailJoint* railjoint) {
	if(this==railjoint->body1) {
		if(railjoint->link1.prev==NULL) return NULL;
		else return railjoint->link1.prev->railjoint;
	}
	if(this==railjoint->body2) {
		if(railjoint->link2.prev==NULL) return NULL;
		else return railjoint->link2.prev->railjoint;
	}
	return NULL;
}

ep_RailJoint* ep_Body::GetNextRailJoint(ep_RailJoint* railjoint) {
	if(this==railjoint->body1) {
		if(railjoint->link1.next==NULL) return NULL;
		else return railjoint->link1.next->railjoint;
	}
	if(this==railjoint->body2) {
		if(railjoint->link2.next==NULL) return NULL;
		else return railjoint->link2.next->railjoint;
	}
	return NULL;
}

// slider joint links

ep_SliderJoint* ep_Body::GetFirstSliderJoint() {
	if(first_sliderjointlink==NULL) return NULL;
	else return first_sliderjointlink->sliderjoint;
}

ep_SliderJoint* ep_Body::GetLastSliderJoint() {
	if(last_sliderjointlink==NULL) return NULL;
	else return last_sliderjointlink->sliderjoint;
}

ep_SliderJoint* ep_Body::GetPreviousSliderJoint(ep_SliderJoint* sliderjoint) {
	if(this==sliderjoint->body1) {
		if(sliderjoint->link1.prev==NULL) return NULL;
		else return sliderjoint->link1.prev->sliderjoint;
	}
	if(this==sliderjoint->body2) {
		if(sliderjoint->link2.prev==NULL) return NULL;
		else return sliderjoint->link2.prev->sliderjoint;
	}
	return NULL;
}

ep_SliderJoint* ep_Body::GetNextSliderJoint(ep_SliderJoint* sliderjoint) {
	if(this==sliderjoint->body1) {
		if(sliderjoint->link1.next==NULL) return NULL;
		else return sliderjoint->link1.next->sliderjoint;
	}
	if(this==sliderjoint->body2) {
		if(sliderjoint->link2.next==NULL) return NULL;
		else return sliderjoint->link2.next->sliderjoint;
	}
	return NULL;
}

//JOINTS//

// other

bool ep_Body::CalculateMass() {
	
#if EP_COMPILE_DEBUGCHECKS
	if(isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not calculate mass/inertia/center of body %lu in world %lu, the body is static.", id, world->id);
#endif
		return false;
	}
#endif
	
	Awake(false, true);
	
	// calculate mass and center of mass
	{
		mass = 0.0;
		double xx = 0.0, yy = 0.0;
		for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
			if(shape->shapetype==EP_SHAPETYPE_BOX) {
				double m = (shape->shapedata.box.w==0.0)? shape->density*shape->shapedata.box.h*2.0 : ((shape->shapedata.box.h==0.0)? shape->density*shape->shapedata.box.w*2.0 : shape->density*shape->shapedata.box.w*shape->shapedata.box.h*4.0);
				xx += m*shape->x;
				yy += m*shape->y;
				mass += m;
			}
			if(shape->shapetype==EP_SHAPETYPE_CIRCLE) {
				double m = (shape->shapedata.circle.r==0.0)? shape->density : shape->density*ep_sqr(shape->shapedata.circle.r)*M_PI;
				xx += m*shape->x;
				yy += m*shape->y;
				mass += m;
			}
			if(shape->shapetype==EP_SHAPETYPE_POLYGON) {
				double m = shape->density*shape->shapedata.polygon.polygon->mass;
				xx += m*(shape->x+ep_transform_x(shape->rot_sin, shape->rot_cos, shape->shapedata.polygon.polygon->xoff, shape->shapedata.polygon.polygon->yoff));
				yy += m*(shape->y+ep_transform_y(shape->rot_sin, shape->rot_cos, shape->shapedata.polygon.polygon->xoff, shape->shapedata.polygon.polygon->yoff));
				mass += m;
			}
		}
#if EP_COMPILE_DEBUGCHECKS
		if(mass<EP_EPSILON_MINBODYMASS) {
			mass = 1.0;
			invmass = 1.0/mass;
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not calculate mass/inertia/center of body %lu in world %lu, the mass of the body is zero.", id, world->id);
#endif
			return false;
		}
#endif
		xx /= mass;
		yy /= mass;
		double vx, vy;
		vx = ep_transform_x(rot_sin, rot_cos, xx-xoff, yy-yoff);
		vy = ep_transform_y(rot_sin, rot_cos, xx-xoff, yy-yoff);
		x += vx;
		y += vy;
		xvel += vy*rotvel;
		yvel -= vx*rotvel;
		xoff = xx;
		yoff = yy;
		invmass = 1.0/mass;
	}
	
	// calculate moment of inertia
	if(!norotation) {
		inertia = 0.0;
		for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
			if(shape->shapetype==EP_SHAPETYPE_BOX) {
				double m = (shape->shapedata.box.w==0.0)? shape->density*shape->shapedata.box.h*2.0 : ((shape->shapedata.box.h==0.0)? shape->density*shape->shapedata.box.w*2.0 : shape->density*shape->shapedata.box.w*shape->shapedata.box.h*4.0);
				double xx = shape->x-xoff;
				double yy = shape->y-yoff;
				inertia += m*((ep_sqr(shape->shapedata.box.w)+ep_sqr(shape->shapedata.box.h))/3.0+ep_sqr(xx)+ep_sqr(yy));
			}
			if(shape->shapetype==EP_SHAPETYPE_CIRCLE) {
				double m = (shape->shapedata.circle.r==0.0)? shape->density : shape->density*ep_sqr(shape->shapedata.circle.r)*M_PI;
				double xx = shape->x-xoff;
				double yy = shape->y-yoff;
				inertia += m*(ep_sqr(shape->shapedata.circle.r)*0.5+ep_sqr(xx)+ep_sqr(yy));
			}
			if(shape->shapetype==EP_SHAPETYPE_POLYGON) {
				double xx = shape->x+ep_transform_x(shape->rot_sin, shape->rot_cos, shape->shapedata.polygon.polygon->xoff, shape->shapedata.polygon.polygon->yoff)-xoff;
				double yy = shape->y+ep_transform_y(shape->rot_sin, shape->rot_cos, shape->shapedata.polygon.polygon->xoff, shape->shapedata.polygon.polygon->yoff)-yoff;
				inertia += shape->density*(shape->shapedata.polygon.polygon->inertia+shape->shapedata.polygon.polygon->mass*(ep_sqr(xx)+ep_sqr(yy)));
			}
		}
#if EP_COMPILE_DEBUGCHECKS
		if(inertia<EP_EPSILON_MINBODYINERTIA) {
			inertia = 1.0;
			invinertia = 1.0/inertia;
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not calculate mass/inertia/center of body %lu in world %lu, the moment of inertia of the body is zero.", id, world->id);
#endif
			return false;
		}
#endif
		invinertia = 1.0/inertia;
	}
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Calculated mass of body %lu in world %lu.", id, world->id);
#endif
	
	return true;
}

bool ep_Body::SetMass(double _mass) {
#if EP_COMPILE_DEBUGCHECKS
	if(isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not change mass of body %lu in world %lu, the body is static.", id, world->id);
#endif
		return false;
	}
	if(_mass<EP_EPSILON_MINBODYMASS) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not change mass of body %lu in world %lu, the new mass is zero.", id, world->id);
#endif
		return false;
	}
#endif
	if(mass!=_mass) {
		mass = _mass;
		invmass = 1.0/mass;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed mass of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::SetInertia(double _inertia) {
#if EP_COMPILE_DEBUGCHECKS
	if(isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not change inertia of body %lu in world %lu, the body is static.", id, world->id);
#endif
		return false;
	}
	if(_inertia<EP_EPSILON_MINBODYINERTIA) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not change inertia of body %lu in world %lu, the new moment of inertia is zero.", id, world->id);
#endif
		return false;
	}
#endif
	if(inertia!=_inertia) {
		inertia = _inertia;
		invinertia = 1.0/inertia;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed moment of inertia of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::SetCenter(double localx, double localy, bool updateinertia) {
	if(xoff!=localx || yoff!=localy) {
		if(updateinertia) {
#if EP_COMPILE_DEBUGCHECKS
			if(norotation) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Could not update inertia while setting center of body %lu in world %lu, the body can not rotate.", id, world->id);
#endif
				return false;
			}
#endif
			inertia += mass*(ep_sqr(localx-xoff)+ep_sqr(localy-yoff));
			invinertia = 1.0/inertia;
		}
		x += ep_transform_x(rot_sin, rot_cos, localx-xoff, localy-yoff);
		y += ep_transform_y(rot_sin, rot_cos, localx-xoff, localy-yoff);
		xoff = localx;
		yoff = localy;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed center of mass of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

void ep_Body::SetPosition(double _x, double _y, double _rot) {
	double newsin = sin(_rot),
	       newcos = cos(_rot),
	       newx = _x+ep_transform_x(newsin, newcos, xoff, yoff),
	       newy = _y+ep_transform_y(newsin, newcos, xoff, yoff);
	if(x!=newx || y!=newy || rot!=_rot) {
		x = newx;
		y = newy;
		rot = _rot;
		rot_sin = newsin;
		rot_cos = newcos;
		Awake(false, true);
		Moved();
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed position of body %lu in world %lu.", id, world->id);
#endif
}

void ep_Body::SetPositionCenter(double _x, double _y, double _rot) {
	if(x!=_x || y!=_y || rot!=_rot) {
		x = _x;
		y = _y;
		rot = _rot;
		rot_sin = sin(_rot);
		rot_cos = cos(_rot);
		Awake(false, true);
		Moved();
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed position of body %lu in world %lu.", id, world->id);
#endif
}

void ep_Body::SetPositionLocalPoint(double _x, double _y, double _rot, double localx, double localy) {
	double newsin = sin(_rot),
	       newcos = cos(_rot),
	       newx = _x+ep_transform_x(newsin, newcos, xoff-localx, yoff-localy),
	       newy = _y+ep_transform_y(newsin, newcos, xoff-localx, yoff-localy);
	if(x!=newx || y!=newy || rot!=_rot) {
		x = newx;
		y = newy;
		rot = _rot;
		rot_sin = newsin;
		rot_cos = newcos;
		Awake(false, true);
		Moved();
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed position of body %lu in world %lu.", id, world->id);
#endif
}

bool ep_Body::SetVelocityCenter(double _xvel, double _yvel, double _rotvel) {
	if(xvel!=_xvel || yvel!=_yvel || rotvel!=_rotvel) {
		xvel = _xvel;
		yvel = _yvel;
		rotvel = _rotvel;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed velocity of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::SetVelocityLocalPoint(double _xvel, double _yvel, double _rotvel, double localx, double localy) {
	double newxvel = _xvel+ep_transform_y(rot_sin, rot_cos, xoff-localx, yoff-localy)*_rotvel,
	       newyvel = _yvel-ep_transform_x(rot_sin, rot_cos, xoff-localx, yoff-localy)*_rotvel;
	if(xvel!=newxvel || yvel!=newyvel || rotvel!=_rotvel) {
		xvel = newxvel;
		yvel = newyvel;
		rotvel = _rotvel;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed velocity of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::SetMaxVelocity(double _maxvel, double _maxrotvel) {
#if EP_COMPILE_DEBUGCHECKS
	if(isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not change maximum velocity of body %lu in world %lu, the body is static.", id, world->id);
#endif
		return false;
	}
#endif
	if(maxvel!=_maxvel || maxrotvel!=_maxrotvel) {
		maxvel = _maxvel;
		maxrotvel = _maxrotvel;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed maximum velocity of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::SetGravity(double _gravity_x, double _gravity_y) {
	if(gravity_x!=_gravity_x || gravity_y!=_gravity_y) {
		gravity_x = _gravity_x;
		gravity_y = _gravity_y;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed gravity of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::SetDamping(double _damping, double _rotdamping) {
	if(damping!=_damping || rotdamping!=_rotdamping) {
		damping = ep_clamp(0.0, 1.0, _damping);
		rotdamping = ep_clamp(0.0, 1.0, _rotdamping);
		if(world->timestep!=0.0) {
			damping_factor = pow(1.0-damping, world->timestep);
			rotdamping_factor = pow(1.0-rotdamping, world->timestep);
		}
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed damping of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

void ep_Body::StoreImpulses(bool _storecontactimpulses, bool _storejointimpulses) {
	if(storecontactimpulses!=_storecontactimpulses || storejointimpulses!=_storejointimpulses) {
		storecontactimpulses = _storecontactimpulses;
		storejointimpulses = _storejointimpulses;
		Awake(false, true);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed stored impulses of body %lu in world %lu.", id, world->id);
#endif
}

bool ep_Body::SetSleeping(bool _sleepstable, bool _sleepoutofview) {
#if EP_COMPILE_DEBUGCHECKS
	if(isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not change damping of body %lu in world %lu, the body is static.", id, world->id);
#endif
		return false;
	}
#endif
	if(sleepstable!=_sleepstable || sleepoutofview!=_sleepoutofview) {
		sleepstable = _sleepstable;
		sleepoutofview = _sleepoutofview;
		// awake is not required, updatesleeping will awake the body 
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed sleeping settings of body %lu in world %lu.", id, world->id);
#endif
	return true;
}

unsigned long ep_Body::CollisionTestBox(double w, double h, double _x, double _y, double _rot, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	dummy.InitVirtualBox(w, h, _x, _y, _rot);
	dummy.UpdateAABB();
	world->ClearCollisionList();
	for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
		if(world->CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
			if(shape->updateaabb) {
				shape->UpdateTransformedCoordinates();
				shape->UpdateAABB();
			}
			if(world->CollisionAABB(shape, &dummy, contact_threshold)) {
				if(!world->AddToCollisionList(shape)) return 0;
			}
		}
	}
	return world->collisionshapecount;
}

unsigned long ep_Body::CollisionTestLine(double x1, double y1, double x2, double y2, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	dummy.InitVirtualLine(x1, y1, x2, y2);
	dummy.UpdateAABB();
	world->ClearCollisionList();
	for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
		if(world->CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
			if(shape->updateaabb) {
				shape->UpdateTransformedCoordinates();
				shape->UpdateAABB();
			}
			if(world->CollisionAABB(shape, &dummy, contact_threshold)) {
				if(!world->AddToCollisionList(shape)) return 0;
			}
		}
	}
	return world->collisionshapecount;
}

unsigned long ep_Body::CollisionTestCircle(double r, double _x, double _y, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	dummy.InitVirtualCircle(r, _x, _y);
	dummy.UpdateAABB();
	world->ClearCollisionList();
	for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
		if(world->CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
			if(shape->updateaabb) {
				shape->UpdateTransformedCoordinates();
				shape->UpdateAABB();
			}
			if(world->CollisionAABB(shape, &dummy, contact_threshold)) {
				if(!world->AddToCollisionList(shape)) return 0;
			}
		}
	}
	return world->collisionshapecount;
}

unsigned long ep_Body::CollisionTestPolygon(ep_Polygon* polygon, double _x, double _y, double _rot, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	if(!dummy.InitVirtualPolygon(polygon, _x, _y, _rot)) return false;
	dummy.UpdateAABB();
	world->ClearCollisionList();
	for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
		if(world->CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
			if(shape->updateaabb) {
				shape->UpdateTransformedCoordinates();
				shape->UpdateAABB();
			}
			if(world->CollisionAABB(shape, &dummy, contact_threshold)) {
				if(!world->AddToCollisionList(shape)) return 0;
			}
		}
	}
	return world->collisionshapecount;
}

double ep_Body::RayCast(double _x, double _y, double vx, double vy, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	double d = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	vx /= d;
	vy /= d;
	world->ClearCollisionList();
	ep_Shape *collisionshape = NULL;
	double distance = DBL_MAX;
	for(ep_Shape *shape = first_shape; shape!=NULL; shape = shape->next) {
		if(world->CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
			shape->UpdateTransformedCoordinates();
			double d = world->RayCast(shape, _x, _y, vx, vy);
			if(d!=-1.0 && d<distance) {
				collisionshape = shape;
				distance = d;
			}
		}
	}
	if(collisionshape==NULL) {
		return -1.0;
	}
	if(!world->AddToCollisionList(collisionshape)) return -1.0;
	return distance;
}

