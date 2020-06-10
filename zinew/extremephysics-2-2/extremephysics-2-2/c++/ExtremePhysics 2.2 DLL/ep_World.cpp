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
 * File: ep_World.cpp
 * Implementation of ep_World.
 */

#include "ExtremePhysics.h"

void ep_World::Init(ep_Main* _main) {
	
	// add to linked list
	main = _main;
	prev = main->last_world;
	next = NULL;
	if(prev==NULL) main->first_world = this;
	else prev->next = this;
	main->last_world = this;
	
	// initialize variables
	id = ++main->idcounter_worlds;
	
	idcounter_polygons = 0;
	polygoncount = 0;
	first_polygon = NULL;
	last_polygon = NULL;
	current_polygon = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_polygons.Init();
#endif
	
	idcounter_bodies = 0;
	bodycount = 0;
	first_body = NULL;
	last_body = NULL;
	current_body = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_bodies.Init();
#endif
	
	idcounter_contacts = 0;
	contactcount = 0;
	first_contact = NULL;
	last_contact = NULL;
	current_contact = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_contacts.Init();
#endif
	
	idcounter_hingejoints = 0;
	hingejointcount = 0;
	first_hingejoint = NULL;
	last_hingejoint = NULL;
	current_hingejoint = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_hingejoints.Init();
#endif
	
	idcounter_distancejoints = 0;
	distancejointcount = 0;
	first_distancejoint = NULL;
	last_distancejoint = NULL;
	current_distancejoint = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_distancejoints.Init();
#endif
	
	idcounter_railjoints = 0;
	railjointcount = 0;
	first_railjoint = NULL;
	last_railjoint = NULL;
	current_railjoint = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_railjoints.Init();
#endif
	
	idcounter_sliderjoints = 0;
	sliderjointcount = 0;
	first_sliderjoint = NULL;
	last_sliderjoint = NULL;
	current_sliderjoint = NULL;
#if EP_USE_IDHASHTABLE
	idhashtable_sliderjoints.Init();
#endif
	
	//JOINTS//
	
	idcounter_views = 0;
	viewcount = 0;
	first_view = NULL;
	last_view = NULL;
	current_view = NULL;
	
	idcounter_water = 0;
	watercount = 0;
	first_water = NULL;
	last_water = NULL;
	current_water = NULL;
	
	shapecount = 0;
	forcecount = 0;
	
	timestep = 1.0;
	velocity_iterations = 20;
	position_iterations = 10;
	contact_threshold = 0.1;
	velocity_threshold = 0.5;
	baumgarte_factor = 0.05;
	mass_bias = 0.5;
	position_factor = 1.0;
	horizontal = true;
	
	enable_sleeping = false;
	time_stable = 0.0;
	time_outofview = 0.0;
	stable_maxvel = 0.0;
	stable_maxrotvel = 0.0;
	
	collisionshapecount = 0;
	collisionshapes = NULL;
	
#if EP_COMPILE_MULTIPOLY
	multipoly_vertexcount = 0;
	multipoly_vertices = NULL;
	multipoly_first_polygon = NULL;
	multipoly_last_polygon = NULL;
#endif // EP_COMPILE_MULTIPOLY
	
#if EP_COMPILE_SERIALIZE
	serializedata = NULL;
	serializedata_length = 0;
#endif // EP_COMPILE_SERIALIZE
	
}

void ep_World::DeInit() {
	
	ClearCollisionList();
	
#if EP_COMPILE_MULTIPOLY
	ep_free(multipoly_vertices);
#endif // EP_COMPILE_MULTIPOLY
	
#if EP_COMPILE_SERIALIZE
	ep_free(serializedata);
#endif // EP_COMPILE_SERIALIZE
	
	// destroy everything (in reverse order)
	while(first_water!=NULL) DestroyWater(first_water);
	while(first_view!=NULL) DestroyView(first_view);
	//JOINTS//
	while(first_sliderjoint!=NULL) DestroySliderJoint(first_sliderjoint);
	while(first_railjoint!=NULL) DestroyRailJoint(first_railjoint);
	while(first_distancejoint!=NULL) DestroyDistanceJoint(first_distancejoint);
	while(first_hingejoint!=NULL) DestroyHingeJoint(first_hingejoint);
	while(first_contact!=NULL) DestroyContact(first_contact);
	while(first_body!=NULL) DestroyBody(first_body);
	while(first_polygon!=NULL) DestroyPolygon(first_polygon);
	
	// remove from linked list
	if(prev==NULL) main->first_world = next;
	else prev->next = next;
	if(next==NULL) main->last_world = prev;
	else next->prev = prev;
	if(main->current_world==this) main->current_world = NULL;
	
}

void ep_World::Clear() {
	
	ClearCollisionList();
	
#if EP_COMPILE_MULTIPOLY
	if(multipoly_vertices!=NULL) {
		ep_free(multipoly_vertices);
		multipoly_vertexcount = 0;
		multipoly_vertices = NULL;
		multipoly_first_polygon = NULL;
		multipoly_last_polygon = NULL;
	}
#endif // EP_COMPILE_MULTIPOLY
	
#if EP_COMPILE_SERIALIZE
	if(serializedata!=NULL) {
		ep_free(serializedata);
		serializedata = NULL;
		serializedata_length = 0;
	}
#endif // EP_COMPILE_SERIALIZE
	
	// destroy everything (in reverse order)
	while(first_water!=NULL) DestroyWater(first_water);
	while(first_view!=NULL) DestroyView(first_view);
	//JOINTS//
	while(first_sliderjoint!=NULL) DestroySliderJoint(first_sliderjoint);
	while(first_railjoint!=NULL) DestroyRailJoint(first_railjoint);
	while(first_distancejoint!=NULL) DestroyDistanceJoint(first_distancejoint);
	while(first_hingejoint!=NULL) DestroyHingeJoint(first_hingejoint);
	while(first_contact!=NULL) DestroyContact(first_contact);
	while(first_body!=NULL) DestroyBody(first_body);
	while(first_polygon!=NULL) DestroyPolygon(first_polygon);
	
	// reset id counters
	idcounter_polygons = 0;
	idcounter_bodies = 0;
	idcounter_hingejoints = 0;
	idcounter_distancejoints = 0;
	idcounter_railjoints = 0;
	idcounter_sliderjoints = 0;
	//JOINTS//
	idcounter_views = 0;
	
}

void ep_World::ClearCollisionList() {
	if(collisionshapes!=NULL) {
		ep_free(collisionshapes);
		collisionshapecount = 0;
		collisionshapes = NULL;
	}
}

bool ep_World::AddToCollisionList(ep_Shape* shape) {
	if((collisionshapecount&63)==0) {
		void *temp = ep_realloc(collisionshapes, (collisionshapecount+64)*sizeof(ep_Shape*));
		if(temp==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not add shape to collision list in world %lu, memory allocation failed.", id);
#endif
			return false;
		}
		collisionshapes = (ep_Shape**)(temp);
	}
	collisionshapes[collisionshapecount] = shape;
	++collisionshapecount;
	return true;
}

// contact

ep_Contact* ep_World::CreateContact(ep_Shape* shape1, ep_Shape* shape2) {
	
	ep_Contact *contact = AllocateContact();
	if(contact==NULL) return NULL;
	contact->Init(this, shape1, shape2, ++idcounter_contacts, true);
	
	return contact;
}

void ep_World::DestroyContact(ep_Contact* contact) {
	
	contact->DeInit();
	
	// store free contact?
	if(main->freecontactcount<EP_MAXFREECONTACTS) {
		// push one contact
		contact->next = main->freecontacts; // 'next' is the only variable used here 
		main->freecontacts = contact;
		++main->freecontactcount;
	} else {
		// free the memory
		ep_free(contact);
	}
	
}

ep_Contact* ep_World::FindContact(unsigned long _id) {
	if(current_contact!=NULL) {
		if(current_contact->id==_id) return current_contact;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_contacts.Find(_id);
	if(e==NULL) return NULL;
	current_contact = static_cast<ep_Contact*>(e);
	return current_contact;
#else
	for(current_contact = first_contact; current_contact!=NULL; current_contact = current_contact->next) {
		if(current_contact->id==_id) return current_contact;
	}
	return NULL;
#endif
}

// polygon

ep_Polygon* ep_World::CreatePolygon(unsigned long vertexcount) {
	
#if EP_COMPILE_DEBUGCHECKS
	// is the vertex count smaller than 3?
	if(vertexcount<3) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create polygon in world %lu, vertex count is smaller than 3.", id);
#endif
		return NULL;
	}
#endif
	
	ep_Polygon *polygon = AllocatePolygon(vertexcount);
	if(polygon==NULL) return NULL;
	polygon->Init(this, vertexcount, ++idcounter_polygons);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created polygon %lu in world %lu.", polygon->id, id);
#endif
	
	return polygon;
}

bool ep_World::DestroyPolygon(ep_Polygon* polygon) {
	
#if EP_COMPILE_MULTIPOLY
	ep_free(multipoly_vertices);
	multipoly_vertexcount = 0;
	multipoly_vertices = NULL;
	multipoly_first_polygon = NULL;
	multipoly_last_polygon = NULL;
#endif // EP_COMPILE_MULTIPOLY
	
#if EP_COMPILE_DEBUGCHECKS
	if(polygon->referencecount!=0) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not destroy polygon %lu in world %lu, polygon is still being used by %lu shapes.", polygon->id, id, polygon->referencecount);
#endif
		return false;
	}
#endif
	
	polygon->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = polygon->id;
#endif
	ep_free(polygon);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed polygon %lu in world %lu.", _id, id);
#endif
	
	return true;
}

ep_Polygon* ep_World::FindPolygon(unsigned long _id) {
	if(current_polygon!=NULL) {
		if(current_polygon->id==_id) return current_polygon;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_polygons.Find(_id);
	if(e==NULL) return NULL;
	current_polygon = static_cast<ep_Polygon*>(e);
	return current_polygon;
#else
	for(current_polygon = first_polygon; current_polygon!=NULL; current_polygon = current_polygon->next) {
		if(current_polygon->id==_id) return current_polygon;
	}
	return NULL;
#endif
}

// body

ep_Body* ep_World::CreateStaticBody() {
	
	ep_Body *body = AllocateBody();
	if(body==NULL) return NULL;
	body->Init(this, true, true, ++idcounter_bodies);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created body %lu in world %lu.", body->id, id);
#endif
	
	return body;
}

ep_Body* ep_World::CreateDynamicBody(bool norotation) {
	
	ep_Body *body = AllocateBody();
	if(body==NULL) return NULL;
	body->Init(this, false, norotation, ++idcounter_bodies);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created body %lu in world %lu.", body->id, id);
#endif
	
	return body;
}

void ep_World::DestroyBody(ep_Body* body) {
	
	body->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = body->id;
#endif
	ep_free(body);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed body %lu in world %lu.", _id, id);
#endif
	
}

ep_Body* ep_World::FindBody(unsigned long _id) {
	if(current_body!=NULL) {
		if(current_body->id==_id) return current_body;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_bodies.Find(_id);
	if(e==NULL) return NULL;
	current_body = static_cast<ep_Body*>(e);
	return current_body;
#else
	for(current_body = first_body; current_body!=NULL; current_body = current_body->next) {
		if(current_body->id==_id) return current_body;
	}
	return NULL;
#endif
}

// hinge joint

ep_HingeJoint* ep_World::CreateHingeJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2, double y2, double referencerotation) {
	
#if EP_COMPILE_DEBUGCHECKS
	// do the bodies belong to this world?
	if(body1->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create hinge joint in world %lu, the first body does not belong to this world.", id);
#endif
		return NULL;
	}
	if(body2->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create hinge joint in world %lu, the second body does not belong to this world.", id);
#endif
		return NULL;
	}
	// are the bodies the same?
	if(body1==body2) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create hinge joint in world %lu, body %lu can't be connected to itself.", id, body1->id);
#endif
		return NULL;
	}
	// are both bodies static?
	if(body1->isstatic && body2->isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create hinge joint in world %lu, body %lu and body %lu are both static.", id, body1->id, body2->id);
#endif
		return NULL;
	}
#endif
	
	ep_HingeJoint *hingejoint = AllocateHingeJoint();
	if(hingejoint==NULL) return NULL;
	hingejoint->Init(this, body1, body2, x1, y1, x2, y2, referencerotation, ++idcounter_hingejoints, true);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created hinge joint %lu in world %lu.", hingejoint->id, id);
#endif
	
	return hingejoint;
}

void ep_World::DestroyHingeJoint(ep_HingeJoint* hingejoint) {
	
	hingejoint->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = hingejoint->id;
#endif
	ep_free(hingejoint);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed hinge joint %lu in world %lu.", _id, id);
#endif
	
}

ep_HingeJoint* ep_World::FindHingeJoint(unsigned long _id) {
	if(current_hingejoint!=NULL) {
		if(current_hingejoint->id==_id) return current_hingejoint;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_hingejoints.Find(_id);
	if(e==NULL) return NULL;
	current_hingejoint = static_cast<ep_HingeJoint*>(e);
	return current_hingejoint;
#else
	for(current_hingejoint = first_hingejoint; current_hingejoint!=NULL; current_hingejoint = current_hingejoint->next) {
		if(current_hingejoint->id==_id) return current_hingejoint;
	}
	return NULL;
#endif
}

// distance joint

ep_DistanceJoint* ep_World::CreateDistanceJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2, double y2) {
	
#if EP_COMPILE_DEBUGCHECKS
	// first some additional error checking
	// do the bodies belong to this world?
	if(body1->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create distance joint in world %lu, the first body does not belong to this world.", id);
#endif
		return NULL;
	}
	if(body2->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create distance joint in world %lu, the second body does not belong to this world.", id);
#endif
		return NULL;
	}
	// are the bodies the same?
	if(body1==body2) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create distance joint in world %lu, body %lu can't be connected to itself.", id, body1->id);
#endif
		return NULL;
	}
	// are both bodies static?
	if(body1->isstatic && body2->isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create distance joint in world %lu, body %lu and body %lu are both static.", id, body1->id, body2->id);
#endif
		return NULL;
	}
#endif
	
	ep_DistanceJoint *distancejoint = AllocateDistanceJoint();
	if(distancejoint==NULL) return NULL;
	distancejoint->Init(this, body1, body2, x1, y1, x2, y2, ++idcounter_distancejoints, true);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created distance joint %lu in world %lu.", distancejoint->id, id);
#endif
	
	return distancejoint;
}

void ep_World::DestroyDistanceJoint(ep_DistanceJoint* distancejoint) {
	
	distancejoint->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = distancejoint->id;
#endif
	ep_free(distancejoint);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed distance joint %lu in world %lu.", _id, id);
#endif
	
}

ep_DistanceJoint* ep_World::FindDistanceJoint(unsigned long _id) {
	if(current_distancejoint!=NULL) {
		if(current_distancejoint->id==_id) return current_distancejoint;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_distancejoints.Find(_id);
	if(e==NULL) return NULL;
	current_distancejoint = static_cast<ep_DistanceJoint*>(e);
	return current_distancejoint;
#else
	for(current_distancejoint = first_distancejoint; current_distancejoint!=NULL; current_distancejoint = current_distancejoint->next) {
		if(current_distancejoint->id==_id) return current_distancejoint;
	}
	return NULL;
#endif
}

// rail joint

ep_RailJoint* ep_World::CreateRailJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2a, double y2a, double x2b, double y2b) {
	
#if EP_COMPILE_DEBUGCHECKS
	// first some additional error checking
	// do the bodies belong to this world?
	if(body1->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create rail joint in world %lu, the first body does not belong to this world.", id);
#endif
		return NULL;
	}
	if(body2->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create rail joint in world %lu, the second body does not belong to this world.", id);
#endif
		return NULL;
	}
	// are the bodies the same?
	if(body1==body2) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create rail joint in world %lu, body %lu can't be connected to itself.", id, body1->id);
#endif
		return NULL;
	}
	// are both bodies static?
	if(body1->isstatic && body2->isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create rail joint in world %lu, body %lu and body %lu are both static.", id, body1->id, body2->id);
#endif
		return NULL;
	}
#endif
	
	double vx, vy, length;
	vx = x2b-x2a;
	vy = y2b-y2a;
	length = sqrt(ep_sqr(vx)+ep_sqr(vy));
#if EP_COMPILE_DEBUGCHECKS
	// is the length zero?
	if(length<EP_EPSILON_MINRAILLENGTH) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create rail joint in world %lu, the length of the rail is zero.", id);
#endif
		return NULL;
	}
#endif
	vx /= length;
	vy /= length;
	
	ep_RailJoint *railjoint = AllocateRailJoint();
	if(railjoint==NULL) return NULL;
	railjoint->Init(this, body1, body2, x1, y1, x2a, y2a, vx, vy, length, ++idcounter_railjoints, true);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created rail joint %lu in world %lu.", railjoint->id, id);
#endif
	
	return railjoint;
}

void ep_World::DestroyRailJoint(ep_RailJoint* railjoint) {
	
	railjoint->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = railjoint->id;
#endif
	ep_free(railjoint);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed rail joint %lu in world %lu.", _id, id);
#endif
	
}

ep_RailJoint* ep_World::FindRailJoint(unsigned long _id) {
	if(current_railjoint!=NULL) {
		if(current_railjoint->id==_id) return current_railjoint;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_railjoints.Find(_id);
	if(e==NULL) return NULL;
	current_railjoint = static_cast<ep_RailJoint*>(e);
	return current_railjoint;
#else
	for(current_railjoint = first_railjoint; current_railjoint!=NULL; current_railjoint = current_railjoint->next) {
		if(current_railjoint->id==_id) return current_railjoint;
	}
	return NULL;
#endif
}

// slider joint

ep_SliderJoint* ep_World::CreateSliderJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2a, double y2a, double x2b, double y2b, double rotation) {
	
#if EP_COMPILE_DEBUGCHECKS
	// first some additional error checking
	// do the bodies belong to this world?
	if(body1->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, the first body does not belong to this world.", id);
#endif
		return NULL;
	}
	if(body2->world!=this) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, the second body does not belong to this world.", id);
#endif
		return NULL;
	}
	// are the bodies the same?
	if(body1==body2) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, body %lu can't be connected to itself.", id, body1->id);
#endif
		return NULL;
	}
	// are both bodies static?
	if(body1->isstatic && body2->isstatic) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, body %lu and body %lu are both static.", id, body1->id, body2->id);
#endif
		return NULL;
	}
	// are both bodies norotation?
	if(body1->norotation && body2->norotation) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, body %lu and body %lu both can't rotate.", id, body1->id, body2->id);
#endif
		return false;
	}
#endif
	
	double vx, vy, length;
	vx = x2b-x2a;
	vy = y2b-y2a;
	length = sqrt(ep_sqr(vx)+ep_sqr(vy));
#if EP_COMPILE_DEBUGCHECKS
	// is the length zero?
	if(length<EP_EPSILON_MINRAILLENGTH) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, the length of the rail is zero.", id);
#endif
		return NULL;
	}
#endif
	vx /= length;
	vy /= length;
	
	ep_SliderJoint *sliderjoint = AllocateSliderJoint();
	if(sliderjoint==NULL) return NULL;
	sliderjoint->Init(this, body1, body2, x1, y1, x2a, y2a, vx, vy, length, rotation, ++idcounter_sliderjoints, true);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created slider joint %lu in world %lu.", sliderjoint->id, id);
#endif
	
	return sliderjoint;
}

void ep_World::DestroySliderJoint(ep_SliderJoint* sliderjoint) {
	
	sliderjoint->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = sliderjoint->id;
#endif
	ep_free(sliderjoint);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed slider joint %lu in world %lu.", _id, id);
#endif
	
}

ep_SliderJoint* ep_World::FindSliderJoint(unsigned long _id) {
	if(current_sliderjoint!=NULL) {
		if(current_sliderjoint->id==_id) return current_sliderjoint;
	}
#if EP_USE_IDHASHTABLE
	ep_IdHashTableEntry *e = idhashtable_sliderjoints.Find(_id);
	if(e==NULL) return NULL;
	current_sliderjoint = static_cast<ep_SliderJoint*>(e);
	return current_sliderjoint;
#else
	for(current_sliderjoint = first_sliderjoint; current_sliderjoint!=NULL; current_sliderjoint = current_sliderjoint->next) {
		if(current_sliderjoint->id==_id) return current_sliderjoint;
	}
	return NULL;
#endif
}

//JOINTS//

// view

ep_View* ep_World::CreateView() {
	
	ep_View *view = AllocateView();
	if(view==NULL) return NULL;
	view->Init(this, ++idcounter_views);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created view %lu in world %lu.", view->id, id);
#endif
	
	return view;
}

void ep_World::DestroyView(ep_View* view) {
	
	view->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = view->id;
#endif
	ep_free(view);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed view %lu in world %lu.", _id, id);
#endif
	
}

ep_View* ep_World::FindView(unsigned long _id) {
	if(current_view!=NULL) {
		if(current_view->id==_id) return current_view;
	}
	for(current_view = first_view; current_view!=NULL; current_view = current_view->next) {
		if(current_view->id==_id) return current_view;
	}
	return NULL;
}

// water

ep_Water* ep_World::CreateWater() {
	
	ep_Water *water = AllocateWater();
	if(water==NULL) return NULL;
	water->Init(this, ++idcounter_water, true);
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created water %lu in world %lu.", water->id, id);
#endif
	
	return water;
}

void ep_World::DestroyWater(ep_Water* water) {
	
	water->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = water->id;
#endif
	ep_free(water);
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed water %lu in world %lu.", _id, id);
#endif
	
}

ep_Water* ep_World::FindWater(unsigned long _id) {
	if(current_water!=NULL) {
		if(current_water->id==_id) return current_water;
	}
	for(current_water = first_water; current_water!=NULL; current_water = current_water->next) {
		if(current_water->id==_id) return current_water;
	}
	return NULL;
}

// other

void ep_World::Awake() {
	// This function has the same effect as calling
	// Awake(false, true) for all bodies, but it's faster.
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		body->stabletimer = 0.0;
	}
}

void ep_World::SetSettings(double _timestep, unsigned long _velocity_iterations, unsigned long _position_iterations, double _contact_threshold, double _velocity_threshold, double _baumgarte_factor, double _mass_bias, double _position_factor) {
	if(_timestep<EP_EPSILON_MINTIMESTEP) _timestep = 0.0;
	if(timestep!=_timestep || velocity_iterations!=_velocity_iterations || position_iterations!=_position_iterations || contact_threshold!=_contact_threshold || velocity_threshold!=_velocity_threshold || baumgarte_factor!=_baumgarte_factor || mass_bias!=_mass_bias || position_factor!=_position_factor) {
		timestep = _timestep;
		velocity_iterations = _velocity_iterations;
		position_iterations = _position_iterations;
		contact_threshold = _contact_threshold;
		velocity_threshold = _velocity_threshold;
		baumgarte_factor = _baumgarte_factor;
		mass_bias = _mass_bias;
		position_factor = _position_factor;
		if(timestep!=0.0) {
			for(ep_Body *body = first_body; body!=NULL; body = body->next) {
				body->damping_factor = pow(1.0-body->damping, timestep);
				body->rotdamping_factor = pow(1.0-body->rotdamping, timestep);
			}
		}
		Awake();
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed settings of world %lu.", id);
#endif
}

void ep_World::SetPrimaryAxis(bool _horizontal) {
	// Awake is not required here because this setting
	// doesn't make any real difference (except performance).
	horizontal = _horizontal;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed settings of world %lu.", id);
#endif
}

void ep_World::SetSleeping(bool _enable_sleeping, double _time_stable, double _time_outofview, double _stable_maxvel, double _stable_maxrotvel) {
	if(enable_sleeping!= _enable_sleeping || time_stable!=_time_stable || time_outofview!=_time_outofview || stable_maxvel!=_stable_maxvel || stable_maxrotvel!=_stable_maxrotvel) {
		enable_sleeping = _enable_sleeping;
		time_stable = _time_stable;
		time_outofview = _time_outofview;
		stable_maxvel = _stable_maxvel;
		stable_maxrotvel = _stable_maxrotvel;
		for(ep_Body *body = first_body; body!=NULL; body = body->next) {
			body->issleeping = false;
			body->stabletimer = 0.0;
			body->outofviewtimer = 0.0;
		}
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed sleeping settings of world %lu.", id);
#endif
}

bool ep_World::UpdateContacts() {
	
	if(!_UpdateContacts()) {
		return false;
	}
	if(enable_sleeping) {
		_UpdateSleeping();
	}
	
	return true;
}

bool ep_World::SimulateStep() {
	
	if(timestep!=0.0) {
		IntegrateVelocity();
		SolveVelocityConstraints();
		IntegratePosition();
		if(position_iterations!=0) SolvePositionConstraints();
	}
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Simulated a step in world %lu.", id);
#endif
	return true;
}

unsigned long ep_World::CollisionTestBox(double w, double h, double _x, double _y, double _rot, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	dummy.InitVirtualBox(w, h, _x, _y, _rot);
	dummy.UpdateAABB();
	ClearCollisionList();
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			if(CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
				if(shape->updateaabb) {
					shape->UpdateTransformedCoordinates();
					shape->UpdateAABB();
				}
				if(CollisionAABB(shape, &dummy, contact_threshold)) {
					if(!AddToCollisionList(shape)) return 0;
				}
			}
		}
	}
	return collisionshapecount;
}

unsigned long ep_World::CollisionTestLine(double x1, double y1, double x2, double y2, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	dummy.InitVirtualLine(x1, y1, x2, y2);
	dummy.UpdateAABB();
	ClearCollisionList();
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			if(CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
				if(shape->updateaabb) {
					shape->UpdateTransformedCoordinates();
					shape->UpdateAABB();
				}
				if(CollisionAABB(shape, &dummy, contact_threshold)) {
					if(!AddToCollisionList(shape)) return 0;
				}
			}
		}
	}
	return collisionshapecount;
}

unsigned long ep_World::CollisionTestCircle(double r, double _x, double _y, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	dummy.InitVirtualCircle(r, _x, _y);
	dummy.UpdateAABB();
	ClearCollisionList();
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			if(CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
				if(shape->updateaabb) {
					shape->UpdateTransformedCoordinates();
					shape->UpdateAABB();
				}
				if(CollisionAABB(shape, &dummy, contact_threshold)) {
					if(!AddToCollisionList(shape)) return 0;
				}
			}
		}
	}
	return collisionshapecount;
}

unsigned long ep_World::CollisionTestPolygon(ep_Polygon* polygon, double _x, double _y, double _rot, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	ep_Shape dummy;
	if(!dummy.InitVirtualPolygon(polygon, _x, _y, _rot)) return 0;
	dummy.UpdateAABB();
	ClearCollisionList();
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			if(CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
				if(shape->updateaabb) {
					shape->UpdateTransformedCoordinates();
					shape->UpdateAABB();
				}
				if(CollisionAABB(shape, &dummy, contact_threshold)) {
					if(!AddToCollisionList(shape)) return 0;
				}
			}
		}
	}
	return collisionshapecount;
}

double ep_World::RayCast(double _x, double _y, double vx, double vy, unsigned long collidemask1, unsigned long collidemask2, unsigned long group) {
	double d = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	vx /= d;
	vy /= d;
	ClearCollisionList();
	ep_Shape *collisionshape = NULL;
	double distance = DBL_MAX;
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			if(CheckCollisionMasks(shape->collidemask1, shape->collidemask2, shape->group, collidemask1, collidemask2, group)) {
				shape->UpdateTransformedCoordinates();
				double d = RayCast(shape, _x, _y, vx, vy);
				if(d!=-1.0 && d<distance) {
					collisionshape = shape;
					distance = d;
				}
			}
		}
	}
	if(collisionshape==NULL) {
		return -1.0;
	}
	if(!AddToCollisionList(collisionshape)) return -1.0;
	return distance;
}

ep_Shape* ep_World::GetCollisionShape(unsigned long index) {
	
#if EP_COMPILE_DEBUGCHECKS
	if(index>=collisionshapecount) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not get collision shape %lu in world %lu, there are only %lu collision shapes.", index, id, collisionshapecount);
#endif
		return NULL;
	}
#endif
	
	return collisionshapes[index];
	
}

