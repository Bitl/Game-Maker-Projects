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
 * File: ep_World_Allocate.cpp
 * Allocates memory for new objects.
 */

#include "ExtremePhysics.h"

ep_Contact* ep_World::AllocateContact() {
	ep_Contact *contact;
	// free contacts left?
	if(main->freecontacts!=NULL) {
		// pop one contact
		contact = main->freecontacts;
		main->freecontacts = contact->next;
		--main->freecontactcount;
	} else {
		// allocate memory
		contact = (ep_Contact*)(ep_malloc(sizeof(ep_Contact)));
		if(contact==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not create contact in world %lu, memory allocation failed.", id);
#endif
			return NULL;
		}
	}
	return contact;
}

ep_Polygon* ep_World::AllocatePolygon(unsigned long vertexcount) {
	ep_Polygon *polygon = (ep_Polygon*)(ep_malloc(sizeof(ep_Polygon)+vertexcount*sizeof(ep_PolygonVertex)+polygon_userdatasize));
	if(polygon==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create polygon in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return polygon;
}

ep_Body* ep_World::AllocateBody() {
	ep_Body *body = (ep_Body*)(ep_malloc(sizeof(ep_Body)+body_userdatasize));
	if(body==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create body in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return body;
}

ep_HingeJoint* ep_World::AllocateHingeJoint() {
	ep_HingeJoint *hingejoint = (ep_HingeJoint*)(ep_malloc(sizeof(ep_HingeJoint)+hingejoint_userdatasize));
	if(hingejoint==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create hinge joint in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return hingejoint;
}

ep_DistanceJoint* ep_World::AllocateDistanceJoint() {
	ep_DistanceJoint *distancejoint = (ep_DistanceJoint*)(ep_malloc(sizeof(ep_DistanceJoint)+distancejoint_userdatasize));
	if(distancejoint==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create distance joint in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return distancejoint;
}

ep_RailJoint* ep_World::AllocateRailJoint() {
	ep_RailJoint *railjoint = (ep_RailJoint*)(ep_malloc(sizeof(ep_RailJoint)+railjoint_userdatasize));
	if(railjoint==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create rail joint in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return railjoint;
}

ep_SliderJoint* ep_World::AllocateSliderJoint() {
	ep_SliderJoint *sliderjoint = (ep_SliderJoint*)(ep_malloc(sizeof(ep_SliderJoint)+sliderjoint_userdatasize));
	if(sliderjoint==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create slider joint in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return sliderjoint;
}

//JOINTS//

ep_View* ep_World::AllocateView() {
	ep_View *view = (ep_View*)(ep_malloc(sizeof(ep_View)+view_userdatasize));
	if(view==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create view in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return view;
}

ep_Water* ep_World::AllocateWater() {
	ep_Water *water = (ep_Water*)(ep_malloc(sizeof(ep_Water)+water_userdatasize));
	if(water==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create water in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return water;
}

ep_Shape* ep_World::AllocateShape() {
	ep_Shape *shape = (ep_Shape*)(ep_malloc(sizeof(ep_Shape)+shape_userdatasize));
	if(shape==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create shape in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return shape;
}

ep_Force* ep_World::AllocateForce() {
	ep_Force *force = (ep_Force*)(ep_malloc(sizeof(ep_Force)+force_userdatasize));
	if(force==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not create force in world %lu, memory allocation failed.", id);
#endif
		return NULL;
	}
	return force;
}


