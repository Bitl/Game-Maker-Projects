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
 * File: ep_Main.cpp
 * Implementation of ep_Main.
 */

#include "ExtremePhysics.h"

const char* ep_Version() {
	return "2.2.13";
}

ep_Main::ep_Main(ep_Callback* _callback) {
	
	callback = _callback;
	
	idcounter_worlds = 0;
	first_world = NULL;
	last_world = NULL;
	current_world = NULL;
	
	maxmessagelevel = 1;
	
	freecontactcount = 0;
	freecontacts = NULL;
	
}

ep_Main::~ep_Main() {
	
	// destroy everything
	while(first_world!=NULL) DestroyWorld(first_world);
	
	// destroy free contacts
	while(freecontacts!=NULL) {
		ep_Contact *t = freecontacts->next;
		ep_free(freecontacts);
		freecontacts = t;
	}
	
}

void ep_Main::Clear() {
	
	// destroy everything
	while(first_world!=NULL) DestroyWorld(first_world);
	
	// reset id counters
	idcounter_worlds = 0;
	
}

// world

ep_World* ep_Main::CreateWorld(unsigned long world_userdatasize, unsigned long polygon_userdatasize, unsigned long body_userdatasize,
	unsigned long hingejoint_userdatasize, unsigned long distancejoint_userdatasize, unsigned long railjoint_userdatasize,
	unsigned long sliderjoint_userdatasize,
	unsigned long view_userdatasize, unsigned long water_userdatasize, unsigned long shape_userdatasize, unsigned long force_userdatasize) { //JOINTS//
	ep_World *world;
	
	// allocate memory
	world = (ep_World*)(ep_malloc(sizeof(ep_World)+world_userdatasize));
	if(world==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		Message(EP_MESSAGELEVEL_ERROR, "Could not create world, memory allocation failed.");
#endif
		return NULL;
	}
	
	world->polygon_userdatasize = polygon_userdatasize;
	world->body_userdatasize = body_userdatasize;
	world->hingejoint_userdatasize = hingejoint_userdatasize;
	world->distancejoint_userdatasize = distancejoint_userdatasize;
	world->railjoint_userdatasize = railjoint_userdatasize;
	world->sliderjoint_userdatasize = sliderjoint_userdatasize;
	//JOINTS//
	world->view_userdatasize = view_userdatasize;
	world->water_userdatasize = water_userdatasize;
	world->shape_userdatasize = shape_userdatasize;
	world->force_userdatasize = force_userdatasize;
	world->Init(this);
	
#if EP_COMPILE_DEBUGMESSAGES
	Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Created world %lu.", world->id);
#endif
	
	return world;
}

bool ep_Main::DestroyWorld(ep_World* world) {
	
	world->DeInit();
#if EP_COMPILE_DEBUGMESSAGES
	unsigned long _id = world->id;
#endif
	ep_free(world);
#if EP_COMPILE_DEBUGMESSAGES
	Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Destroyed world %lu.", _id);
#endif
	
	return true;
}

ep_World* ep_Main::FindWorld(unsigned long _id) {
	if(current_world!=NULL) {
		if(current_world->id==_id) return current_world;
	}
	for(current_world = first_world; current_world!=NULL; current_world = current_world->next) {
		if(current_world->id==_id) return current_world;
	}
	return NULL;
}

// other

void ep_Main::Message(int level, const char* format, ...) {
	if(level<=maxmessagelevel) {
		va_list args;
		va_start(args, format);
		char messagebuffer[EP_MESSAGE_BUFFERSIZE+1];
#ifdef _MSC_VER
		// VC++ complains about deprecation
		_vsnprintf_s(messagebuffer, EP_MESSAGE_BUFFERSIZE+1, EP_MESSAGE_BUFFERSIZE, format, args);
#else
		vsnprintf(messagebuffer, EP_MESSAGE_BUFFERSIZE, format, args);
#endif
		messagebuffer[EP_MESSAGE_BUFFERSIZE] = '\0';
		va_end(args);
		callback->DebugMessage(level, messagebuffer);
	}
}

void ep_Main::PrintObjectTree() {
	
	/*
	Object tree:
	+- Main
	   +- World 1
	   |  +- Body 1
	   |  |  +- Shape 1
	   |  |  +- Shape 2
	   |  +- Body 2
	   |  |  +- Shape 1
	   |  +- Body 3
	   |     +- Shape 1
	   |     +- Shape 2
	   |     +- Shape 3
	   +- World 2
	End of object tree.
	*/
	
	Message(EP_MESSAGELEVEL_EXPLICIT, "Object tree:");
	Message(EP_MESSAGELEVEL_EXPLICIT, "+- Main");
	for(ep_World *world = first_world; world!=NULL; world = world->next) {
		Message(EP_MESSAGELEVEL_EXPLICIT, "   +- World %lu",
		        world->id);
		for(ep_Polygon *polygon = world->first_polygon; polygon!=NULL; polygon = polygon->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Polygon %lu (%lu vertices, %s)",
			        (world->next==NULL)? ' ' : '|',
			        polygon->id, polygon->vertexcount,
			        (polygon->initialized)? "initialized" : "not initialized");
		}
		for(ep_Body *body = world->first_body; body!=NULL; body = body->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Body %lu (%s)",
			        (world->next==NULL)? ' ' : '|',
			        body->id,
			        (body->isstatic)? "static" : "dynamic");
			for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
				switch(shape->shapetype) {
					case EP_SHAPETYPE_BOX: {
						Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  %c  +- Shape %lu (box)",
						        (world->next==NULL)? ' ' : '|',
						        (body->next==NULL)? ' ' : '|',
						        shape->id);
						break;
					}
					case EP_SHAPETYPE_CIRCLE: {
						Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  %c  +- Shape %lu (circle)",
						        (world->next==NULL)? ' ' : '|',
						        (body->next==NULL)? ' ' : '|',
						        shape->id);
						break;
					}
					case EP_SHAPETYPE_POLYGON: {
						Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  %c  +- Shape %lu (polygon %lu)",
						        (world->next==NULL)? ' ' : '|',
						        (body->next==NULL)? ' ' : '|',
						        shape->id, shape->shapedata.polygon.polygon->id);
						break;
					}
				}
			}
			for(ep_Force *force = body->first_force; force!=NULL; force = force->next) {
				Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  %c  +- Force %lu (%s)",
				        (world->next==NULL)? ' ' : '|',
				        (body->next==NULL)? ' ' : '|',
				        force->id,
				        (force->local)? ((force->ignoremass)? "local + ignoring mass" : "local")
				                      : ((force->ignoremass)? "global + ignoring mass" : "global"));
			}
		}
		for(ep_Contact *contact = world->first_contact; contact!=NULL; contact = contact->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Contact %lu (connected to shape %lu of body %lu and shape %lu of body %lu)",
			        (world->next==NULL)? ' ' : '|',
			        contact->id, contact->shape1->id, contact->body1->id, contact->shape2->id, contact->body2->id);
		}
		for(ep_HingeJoint *hingejoint = world->first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Hinge Joint %lu (connected to body %lu and body %lu)",
			        (world->next==NULL)? ' ' : '|',
			        hingejoint->id, hingejoint->body1->id, hingejoint->body2->id);
		}
		for(ep_DistanceJoint *distancejoint = world->first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Distance Joint %lu (connected to body %lu and body %lu)",
			        (world->next==NULL)? ' ' : '|',
			        distancejoint->id, distancejoint->body1->id, distancejoint->body2->id);
		}
		for(ep_RailJoint *railjoint = world->first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Rail Joint %lu (connected to body %lu and body %lu)",
			        (world->next==NULL)? ' ' : '|',
			        railjoint->id, railjoint->body1->id, railjoint->body2->id);
		}
		for(ep_SliderJoint *sliderjoint = world->first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- Slider Joint %lu (connected to body %lu and body %lu)",
			        (world->next==NULL)? ' ' : '|',
			        sliderjoint->id, sliderjoint->body1->id, sliderjoint->body2->id);
		}
		//JOINTS//
		for(ep_View *view = world->first_view; view!=NULL; view = view->next) {
			Message(EP_MESSAGELEVEL_EXPLICIT, "   %c  +- View %lu",
			        (world->next==NULL)? ' ' : '|',
			        view->id);
		}
	}
	Message(EP_MESSAGELEVEL_EXPLICIT, "End of object tree.");
	
}

bool ep_Main::CollisionTestBoxBox(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_w, double shape2_h, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualBox(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot);
	dummy2.InitVirtualBox(shape2_w, shape2_h, shape2_x, shape2_y, shape2_rot);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestBoxLine(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_x1, double shape2_y1, double shape2_x2, double shape2_y2, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualBox(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot);
	dummy2.InitVirtualLine(shape2_x1, shape2_y1, shape2_x2, shape2_y2);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestBoxCircle(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_r, double shape2_x, double shape2_y, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualBox(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot);
	dummy2.InitVirtualCircle(shape2_r, shape2_x, shape2_y);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestBoxPolygon(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualBox(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot);
	dummy2.InitVirtualPolygon(shape2_polygon, shape2_x, shape2_y, shape2_rot);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestLineLine(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
	double shape2_x1, double shape2_y1, double shape2_x2, double shape2_y2, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualLine(shape1_x1, shape1_y1, shape1_x2, shape1_y2);
	dummy2.InitVirtualLine(shape2_x1, shape2_y1, shape2_x2, shape2_y2);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestLineCircle(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
	double shape2_r, double shape2_x, double shape2_y, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualLine(shape1_x1, shape1_y1, shape1_x2, shape1_y2);
	dummy2.InitVirtualCircle(shape2_r, shape2_x, shape2_y);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestLinePolygon(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
	ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualLine(shape1_x1, shape1_y1, shape1_x2, shape1_y2);
	dummy2.InitVirtualPolygon(shape2_polygon, shape2_x, shape2_y, shape2_rot);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestCircleCircle(double shape1_r, double shape1_x, double shape1_y,
	double shape2_r, double shape2_x, double shape2_y, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualCircle(shape1_r, shape1_x, shape1_y);
	dummy2.InitVirtualCircle(shape2_r, shape2_x, shape2_y);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestCirclePolygon(double shape1_r, double shape1_x, double shape1_y,
	ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualCircle(shape1_r, shape1_x, shape1_y);
	dummy2.InitVirtualPolygon(shape2_polygon, shape2_x, shape2_y, shape2_rot);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}
bool ep_Main::CollisionTestPolygonPolygon(ep_Polygon* shape1_polygon, double shape1_x, double shape1_y, double shape1_rot,
	ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_Shape dummy1, dummy2;
	dummy1.InitVirtualPolygon(shape1_polygon, shape1_x, shape1_y, shape1_rot);
	dummy2.InitVirtualPolygon(shape2_polygon, shape2_x, shape2_y, shape2_rot);
	return ep_World::Collision(NULL, &dummy1, &dummy2, contact_threshold);
}

double ep_Main::RayCastBox(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_w, double shape_h, double shape_x, double shape_y, double shape_rot) {
	double d = sqrt(ep_sqr(ray_vx)+ep_sqr(ray_vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	ray_vx /= d;
	ray_vy /= d;
	ep_Shape dummy;
	dummy.InitVirtualBox(shape_w, shape_h, shape_x, shape_y, shape_rot);
	return ep_World::RayCast(&dummy, ray_x, ray_y, ray_vx, ray_vy);
}
double ep_Main::RayCastLine(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_x1, double shape_y1, double shape_x2, double shape_y2) {
	double d = sqrt(ep_sqr(ray_vx)+ep_sqr(ray_vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	ray_vx /= d;
	ray_vy /= d;
	ep_Shape dummy;
	dummy.InitVirtualLine(shape_x1, shape_y1, shape_x2, shape_y2);
	return ep_World::RayCast(&dummy, ray_x, ray_y, ray_vx, ray_vy);
}
double ep_Main::RayCastCircle(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_r, double shape_x, double shape_y) {
	double d = sqrt(ep_sqr(ray_vx)+ep_sqr(ray_vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	ray_vx /= d;
	ray_vy /= d;
	ep_Shape dummy;
	dummy.InitVirtualCircle(shape_r, shape_x, shape_y);
	return ep_World::RayCast(&dummy, ray_x, ray_y, ray_vx, ray_vy);
}
double ep_Main::RayCastPolygon(double ray_x, double ray_y, double ray_vx, double ray_vy,
	ep_Polygon* shape_polygon, double shape_x, double shape_y, double shape_rot) {
	double d = sqrt(ep_sqr(ray_vx)+ep_sqr(ray_vy));
	if(d<EP_EPSILON_MINNORMALVECTOR) return -1.0;
	ray_vx /= d;
	ray_vy /= d;
	ep_Shape dummy;
	dummy.InitVirtualPolygon(shape_polygon, shape_x, shape_y, shape_rot);
	return ep_World::RayCast(&dummy, ray_x, ray_y, ray_vx, ray_vy);
}

