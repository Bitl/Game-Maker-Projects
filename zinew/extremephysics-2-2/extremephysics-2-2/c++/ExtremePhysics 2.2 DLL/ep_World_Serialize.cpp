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
 * File: ep_World_Serialize.cpp
 * Polygon decomposition algorithm (extension).
 */

#include "ExtremePhysics.h"

#if EP_COMPILE_SERIALIZE

#define EP_WORLD_SERIALIZE_WRITE(data, type, value) {\
	*((type*)(data)) = (value);\
	(data) += sizeof(type);\
}
#define EP_WORLD_SERIALIZE_READ(data, length, type, value) {\
	if(sizeof(type)>(length)) goto onendofdata;\
	(value) = *((type*)(data));\
	(data) += sizeof(type);\
	(length) -= sizeof(type);\
}

/*#define EP_WORLD_SERIALIZE_WRITE_MEM(data, memory, len) {\
	memcpy((data), (memory), (len));\
	(data) += (len);\
}
#define EP_WORLD_SERIALIZE_READ_MEM(data, length, memory, len) {\
	memcpy((memory), (data), (len));\
	(data) += (len);\
	(length) -= (len);\
}*/

union ep_World_Serialize_shapedata {
	struct {
		double w, h;
	} box;
	struct {
		double r;
	} circle;
	struct {
		unsigned long polygon_id;
	} polygon;
};

#define EP_SERIALIZE_DEBUG 0 // used only for debugging

#if EP_SERIALIZE_DEBUG
const unsigned long ep_World_Serialize_version = *((const unsigned long*)("EPd5"));
const unsigned long ep_World_Serialize_marker  = *((const unsigned long*)("<<>>"));
#else
const unsigned long ep_World_Serialize_version = *((const unsigned long*)("EPs5"));
#endif

bool ep_World::Serialize() {
	
	ep_free(serializedata);
	serializedata = NULL;
	serializedata_length = sizeof(unsigned long);
	
	// world
	serializedata_length += sizeof(unsigned long)*18; //JOINTS//
	serializedata_length += sizeof(double)*1+sizeof(unsigned long)*2+sizeof(double)*5+sizeof(bool)*1;
	serializedata_length += sizeof(bool)*1+sizeof(double)*4;
#if EP_SERIALIZE_DEBUG
	serializedata_length += sizeof(unsigned long);
#endif
	// polygons
	serializedata_length += (sizeof(unsigned long)*2+sizeof(double)*4+sizeof(bool)*1)*polygoncount;
	for(ep_Polygon *polygon = first_polygon; polygon!=NULL; polygon = polygon->next) {
		serializedata_length += polygon->vertexcount*(sizeof(double)*7);
	}
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*polygoncount;
#endif
	// bodies
	serializedata_length += (sizeof(unsigned long)*1+sizeof(bool)*2)*bodycount;
	serializedata_length += (sizeof(unsigned long)*4+sizeof(double)*10)*bodycount;
	serializedata_length += (sizeof(double)*6)*bodycount;
	serializedata_length += (sizeof(bool)*5+sizeof(double)*2)*bodycount;
	serializedata_length += (sizeof(unsigned long)*1)*bodycount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*bodycount;
#endif
	// shapes
	serializedata_length += (sizeof(unsigned long)*1+sizeof(int)*1+sizeof(ep_World_Serialize_shapedata)*1)*shapecount;
	serializedata_length += (sizeof(double)*4)*shapecount;
	serializedata_length += (sizeof(float)*2+sizeof(double)*2+sizeof(unsigned long)*3)*shapecount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*shapecount;
#endif
	// forces
	serializedata_length += (sizeof(unsigned long)*1+sizeof(double)*2+sizeof(bool)*2)*forcecount;
	serializedata_length += (sizeof(double)*3)*forcecount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*forcecount;
#endif
	// contacts
	serializedata_length += (sizeof(unsigned long)*5+sizeof(double)*4)*contactcount;
	serializedata_length += (2*(sizeof(bool)*1+sizeof(double)*12))*contactcount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*contactcount;
#endif
	// hinge joints
	serializedata_length += (sizeof(unsigned long)*3+sizeof(double)*6)*hingejointcount;
	serializedata_length += (sizeof(double)*2)*hingejointcount;
	serializedata_length += (sizeof(double)*16)*hingejointcount;
	serializedata_length += (sizeof(double)*4)*hingejointcount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*hingejointcount;
#endif
	// distance joints
	serializedata_length += (sizeof(unsigned long)*3+sizeof(double)*4)*distancejointcount;
	serializedata_length += (sizeof(double)*2)*distancejointcount;
	serializedata_length += (sizeof(double)*16)*distancejointcount;
	serializedata_length += (sizeof(double)*2)*distancejointcount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*distancejointcount;
#endif
	// rail joints
	serializedata_length += (sizeof(unsigned long)*3+sizeof(double)*7)*railjointcount;
	serializedata_length += (sizeof(double)*3)*railjointcount;
	serializedata_length += (sizeof(double)*16)*railjointcount;
	serializedata_length += (sizeof(double)*3)*railjointcount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*railjointcount;
#endif
	// slider joints
	serializedata_length += (sizeof(unsigned long)*3+sizeof(double)*8)*sliderjointcount;
	serializedata_length += (sizeof(double)*4)*sliderjointcount;
	serializedata_length += (sizeof(double)*16)*sliderjointcount;
	serializedata_length += (sizeof(double)*4)*sliderjointcount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*sliderjointcount;
#endif
	//JOINTS//
	// view
	serializedata_length += (sizeof(unsigned long)*1)*viewcount;
	serializedata_length += (sizeof(double)*4)*viewcount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*viewcount;
#endif
	// water
	serializedata_length += (sizeof(unsigned long)*1)*watercount;
	serializedata_length += (sizeof(double)*11)*watercount;
#if EP_SERIALIZE_DEBUG
	serializedata_length += (sizeof(unsigned long))*watercount;
#endif
	
	serializedata = (char*)(ep_malloc(serializedata_length));
	if(serializedata==NULL) {
		serializedata_length = 0;
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Serialization of world %lu failed, memory allocation failed.", id);
#endif
		return false;
	}
	
	char *data = serializedata;
	
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_version);
	
	// world
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_polygons);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, polygoncount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_bodies);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, bodycount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_contacts);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, contactcount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_hingejoints);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, hingejointcount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_distancejoints);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, distancejointcount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_railjoints);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, railjointcount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_sliderjoints);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, sliderjointcount);
	//JOINTS//
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_views);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, viewcount);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, idcounter_water);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, watercount);
	
	EP_WORLD_SERIALIZE_WRITE(data, double, timestep);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, velocity_iterations);
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, position_iterations);
	EP_WORLD_SERIALIZE_WRITE(data, double, contact_threshold);
	EP_WORLD_SERIALIZE_WRITE(data, double, velocity_threshold);
	EP_WORLD_SERIALIZE_WRITE(data, double, baumgarte_factor);
	EP_WORLD_SERIALIZE_WRITE(data, double, mass_bias);
	EP_WORLD_SERIALIZE_WRITE(data, double, position_factor);
	EP_WORLD_SERIALIZE_WRITE(data, bool, horizontal);
	
	EP_WORLD_SERIALIZE_WRITE(data, bool, enable_sleeping);
	EP_WORLD_SERIALIZE_WRITE(data, double, time_stable);
	EP_WORLD_SERIALIZE_WRITE(data, double, time_outofview);
	EP_WORLD_SERIALIZE_WRITE(data, double, stable_maxvel);
	EP_WORLD_SERIALIZE_WRITE(data, double, stable_maxrotvel);
	
#if EP_SERIALIZE_DEBUG
	EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
	
	// polygons
	for(ep_Polygon *polygon = first_polygon; polygon!=NULL; polygon = polygon->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, polygon->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, polygon->vertexcount);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, polygon->xoff);
		EP_WORLD_SERIALIZE_WRITE(data, double, polygon->yoff);
		EP_WORLD_SERIALIZE_WRITE(data, double, polygon->mass);
		EP_WORLD_SERIALIZE_WRITE(data, double, polygon->inertia);
#if EP_COMPILE_DEBUGCHECKS
		EP_WORLD_SERIALIZE_WRITE(data, bool, polygon->initialized);
#else
		EP_WORLD_SERIALIZE_WRITE(data, bool, false);
#endif
		
		for(unsigned long i = 0; i<polygon->vertexcount; ++i) {
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].x);
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].y);
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].nx);
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].ny);
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].len);
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].dist);
			EP_WORLD_SERIALIZE_WRITE(data, double, polygon->vertices[i].pos);
		}
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	// bodies
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, body->id);
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->isstatic);
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->norotation);
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, body->idcounter_shapes);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, body->shapecount);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, body->idcounter_forces);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, body->forcecount);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->xoff);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->yoff);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->mass);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->inertia);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->x);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->y);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->rot);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->xvel);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->yvel);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->rotvel);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, body->maxvel);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->maxrotvel);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->gravity_x);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->gravity_y);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->damping);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->rotdamping);
		
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->storecontactimpulses);
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->storejointimpulses);
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->sleepstable);
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->sleepoutofview);
		EP_WORLD_SERIALIZE_WRITE(data, bool, body->issleeping);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->stabletimer);
		EP_WORLD_SERIALIZE_WRITE(data, double, body->outofviewtimer);
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, (body->nextislandbody==NULL)? 0 : body->nextislandbody->id);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
		// shapes
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			
			ep_World_Serialize_shapedata shapedata;
			switch(shape->shapetype) {
				case EP_SHAPETYPE_BOX: {
					shapedata.box.w = shape->shapedata.box.w;
					shapedata.box.h = shape->shapedata.box.h;
					break;
				}
				case EP_SHAPETYPE_CIRCLE: {
					shapedata.circle.r = shape->shapedata.circle.r;
					break;
				}
				case EP_SHAPETYPE_POLYGON: {
					shapedata.polygon.polygon_id = shape->shapedata.polygon.polygon->id;
					break;
				}
			}
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, shape->id);
			EP_WORLD_SERIALIZE_WRITE(data, int, shape->shapetype);
			EP_WORLD_SERIALIZE_WRITE(data, ep_World_Serialize_shapedata, shapedata);
			
			EP_WORLD_SERIALIZE_WRITE(data, double, shape->x);
			EP_WORLD_SERIALIZE_WRITE(data, double, shape->y);
			EP_WORLD_SERIALIZE_WRITE(data, double, shape->rot);
			EP_WORLD_SERIALIZE_WRITE(data, double, shape->density);
			
			EP_WORLD_SERIALIZE_WRITE(data, float, shape->restitution);
			EP_WORLD_SERIALIZE_WRITE(data, float, shape->friction);
			EP_WORLD_SERIALIZE_WRITE(data, double, shape->normalvelocity);
			EP_WORLD_SERIALIZE_WRITE(data, double, shape->tangentvelocity);
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, shape->collidemask1);
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, shape->collidemask2);
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, shape->group);
			
#if EP_SERIALIZE_DEBUG
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
			
		}
		
		// forces
		for(ep_Force *force = body->first_force; force!=NULL; force = force->next) {
			
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, force->id);
			EP_WORLD_SERIALIZE_WRITE(data, double, force->x);
			EP_WORLD_SERIALIZE_WRITE(data, double, force->y);
			EP_WORLD_SERIALIZE_WRITE(data, bool, force->local);
			EP_WORLD_SERIALIZE_WRITE(data, bool, force->ignoremass);
			
			EP_WORLD_SERIALIZE_WRITE(data, double, force->xforce);
			EP_WORLD_SERIALIZE_WRITE(data, double, force->yforce);
			EP_WORLD_SERIALIZE_WRITE(data, double, force->torque);
			
#if EP_SERIALIZE_DEBUG
			EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
			
		}
		
	}
	
	// contacts
	for(ep_Contact *contact = first_contact; contact!=NULL; contact = contact->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, contact->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, contact->shape1->body->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, contact->shape1->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, contact->shape2->body->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, contact->shape2->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, contact->nx);
		EP_WORLD_SERIALIZE_WRITE(data, double, contact->ny);
		EP_WORLD_SERIALIZE_WRITE(data, double, contact->restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, contact->friction);
		
		for(unsigned long i = 0; i<2; ++i) {
			EP_WORLD_SERIALIZE_WRITE(data, bool, contact->contactpoints[i].active);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].x1);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].y1);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].x2);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].y2);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].xx1);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].yy1);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].xx2);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].yy2);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].normalforce);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].tangentforce);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].normalveldelta);
			EP_WORLD_SERIALIZE_WRITE(data, double, contact->contactpoints[i].tangentveldelta);
		}
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	// hinge joints
	for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, hingejoint->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, hingejoint->body1->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, hingejoint->body2->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->x1);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->y1);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->x2);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->y2);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->referencerotation);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->maxforce);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->maxmotortorque);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->motorvel);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit1_maxtorque);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit1_rot);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit1_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit1_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit2_maxtorque);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit2_rot);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit2_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit2_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->spring1_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->spring1_rotation);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->spring1_damping);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->spring2_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->spring2_rotation);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->xforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->yforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->motortorque);
		EP_WORLD_SERIALIZE_WRITE(data, double, hingejoint->limittorque);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	// distance joints
	for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, distancejoint->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, distancejoint->body1->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, distancejoint->body2->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->x1);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->y1);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->x2);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->y2);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->maxmotorforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->motorvel);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit1_maxforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit1_distance);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit1_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit1_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit2_maxforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit2_distance);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit2_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit2_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->spring1_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->spring1_distance);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->spring1_damping);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->spring2_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->spring2_distance);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->motorforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, distancejoint->limitforce);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	// rail joints
	for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, railjoint->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, railjoint->body1->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, railjoint->body2->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->x1);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->y1);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->x2);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->y2);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->vx);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->vy);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->length);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->maxnormalforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->maxmotorforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->motorvel);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit1_maxforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit1_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit1_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit1_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit2_maxforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit2_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit2_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit2_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->spring1_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->spring1_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->spring1_damping);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->spring2_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->spring2_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->normalforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->motorforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, railjoint->limitforce);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	// slider joints
	for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, sliderjoint->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, sliderjoint->body1->id);
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, sliderjoint->body2->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->x1);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->y1);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->x2);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->y2);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->vx);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->vy);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->length);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->rotation);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->maxnormalforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->torqueradius);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->maxmotorforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->motorvel);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit1_maxforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit1_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit1_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit1_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit2_maxforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit2_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit2_restitution);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit2_velocity);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->spring1_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->spring1_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->spring1_damping);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->spring2_k);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->spring2_position);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->normalforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->torque);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->motorforce);
		EP_WORLD_SERIALIZE_WRITE(data, double, sliderjoint->limitforce);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	//JOINTS//
	
	// views
	for(ep_View *view = first_view; view!=NULL; view = view->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, view->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, view->x1);
		EP_WORLD_SERIALIZE_WRITE(data, double, view->y1);
		EP_WORLD_SERIALIZE_WRITE(data, double, view->x2);
		EP_WORLD_SERIALIZE_WRITE(data, double, view->y2);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
	// water
	for(ep_Water *water = first_water; water!=NULL; water = water->next) {
		
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, water->id);
		
		EP_WORLD_SERIALIZE_WRITE(data, double, water->density);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->lineardrag);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->quadraticdrag);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->xvel);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->yvel);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->gravity_x);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->gravity_y);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->x1);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->y1);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->x2);
		EP_WORLD_SERIALIZE_WRITE(data, double, water->y2);
		
#if EP_SERIALIZE_DEBUG
		EP_WORLD_SERIALIZE_WRITE(data, unsigned long, ep_World_Serialize_marker);
#endif
		
	}
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Serialized world %lu.", id);
#endif
	
	return true;
}

bool ep_World::Unserialize(const char* data, unsigned long length) {
	
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Unserialization of world %lu started.", id);
	Clear();
	
	unsigned long v;
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, v);
	if(v!=ep_World_Serialize_version) {
		main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid.", id);
		return false;
	}
	
	// world
	unsigned long polygons, bodies, contacts, hingejoints, distancejoints, railjoints, sliderjoints, views, waters;
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_polygons);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, polygons);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_bodies);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, bodies);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_contacts);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, contacts);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_hingejoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, hingejoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_distancejoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, distancejoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_railjoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, railjoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_sliderjoints);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, sliderjoints);
	//JOINTS//
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_views);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, views);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, idcounter_water);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, waters);
	
	EP_WORLD_SERIALIZE_READ(data, length, double, timestep);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, velocity_iterations);
	EP_WORLD_SERIALIZE_READ(data, length, unsigned long, position_iterations);
	EP_WORLD_SERIALIZE_READ(data, length, double, contact_threshold);
	EP_WORLD_SERIALIZE_READ(data, length, double, velocity_threshold);
	EP_WORLD_SERIALIZE_READ(data, length, double, baumgarte_factor);
	EP_WORLD_SERIALIZE_READ(data, length, double, mass_bias);
	EP_WORLD_SERIALIZE_READ(data, length, double, position_factor);
	EP_WORLD_SERIALIZE_READ(data, length, bool, horizontal);
	
	EP_WORLD_SERIALIZE_READ(data, length, bool, enable_sleeping);
	EP_WORLD_SERIALIZE_READ(data, length, double, time_stable);
	EP_WORLD_SERIALIZE_READ(data, length, double, time_outofview);
	EP_WORLD_SERIALIZE_READ(data, length, double, stable_maxvel);
	EP_WORLD_SERIALIZE_READ(data, length, double, stable_maxrotvel);
	
#if EP_SERIALIZE_DEBUG
	{
		unsigned long m;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
		if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (world marker missing).", id);
#endif
			goto onerror;
		}
	}
#endif
	
	// polygons
	for(unsigned long j = 0; j<polygons; ++j) {
		
		unsigned long _id, vertexcount;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, vertexcount);
		if(vertexcount<3) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (polygon vertexcount < 3).", id);
#endif
			goto onerror;
		}
		
		// create the polygon
		ep_Polygon *polygon = AllocatePolygon(vertexcount);
		if(polygon==NULL) {
			goto onerror;
		}
		polygon->Init(this, vertexcount, _id);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, polygon->xoff);
		EP_WORLD_SERIALIZE_READ(data, length, double, polygon->yoff);
		EP_WORLD_SERIALIZE_READ(data, length, double, polygon->mass);
		EP_WORLD_SERIALIZE_READ(data, length, double, polygon->inertia);
		bool init;
		EP_WORLD_SERIALIZE_READ(data, length, bool, init);
#if EP_COMPILE_DEBUGCHECKS
		polygon->initialized = init;
#endif
		
		for(unsigned long i = 0; i<polygon->vertexcount; ++i) {
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].x);
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].y);
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].nx);
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].ny);
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].len);
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].dist);
			EP_WORLD_SERIALIZE_READ(data, length, double, polygon->vertices[i].pos);
		}
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (polygon marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	// bodies
	for(unsigned long j = 0; j<bodies; ++j) {
		
		unsigned long _id;
		bool isstatic, norotation;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, bool, isstatic);
		EP_WORLD_SERIALIZE_READ(data, length, bool, norotation);
		
		// create the body
		ep_Body *body = AllocateBody();
		if(body==NULL) {
			goto onerror;
		}
		body->Init(this, isstatic, norotation, _id);
		
		unsigned long shapes, forces;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body->idcounter_shapes);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, shapes);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body->idcounter_forces);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, forces);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->xoff);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->yoff);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->mass);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->inertia);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->x);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->y);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->rot);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->xvel);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->yvel);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->rotvel);
		body->invmass = (isstatic)? 0.0 : 1.0/body->mass;
		body->invinertia = (norotation)? 0.0 : 1.0/body->inertia;
		body->rot_sin = sin(body->rot);
		body->rot_cos = cos(body->rot);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, body->maxvel);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->maxrotvel);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->gravity_x);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->gravity_y);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->damping);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->rotdamping);
		if(timestep!=0.0) {
			body->damping_factor = pow(1.0-body->damping, timestep);
			body->rotdamping_factor = pow(1.0-body->rotdamping, timestep);
		}
		
		EP_WORLD_SERIALIZE_READ(data, length, bool, body->storecontactimpulses);
		EP_WORLD_SERIALIZE_READ(data, length, bool, body->storejointimpulses);
		EP_WORLD_SERIALIZE_READ(data, length, bool, body->sleepstable);
		EP_WORLD_SERIALIZE_READ(data, length, bool, body->sleepoutofview);
		EP_WORLD_SERIALIZE_READ(data, length, bool, body->issleeping);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->stabletimer);
		EP_WORLD_SERIALIZE_READ(data, length, double, body->outofviewtimer);
		
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body->nextislandbody_id);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (body marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
		// shapes
		for(unsigned long k = 0; k<shapes; ++k) {
			
			unsigned long _id2;
			int shapetype;
			double _x, _y, _rot, density;
			ep_World_Serialize_shapedata shapedata;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id2);
			EP_WORLD_SERIALIZE_READ(data, length, int, shapetype);
			EP_WORLD_SERIALIZE_READ(data, length, ep_World_Serialize_shapedata, shapedata);
			EP_WORLD_SERIALIZE_READ(data, length, double, _x);
			EP_WORLD_SERIALIZE_READ(data, length, double, _y);
			EP_WORLD_SERIALIZE_READ(data, length, double, _rot);
			EP_WORLD_SERIALIZE_READ(data, length, double, density);
			ep_Polygon *polygon;
#ifdef __GNUC__
			polygon = NULL;
#endif
			if(shapetype==EP_SHAPETYPE_POLYGON) {
				polygon = FindPolygon(shapedata.polygon.polygon_id);
				if(polygon==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
					main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (shape polygon does not exist).", id);
#endif
					goto onerror;
				}
			}
			
			// create the shape
			ep_Shape *shape = AllocateShape();
			if(shape==NULL) {
				goto onerror;
			}
			switch(shapetype) {
				case EP_SHAPETYPE_BOX: {
					shape->Init(body, shapetype, _x, _y, _rot, density, shapedata.box.w, shapedata.box.h, NULL, _id2);
					break;
				}
				case EP_SHAPETYPE_CIRCLE: {
					shape->Init(body, shapetype, _x, _y, _rot, density, shapedata.circle.r, 0.0, NULL, _id2);
					break;
				}
				case EP_SHAPETYPE_POLYGON: {
					shape->Init(body, shapetype, _x, _y, _rot, density, 0.0, 0.0, polygon, _id2);
					break;
				}
			}
			
			EP_WORLD_SERIALIZE_READ(data, length, float, shape->restitution);
			EP_WORLD_SERIALIZE_READ(data, length, float, shape->friction);
			EP_WORLD_SERIALIZE_READ(data, length, double, shape->normalvelocity);
			EP_WORLD_SERIALIZE_READ(data, length, double, shape->tangentvelocity);
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, shape->collidemask1);
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, shape->collidemask2);
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, shape->group);
			
#if EP_SERIALIZE_DEBUG
			{
				unsigned long m;
				EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
				if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
					main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (shape marker missing).", id);
#endif
					goto onerror;
				}
			}
#endif
			
		}
		
		// forces
		for(unsigned long k = 0; k<forces; ++k) {
			
			unsigned long _id2;
			double _x, _y;
			bool _local, _ignoremass;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id2);
			EP_WORLD_SERIALIZE_READ(data, length, double, _x);
			EP_WORLD_SERIALIZE_READ(data, length, double, _y);
			EP_WORLD_SERIALIZE_READ(data, length, bool, _local);
			EP_WORLD_SERIALIZE_READ(data, length, bool, _ignoremass);
			
			// create the force
			ep_Force *force = AllocateForce();
			if(force==NULL) {
				goto onerror;
			}
			force->Init(body, _x, _y, _local, _ignoremass, _id);
			
			EP_WORLD_SERIALIZE_READ(data, length, double, force->xforce);
			EP_WORLD_SERIALIZE_READ(data, length, double, force->yforce);
			EP_WORLD_SERIALIZE_READ(data, length, double, force->torque);
			
#if EP_SERIALIZE_DEBUG
			{
				unsigned long m;
				EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
				if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
					main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (force marker missing).", id);
#endif
					goto onerror;
				}
			}
#endif
			
		}
		
	}
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		// remember nextislandbody and nextislandbody_id are a union
		ep_Body *other = (body->nextislandbody_id==0)? NULL : FindBody(body->nextislandbody_id);
		body->nextislandbody = other;
	}
	
	// contacts
	for(unsigned long j = 0; j<contacts; ++j) {
		
		unsigned long _id, body1_id, shape1_id, body2_id, shape2_id;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body1_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, shape1_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body2_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, shape2_id);
		
		ep_Body *body1, *body2;
		ep_Shape *shape1, *shape2;
		body1 = FindBody(body1_id);
		if(body1==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (contact body 1 does not exist).", id);
#endif
			goto onerror;
		}
		shape1 = body1->FindShape(shape1_id);
		if(shape1==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (contact shape 1 does not exist).", id);
#endif
			goto onerror;
		}
		body2 = FindBody(body2_id);
		if(body2==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (contact body 2 does not exist).", id);
#endif
			goto onerror;
		}
		shape2 = body2->FindShape(shape2_id);
		if(shape2==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (contact shape 2 does not exist).", id);
#endif
			goto onerror;
		}
		
		ep_Contact *contact = AllocateContact();
		if(contact==NULL) {
			goto onerror;
		}
		contact->Init(this, shape1, shape2, _id, false);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, contact->nx);
		EP_WORLD_SERIALIZE_READ(data, length, double, contact->ny);
		EP_WORLD_SERIALIZE_READ(data, length, double, contact->restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, contact->friction);
		
		for(unsigned long i = 0; i<2; ++i) {
			EP_WORLD_SERIALIZE_READ(data, length, bool, contact->contactpoints[i].active);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].x1);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].y1);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].x2);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].y2);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].xx1);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].yy1);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].xx2);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].yy2);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].normalforce);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].tangentforce);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].normalveldelta);
			EP_WORLD_SERIALIZE_READ(data, length, double, contact->contactpoints[i].tangentveldelta);
		}
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (contact marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	// hinge joints
	for(unsigned long j = 0; j<hingejoints; ++j) {
		
		unsigned long _id, body1_id, body2_id;
		double _x1, _y1, _x2, _y2, _referencerotation;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body1_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body2_id);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _referencerotation);
		
		ep_Body *body1, *body2;
		body1 = FindBody(body1_id);
		if(body1==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (hinge joint body 1 does not exist).", id);
#endif
			goto onerror;
		}
		body2 = FindBody(body2_id);
		if(body2==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (hinge joint body 2 does not exist).", id);
#endif
			goto onerror;
		}
		
		ep_HingeJoint *hingejoint = AllocateHingeJoint();
		if(hingejoint==NULL) {
			goto onerror;
		}
		hingejoint->Init(this, body1, body2, _x1, _y1, _x2, _y2, _referencerotation, _id, false);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->maxforce);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->maxmotortorque);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->motorvel);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit1_maxtorque);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit1_rot);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit1_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit1_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit2_maxtorque);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit2_rot);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit2_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit2_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->spring1_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->spring1_rotation);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->spring1_damping);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->spring2_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->spring2_rotation);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->xforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->yforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->motortorque);
		EP_WORLD_SERIALIZE_READ(data, length, double, hingejoint->limittorque);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (hinge joint marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	// distance joints
	for(unsigned long j = 0; j<distancejoints; ++j) {
		
		unsigned long _id, body1_id, body2_id;
		double _x1, _y1, _x2, _y2;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body1_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body2_id);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y2);
		
		ep_Body *body1, *body2;
		body1 = FindBody(body1_id);
		if(body1==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (distance joint body 1 does not exist).", id);
#endif
			goto onerror;
		}
		body2 = FindBody(body2_id);
		if(body2==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (distance joint body 2 does not exist).", id);
#endif
			goto onerror;
		}
		
		ep_DistanceJoint *distancejoint = AllocateDistanceJoint();
		if(distancejoint==NULL) {
			goto onerror;
		}
		distancejoint->Init(this, body1, body2, _x1, _y1, _x2, _y2, _id, false);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->maxmotorforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->motorvel);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit1_maxforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit1_distance);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit1_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit1_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit2_maxforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit2_distance);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit2_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit2_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->spring1_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->spring1_distance);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->spring1_damping);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->spring2_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->spring2_distance);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->motorforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, distancejoint->limitforce);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (distance joint marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	// rail joints
	for(unsigned long j = 0; j<railjoints; ++j) {
		
		unsigned long _id, body1_id, body2_id;
		double _x1, _y1, _x2, _y2, _vx, _vy, _length;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body1_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body2_id);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _vx);
		EP_WORLD_SERIALIZE_READ(data, length, double, _vy);
		EP_WORLD_SERIALIZE_READ(data, length, double, _length);
		
		ep_Body *body1, *body2;
		body1 = FindBody(body1_id);
		if(body1==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (rail joint body 1 does not exist).", id);
#endif
			goto onerror;
		}
		body2 = FindBody(body2_id);
		if(body2==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (rail joint body 2 does not exist).", id);
#endif
			goto onerror;
		}
		
		ep_RailJoint *railjoint = AllocateRailJoint();
		if(railjoint==NULL) {
			goto onerror;
		}
		railjoint->Init(this, body1, body2, _x1, _y1, _x2, _y2, _vx, _vy, _length, _id, false);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->maxnormalforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->maxmotorforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->motorvel);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit1_maxforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit1_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit1_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit1_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit2_maxforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit2_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit2_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit2_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->spring1_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->spring1_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->spring1_damping);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->spring2_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->spring2_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->normalforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->motorforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, railjoint->limitforce);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (rail joint marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	// slider joints
	for(unsigned long j = 0; j<sliderjoints; ++j) {
		
		unsigned long _id, body1_id, body2_id;
		double _x1, _y1, _x2, _y2, _vx, _vy, _length, _rotation;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body1_id);
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, body2_id);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y1);
		EP_WORLD_SERIALIZE_READ(data, length, double, _x2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _y2);
		EP_WORLD_SERIALIZE_READ(data, length, double, _vx);
		EP_WORLD_SERIALIZE_READ(data, length, double, _vy);
		EP_WORLD_SERIALIZE_READ(data, length, double, _length);
		EP_WORLD_SERIALIZE_READ(data, length, double, _rotation);
		
		ep_Body *body1, *body2;
		body1 = FindBody(body1_id);
		if(body1==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (slider joint body 1 does not exist).", id);
#endif
			goto onerror;
		}
		body2 = FindBody(body2_id);
		if(body2==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (slider joint body 2 does not exist).", id);
#endif
			goto onerror;
		}
		
		ep_SliderJoint *sliderjoint = AllocateSliderJoint();
		if(sliderjoint==NULL) {
			goto onerror;
		}
		sliderjoint->Init(this, body1, body2, _x1, _y1, _x2, _y2, _vx, _vy, _length, _rotation, _id, false);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->maxnormalforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->torqueradius);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->maxmotorforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->motorvel);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit1_maxforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit1_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit1_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit1_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit2_maxforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit2_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit2_restitution);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit2_velocity);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit_contact_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limit_velocity_threshold);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->spring1_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->spring1_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->spring1_damping);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->spring2_k);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->spring2_position);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->spring2_damping);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->normalforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->torque);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->motorforce);
		EP_WORLD_SERIALIZE_READ(data, length, double, sliderjoint->limitforce);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (slider joint marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	//JOINTS//
	
	// views
	for(unsigned long j = 0; j<views; ++j) {
		
		unsigned long _id;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		
		// create the view
		ep_View *view = AllocateView();
		if(view==NULL) {
			goto onerror;
		}
		view->Init(this, _id);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, view->x1);
		EP_WORLD_SERIALIZE_READ(data, length, double, view->y1);
		EP_WORLD_SERIALIZE_READ(data, length, double, view->x2);
		EP_WORLD_SERIALIZE_READ(data, length, double, view->y2);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (view marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	// water
	for(unsigned long j = 0; j<waters; ++j) {
		
		unsigned long _id;
		EP_WORLD_SERIALIZE_READ(data, length, unsigned long, _id);
		
		// create the water object
		ep_Water *water = AllocateWater();
		if(water==NULL) {
			goto onerror;
		}
		water->Init(this, _id, false);
		
		EP_WORLD_SERIALIZE_READ(data, length, double, water->density);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->lineardrag);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->quadraticdrag);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->xvel);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->yvel);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->gravity_x);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->gravity_y);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->x1);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->y1);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->x2);
		EP_WORLD_SERIALIZE_READ(data, length, double, water->y2);
		
#if EP_SERIALIZE_DEBUG
		{
			unsigned long m;
			EP_WORLD_SERIALIZE_READ(data, length, unsigned long, m);
			if(m!=ep_World_Serialize_marker) {
#if EP_COMPILE_DEBUGMESSAGES
				main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, the data is not valid (water marker missing).", id);
#endif
				goto onerror;
			}
		}
#endif
		
	}
	
	if(length!=0) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, end of data expected.", id);
#endif
		goto onerror;
	}
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_IMPORTANTCHANGE, "Unserialization of world %lu succeeded.", id);
#endif
	return true;
	
	onendofdata:
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_ERROR, "Unserialization of world %lu failed, unexpected end of data.", id);
#endif
	goto onerror;
	
	onerror:
	Clear();
	return false;
}

void ep_World::FreeSerializeData() {
	ep_free(serializedata);
	serializedata = NULL;
}

#endif // EP_COMPILE_SERIALIZE

