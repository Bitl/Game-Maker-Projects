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
 * File: ep_DistanceJoint.cpp
 * Implementation of ep_DistanceJoint.
 */

#include "ExtremePhysics.h"

void ep_DistanceJoint::Init(ep_World* _world, ep_Body* _body1, ep_Body* _body2, double _x1, double _y1, double _x2, double _y2, unsigned long _id, bool awake) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_distancejoint;
	next = NULL;
	if(prev==NULL) world->first_distancejoint = this;
	else prev->next = this;
	world->last_distancejoint = this;
	
	// initialize variables
	id = _id;
#if EP_USE_IDHASHTABLE
	world->idhashtable_distancejoints.Insert(this);
#endif
	
	body1 = _body1;
	body2 = _body2;
	
	x1 = _x1;
	y1 = _y1;
	x2 = _x2;
	y2 = _y2;
	
	maxmotorforce = 0.0;
	motorvel = 0.0;
	
	limit1_maxforce = 0.0;
	limit1_distance = 0.0;
	limit1_restitution = 0.0;
	limit1_velocity = 0.0;
	limit2_maxforce = 0.0;
	limit2_distance = 0.0;
	limit2_restitution = 0.0;
	limit2_velocity = 0.0;
	limit_contact_threshold = 0.1;
	limit_velocity_threshold = 0.5;
	spring1_k = 0.0;
	spring1_distance = 0.0;
	spring1_damping = 0.0;
	spring2_k = 0.0;
	spring2_distance = 0.0;
	spring2_damping = 0.0;
	
	motorforce = 0.0;
	limitforce = 0.0;
	
	// initialize links
	link1.distancejoint = this;
	link2.distancejoint = this;
	link1.other_body = body2;
	link2.other_body = body1;
	
	// add link1 to body1's joint list
	link1.prev = body1->last_distancejointlink;
	link1.next = NULL;
	if(link1.prev==NULL) body1->first_distancejointlink = &link1;
	else link1.prev->next = &link1;
	body1->last_distancejointlink = &link1;
	
	// add link2 to body1's joint list
	link2.prev = body2->last_distancejointlink;
	link2.next = NULL;
	if(link2.prev==NULL) body2->first_distancejointlink = &link2;
	else link2.prev->next = &link2;
	body2->last_distancejointlink = &link2;
	
	// awake connected bodies
	if(awake) {
		body1->Awake(true, false);
		body2->Awake(true, false);
	}
	
	++world->distancejointcount;
	
}

void ep_DistanceJoint::DeInit() {
	
	// remove from linked list
	if(prev==NULL) world->first_distancejoint = next;
	else prev->next = next;
	if(next==NULL) world->last_distancejoint = prev;
	else next->prev = prev;
	if(world->current_distancejoint==this) world->current_distancejoint = NULL;
#if EP_USE_IDHASHTABLE
	world->idhashtable_distancejoints.Remove(this);
#endif
	
	// remove link1 from body1's joint list
	if(link1.prev==NULL) body1->first_distancejointlink = link1.next;
	else link1.prev->next = link1.next;
	if(link1.next==NULL) body1->last_distancejointlink = link1.prev;
	else link1.next->prev = link1.prev;
	
	// remove link2 from body2's joint list
	if(link2.prev==NULL) body2->first_distancejointlink = link2.next;
	else link2.prev->next = link2.next;
	if(link2.next==NULL) body2->last_distancejointlink = link2.prev;
	else link2.next->prev = link2.prev;
	
	// awake connected bodies
	body1->Awake(true, false);
	body2->Awake(true, false);
	
	--world->distancejointcount;
	
}

void ep_DistanceJoint::SetMotor(double _maxmotorforce, double _motorvel) {
	if(_maxmotorforce<0.0) _maxmotorforce = 0.0;
	if(maxmotorforce!=_maxmotorforce || motorvel!=_motorvel) {
		maxmotorforce = _maxmotorforce;
		motorvel = _motorvel;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed motor settings of distance joint %lu in world %lu.", id, world->id);
#endif
}

void ep_DistanceJoint::SetLimitSettings(double contact_threshold, double velocity_threshold) {
	if(limit_contact_threshold!=contact_threshold || limit_velocity_threshold!=velocity_threshold) {
		limit_contact_threshold = contact_threshold;
		limit_velocity_threshold = velocity_threshold;
		if(limit1_maxforce!=0.0 || limit2_maxforce!=0.0) {
			body1->Awake(false, false);
			body2->Awake(false, false);
		}
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed limit settings of distance joint %lu in world %lu.", id, world->id);
#endif
}

void ep_DistanceJoint::SetLowerLimit(double maxlimitforce, double distance, double restitution, double velocity) {
	if(maxlimitforce<0.0) maxlimitforce = 0.0;
	if(distance<0.0) distance = 0.0;
	if(limit1_maxforce!=maxlimitforce || limit1_distance!=distance || limit1_restitution!=restitution || limit1_velocity!=velocity) {
		limit1_maxforce = maxlimitforce;
		limit1_distance = distance;
		limit1_restitution = restitution;
		limit1_velocity = velocity;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed lower limit of distance joint %lu in world %lu.", id, world->id);
#endif
}

void ep_DistanceJoint::SetUpperLimit(double maxlimitforce, double distance, double restitution, double velocity) {
	if(maxlimitforce<0.0) maxlimitforce = 0.0;
	if(distance<0.0) distance = 0.0;
	if(limit2_maxforce!=maxlimitforce || limit2_distance!=distance || limit2_restitution!=restitution || limit2_velocity!=velocity) {
		limit2_maxforce = maxlimitforce;
		limit2_distance = distance;
		limit2_restitution = restitution;
		limit2_velocity = velocity;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed upper limit of distance joint %lu in world %lu.", id, world->id);
#endif
}

void ep_DistanceJoint::SetLowerSpring(double k, double distance, double damping) {
	if(k<0.0) k = 0.0;
	if(distance<0.0) distance = 0.0;
	if(damping<0.0) damping = 0.0;
	if(spring1_k!=k || spring1_distance!=distance || spring1_damping!=damping) {
		spring1_k = k;
		spring1_distance = distance;
		spring1_damping = damping;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed lower spring of distance joint %lu in world %lu.", id, world->id);
#endif
}

void ep_DistanceJoint::SetUpperSpring(double k, double distance, double damping) {
	if(k<0.0) k = 0.0;
	if(distance<0.0) distance = 0.0;
	if(damping<0.0) damping = 0.0;
	if(spring2_k!=k || spring2_distance!=distance || spring2_damping!=damping) {
		spring2_k = k;
		spring2_distance = distance;
		spring2_damping = damping;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed upper spring of distance joint %lu in world %lu.", id, world->id);
#endif
}

