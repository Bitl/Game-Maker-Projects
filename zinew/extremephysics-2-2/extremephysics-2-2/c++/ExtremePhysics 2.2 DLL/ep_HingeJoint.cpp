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
 * File: ep_HingeJoint.cpp
 * Implementation of ep_HingeJoint.
 */

#include "ExtremePhysics.h"

void ep_HingeJoint::Init(ep_World* _world, ep_Body* _body1, ep_Body* _body2, double _x1, double _y1, double _x2, double _y2, double _referencerotation, unsigned long _id, bool awake) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_hingejoint;
	next = NULL;
	if(prev==NULL) world->first_hingejoint = this;
	else prev->next = this;
	world->last_hingejoint = this;
	
	// initialize variables
	id = _id;
#if EP_USE_IDHASHTABLE
	world->idhashtable_hingejoints.Insert(this);
#endif
	
	body1 = _body1;
	body2 = _body2;
	
	x1 = _x1;
	y1 = _y1;
	x2 = _x2;
	y2 = _y2;
	referencerotation = _referencerotation;
	maxforce = 0.0;
	
	maxmotortorque = 0.0;
	motorvel = 0.0;
	
	limit1_maxtorque = 0.0;
	limit1_rot = 0.0;
	limit1_restitution = 0.0;
	limit1_velocity = 0.0;
	limit2_maxtorque = 0.0;
	limit2_rot = 0.0;
	limit2_restitution = 0.0;
	limit2_velocity = 0.0;
	limit_contact_threshold = 0.001;
	limit_velocity_threshold = 0.005;
	spring1_k = 0.0;
	spring1_rotation = 0.0;
	spring1_damping = 0.0;
	spring2_k = 0.0;
	spring2_rotation = 0.0;
	spring2_damping = 0.0;
	
	xforce = 0.0;
	yforce = 0.0;
	motortorque = 0.0;
	limittorque = 0.0;
	
	// initialize links
	link1.hingejoint = this;
	link2.hingejoint = this;
	link1.other_body = body2;
	link2.other_body = body1;
	
	// add link1 to body1's joint list
	link1.prev = body1->last_hingejointlink;
	link1.next = NULL;
	if(link1.prev==NULL) body1->first_hingejointlink = &link1;
	else link1.prev->next = &link1;
	body1->last_hingejointlink = &link1;
	
	// add link2 to body1's joint list
	link2.prev = body2->last_hingejointlink;
	link2.next = NULL;
	if(link2.prev==NULL) body2->first_hingejointlink = &link2;
	else link2.prev->next = &link2;
	body2->last_hingejointlink = &link2;
	
	// awake connected bodies
	if(awake) {
		body1->Awake(true, false);
		body2->Awake(true, false);
	}
	
	++world->hingejointcount;
	
}

void ep_HingeJoint::DeInit() {
	
	// remove from linked list
	if(prev==NULL) world->first_hingejoint = next;
	else prev->next = next;
	if(next==NULL) world->last_hingejoint = prev;
	else next->prev = prev;
	if(world->current_hingejoint==this) world->current_hingejoint = NULL;
#if EP_USE_IDHASHTABLE
	world->idhashtable_hingejoints.Remove(this);
#endif
	
	// remove link1 from body1's joint list
	if(link1.prev==NULL) body1->first_hingejointlink = link1.next;
	else link1.prev->next = link1.next;
	if(link1.next==NULL) body1->last_hingejointlink = link1.prev;
	else link1.next->prev = link1.prev;
	
	// remove link2 from body2's joint list
	if(link2.prev==NULL) body2->first_hingejointlink = link2.next;
	else link2.prev->next = link2.next;
	if(link2.next==NULL) body2->last_hingejointlink = link2.prev;
	else link2.next->prev = link2.prev;
	
	// awake connected bodies
	body1->Awake(true, false);
	body2->Awake(true, false);
	
	--world->hingejointcount;
	
}

void ep_HingeJoint::SetMaxForce(double _maxforce) {
	if(_maxforce<0.0) _maxforce = 0.0;
	if(maxforce!=_maxforce) {
		maxforce = _maxforce;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed maximum force of hinge joint %lu in world %lu.", id, world->id);
#endif
}

bool ep_HingeJoint::SetMotor(double _maxmotortorque, double _motorvel) {
#if EP_COMPILE_DEBUGCHECKS
	if(body1->norotation && body2->norotation) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set motor of hinge joint %lu in world %lu, both bodies can't rotate.", id, world->id);
#endif
		return false;
	}
#endif
	if(_maxmotortorque<0.0) _maxmotortorque = 0.0;
	if(maxmotortorque!=_maxmotortorque || motorvel!=_motorvel) {
		maxmotortorque = _maxmotortorque;
		motorvel = _motorvel;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed motor settings of hinge joint %lu in world %lu.", id, world->id);
#endif
	return true;
}

void ep_HingeJoint::SetLimitSettings(double contact_threshold, double velocity_threshold) {
	if(limit_contact_threshold!=contact_threshold || limit_velocity_threshold!=velocity_threshold) {
		limit_contact_threshold = contact_threshold;
		limit_velocity_threshold = velocity_threshold;
		if(limit1_maxtorque!=0.0 || limit2_maxtorque!=0.0) {
			body1->Awake(false, false);
			body2->Awake(false, false);
		}
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed limit settings of hinge joint %lu in world %lu.", id, world->id);
#endif
}

bool ep_HingeJoint::SetLowerLimit(double maxlimittorque, double rotation, double restitution, double velocity) {
#if EP_COMPILE_DEBUGCHECKS
	if(body1->norotation && body2->norotation) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set lower limit of hinge joint %lu in world %lu, both bodies can't rotate.", id, world->id);
#endif
		return false;
	}
#endif
	if(maxlimittorque<0.0) maxlimittorque = 0.0;
	if(limit1_maxtorque!=maxlimittorque || limit1_rot!=rotation || limit1_restitution!=restitution || limit1_velocity!=velocity) {
		limit1_maxtorque = maxlimittorque;
		limit1_rot = rotation;
		limit1_restitution = restitution;
		limit1_velocity = velocity;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed lower limit of hinge joint %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_HingeJoint::SetUpperLimit(double maxlimittorque, double rotation, double restitution, double velocity) {
#if EP_COMPILE_DEBUGCHECKS
	if(body1->norotation && body2->norotation) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set upper limit of hinge joint %lu in world %lu, both bodies can't rotate.", id, world->id);
#endif
		return false;
	}
#endif
	if(maxlimittorque<0.0) maxlimittorque = 0.0;
	if(limit2_maxtorque!=maxlimittorque || limit2_rot!=rotation || limit2_restitution!=restitution || limit2_velocity!=velocity) {
		limit2_maxtorque = maxlimittorque;
		limit2_rot = rotation;
		limit2_restitution = restitution;
		limit2_velocity = velocity;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed upper limit of hinge joint %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_HingeJoint::SetLowerSpring(double k, double rotation, double damping) {
#if EP_COMPILE_DEBUGCHECKS
	if(body1->norotation && body2->norotation) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set lower spring of hinge joint %lu in world %lu, both bodies can't rotate.", id, world->id);
#endif
		return false;
	}
#endif
	if(k<0.0) k = 0.0;
	if(damping<0.0) damping = 0.0;
	if(spring1_k!=k || spring1_rotation!=rotation || spring1_damping!=damping) {
		spring1_k = k;
		spring1_rotation = rotation;
		spring1_damping = damping;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed lower spring of hinge joint %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_HingeJoint::SetUpperSpring(double k, double rotation, double damping) {
#if EP_COMPILE_DEBUGCHECKS
	if(body1->norotation && body2->norotation) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set upper spring of hinge joint %lu in world %lu, both bodies can't rotate.", id, world->id);
#endif
		return false;
	}
#endif
	if(k<0.0) k = 0.0;
	if(damping<0.0) damping = 0.0;
	if(spring2_k!=k || spring2_rotation!=rotation || spring2_damping!=damping) {
		spring2_k = k;
		spring2_rotation = rotation;
		spring2_damping = damping;
		body1->Awake(false, false);
		body2->Awake(false, false);
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed upper spring of hinge joint %lu in world %lu.", id, world->id);
#endif
	return true;
}

