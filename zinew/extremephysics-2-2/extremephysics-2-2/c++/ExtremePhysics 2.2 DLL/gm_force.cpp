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
 * File: gm_force.cpp
 * Wrapper for ep_Force.
 */

#include "gm.h"

gmexport double ep_force_create(double world_id, double body_id, double x, double y, double local, double ignoremass) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->CreateForce(x, y, gm_cast<bool>(local), gm_cast<bool>(ignoremass)))==NULL) {
		return 0;
	}
	((gmuserdata*)(force->GetUserData()))->Clear();
	force->CacheID();
	return force->GetID();
}

gmexport double ep_force_destroy(double world_id, double body_id, double force_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->FindForce(gm_cast<unsigned long>(force_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_destroy: Force %lu doesn't exist in world %lu.", gm_cast<unsigned long>(force_id), world->GetID());
		return 0;
	}
	body->DestroyForce(force);
	return 1;
}

gmexport double ep_force_exists(double world_id, double body_id, double force_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->FindForce(gm_cast<unsigned long>(force_id))!=NULL)? 1 : 0;
}

gmexport double ep_force_set_force(double world_id, double body_id, double force_id, double xforce, double yforce, double torque, double awake) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_set_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_set_force: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->FindForce(gm_cast<unsigned long>(force_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_set_force: Force %lu doesn't exist in world %lu.", gm_cast<unsigned long>(force_id), world->GetID());
		return 0;
	}
	force->SetForce(xforce, yforce, torque, gm_cast<bool>(awake));
	return 1;
}

gmexport double ep_force_previous(double world_id, double body_id, double force_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_previous: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->FindForce(gm_cast<unsigned long>(force_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_previous: Force %lu doesn't exist in world %lu.", gm_cast<unsigned long>(force_id), world->GetID());
		return 0;
	}
	if(force->GetPrevious()==NULL) {
		return 0;
	}
	force->GetPrevious()->CacheID();
	return force->GetPrevious()->GetID();
}

gmexport double ep_force_next(double world_id, double body_id, double force_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_next: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->FindForce(gm_cast<unsigned long>(force_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_next: Force %lu doesn't exist in world %lu.", gm_cast<unsigned long>(force_id), world->GetID());
		return 0;
	}
	if(force->GetNext()==NULL) {
		return 0;
	}
	force->GetNext()->CacheID();
	return force->GetNext()->GetID();
}

gmexport double ep_force_set_uservar(double world_id, double body_id, double force_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_set_uservar: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->FindForce(gm_cast<unsigned long>(force_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_set_uservar: Force %lu doesn't exist in world %lu.", gm_cast<unsigned long>(force_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of force of body %lu %lu in world %lu, index is out of range.", force->GetID(), body->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(force->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_force_get_uservar(double world_id, double body_id, double force_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_get_uservar: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Force *force;
	if((force = body->FindForce(gm_cast<unsigned long>(force_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_force_get_uservar: Force %lu doesn't exist in world %lu.", gm_cast<unsigned long>(force_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of force %lu of body %lu in world %lu, index is out of range.", force->GetID(), body->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(force->GetUserData()))->var[i];
}

