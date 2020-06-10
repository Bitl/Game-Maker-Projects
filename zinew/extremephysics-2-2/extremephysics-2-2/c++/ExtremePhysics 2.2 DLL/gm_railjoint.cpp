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
 * File: gm_railjoint.cpp
 * Wrapper for ep_RailJoint.
 */

#include "gm.h"

gmexport double ep_railjoint_create(double world_id, double body1_id, double body2_id, double x1, double y1, double x2a, double y2a, double x2b, double y2b) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body1, *body2;
	if((body1 = world->FindBody(gm_cast<unsigned long>(body1_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body1_id), world->GetID());
		return 0;
	}
	if((body2 = world->FindBody(gm_cast<unsigned long>(body2_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body2_id), world->GetID());
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->CreateRailJoint(body1, body2, x1, y1, x2a, y2a, x2b, y2b))==NULL) {
		return 0;
	}
	((gmuserdata*)(railjoint->GetUserData()))->Clear();
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_railjoint_destroy(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_destroy: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	world->DestroyRailJoint(railjoint);
	return 1;
}

gmexport double ep_railjoint_exists(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindRailJoint(gm_cast<unsigned long>(railjoint_id))!=NULL)? 1 : 0;
}

gmexport double ep_railjoint_set_max_normal_force(double world_id, double railjoint_id, double maxnormalforce) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_max_normal_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_max_normal_force: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetMaxNormalForce(maxnormalforce);
	return 1;
}

gmexport double ep_railjoint_set_motor(double world_id, double railjoint_id, double maxmotorforce, double motorvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_motor: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_motor: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetMotor(maxmotorforce, motorvel);
	return 1;
}

gmexport double ep_railjoint_set_limit_settings(double world_id, double railjoint_id, double contact_threshold, double velocity_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_limit_settings: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_limit_settings: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetLimitSettings(contact_threshold, velocity_threshold);
	return 1;
}

gmexport double ep_railjoint_set_lower_limit(double world_id, double railjoint_id, double maxlimitforce, double position, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_lower_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_lower_limit: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetLowerLimit(maxlimitforce, position, restitution, velocity);
	return 1;
}

gmexport double ep_railjoint_set_upper_limit(double world_id, double railjoint_id, double maxlimitforce, double position, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_upper_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_upper_limit: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetUpperLimit(maxlimitforce, position, restitution, velocity);
	return 1;
}

gmexport double ep_railjoint_set_lower_spring(double world_id, double railjoint_id, double k, double position, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_lower_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_lower_spring: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetLowerSpring(k, position, damping);
	return 1;
}

gmexport double ep_railjoint_set_upper_spring(double world_id, double railjoint_id, double k, double position, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_upper_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_upper_spring: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint->SetUpperSpring(k, position, damping);
	return 1;
}

gmexport double ep_railjoint_get_body1(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_body1: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_body1: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	if(railjoint->GetBody1()==NULL) {
		return 0;
	}
	railjoint->GetBody1()->CacheID();
	return railjoint->GetBody1()->GetID();
}

gmexport double ep_railjoint_get_body2(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_body2: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_body2: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	if(railjoint->GetBody2()==NULL) {
		return 0;
	}
	railjoint->GetBody2()->CacheID();
	return railjoint->GetBody2()->GetID();
}

gmexport double ep_railjoint_get_position(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_position: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_position: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	return railjoint->GetPosition();
}

gmexport double ep_railjoint_get_normal_force(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_normal_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_normal_force: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	return railjoint->GetNormalForce();
}

gmexport double ep_railjoint_get_motor_force(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_motor_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_motor_force: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	return railjoint->GetMotorForce();
}

gmexport double ep_railjoint_get_limit_force(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_limit_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_limit_force: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	return railjoint->GetLimitForce();
}

gmexport double ep_railjoint_previous(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_previous: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	if(railjoint->GetPrevious()==NULL) {
		return 0;
	}
	railjoint->GetPrevious()->CacheID();
	return railjoint->GetPrevious()->GetID();
}

gmexport double ep_railjoint_next(double world_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_next: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	if(railjoint->GetNext()==NULL) {
		return 0;
	}
	railjoint->GetNext()->CacheID();
	return railjoint->GetNext()->GetID();
}

gmexport double ep_railjoint_set_uservar(double world_id, double railjoint_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_set_uservar: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of rail joint %lu in world %lu, index is out of range.", railjoint->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(railjoint->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_railjoint_get_uservar(double world_id, double railjoint_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_railjoint_get_uservar: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of rail joint %lu in world %lu, index is out of range.", railjoint->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(railjoint->GetUserData()))->var[i];
}

