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
 * File: gm_distancejoint.cpp
 * Wrapper for ep_DistanceJoint.
 */

#include "gm.h"

gmexport double ep_distancejoint_create(double world_id, double body1_id, double body2_id, double x1, double y1, double x2, double y2) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body1, *body2;
	if((body1 = world->FindBody(gm_cast<unsigned long>(body1_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body1_id), world->GetID());
		return 0;
	}
	if((body2 = world->FindBody(gm_cast<unsigned long>(body2_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body2_id), world->GetID());
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->CreateDistanceJoint(body1, body2, x1, y1, x2, y2))==NULL) {
		return 0;
	}
	((gmuserdata*)(distancejoint->GetUserData()))->Clear();
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_distancejoint_destroy(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_destroy: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	world->DestroyDistanceJoint(distancejoint);
	return 1;
}

gmexport double ep_distancejoint_exists(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id))!=NULL)? 1 : 0;
}

gmexport double ep_distancejoint_set_motor(double world_id, double distancejoint_id, double maxmotorforce, double motorvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_motor: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_motor: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint->SetMotor(maxmotorforce, motorvel);
	return 1;
}

gmexport double ep_distancejoint_set_limit_settings(double world_id, double distancejoint_id, double contact_threshold, double velocity_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_limit_settings: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_limit_settings: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint->SetLimitSettings(contact_threshold, velocity_threshold);
	return 1;
}

gmexport double ep_distancejoint_set_lower_limit(double world_id, double distancejoint_id, double maxlimitforce, double distance, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_lower_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_lower_limit: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint->SetLowerLimit(maxlimitforce, distance, restitution, velocity);
	return 1;
}

gmexport double ep_distancejoint_set_upper_limit(double world_id, double distancejoint_id, double maxlimitforce, double distance, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_upper_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_upper_limit: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint->SetUpperLimit(maxlimitforce, distance, restitution, velocity);
	return 1;
}

gmexport double ep_distancejoint_set_lower_spring(double world_id, double distancejoint_id, double k, double distance, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_lower_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_lower_spring: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint->SetLowerSpring(k, distance, damping);
	return 1;
}

gmexport double ep_distancejoint_set_upper_spring(double world_id, double distancejoint_id, double k, double distance, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_upper_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_upper_spring: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint->SetUpperSpring(k, distance, damping);
	return 1;
}

gmexport double ep_distancejoint_get_body1(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_body1: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_body1: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	if(distancejoint->GetBody1()==NULL) {
		return 0;
	}
	distancejoint->GetBody1()->CacheID();
	return distancejoint->GetBody1()->GetID();
}

gmexport double ep_distancejoint_get_body2(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_body2: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_body2: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	if(distancejoint->GetBody2()==NULL) {
		return 0;
	}
	distancejoint->GetBody2()->CacheID();
	return distancejoint->GetBody2()->GetID();
}

gmexport double ep_distancejoint_get_distance(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_distance: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_distance: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	return distancejoint->GetDistance();
}

gmexport double ep_distancejoint_get_motor_force(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_motor_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_motor_force: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	return distancejoint->GetMotorForce();
}

gmexport double ep_distancejoint_get_limit_force(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_limit_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_limit_force: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	return distancejoint->GetLimitForce();
}

gmexport double ep_distancejoint_previous(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_previous: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	if(distancejoint->GetPrevious()==NULL) {
		return 0;
	}
	distancejoint->GetPrevious()->CacheID();
	return distancejoint->GetPrevious()->GetID();
}

gmexport double ep_distancejoint_next(double world_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_next: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	if(distancejoint->GetNext()==NULL) {
		return 0;
	}
	distancejoint->GetNext()->CacheID();
	return distancejoint->GetNext()->GetID();
}

gmexport double ep_distancejoint_set_uservar(double world_id, double distancejoint_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_set_uservar: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of distance joint %lu in world %lu, index is out of range.", distancejoint->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(distancejoint->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_distancejoint_get_uservar(double world_id, double distancejoint_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_distancejoint_get_uservar: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of distance joint %lu in world %lu, index is out of range.", distancejoint->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(distancejoint->GetUserData()))->var[i];
}

