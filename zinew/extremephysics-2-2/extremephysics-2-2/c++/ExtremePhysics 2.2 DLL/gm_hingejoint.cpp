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
 * File: gm_hingejoint.cpp
 * Wrapper for ep_HingeJoint.
 */

#include "gm.h"

gmexport double ep_hingejoint_create(double world_id, double body1_id, double body2_id, double x1, double y1, double x2, double y2, double referencerotation) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body1, *body2;
	if((body1 = world->FindBody(gm_cast<unsigned long>(body1_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body1_id), world->GetID());
		return 0;
	}
	if((body2 = world->FindBody(gm_cast<unsigned long>(body2_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body2_id), world->GetID());
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->CreateHingeJoint(body1, body2, x1, y1, x2, y2, referencerotation))==NULL) {
		return 0;
	}
	((gmuserdata*)(hingejoint->GetUserData()))->Clear();
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_hingejoint_destroy(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_destroy: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	world->DestroyHingeJoint(hingejoint);
	return 1;
}

gmexport double ep_hingejoint_exists(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id))!=NULL)? 1 : 0;
}

gmexport double ep_hingejoint_set_max_force(double world_id, double hingejoint_id, double maxforce) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_max_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_destroy: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	hingejoint->SetMaxForce(maxforce);
	return 1;
}

gmexport double ep_hingejoint_set_motor(double world_id, double hingejoint_id, double maxmotortorque, double motorvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_motor: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_motor: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return (hingejoint->SetMotor(maxmotortorque, motorvel))? 1 : 0;
}

gmexport double ep_hingejoint_set_limit_settings(double world_id, double hingejoint_id, double contact_threshold, double velocity_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_limit_settings: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_limit_settings: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	hingejoint->SetLimitSettings(contact_threshold, velocity_threshold);
	return 1;
}

gmexport double ep_hingejoint_set_lower_limit(double world_id, double hingejoint_id, double maxlimittorque, double rotation, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_lower_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_lower_limit: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return (hingejoint->SetLowerLimit(maxlimittorque, rotation, restitution, velocity))? 1 : 0;
}

gmexport double ep_hingejoint_set_upper_limit(double world_id, double hingejoint_id, double maxlimittorque, double rotation, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_upper_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_upper_limit: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return (hingejoint->SetUpperLimit(maxlimittorque, rotation, restitution, velocity))? 1 : 0;
}

gmexport double ep_hingejoint_set_lower_spring(double world_id, double hingejoint_id, double k, double rotation, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_lower_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_lower_spring: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return (hingejoint->SetLowerSpring(k, rotation, damping))? 1 : 0;
}

gmexport double ep_hingejoint_set_upper_spring(double world_id, double hingejoint_id, double k, double rotation, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_upper_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_upper_spring: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return (hingejoint->SetUpperSpring(k, rotation, damping))? 1 : 0;
}

gmexport double ep_hingejoint_get_body1(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_body1: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_body1: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	if(hingejoint->GetBody1()==NULL) {
		return 0;
	}
	hingejoint->GetBody1()->CacheID();
	return hingejoint->GetBody1()->GetID();
}

gmexport double ep_hingejoint_get_body2(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_body2: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_body2: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	if(hingejoint->GetBody2()==NULL) {
		return 0;
	}
	hingejoint->GetBody2()->CacheID();
	return hingejoint->GetBody2()->GetID();
}

gmexport double ep_hingejoint_get_rotation(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_rotation: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_rotation: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return hingejoint->GetRotation();
}

gmexport double ep_hingejoint_get_xforce(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_xforce: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_xforce: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return hingejoint->GetXForce();
}

gmexport double ep_hingejoint_get_yforce(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_yforce: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_yforce: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return hingejoint->GetYForce();
}

gmexport double ep_hingejoint_get_motor_torque(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_motortorque: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_motortorque: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return hingejoint->GetMotorTorque();
}

gmexport double ep_hingejoint_get_limit_torque(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_limittorque: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_limittorque: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	return hingejoint->GetLimitTorque();
}

gmexport double ep_hingejoint_previous(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_previous: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	if(hingejoint->GetPrevious()==NULL) {
		return 0;
	}
	hingejoint->GetPrevious()->CacheID();
	return hingejoint->GetPrevious()->GetID();
}

gmexport double ep_hingejoint_next(double world_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_next: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	if(hingejoint->GetNext()==NULL) {
		return 0;
	}
	hingejoint->GetNext()->CacheID();
	return hingejoint->GetNext()->GetID();
}

gmexport double ep_hingejoint_set_uservar(double world_id, double hingejoint_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_set_uservar: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of hinge joint %lu in world %lu, index is out of range.", hingejoint->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(hingejoint->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_hingejoint_get_uservar(double world_id, double hingejoint_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_hingejoint_get_uservar: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of hinge joint %lu in world %lu, index is out of range.", hingejoint->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(hingejoint->GetUserData()))->var[i];
}

