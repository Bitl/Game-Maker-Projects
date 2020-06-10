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
 * File: gm_sliderjoint.cpp
 * Wrapper for ep_SliderJoint.
 */

#include "gm.h"

gmexport double ep_sliderjoint_create(double world_id, double body1_id, double body2_id, double x1, double y1, double x2a, double y2a, double x2b, double y2b, double rotation) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body1, *body2;
	if((body1 = world->FindBody(gm_cast<unsigned long>(body1_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body1_id), world->GetID());
		return 0;
	}
	if((body2 = world->FindBody(gm_cast<unsigned long>(body2_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_create: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body2_id), world->GetID());
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->CreateSliderJoint(body1, body2, x1, y1, x2a, y2a, x2b, y2b, rotation))==NULL) {
		return 0;
	}
	((gmuserdata*)(sliderjoint->GetUserData()))->Clear();
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

gmexport double ep_sliderjoint_destroy(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_destroy: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	world->DestroySliderJoint(sliderjoint);
	return 1;
}

gmexport double ep_sliderjoint_exists(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id))!=NULL)? 1 : 0;
}

gmexport double ep_sliderjoint_set_max_combined_force(double world_id, double sliderjoint_id, double maxnormalforce, double torqueradius) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_max_combined_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_max_combined_force: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetMaxCombinedForce(maxnormalforce, torqueradius);
	return 1;
}

gmexport double ep_sliderjoint_set_motor(double world_id, double sliderjoint_id, double maxmotorforce, double motorvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_motor: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_motor: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetMotor(maxmotorforce, motorvel);
	return 1;
}

gmexport double ep_sliderjoint_set_limit_settings(double world_id, double sliderjoint_id, double contact_threshold, double velocity_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_limit_settings: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_limit_settings: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetLimitSettings(contact_threshold, velocity_threshold);
	return 1;
}

gmexport double ep_sliderjoint_set_lower_limit(double world_id, double sliderjoint_id, double maxlimitforce, double position, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_lower_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_lower_limit: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetLowerLimit(maxlimitforce, position, restitution, velocity);
	return 1;
}

gmexport double ep_sliderjoint_set_upper_limit(double world_id, double sliderjoint_id, double maxlimitforce, double position, double restitution, double velocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_upper_limit: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_upper_limit: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetUpperLimit(maxlimitforce, position, restitution, velocity);
	return 1;
}

gmexport double ep_sliderjoint_set_lower_spring(double world_id, double sliderjoint_id, double k, double position, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_lower_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_lower_spring: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetLowerSpring(k, position, damping);
	return 1;
}

gmexport double ep_sliderjoint_set_upper_spring(double world_id, double sliderjoint_id, double k, double position, double damping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_upper_spring: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_upper_spring: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint->SetUpperSpring(k, position, damping);
	return 1;
}

gmexport double ep_sliderjoint_get_body1(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_body1: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_body1: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	if(sliderjoint->GetBody1()==NULL) {
		return 0;
	}
	sliderjoint->GetBody1()->CacheID();
	return sliderjoint->GetBody1()->GetID();
}

gmexport double ep_sliderjoint_get_body2(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_body2: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_body2: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	if(sliderjoint->GetBody2()==NULL) {
		return 0;
	}
	sliderjoint->GetBody2()->CacheID();
	return sliderjoint->GetBody2()->GetID();
}

gmexport double ep_sliderjoint_get_position(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_position: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_position: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	return sliderjoint->GetPosition();
}

gmexport double ep_sliderjoint_get_normal_force(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_normal_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_normal_force: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	return sliderjoint->GetNormalForce();
}

gmexport double ep_sliderjoint_get_torque(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_torque: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_torque: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	return sliderjoint->GetTorque();
}

gmexport double ep_sliderjoint_get_combined_force(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_combined_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_combined_force: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	return sliderjoint->GetCombinedForce();
}

gmexport double ep_sliderjoint_get_motor_force(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_motor_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_motor_force: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	return sliderjoint->GetMotorForce();
}

gmexport double ep_sliderjoint_get_limit_force(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_limit_force: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_limit_force: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	return sliderjoint->GetLimitForce();
}

gmexport double ep_sliderjoint_previous(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_previous: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	if(sliderjoint->GetPrevious()==NULL) {
		return 0;
	}
	sliderjoint->GetPrevious()->CacheID();
	return sliderjoint->GetPrevious()->GetID();
}

gmexport double ep_sliderjoint_next(double world_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_next: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	if(sliderjoint->GetNext()==NULL) {
		return 0;
	}
	sliderjoint->GetNext()->CacheID();
	return sliderjoint->GetNext()->GetID();
}

gmexport double ep_sliderjoint_set_uservar(double world_id, double sliderjoint_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_set_uservar: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of slider joint %lu in world %lu, index is out of range.", sliderjoint->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(sliderjoint->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_sliderjoint_get_uservar(double world_id, double sliderjoint_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_sliderjoint_get_uservar: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of slider joint %lu in world %lu, index is out of range.", sliderjoint->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(sliderjoint->GetUserData()))->var[i];
}

