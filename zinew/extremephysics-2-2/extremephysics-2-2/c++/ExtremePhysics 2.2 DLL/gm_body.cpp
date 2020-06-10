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
 * File: gm_body.cpp
 * Wrapper for ep_Body.
 */

#include "gm.h"

gmexport double ep_body_create_static(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_create_static: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->CreateStaticBody())==NULL) {
		return 0;
	}
	((gmuserdata*)(body->GetUserData()))->Clear();
	body->CacheID();
	return body->GetID();
}

gmexport double ep_body_create_dynamic(double world_id, double norotation) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_create_dynamic: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->CreateDynamicBody(gm_cast<bool>(norotation)))==NULL) {
		return 0;
	}
	((gmuserdata*)(body->GetUserData()))->Clear();
	body->CacheID();
	return body->GetID();
}

gmexport double ep_body_destroy(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_destroy: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	world->DestroyBody(body);
	return 1;
}

gmexport double ep_body_exists(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindBody(gm_cast<unsigned long>(body_id))!=NULL)? 1 : 0;
}

gmexport double ep_body_get_first_hingejoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_hingejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_hingejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_HingeJoint *hingejoint = body->GetFirstHingeJoint();
	if(hingejoint==NULL) {
		return 0;
	}
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_body_get_last_hingejoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_hingejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_hingejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_HingeJoint *hingejoint = body->GetLastHingeJoint();
	if(hingejoint==NULL) {
		return 0;
	}
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_body_get_previous_hingejoint(double world_id, double body_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_hingejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_hingejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_hingejoint: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	hingejoint = body->GetPreviousHingeJoint(hingejoint);
	if(hingejoint==NULL) {
		return 0;
	}
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_body_get_next_hingejoint(double world_id, double body_id, double hingejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_hingejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_hingejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_HingeJoint *hingejoint;
	if((hingejoint = world->FindHingeJoint(gm_cast<unsigned long>(hingejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_hingejoint: Hinge joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(hingejoint_id), world->GetID());
		return 0;
	}
	hingejoint = body->GetNextHingeJoint(hingejoint);
	if(hingejoint==NULL) {
		return 0;
	}
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_body_get_first_distancejoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_distancejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_distancejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_DistanceJoint *distancejoint = body->GetFirstDistanceJoint();
	if(distancejoint==NULL) {
		return 0;
	}
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_body_get_last_distancejoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_distancejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_distancejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_DistanceJoint *distancejoint = body->GetLastDistanceJoint();
	if(distancejoint==NULL) {
		return 0;
	}
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_body_get_previous_distancejoint(double world_id, double body_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_distancejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_distancejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_distancejoint: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint = body->GetPreviousDistanceJoint(distancejoint);
	if(distancejoint==NULL) {
		return 0;
	}
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_body_get_next_distancejoint(double world_id, double body_id, double distancejoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_distancejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_distancejoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_DistanceJoint *distancejoint;
	if((distancejoint = world->FindDistanceJoint(gm_cast<unsigned long>(distancejoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_distancejoint: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(distancejoint_id), world->GetID());
		return 0;
	}
	distancejoint = body->GetNextDistanceJoint(distancejoint);
	if(distancejoint==NULL) {
		return 0;
	}
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_body_get_first_railjoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_railjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_railjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_RailJoint *railjoint = body->GetFirstRailJoint();
	if(railjoint==NULL) {
		return 0;
	}
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_body_get_last_railjoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_railjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_railjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_RailJoint *railjoint = body->GetLastRailJoint();
	if(railjoint==NULL) {
		return 0;
	}
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_body_get_previous_railjoint(double world_id, double body_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_railjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_railjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_railjoint: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint = body->GetPreviousRailJoint(railjoint);
	if(railjoint==NULL) {
		return 0;
	}
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_body_get_next_railjoint(double world_id, double body_id, double railjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_railjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_railjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_RailJoint *railjoint;
	if((railjoint = world->FindRailJoint(gm_cast<unsigned long>(railjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_railjoint: Rail joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(railjoint_id), world->GetID());
		return 0;
	}
	railjoint = body->GetNextRailJoint(railjoint);
	if(railjoint==NULL) {
		return 0;
	}
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_body_get_first_sliderjoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_sliderjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_first_sliderjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_SliderJoint *sliderjoint = body->GetFirstSliderJoint();
	if(sliderjoint==NULL) {
		return 0;
	}
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

gmexport double ep_body_get_last_sliderjoint(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_sliderjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_last_sliderjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_SliderJoint *sliderjoint = body->GetLastSliderJoint();
	if(sliderjoint==NULL) {
		return 0;
	}
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

gmexport double ep_body_get_previous_sliderjoint(double world_id, double body_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_sliderjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_sliderjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_previous_railjoint: Distance joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint = body->GetPreviousSliderJoint(sliderjoint);
	if(sliderjoint==NULL) {
		return 0;
	}
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

gmexport double ep_body_get_next_sliderjoint(double world_id, double body_id, double sliderjoint_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_sliderjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_sliderjoint: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_SliderJoint *sliderjoint;
	if((sliderjoint = world->FindSliderJoint(gm_cast<unsigned long>(sliderjoint_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_next_sliderjoint: Slider joint %lu doesn't exist in world %lu.", gm_cast<unsigned long>(sliderjoint_id), world->GetID());
		return 0;
	}
	sliderjoint = body->GetNextSliderJoint(sliderjoint);
	if(sliderjoint==NULL) {
		return 0;
	}
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

//JOINTS//

gmexport double ep_body_calculate_mass(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_calculate_mass: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_calculate_mass: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->CalculateMass())? 1 : 0;
}

gmexport double ep_body_set_mass(double world_id, double body_id, double mass) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_mass: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_mass: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetMass(mass))? 1 : 0;
}

gmexport double ep_body_set_inertia(double world_id, double body_id, double inertia) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_inertia: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_inertia: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetInertia(inertia))? 1 : 0;
}

gmexport double ep_body_set_center(double world_id, double body_id, double localx, double localy, double updateinertia) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetCenter(localx, localy, gm_cast<bool>(updateinertia)));
}

gmexport double ep_body_set_position(double world_id, double body_id, double x, double y, double rot) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	body->SetPosition(x, y, rot);
	return 1;
}

gmexport double ep_body_set_position_center(double world_id, double body_id, double x, double y, double rot) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	body->SetPositionCenter(x, y, rot);
	return 1;
}

gmexport double ep_body_set_position_local_point(double world_id, double body_id, double x, double y, double rot, double localx, double localy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position_local_point: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position_local_point: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	body->SetPositionLocalPoint(x, y, rot, localx, localy);
	return 1;
}

gmexport double ep_body_set_velocity_center(double world_id, double body_id, double xvel, double yvel, double rotvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_velocity_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_velocity_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetVelocityCenter(xvel, yvel, rotvel))? 1 : 0;
}

gmexport double ep_body_set_velocity_local_point(double world_id, double body_id, double xvel, double yvel, double rotvel, double localx, double localy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position_local_point: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_position_local_point: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetVelocityLocalPoint(xvel, yvel, rotvel, localx, localy))? 1 : 0;
}

gmexport double ep_body_set_max_velocity(double world_id, double body_id, double maxvel, double maxrotvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_max_velocity: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_max_velocity: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetMaxVelocity(maxvel, maxrotvel))? 1 : 0;
}

gmexport double ep_body_set_gravity(double world_id, double body_id, double gravity_x, double gravity_y) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_gravity: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_gravity: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetGravity(gravity_x, gravity_y))? 1 : 0;
}

gmexport double ep_body_set_damping(double world_id, double body_id, double damping, double rotdamping) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_damping: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_damping: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetDamping(damping, rotdamping))? 1 : 0;
}

gmexport double ep_body_store_impulses(double world_id, double body_id, double storecontactimpulses, double storejointimpulses) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_store_impulses: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_store_impulses: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	body->StoreImpulses(gm_cast<bool>(storecontactimpulses), gm_cast<bool>(storejointimpulses));
	return 1;
}

gmexport double ep_body_set_sleeping(double world_id, double body_id, double sleepstable, double sleepoutofview) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_sleeping: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_sleeping: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->SetSleeping(gm_cast<bool>(sleepstable), gm_cast<bool>(sleepoutofview)))? 1 : 0;
}

gmexport double ep_body_collision_test_box(double world_id, double body_id, double w, double h, double x, double y, double rot, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_box: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_box: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CollisionTestBox(w, h, x, y, rot, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_body_collision_test_line(double world_id, double body_id, double x1, double y1, double x2, double y2, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_line: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_line: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CollisionTestLine(x1, y1, x2, y2, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_body_collision_test_circle(double world_id, double body_id, double r, double x, double y, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_circle: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_circle: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CollisionTestCircle(r, x, y, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_body_collision_test_polygon(double world_id, double body_id, double polygon_id, double x, double y, double rot, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_polygon: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_polygon: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return body->CollisionTestPolygon(polygon, x, y, rot, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_body_ray_cast(double world_id, double body_id, double x, double y, double vx, double vy, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_ray_cast: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_ray_cast: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->RayCast(x, y, vx, vy, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_body_apply_impulse(double world_id, double body_id, double localx, double localy, double xforce, double yforce, double torque, double local, double ignoremass, double awake) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_apply_impulse: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_apply_impulse: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	body->ApplyImpulse(localx, localy, xforce, yforce, torque, gm_cast<bool>(local), gm_cast<bool>(ignoremass), gm_cast<bool>(awake));
	return 1;
}

gmexport double ep_body_apply_impulse_relative(double world_id, double body_id, double relativex, double relativey, double xforce, double yforce, double torque, double ignoremass, double awake) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_apply_impulse_relative: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_apply_impulse_relative: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	body->ApplyImpulseRelative(relativex, relativey, xforce, yforce, torque, gm_cast<bool>(ignoremass), gm_cast<bool>(awake));
	return 1;
}

gmexport double ep_body_get_mass(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_mass: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_mass: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetMass();
}

gmexport double ep_body_get_inertia(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_inertia: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_inertia: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetInertia();
}

gmexport double ep_body_get_center_of_mass_x(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_center_of_mass_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_center_of_mass_x: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetCenterOfMassX();
}

gmexport double ep_body_get_center_of_mass_y(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_center_of_mass_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_center_of_mass_y: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetCenterOfMassY();
}

gmexport double ep_body_get_x(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_x: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetX();
}

gmexport double ep_body_get_y(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_y: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetY();
}

gmexport double ep_body_get_x_center(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_x_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_x_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetXCenter();
}

gmexport double ep_body_get_y_center(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_y_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_y_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetYCenter();
}

gmexport double ep_body_get_rot(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_rot: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_rot: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetRot();
}

gmexport double ep_body_get_xvel_center(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_xvel_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_xvel_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetXVelCenter();
}

gmexport double ep_body_get_yvel_center(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_yvel_center: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_yvel_center: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetYVelCenter();
}

gmexport double ep_body_get_xvel_local_point(double world_id, double body_id, double localx, double localy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_xvel_local_point: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_xvel_local_point: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetXVelLocalPoint(localx, localy);
}

gmexport double ep_body_get_yvel_local_point(double world_id, double body_id, double localx, double localy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_yvel_local_point: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_yvel_local_point: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetYVelLocalPoint(localx, localy);
}

gmexport double ep_body_get_rotvel(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_rotvel: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_rotvel: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetRotVel();
}

gmexport double ep_body_is_static(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_is_static: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_is_static: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->IsStatic())? 1 : 0;
}

gmexport double ep_body_is_norotation(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_is_norotation: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_is_norotation: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->IsNoRotation())? 1 : 0;
}

gmexport double ep_body_is_sleeping(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_is_sleeping: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_is_sleeping: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->IsSleeping())? 1 : 0;
}

gmexport double ep_body_stable_timer(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_stable_timer: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_stable_timer: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->StableTimer();
}

gmexport double ep_body_out_of_view_timer(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_out_of_view_timer: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_out_of_view_timer: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->OutOfViewTimer();
}

gmexport double ep_body_coord_local_to_world_x(double world_id, double body_id, double localx, double localy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_local_to_world_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_local_to_world_x: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CoordLocalToWorldX(localx, localy);
}

gmexport double ep_body_coord_local_to_world_y(double world_id, double body_id, double localx, double localy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_local_to_world_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_local_to_world_y: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CoordLocalToWorldY(localx, localy);
}

gmexport double ep_body_coord_world_to_local_x(double world_id, double body_id, double worldx, double worldy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_world_to_local_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_world_to_local_x: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CoordWorldToLocalX(worldx, worldy);
}

gmexport double ep_body_coord_world_to_local_y(double world_id, double body_id, double worldx, double worldy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_world_to_local_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_coord_world_to_local_y: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->CoordWorldToLocalY(worldx, worldy);
}

gmexport double ep_body_vect_local_to_world_x(double world_id, double body_id, double vx, double vy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_local_to_world_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_local_to_world_x: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->VectLocalToWorldX(vx, vy);
}

gmexport double ep_body_vect_local_to_world_y(double world_id, double body_id, double vx, double vy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_local_to_world_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_local_to_world_y: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->VectLocalToWorldY(vx, vy);
}

gmexport double ep_body_vect_world_to_local_x(double world_id, double body_id, double vx, double vy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_world_to_local_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_world_to_local_x: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->VectWorldToLocalX(vx, vy);
}

gmexport double ep_body_vect_world_to_local_y(double world_id, double body_id, double vx, double vy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_world_to_local_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_vect_world_to_local_y: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->VectWorldToLocalY(vx, vy);
}

gmexport double ep_body_boxchain_begin(double world_id, double body_id, double vertexcount) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_begin: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_begin: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->BoxChainBegin(gm_cast<unsigned long>(vertexcount)))? 1 : 0;
}

gmexport double ep_body_boxchain_end(double world_id, double body_id, double circular, double ignorefirstlast, double width_top, double width_bottom, double density) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_end: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_end: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(!body->BoxChainEnd(gm_cast<bool>(circular), gm_cast<bool>(ignorefirstlast), width_top, width_bottom, density)) {
		return 0;
	}
	for(ep_Shape *shape = body->BoxChainGetFirst(); shape!=NULL ; shape = shape->GetNext()) {
		((gmuserdata*)(shape->GetUserData()))->Clear();
		if(shape==body->BoxChainGetLast()) break;
	}
	return 1;
}

gmexport double ep_body_boxchain_set_vertex(double world_id, double body_id, double index, double x, double y) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_set_vertex: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_set_vertex: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->BoxChainSetVertex(gm_cast<unsigned long>(index), x, y))? 1 : 0;
}

gmexport double ep_body_boxchain_get_first(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_get_first: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_get_first: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->BoxChainGetFirst()==NULL) {
		return 0;
	}
	body->BoxChainGetFirst()->CacheID();
	return body->BoxChainGetFirst()->GetID();
}

gmexport double ep_body_boxchain_get_last(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_get_last: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_boxchain_get_last: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->BoxChainGetLast()==NULL) {
		return 0;
	}
	body->BoxChainGetLast()->CacheID();
	return body->BoxChainGetLast()->GetID();
}

gmexport double ep_body_previous(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_previous: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->GetPrevious()==NULL) {
		return 0;
	}
	body->GetPrevious()->CacheID();
	return body->GetPrevious()->GetID();
}

gmexport double ep_body_next(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_next: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->GetNext()==NULL) {
		return 0;
	}
	body->GetNext()->CacheID();
	return body->GetNext()->GetID();
}

gmexport double ep_body_set_uservar(double world_id, double body_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_set_uservar: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of body %lu in world %lu, index is out of range.", body->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(body->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_body_get_uservar(double world_id, double body_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_get_uservar: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of body %lu in world %lu, index is out of range.", body->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(body->GetUserData()))->var[i];
}

gmexport double ep_body_first_shape(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_first_shape: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_first_shape: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->GetFirstShape()==NULL) {
		return 0;
	}
	body->GetFirstShape()->CacheID();
	return body->GetFirstShape()->GetID();
}

gmexport double ep_body_last_shape(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_last_shape: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_last_shape: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->GetLastShape()==NULL) {
		return 0;
	}
	body->GetLastShape()->CacheID();
	return body->GetLastShape()->GetID();
}

gmexport double ep_body_shape_count(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_shape_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_shape_count: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetShapeCount();
}

gmexport double ep_body_first_force(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_first_shape: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_first_shape: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->GetFirstForce()==NULL) {
		return 0;
	}
	body->GetFirstForce()->CacheID();
	return body->GetFirstForce()->GetID();
}

gmexport double ep_body_last_force(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_last_shape: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_last_shape: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	if(body->GetLastForce()==NULL) {
		return 0;
	}
	body->GetLastForce()->CacheID();
	return body->GetLastForce()->GetID();
}

gmexport double ep_body_force_count(double world_id, double body_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_shape_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_shape_count: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return body->GetForceCount();
}

