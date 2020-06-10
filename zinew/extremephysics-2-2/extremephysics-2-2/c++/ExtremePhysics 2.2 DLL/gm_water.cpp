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
 * File: gm_water.cpp
 * Wrapper for ep_Water.
 */

#include "gm.h"

gmexport double ep_water_create(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->CreateWater())==NULL) {
		return 0;
	}
	((gmuserdata*)(water->GetUserData()))->Clear();
	water->CacheID();
	return water->GetID();
}

gmexport double ep_water_destroy(double world_id, double water_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_destroy: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	world->DestroyWater(water);
	return 1;
}

gmexport double ep_water_exists(double world_id, double water_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindWater(gm_cast<unsigned long>(water_id))!=NULL)? 1 : 0;
}

gmexport double ep_water_set_parameters(double world_id, double water_id, double density, double lineardrag, double quadraticdrag, double xvel, double yvel, double gravity_x, double gravity_y) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_set_parameters: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_set_parameters: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	water->SetParameters(density, lineardrag, quadraticdrag, xvel, yvel, gravity_x, gravity_y);
	return 1;
}

gmexport double ep_water_set_rectangle(double world_id, double water_id, double x1, double y1, double x2, double y2) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_set_rectangle: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_set_rectangle: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	water->SetRectangle(x1, y1, x2, y2);
	return 1;
}

gmexport double ep_water_previous(double world_id, double water_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_previous: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	if(water->GetPrevious()==NULL) {
		return 0;
	}
	water->GetPrevious()->CacheID();
	return water->GetPrevious()->GetID();
}

gmexport double ep_water_next(double world_id, double water_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_next: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	if(water->GetNext()==NULL) {
		return 0;
	}
	water->GetNext()->CacheID();
	return water->GetNext()->GetID();
}

gmexport double ep_water_set_uservar(double world_id, double water_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_set_uservar: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of water %lu in world %lu, index is out of range.", water->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(water->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_water_get_uservar(double world_id, double water_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Water *water;
	if((water = world->FindWater(gm_cast<unsigned long>(water_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_water_get_uservar: Water %lu doesn't exist in world %lu.", gm_cast<unsigned long>(water_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of water %lu in world %lu, index is out of range.", water->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(water->GetUserData()))->var[i];
}

