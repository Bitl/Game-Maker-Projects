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
 * File: gm_view.cpp
 * Wrapper for ep_View.
 */

#include "gm.h"

gmexport double ep_view_create(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->CreateView())==NULL) {
		return 0;
	}
	((gmuserdata*)(view->GetUserData()))->Clear();
	view->CacheID();
	return view->GetID();
}

gmexport double ep_view_destroy(double world_id, double view_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->FindView(gm_cast<unsigned long>(view_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_destroy: View %lu doesn't exist in world %lu.", gm_cast<unsigned long>(view_id), world->GetID());
		return 0;
	}
	world->DestroyView(view);
	return 1;
}

gmexport double ep_view_exists(double world_id, double view_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindView(gm_cast<unsigned long>(view_id))!=NULL)? 1 : 0;
}

gmexport double ep_view_set_rectangle(double world_id, double view_id, double x1, double y1, double x2, double y2) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_set_rectangle: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->FindView(gm_cast<unsigned long>(view_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_set_rectangle: View %lu doesn't exist in world %lu.", gm_cast<unsigned long>(view_id), world->GetID());
		return 0;
	}
	view->SetRectangle(x1, y1, x2, y2);
	return 1;
}

gmexport double ep_view_previous(double world_id, double view_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->FindView(gm_cast<unsigned long>(view_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_previous: View %lu doesn't exist in world %lu.", gm_cast<unsigned long>(view_id), world->GetID());
		return 0;
	}
	if(view->GetPrevious()==NULL) {
		return 0;
	}
	view->GetPrevious()->CacheID();
	return view->GetPrevious()->GetID();
}

gmexport double ep_view_next(double world_id, double view_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->FindView(gm_cast<unsigned long>(view_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_next: View %lu doesn't exist in world %lu.", gm_cast<unsigned long>(view_id), world->GetID());
		return 0;
	}
	if(view->GetNext()==NULL) {
		return 0;
	}
	view->GetNext()->CacheID();
	return view->GetNext()->GetID();
}

gmexport double ep_view_set_uservar(double world_id, double view_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->FindView(gm_cast<unsigned long>(view_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_set_uservar: View %lu doesn't exist in world %lu.", gm_cast<unsigned long>(view_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of view %lu in world %lu, index is out of range.", view->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(view->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_view_get_uservar(double world_id, double view_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_View *view;
	if((view = world->FindView(gm_cast<unsigned long>(view_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_view_get_uservar: View %lu doesn't exist in world %lu.", gm_cast<unsigned long>(view_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of view %lu in world %lu, index is out of range.", view->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(view->GetUserData()))->var[i];
}

