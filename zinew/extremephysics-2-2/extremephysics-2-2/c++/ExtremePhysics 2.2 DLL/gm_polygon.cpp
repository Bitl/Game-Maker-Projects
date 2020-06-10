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
 * File: gm_polygon.cpp
 * Wrapper for ep_Polygon.
 */

#include "gm.h"

gmexport double ep_polygon_create(double world_id, double vertex_count) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_create: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->CreatePolygon(gm_cast<unsigned long>(vertex_count)))==NULL) {
		return 0;
	}
	((gmuserdata*)(polygon->GetUserData()))->Clear();
	polygon->CacheID();
	return polygon->GetID();
}

gmexport double ep_polygon_destroy(double world_id, double polygon_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_destroy: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id));
		return 0;
	}
	return (world->DestroyPolygon(polygon))? 1 : 0;
}

gmexport double ep_polygon_exists(double world_id, double polygon_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindPolygon(gm_cast<unsigned long>(polygon_id))!=NULL)? 1 : 0;
}

gmexport double ep_polygon_set_vertex(double world_id, double polygon_id, double index, double x, double y) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_set_vertex: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_set_vertex: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return (polygon->SetVertex(gm_cast<unsigned long>(index), x, y))? 1 : 0;
}

gmexport double ep_polygon_initialize(double world_id, double polygon_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_initialize: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_initialize: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return (polygon->Initialize())? 1 : 0;
}

gmexport double ep_polygon_get_vertex_count(double world_id, double polygon_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_count: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return polygon->GetVertexCount();
}

gmexport double ep_polygon_get_vertex_x(double world_id, double polygon_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_x: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return polygon->GetVertexX(gm_cast<unsigned long>(index));
}

gmexport double ep_polygon_get_vertex_y(double world_id, double polygon_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_y: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return polygon->GetVertexY(gm_cast<unsigned long>(index));
}

gmexport double ep_polygon_get_vertex_normal_x(double world_id, double polygon_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_normal_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_normal_x: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return polygon->GetVertexNormalX(gm_cast<unsigned long>(index));
}

gmexport double ep_polygon_get_vertex_normal_y(double world_id, double polygon_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_normal_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_vertex_normal_y: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return polygon->GetVertexNormalY(gm_cast<unsigned long>(index));
}

gmexport double ep_polygon_get_edge_length(double world_id, double polygon_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_edge_length: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_edge_length: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return polygon->GetEdgeLength(gm_cast<unsigned long>(index));
}

gmexport double ep_polygon_previous(double world_id, double polygon_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_previous: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	if(polygon->GetPrevious()==NULL) {
		return 0;
	}
	polygon->GetPrevious()->CacheID();
	return polygon->GetPrevious()->GetID();
}

gmexport double ep_polygon_next(double world_id, double polygon_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_next: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	if(polygon->GetNext()==NULL) {
		return 0;
	}
	polygon->GetNext()->CacheID();
	return polygon->GetNext()->GetID();
}

gmexport double ep_polygon_set_uservar(double world_id, double polygon_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_set_uservar: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of polygon %lu in world %lu, index is out of range.", polygon->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(polygon->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_polygon_get_uservar(double world_id, double polygon_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_polygon_get_uservar: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of polygon %lu in world %lu, index is out of range.", polygon->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(polygon->GetUserData()))->var[i];
}

