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
 * File: gm_world.cpp
 * Wrapper for ep_World.
 */

#include "gm.h"

gmexport double ep_world_create() {
	ep_World *world;
	if((world = epmain.CreateWorld(sizeof(gmuserdata), sizeof(gmuserdata), sizeof(gmuserdata),
		sizeof(gmuserdata), sizeof(gmuserdata), sizeof(gmuserdata),
		sizeof(gmuserdata),
		sizeof(gmuserdata), sizeof(gmuserdata), sizeof(gmuserdata), sizeof(gmuserdata)))==NULL) {
		return 0;
	}
	((gmuserdata*)(world->GetUserData()))->Clear();
	world->CacheID();
	return world->GetID();
}

gmexport double ep_world_destroy(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (epmain.DestroyWorld(world))? 1 : 0;
}

gmexport double ep_world_exists(double world_id) {
	return (epmain.FindWorld(gm_cast<unsigned long>(world_id))!=NULL)? 1 : 0;
}

gmexport double ep_world_clear(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_clear: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	world->Clear();
	return 1;
}

gmexport double ep_world_set_settings(double world_id, double timestep, double velocity_iterations, double position_iterations, double contact_threshold, double velocity_threshold, double baumgarte_factor, double mass_bias, double position_factor) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_set_settings: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	world->SetSettings(timestep, gm_cast<unsigned long>(velocity_iterations), gm_cast<unsigned long>(position_iterations), contact_threshold, velocity_threshold, baumgarte_factor, mass_bias, position_factor);
	return 1;
}

gmexport double ep_world_set_primary_axis(double world_id, double horizontal) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_set_primary_axis: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	world->SetPrimaryAxis(gm_cast<bool>(horizontal));
	return 1;
}

gmexport double ep_world_set_sleeping(double world_id, double enable_sleeping, double time_stable, double time_outofview, double stable_maxvel, double stable_maxrotvel) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_set_sleeping: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	world->SetSleeping(gm_cast<bool>(enable_sleeping), time_stable, time_outofview, stable_maxvel, stable_maxrotvel);
	return 1;
}

gmexport double ep_world_update_contacts(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_update_contacts: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->UpdateContacts())? 1 : 0;
}

gmexport double ep_world_simulate_step(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_simulate_step: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->SimulateStep())? 1 : 0;
}

gmexport double ep_world_collision_test_box(double world_id, double w, double h, double x, double y, double rot, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_box: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->CollisionTestBox(w, h, x, y, rot, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_world_collision_test_line(double world_id, double x1, double y1, double x2, double y2, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_line: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->CollisionTestLine(x1, y1, x2, y2, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_world_collision_test_circle(double world_id, double r, double x, double y, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_collision_test_circle: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->CollisionTestCircle(r, x, y, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_world_collision_test_polygon(double world_id, double polygon_id, double x, double y, double rot, double contact_threshold, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_polygon: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return world->CollisionTestPolygon(polygon, x, y, rot, contact_threshold, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_world_ray_cast(double world_id, double x, double y, double vx, double vy, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_body_ray_cast: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->RayCast(x, y, vx, vy, gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
}

gmexport double ep_world_get_collision_body(double world_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_get_collision_body: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Shape *shape = world->GetCollisionShape(gm_cast<unsigned long>(index));
	return (shape==NULL)? 0 : shape->GetBody()->GetID();
}

gmexport double ep_world_get_collision_shape(double world_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_get_collision_shape: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Shape *shape = world->GetCollisionShape(gm_cast<unsigned long>(index));
	return (shape==NULL)? 0 : shape->GetID();
}

gmexport double ep_world_multipoly_begin(double world_id, double vertexcount) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_multipoly_begin: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->MultipolyBegin(gm_cast<unsigned long>(vertexcount)))? 1 : 0;
}

gmexport double ep_world_multipoly_end(double world_id, double showerrors) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_multipoly_end: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(!world->MultipolyEnd(gm_cast<bool>(showerrors))) {
		return 0;
	}
	for(ep_Polygon *p = world->MultipolyGetFirst(); p!=NULL ; p = p->GetNext()) {
		((gmuserdata*)(p->GetUserData()))->Clear();
		if(p==world->MultipolyGetLast()) break;
	}
	return 1;
}

gmexport double ep_world_multipoly_set_vertex(double world_id, double index, double x, double y) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_multipoly_set_vertex: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->MultipolySetVertex(gm_cast<unsigned long>(index), x, y))? 1 : 0;
}

gmexport double ep_world_multipoly_get_first(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_multipoly_get_first: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->MultipolyGetFirst()==NULL) {
		return 0;
	}
	world->MultipolyGetFirst()->CacheID();
	return world->MultipolyGetFirst()->GetID();
}

gmexport double ep_world_multipoly_get_last(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_multipoly_get_last: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->MultipolyGetLast()==NULL) {
		return 0;
	}
	world->MultipolyGetLast()->CacheID();
	return world->MultipolyGetLast()->GetID();
}

void bintohex(const char* input, char* output, unsigned long inputlen) {
	for(unsigned long i = 0; i<inputlen; ++i) {
		unsigned char a = input[i];
		output[2*i]   = ((a>>4)>9)? 'a'+(a>>4)-10 : '0'+(a>>4);
		output[2*i+1] = ((a&15)>9)? 'a'+(a&15)-10 : '0'+(a&15);
	}
}
void hextobin(const char* input, char* output, unsigned long outputlen) {
	for(unsigned long i = 0; i<outputlen; ++i) {
		unsigned char a = input[2*i];
		unsigned char b = input[2*i+1];
		output[i] =
			((('0'<=a && a<='9')? a-'0' : (('a'<=a && a<='f')? 10+a-'a' : (('A'<=a && a<='F')? 10+a-'A' : 0)))<<4) |
			(('0'<=b && b<='9')? b-'0' : (('a'<=b && b<='f')? 10+b-'a' : (('A'<=b && b<='F')? 10+b-'A' : 0)));
	}
}

gmexport char* ep_world_serialize(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_serialize: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(!world->Serialize()) return "";
	const char *data = world->GetSerializeData();
	unsigned long len = world->GetSerializeDataLength();
	char *ret = gmcallback.MakeReturnString(len*2+1);
	if(ret==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_serialize: Can't serialize world %lu, memory allocation failed.", world->GetID());
		return "";
	}
	bintohex(data, ret, len);
	ret[len*2] = '\0';
	world->FreeSerializeData();
	return ret;
}

gmexport double ep_world_unserialize(double world_id, char* data) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_unserialize: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	unsigned long len = strlen(data)/2;
	char *ret = gmcallback.MakeReturnString(len);
	if(ret==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_unserialize: Can't unserialize world %lu, memory allocation failed.", world->GetID());
		return 0;
	}
	hextobin(data, ret, len);
	if(world->Unserialize(ret, len)) {
		((gmuserdata*)(world->GetUserData()))->Clear();
		for(ep_Polygon *polygon = world->GetFirstPolygon(); polygon!=NULL; polygon = polygon->GetNext()) {
			((gmuserdata*)(polygon->GetUserData()))->Clear();
		}
		for(ep_Body *body = world->GetFirstBody(); body!=NULL; body = body->GetNext()) {
			((gmuserdata*)(body->GetUserData()))->Clear();
			for(ep_Shape *shape = body->GetFirstShape(); shape!=NULL; shape = shape->GetNext()) {
				((gmuserdata*)(shape->GetUserData()))->Clear();
			}
			for(ep_Force *force = body->GetFirstForce(); force!=NULL; force = force->GetNext()) {
				((gmuserdata*)(force->GetUserData()))->Clear();
			}
		}
		for(ep_HingeJoint *hingejoint = world->GetFirstHingeJoint(); hingejoint!=NULL; hingejoint = hingejoint->GetNext()) {
			((gmuserdata*)(hingejoint->GetUserData()))->Clear();
		}
		for(ep_DistanceJoint *distancejoint = world->GetFirstDistanceJoint(); distancejoint!=NULL; distancejoint = distancejoint->GetNext()) {
			((gmuserdata*)(distancejoint->GetUserData()))->Clear();
		}
		for(ep_RailJoint *railjoint = world->GetFirstRailJoint(); railjoint!=NULL; railjoint = railjoint->GetNext()) {
			((gmuserdata*)(railjoint->GetUserData()))->Clear();
		}
		for(ep_SliderJoint *sliderjoint = world->GetFirstSliderJoint(); sliderjoint!=NULL; sliderjoint = sliderjoint->GetNext()) {
			((gmuserdata*)(sliderjoint->GetUserData()))->Clear();
		}
		//JOINTS//
		for(ep_View *view = world->GetFirstView(); view!=NULL; view = view->GetNext()) {
			((gmuserdata*)(view->GetUserData()))->Clear();
		}
		return 1;
	} else {
		return 0;
	}
}

gmexport double ep_world_previous(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetPrevious()==NULL) {
		return 0;
	}
	world->GetPrevious()->CacheID();
	return world->GetPrevious()->GetID();
}

gmexport double ep_world_next(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetNext()==NULL) {
		return 0;
	}
	world->GetNext()->CacheID();
	return world->GetNext()->GetID();
}

gmexport double ep_world_set_uservar(double world_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of world %lu, index is out of range.", world->GetID());
		return 0;
	}
	((gmuserdata*)(world->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_world_get_uservar(double world_id, double body_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of world %lu, index is out of range.", world->GetID());
		return 0;
	}
	return ((gmuserdata*)(world->GetUserData()))->var[i];
}

gmexport double ep_world_first_polygon(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetFirstPolygon()==NULL) {
		return 0;
	}
	world->GetFirstPolygon()->CacheID();
	return world->GetFirstPolygon()->GetID();
}

gmexport double ep_world_last_polygon(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetLastPolygon()==NULL) {
		return 0;
	}
	world->GetLastPolygon()->CacheID();
	return world->GetLastPolygon()->GetID();
}

gmexport double ep_world_polygon_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_polygon_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetPolygonCount();
}

gmexport double ep_world_first_body(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_body: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetFirstBody()==NULL) {
		return 0;
	}
	world->GetFirstBody()->CacheID();
	return world->GetFirstBody()->GetID();
}

gmexport double ep_world_last_body(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_body: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetLastBody()==NULL) {
		return 0;
	}
	world->GetLastBody()->CacheID();
	return world->GetLastBody()->GetID();
}

gmexport double ep_world_body_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_body_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetBodyCount();
}

gmexport double ep_world_first_contact(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_contact: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetFirstContact()==NULL) {
		return 0;
	}
	world->GetFirstContact()->CacheID();
	return world->GetFirstContact()->GetID();
}

gmexport double ep_world_last_contact(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_contact: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetLastContact()==NULL) {
		return 0;
	}
	world->GetLastContact()->CacheID();
	return world->GetLastContact()->GetID();
}

gmexport double ep_world_contact_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_contact_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetContactCount();
}

gmexport double ep_world_first_hingejoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_hingejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint = world->GetFirstHingeJoint();
	if(hingejoint==NULL) {
		return 0;
	}
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_world_last_hingejoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_hingejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_HingeJoint *hingejoint = world->GetLastHingeJoint();
	if(hingejoint==NULL) {
		return 0;
	}
	hingejoint->CacheID();
	return hingejoint->GetID();
}

gmexport double ep_world_hingejoint_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_hingejoint_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetHingeJointCount();
}

gmexport double ep_world_first_distancejoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_distancejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_DistanceJoint *distancejoint = world->GetFirstDistanceJoint();
	if(distancejoint==NULL) {
		return 0;
	}
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_world_last_distancejoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_distancejoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetLastDistanceJoint()==NULL) {
		return 0;
	}
	ep_DistanceJoint *distancejoint = world->GetLastDistanceJoint();
	if(distancejoint==NULL) {
		return 0;
	}
	distancejoint->CacheID();
	return distancejoint->GetID();
}

gmexport double ep_world_distancejoint_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_distancejoint_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetDistanceJointCount();
}

gmexport double ep_world_first_railjoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_railjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint = world->GetFirstRailJoint();
	if(railjoint==NULL) {
		return 0;
	}
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_world_last_railjoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_railjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_RailJoint *railjoint = world->GetLastRailJoint();
	if(railjoint==NULL) {
		return 0;
	}
	railjoint->CacheID();
	return railjoint->GetID();
}

gmexport double ep_world_railjoint_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_railjoint_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetRailJointCount();
}

gmexport double ep_world_first_sliderjoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_sliderjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint = world->GetFirstSliderJoint();
	if(sliderjoint==NULL) {
		return 0;
	}
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

gmexport double ep_world_last_sliderjoint(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_sliderjoint: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_SliderJoint *sliderjoint = world->GetLastSliderJoint();
	if(sliderjoint==NULL) {
		return 0;
	}
	sliderjoint->CacheID();
	return sliderjoint->GetID();
}

gmexport double ep_world_sliderjoint_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_sliderjoint_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetSliderJointCount();
}

//JOINTS//

gmexport double ep_world_first_view(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_view: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetFirstView()==NULL) {
		return 0;
	}
	world->GetFirstView()->CacheID();
	return world->GetFirstView()->GetID();
}

gmexport double ep_world_last_view(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_view: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetLastView()==NULL) {
		return 0;
	}
	world->GetLastView()->CacheID();
	return world->GetLastView()->GetID();
}

gmexport double ep_world_view_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_view_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetViewCount();
}

gmexport double ep_world_first_water(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_first_water: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetFirstWater()==NULL) {
		return 0;
	}
	world->GetFirstWater()->CacheID();
	return world->GetFirstWater()->GetID();
}

gmexport double ep_world_last_water(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_last_water: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	if(world->GetLastWater()==NULL) {
		return 0;
	}
	world->GetLastWater()->CacheID();
	return world->GetLastWater()->GetID();
}

gmexport double ep_world_water_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_water_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetWaterCount();
}

gmexport double ep_world_shape_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_shape_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetShapeCount();
}

gmexport double ep_world_force_count(double world_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_world_force_count: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return world->GetForceCount();
}

