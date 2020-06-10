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
 * File: gm_main.cpp
 * Wrapper for ep_Main.
 */

#include "gm.h"

gmexport char* ep_version() {
	return (char*)(ep_Version());
}

gmexport double ep_set_log_file(double enable, char* filename, double level) {
	free(gmcallback.logfile);
	if(gm_cast<bool>(enable)) {
		unsigned long len = strlen(filename);
		gmcallback.logfile = (char*)(malloc(len+1));
		if(gmcallback.logfile==NULL) {
			epmain.SetMessageFilter(1);
			return 0;
		}
		memcpy(gmcallback.logfile, filename, len);
		gmcallback.logfile[len] = '\0';
		epmain.SetMessageFilter((gm_cast<int>(level)>1)? gm_cast<int>(level) : 1);
	} else {
		gmcallback.logfile = NULL;
		epmain.SetMessageFilter(1);
	}
	return 1;
}

gmexport double ep_set_show_errors(double enable) {
	gmcallback.showerrors = gm_cast<bool>(enable);
	return 1;
}

gmexport double ep_message(double level, char* string) {
	epmain.Message(gm_cast<int>(level), "%s", string);
	return 1;
}

gmexport double ep_print_object_tree() {
	epmain.PrintObjectTree();
	return 1;
}

gmexport double ep_collision_test_box_box(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_w, double shape2_h, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	return (ep_Main::CollisionTestBoxBox(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot,
		shape2_w, shape2_h, shape2_x, shape2_y, shape2_rot, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_box_line(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_x1, double shape2_y1, double shape2_x2, double shape2_y2, double contact_threshold) {
	return (ep_Main::CollisionTestBoxLine(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot,
		shape2_x1, shape2_y1, shape2_x2, shape2_y2, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_box_circle(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_r, double shape2_x, double shape2_y, double contact_threshold) {
	return (ep_Main::CollisionTestBoxCircle(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot,
		shape2_r, shape2_x, shape2_y, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_box_polygon(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_world_id, double shape2_polygon_id, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_World *world2;
	if((world2 = epmain.FindWorld(gm_cast<unsigned long>(shape2_world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_box_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(shape2_world_id));
		return 0;
	}
	ep_Polygon *polygon2;
	if((polygon2 = world2->FindPolygon(gm_cast<unsigned long>(shape2_polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_box_polygon: Polygon %lu doesn't exist in world %lu..", gm_cast<unsigned long>(shape2_polygon_id), world2->GetID());
		return 0;
	}
	return (ep_Main::CollisionTestBoxPolygon(shape1_w, shape1_h, shape1_x, shape1_y, shape1_rot,
		polygon2, shape2_x, shape2_y, shape2_rot, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_line_line(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
	double shape2_x1, double shape2_y1, double shape2_x2, double shape2_y2, double contact_threshold) {
	return (ep_Main::CollisionTestLineLine(shape1_x1, shape1_y1, shape1_x2, shape1_y2,
		shape2_x1, shape2_y1, shape2_x2, shape2_y2, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_line_circle(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
	double shape2_r, double shape2_x, double shape2_y, double contact_threshold) {
	return (ep_Main::CollisionTestLineCircle(shape1_x1, shape1_y1, shape1_x2, shape1_y2,
		shape2_r, shape2_x, shape2_y, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_line_polygon(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
	double shape2_world_id, double shape2_polygon_id, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_World *world2;
	if((world2 = epmain.FindWorld(gm_cast<unsigned long>(shape2_world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_line_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(shape2_world_id));
		return 0;
	}
	ep_Polygon *polygon2;
	if((polygon2 = world2->FindPolygon(gm_cast<unsigned long>(shape2_polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_line_polygon: Polygon %lu doesn't exist in world %lu..", gm_cast<unsigned long>(shape2_polygon_id), world2->GetID());
		return 0;
	}
	return (ep_Main::CollisionTestLinePolygon(shape1_x1, shape1_y1, shape1_x2, shape1_y2,
		polygon2, shape2_x, shape2_y, shape2_rot, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_circle_circle(double shape1_r, double shape1_x, double shape1_y,
	double shape2_r, double shape2_x, double shape2_y, double contact_threshold) {
	return (ep_Main::CollisionTestCircleCircle(shape1_r, shape1_x, shape1_y,
		shape2_r, shape2_x, shape2_y, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_circle_polygon(double shape1_r, double shape1_x, double shape1_y,
	double shape2_world_id, double shape2_polygon_id, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_World *world2;
	if((world2 = epmain.FindWorld(gm_cast<unsigned long>(shape2_world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_circle_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(shape2_world_id));
		return 0;
	}
	ep_Polygon *polygon2;
	if((polygon2 = world2->FindPolygon(gm_cast<unsigned long>(shape2_polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_circle_polygon: Polygon %lu doesn't exist in world %lu..", gm_cast<unsigned long>(shape2_polygon_id), world2->GetID());
		return 0;
	}
	return (ep_Main::CollisionTestCirclePolygon(shape1_r, shape1_x, shape1_y,
		polygon2, shape2_x, shape2_y, shape2_rot, contact_threshold))? 1 : 0;
}

gmexport double ep_collision_test_polygon_polygon(double shape1_world_id, double shape1_polygon_id, double shape1_x, double shape1_y, double shape1_rot,
	double shape2_world_id, double shape2_polygon_id, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold) {
	ep_World *world1;
	if((world1 = epmain.FindWorld(gm_cast<unsigned long>(shape1_world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_polygon_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(shape1_world_id));
		return 0;
	}
	ep_Polygon *polygon1;
	if((polygon1 = world1->FindPolygon(gm_cast<unsigned long>(shape1_polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_polygon_polygon: Polygon %lu doesn't exist in world %lu..", gm_cast<unsigned long>(shape1_polygon_id), world1->GetID());
		return 0;
	}
	ep_World *world2;
	if((world2 = epmain.FindWorld(gm_cast<unsigned long>(shape2_world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_polygon_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(shape2_world_id));
		return 0;
	}
	ep_Polygon *polygon2;
	if((polygon2 = world2->FindPolygon(gm_cast<unsigned long>(shape2_polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_collision_test_polygon_polygon: Polygon %lu doesn't exist in world %lu..", gm_cast<unsigned long>(shape2_polygon_id), world2->GetID());
		return 0;
	}
	return (ep_Main::CollisionTestPolygonPolygon(polygon1, shape1_x, shape1_y, shape1_rot,
		polygon2, shape2_x, shape2_y, shape2_rot, contact_threshold))? 1 : 0;
}

gmexport double ep_ray_cast_box(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_w, double shape_h, double shape_x, double shape_y, double shape_rot) {
	return ep_Main::RayCastBox(ray_x, ray_y, ray_vx, ray_vy,
		shape_w, shape_h, shape_x, shape_y, shape_rot);
}

gmexport double ep_ray_cast_line(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_x1, double shape_y1, double shape_x2, double shape_y2) {
	return ep_Main::RayCastLine(ray_x, ray_y, ray_vx, ray_vy,
		shape_x1, shape_y1, shape_x2, shape_y2);
}

gmexport double ep_ray_cast_circle(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_r, double shape_x, double shape_y) {
	return ep_Main::RayCastCircle(ray_x, ray_y, ray_vx, ray_vy,
		shape_r, shape_x, shape_y);
}

gmexport double ep_ray_cast_polygon(double ray_x, double ray_y, double ray_vx, double ray_vy,
	double shape_world_id, double shape_polygon_id, double shape_x, double shape_y, double shape_rot) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(shape_world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_ray_cast_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(shape_world_id));
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(shape_polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_ray_cast_polygon: Polygon %lu doesn't exist in world %lu..", gm_cast<unsigned long>(shape_polygon_id), world->GetID());
		return 0;
	}
	return ep_Main::RayCastPolygon(ray_x, ray_y, ray_vx, ray_vy,
		polygon, shape_x, shape_y, shape_rot);
}

gmexport double ep_first_world() {
	if(epmain.GetFirstWorld()==NULL) {
		return 0;
	}
	epmain.GetFirstWorld()->CacheID();
	return epmain.GetFirstWorld()->GetID();
}

gmexport double ep_last_world() {
	if(epmain.GetLastWorld()==NULL) {
		return 0;
	}
	epmain.GetLastWorld()->CacheID();
	return epmain.GetLastWorld()->GetID();
}

