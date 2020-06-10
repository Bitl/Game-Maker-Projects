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
 * File: gm_debugdraw.cpp
 * Wrapper for debug drawing functions of ep_World.
 */

#include "gm.h"

gmexport double ep_debugdraw_set_transformation(double translation_x, double translation_y, double rotation, double scale) {
	gmcallback.translation_x = translation_x;
	gmcallback.translation_y = translation_y;
	gmcallback.rotation = rotation;
	gmcallback.scale = scale;
	gmcallback.m[0][0] = +cos(rotation)*scale;
	gmcallback.m[0][1] = +sin(rotation)*scale;
	gmcallback.m[1][0] = -sin(rotation)*scale;
	gmcallback.m[1][1] = +cos(rotation)*scale;
	return 1;
}

gmexport double ep_debugdraw_bodies(double world_id, double color_static, double color_dynamic) {
	if(!gmfunctions_initialized) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_bodies: GM functions have not been initialized, check the result of ep_gmfunctions_init.");
		return 0;
	}
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_bodies: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	gmcallback.d1 = color_static;
	gmcallback.d2 = color_dynamic;
	world->DebugDrawBodies();
	return 1;
}

gmexport double ep_debugdraw_links(double world_id, double color_shapelink, double color_contactlink, double color_jointlink) {
	if(!gmfunctions_initialized) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_links: GM functions have not been initialized, check the result of ep_gmfunctions_init.");
		return 0;
	}
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_links: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	gmcallback.d1 = color_shapelink;
	gmcallback.d2 = color_contactlink;
	gmcallback.d3 = color_jointlink;
	world->DebugDrawLinks();
	return 1;
}

gmexport double ep_debugdraw_views(double world_id, double color_view) {
	if(!gmfunctions_initialized) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_views: GM functions have not been initialized, check the result of ep_gmfunctions_init.");
		return 0;
	}
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_views: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	GM_draw_set_color(color_view);
	world->DebugDrawViews();
	return 1;
}

gmexport double ep_debugdraw_velocity(double world_id, double color_velocity, double scale, double rotscale) {
	if(!gmfunctions_initialized) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_velocity: GM functions have not been initialized, check the result of ep_gmfunctions_init.");
		return 0;
	}
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_velocity: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	GM_draw_set_color(color_velocity);
	gmcallback.d1 = scale;
	gmcallback.d2 = rotscale;
	world->DebugDrawVelocity();
	return 1;
}

gmexport double ep_debugdraw_forces(double world_id, double color_force, double scale, double rotscale) {
	if(!gmfunctions_initialized) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_forces: GM functions have not been initialized, check the result of ep_gmfunctions_init.");
		return 0;
	}
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_forces: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	GM_draw_set_color(color_force);
	gmcallback.d1 = scale;
	gmcallback.d2 = rotscale;
	world->DebugDrawForces();
	return 1;
}

gmexport double ep_debugdraw_constraints(double world_id, double color_contactpoint, double color_joint) {
	if(!gmfunctions_initialized) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_constraints: GM functions have not been initialized, check the result of ep_gmfunctions_init.");
		return 0;
	}
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_debugdraw_constraints: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	gmcallback.d1 = color_contactpoint;
	gmcallback.d2 = color_joint;
	world->DebugDrawConstraints();
	return 1;
}

