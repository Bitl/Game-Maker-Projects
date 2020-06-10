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
 * File: ep_CollisionData.h
 * Structs that are used to store collision and raycasting data.
 * Included in ExtremePhysics.h.
 */

#ifndef EP_COLLISIONDATA_H
#define EP_COLLISIONDATA_H

struct ep_CollisionData {
	double nx, ny;
	bool cp_active[2];
	double cp_x[2][2], cp_y[2][2];
};

struct ep_RayCastData {
	double nx, ny;
	bool cp_active[2];
	double cp_x[2][2], cp_y[2][2];
};

#endif // EP_COLLISIONDATA_H

