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
 * File: ep_Water.h
 * Definition of ep_Water.
 * Included in ExtremePhysics.h.
 */

#ifndef EP_WATER_H
#define EP_WATER_H

class ep_Water {
	
	friend class ep_Main;
	friend class ep_World;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_Water *prev, *next;
	unsigned long id;
	
	double density, lineardrag, quadraticdrag;
	double xvel, yvel, gravity_x, gravity_y;
	double x1, y1, x2, y2;
	
	// private functions
	private:
	
	void Init(ep_World* _world, unsigned long _id, bool awake);
	void DeInit();
	
	// public functions
	public:
	
	/// Changes the parameters of the water.
	/// @param _density The density of the water.
	/// @param _lineardrag The amount of linear drag.
	/// @param _quadraticdrag The amount of quadratic drag.
	/// @param _xvel The x component of the velocity of the water.
	/// @param _yvel The y component of the velocity of the water.
	/// @param _gravity_x The x component of the gravity of the water.
	/// @param _gravity_y The y component of the gravity of the water.
	void SetParameters(double _density, double _lineardrag, double _quadraticdrag, double _xvel, double _yvel, double _gravity_x, double _gravity_y);
	/// Changes the rectangle of the water.
	/// @param _x1 The left side of the water rectangle.
	/// @param _y1 The top side of the water rectangle.
	/// @param _x2 The right side of the water rectangle.
	/// @param _y2 The bottom side of the water rectangle.
	void SetRectangle(double _x1, double _y1, double _x2, double _y2);
	
	// inline functions
	public:
	
	/// Returns the id of this view.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this view to speed up the next FindView call (if the id matches the id of this view).
	inline void CacheID() { world->current_water = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this view.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this view.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous view.
	inline ep_Water* GetPrevious() { return prev; }
	/// Returns a pointer to the next view.
	inline ep_Water* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
};

#endif // EP_WATER_H

