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
 * File: ep_Force.h
 * Definition of ep_Force.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_FORCE_H
#define EP_FORCE_H

class ep_Force {
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Body;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_Body *body;
	ep_Force *prev, *next;
	unsigned long id;
	
	double x, y;
	double xforce, yforce, torque;
	bool local, ignoremass;
	
	// private functions
	private:
	
	void Init(ep_Body* _body, double _x, double _y, bool _local, bool _ignoremass, unsigned long _id);
	void DeInit();
	
	void IntegrateVelocity();
	
	// inline functions
	public:
	
	/// Returns the id of this force.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this force to speed up the next FindForce call (if the id matches the id of this force).
	inline void CacheID() { body->current_force = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this force.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this force.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the body associated with this force.
	inline ep_Body* GetBody() { return body; }
	/// Returns a pointer to the previous force.
	inline ep_Force* GetPrevious() { return prev; }
	/// Returns a pointer to the next force.
	inline ep_Force* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
	/// Changes the force.
	/// @param _xforce The x component of the new force.
	/// @param _yforce The y component of the new force.
	/// @param _torque The new torque.
	inline void SetForce(double _xforce, double _yforce, double _torque, double awake) {
		if(xforce!=_xforce || yforce!=_yforce || torque!=_torque) {
			xforce = _xforce;
			yforce = _yforce;
			torque = _torque;
			if(awake) body->Awake(false, false);
		}
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed force of force %lu in world %lu.", id, world->id);
#endif
	}
	
};

#endif // EP_FORCE_H

