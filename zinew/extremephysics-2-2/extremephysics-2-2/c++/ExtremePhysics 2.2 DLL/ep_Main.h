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
 * File: ep_Main.h
 * Definition of ep_Main.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_MAIN_H
#define EP_MAIN_H

/// Returns the version number as a string (e.g. "1.2.34").
const char* ep_Version();

/// The main ExtremePhysics class.
/// Create one instance of this class (on the stack or on the heap).
class ep_Main {
	
	friend class ep_World;
	friend class ep_Polygon;
	friend class ep_Body;
	friend class ep_Contact;
	friend class ep_HingeJoint;
	friend class ep_DistanceJoint;
	friend class ep_RailJoint;
	friend class ep_View;
	friend class ep_Shape;
	friend class ep_Force;
	
	// variables
	private:
	
	ep_Callback *callback;
	
	unsigned long idcounter_worlds;
	ep_World *first_world, *last_world, *current_world;
	
	int maxmessagelevel;
	
	unsigned long freecontactcount;
	ep_Contact *freecontacts;
	
	// public functions
	public:
	
	/// Constructor
	/// @param _callback A pointer to an instance of ep_Callback (a derived class).
	ep_Main(ep_Callback* _callback = NULL);
	
	/// Destructor
	~ep_Main();
	
	/// Destroys all ExtremePhysics objects in this world and resets id counters.
	void Clear();
	
	/// Creates a new world.
	/// @param world_userdatasize Size of the userdata memory block for the world.
	/// @param polygon_userdatasize Size of the userdata memory block for polygons.
	/// @param body_userdatasize Size of the userdata memory block for bodies.
	/// @param hingejoint_userdatasize Size of the userdata memory block for hinge joints.
	/// @param distancejoint_userdatasize Size of the userdata memory block for distance joints.
	/// @param railjoint_userdatasize Size of the userdata memory block for rail joints.
	/// @param sliderjoint_userdatasize Size of the userdata memory block for slider joints.
	/// @param view_userdatasize Size of the userdata memory block for views.
	/// @param water_userdatasize Size of the userdata memory block for water.
	/// @param shape_userdatasize Size of the userdata memory block for shapes.
	/// @param force_userdatasize Size of the userdata memory block for forces.
	/// @return Returns a pointer to the new world, or NULL if an error occurred.
	ep_World* CreateWorld(unsigned long world_userdatasize, unsigned long polygon_userdatasize, unsigned long body_userdatasize,
		unsigned long hingejoint_userdatasize, unsigned long distancejoint_userdatasize, unsigned long railjoint_userdatasize,
		unsigned long sliderjoint_userdatasize,
		unsigned long view_userdatasize, unsigned long water_userdatasize, unsigned long shape_userdatasize, unsigned long force_userdatasize); //JOINTS//
	/// Destroys a world.
	/// This function will also destroy all bodies, joints, ... associated with this world.
	/// @param world Pointer to the world to destroy.
	/// @return Returns whether successful.
	bool DestroyWorld(ep_World* world);
	/// Finds the world that corresponds to the given id.
	/// @param _id The id of the world.
	/// @return Returns a pointer to the world, or NULL if the world does not exist.
	ep_World* FindWorld(unsigned long _id);
	
	/// Shows a debug message. The format system is identical to printf().
	/// @param level The level of the message.
	/// @param format The format of the message.
	void Message(int level, const char* format, ...);
	/// Lists all ExtremePhysics objects (using debug messages).
	/// This is useful to detect memory leaks.
	void PrintObjectTree();
	
	/// Performs a collision test with two 'virtual' shapes.
	static bool CollisionTestBoxBox(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
		double shape2_w, double shape2_h, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold);
	static bool CollisionTestBoxLine(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
		double shape2_x1, double shape2_y1, double shape2_x2, double shape2_y2, double contact_threshold);
	static bool CollisionTestBoxCircle(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
		double shape2_r, double shape2_x, double shape2_y, double contact_threshold);
	static bool CollisionTestBoxPolygon(double shape1_w, double shape1_h, double shape1_x, double shape1_y, double shape1_rot,
		ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold);
	static bool CollisionTestLineLine(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
		double shape2_x1, double shape2_y1, double shape2_x2, double shape2_y2, double contact_threshold);
	static bool CollisionTestLineCircle(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
		double shape2_r, double shape2_x, double shape2_y, double contact_threshold);
	static bool CollisionTestLinePolygon(double shape1_x1, double shape1_y1, double shape1_x2, double shape1_y2,
		ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold);
	static bool CollisionTestCircleCircle(double shape1_r, double shape1_x, double shape1_y,
		double shape2_r, double shape2_x, double shape2_y, double contact_threshold);
	static bool CollisionTestCirclePolygon(double shape1_r, double shape1_x, double shape1_y,
		ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold);
	static bool CollisionTestPolygonPolygon(ep_Polygon* shape1_polygon, double shape1_x, double shape1_y, double shape1_rot,
		ep_Polygon* shape2_polygon, double shape2_x, double shape2_y, double shape2_rot, double contact_threshold);
	
	/// Performs a ray cast with a 'virtual' shape.
	static double RayCastBox(double ray_x, double ray_y, double ray_vx, double ray_vy,
		double shape_w, double shape_h, double shape_x, double shape_y, double shape_rot);
	static double RayCastLine(double ray_x, double ray_y, double ray_vx, double ray_vy,
		double shape_x1, double shape_y1, double shape_x2, double shape_y2);
	static double RayCastCircle(double ray_x, double ray_y, double ray_vx, double ray_vy,
		double shape_r, double shape_x, double shape_y);
	static double RayCastPolygon(double ray_x, double ray_y, double ray_vx, double ray_vy,
		ep_Polygon* shape_polygon, double shape_x, double shape_y, double shape_rot);
	
	// public inline functions
	public:
	
	/// Returns a pointer to the first world.
	inline ep_World* GetFirstWorld() { return first_world; }
	/// Returns a pointer to the last world.
	inline ep_World* GetLastWorld() { return last_world; }
	
	/// Sets the message filter.
	/// @param maxmessagelevel Messages with a level higher than this value will be ignored (performance).
	inline void SetMessageFilter(int _maxmessagelevel) { maxmessagelevel = _maxmessagelevel; }
	
};

#endif // EP_MAIN_H

