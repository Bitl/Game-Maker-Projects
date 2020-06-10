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
 * File: ep_World.h
 * Definition of ep_World.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_WORLD_H
#define EP_WORLD_H

#if EP_COMPILE_MULTIPOLY
struct ep_Multipoly_Vertex {
	double x, y;
	double nx, ny;
	double segment_nx, segment_ny;
	bool convex;
	unsigned long prev, next;
};
struct ep_Multipoly_Task {
	unsigned long i, j;
	unsigned long a, b;
	ep_Multipoly_Cut *cuts_list1, *cuts_list2;
	int part;
};
struct ep_Multipoly_Cut {
	unsigned long i, j;
	ep_Multipoly_Cut *next;
	int rating1;
	double rating2;
};
#endif // EP_COMPILE_MULTIPOLY

class ep_World {
	
	friend class ep_Main;
	friend class ep_Polygon;
	friend class ep_Body;
	friend class ep_Contact;
	friend class ep_HingeJoint;
	friend class ep_DistanceJoint;
	friend class ep_RailJoint;
	friend class ep_SliderJoint;
	//JOINTS//
	friend class ep_View;
	friend class ep_Water;
	friend class ep_Shape;
	friend class ep_Force;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *prev, *next;
	unsigned long id;
	
	unsigned long polygon_userdatasize;
	unsigned long body_userdatasize;
	unsigned long hingejoint_userdatasize;
	unsigned long distancejoint_userdatasize;
	unsigned long railjoint_userdatasize;
	unsigned long sliderjoint_userdatasize;
	//JOINTS//
	unsigned long view_userdatasize;
	unsigned long water_userdatasize;
	unsigned long shape_userdatasize;
	unsigned long force_userdatasize;
	
	unsigned long idcounter_polygons, polygoncount;
	ep_Polygon *first_polygon, *last_polygon, *current_polygon;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_polygons;
#endif
	
	unsigned long idcounter_bodies, bodycount;
	ep_Body *first_body, *last_body, *current_body;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_bodies;
#endif
	
	unsigned long idcounter_contacts, contactcount;
	ep_Contact *first_contact, *last_contact, *current_contact;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_contacts;
#endif
	
	unsigned long idcounter_hingejoints, hingejointcount;
	ep_HingeJoint *first_hingejoint, *last_hingejoint, *current_hingejoint;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_hingejoints;
#endif
	
	unsigned long idcounter_distancejoints, distancejointcount;
	ep_DistanceJoint *first_distancejoint, *last_distancejoint, *current_distancejoint;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_distancejoints;
#endif
	
	unsigned long idcounter_railjoints, railjointcount;
	ep_RailJoint *first_railjoint, *last_railjoint, *current_railjoint;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_railjoints;
#endif
	
	unsigned long idcounter_sliderjoints, sliderjointcount;
	ep_SliderJoint *first_sliderjoint, *last_sliderjoint, *current_sliderjoint;
#if EP_USE_IDHASHTABLE
	ep_IdHashTable idhashtable_sliderjoints;
#endif
	
	//JOINTS//
	
	unsigned long idcounter_views, viewcount;
	ep_View *first_view, *last_view, *current_view;
	
	unsigned long idcounter_water, watercount;
	ep_Water *first_water, *last_water, *current_water;
	
	unsigned long shapecount;
	unsigned long forcecount;
	
	double timestep;
	unsigned long velocity_iterations, position_iterations;
	double contact_threshold, velocity_threshold, baumgarte_factor, mass_bias, position_factor;
	bool horizontal;
	
	bool enable_sleeping;
	double time_stable, time_outofview;
	double stable_maxvel, stable_maxrotvel;
	
	unsigned long collisionshapecount;
	ep_Shape **collisionshapes;
	
#if EP_COMPILE_MULTIPOLY
	unsigned long multipoly_vertexcount;
	ep_Multipoly_Vertex *multipoly_vertices;
	ep_Polygon *multipoly_first_polygon, *multipoly_last_polygon;
#endif // EP_COMPILE_MULTIPOLY
	
#if EP_COMPILE_SERIALIZE
	char *serializedata;
	unsigned long serializedata_length;
#endif // EP_COMPILE_SERIALIZE
	
	// private functions
	private:
	
	void Init(ep_Main* _main);
	void DeInit();
	
	void ClearCollisionList();
	bool AddToCollisionList(ep_Shape* shape);
	
	bool _UpdateContacts();
	void _UpdateSleeping();
	void IntegrateVelocity();
	void IntegratePosition();
	void SolveVelocityConstraints();
	void SolvePositionConstraints();
	void SimulateWater();
	
	ep_Contact* AllocateContact();
	ep_Polygon* AllocatePolygon(unsigned long vertexcount);
	ep_Body* AllocateBody();
	ep_HingeJoint* AllocateHingeJoint();
	ep_DistanceJoint* AllocateDistanceJoint();
	ep_RailJoint* AllocateRailJoint();
	ep_SliderJoint* AllocateSliderJoint();
	//JOINTS//
	ep_View* AllocateView();
	ep_Water* AllocateWater();
	ep_Shape* AllocateShape();
	ep_Force* AllocateForce();
	
	ep_Contact* CreateContact(ep_Shape* shape1, ep_Shape* shape2);
	
	static bool CollisionAABB(ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool Collision(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool CollisionBoxBox(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool CollisionBoxCircle(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool CollisionBoxPolygon(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool CollisionCircleCircle(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool CollisionCirclePolygon(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	static bool CollisionPolygonPolygon(ep_CollisionData* data, ep_Shape* shape1, ep_Shape* shape2, double contact_threshold);
	
	static double RayCast(ep_Shape* shape, double x, double y, double vx, double vy);
	static double RayCastBox(ep_Shape* shape, double x, double y, double vx, double vy);
	static double RayCastCircle(ep_Shape* shape, double x, double y, double vx, double vy);
	static double RayCastPolygon(ep_Shape* shape, double x, double y, double vx, double vy);
	
	void Awake();
	
	static EP_FORCEINLINE bool CheckCollisionMasks(unsigned long shape1_collidemask1, unsigned long shape1_collidemask2, unsigned long shape1_group,
		unsigned long shape2_collidemask1, unsigned long shape2_collidemask2, unsigned long shape2_group) {
		return (((shape1_collidemask1&shape2_collidemask2)!=0 || (shape2_collidemask1&shape1_collidemask2)!=0)
			&& (shape1_group!=shape2_group || shape1_group==0));
	}
	
	// public functions
	public:
	
	/// Destroys all objects in the world and resets id counters.
	void Clear();
	
	/// Destroys a contact.
	/// Destroying contacts is only useful when done after calling UpdateContacts but before calling SimulateStep.
	/// @param contact Pointer to the contact to destroy.
	void DestroyContact(ep_Contact* contact);
	/// Finds the contact that corresponds to the given id.
	/// @param _id The id of the contact.
	/// @return Returns a pointer to the contact, or NULL if the contact does not exist.
	ep_Contact* FindContact(unsigned long _id);
	
	/// Creates a new polygon.
	/// @param vertexcount The number of vertices.
	/// @return Returns a pointer to the new polygon, or NULL if an error occurred.
	ep_Polygon* CreatePolygon(unsigned long vertexcount);
	/// Destroys a polygon.
	/// Warning: This function will fail if the polygon is still in use.
	/// @param polygon Pointer to the polygon to destroy.
	/// @return Returns whether successful.
	bool DestroyPolygon(ep_Polygon* polygon);
	/// Finds the polygon that corresponds to the given id.
	/// @param _id The id of the polygon.
	/// @return Returns a pointer to the polygon, or NULL if the polygon does not exist.
	ep_Polygon* FindPolygon(unsigned long _id);
	
	/// Creates a new static body.
	/// @return Returns a pointer to the new body, or NULL if an error occurred.
	ep_Body* CreateStaticBody();
	/// Creates a new dynamic body.
	/// @param norotation Indicates whether the body can rotate.
	/// @return Returns a pointer to the new body, or NULL if an error occurred.
	ep_Body* CreateDynamicBody(bool norotation);
	/// Destroys a body.
	/// This function will also destroy all shapes, forces and joints associated with this body.
	/// @param body Pointer to the body to destroy.
	void DestroyBody(ep_Body* body);
	/// Finds the body that corresponds to the given id.
	/// @param _id The id of the body.
	/// @return Returns a pointer to the body, or NULL if the body does not exist.
	ep_Body* FindBody(unsigned long _id);
	
	/// Creates a new hinge joint.
	/// @param _body1 Pointer to the first body.
	/// @param _body2 Pointer to the second body.
	/// @param _x1 The x component of the anchor point on the first body.
	/// @param _y1 The y component of the anchor point on the first body.
	/// @param _x2 The x component of the anchor point on the second body.
	/// @param _y2 The y component of the anchor point on the second body.
	/// @param _referencerotation Reference used to calculate the relative rotation of the bodies (rot1-rot2).
	/// @return Returns a pointer to the new hinge joint, or NULL if an error occurred.
	ep_HingeJoint* CreateHingeJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2, double y2, double referencerotation);
	/// Destroys a hinge joint.
	/// @param hingejoint Pointer to the hinge joint to destroy.
	void DestroyHingeJoint(ep_HingeJoint* hingejoint);
	/// Finds the hinge joint that corresponds to the given id.
	/// @param _id The id of the hinge joint.
	/// @return Returns a pointer to the hinge joint, or NULL if the hinge joint does not exist.
	ep_HingeJoint* FindHingeJoint(unsigned long _id);
	
	/// Creates a new distance joint.
	/// @param _body1 Pointer to the first body.
	/// @param _body2 Pointer to the second body.
	/// @param _x1 The x component of the anchor point on the first body.
	/// @param _y1 The y component of the anchor point on the first body.
	/// @param _x2 The x component of the anchor point on the second body.
	/// @param _y2 The y component of the anchor point on the second body.
	/// @return Returns a pointer to the new distance joint, or NULL if an error occurred.
	ep_DistanceJoint* CreateDistanceJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2, double y2);
	/// Destroys a distance joint.
	/// @param distancejoint Pointer to the distance joint to destroy.
	void DestroyDistanceJoint(ep_DistanceJoint* distancejoint);
	/// Finds the distance joint that corresponds to the given id.
	/// @param _id The id of the distance joint.
	/// @return Returns a pointer to the distance joint, or NULL if the hinge joint does not exist.
	ep_DistanceJoint* FindDistanceJoint(unsigned long _id);
	
	/// Creates a new rail joint.
	/// @param _body1 Pointer to the first body.
	/// @param _body2 Pointer to the second body.
	/// @param _x1 The x component of the anchor point on the first body.
	/// @param _y1 The y component of the anchor point on the first body.
	/// @param _x2a The x component of the first anchor point on the second body.
	/// @param _y2a The y component of the first anchor point on the second body.
	/// @param _x2b The x component of the second anchor point on the second body.
	/// @param _y2b The y component of the second anchor point on the second body.
	/// @return Returns a pointer to the new rail joint, or NULL if an error occurred.
	ep_RailJoint* CreateRailJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2a, double y2a, double x2b, double y2b);
	/// Destroys a rail joint.
	/// @param railjoint Pointer to the rail joint to destroy.
	void DestroyRailJoint(ep_RailJoint* railjoint);
	/// Finds the rail joint that corresponds to the given id.
	/// @param _id The id of the rail joint.
	/// @return Returns a pointer to the rail joint, or NULL if the rail joint does not exist.
	ep_RailJoint* FindRailJoint(unsigned long _id);
	
	/// Creates a new slider joint.
	/// @param body1 Pointer to the first body.
	/// @param body2 Pointer to the second body.
	/// @param x1 The x component of the anchor point on the first body.
	/// @param y1 The y component of the anchor point on the first body.
	/// @param x2a The x component of the first anchor point on the second body.
	/// @param y2a The y component of the first anchor point on the second body.
	/// @param x2b The x component of the second anchor point on the second body.
	/// @param y2b The y component of the second anchor point on the second body.
	/// @param rotation The relative rotation of the bodies (rot1-rot2).
	/// @return Returns a pointer to the new slider joint, or NULL if an error occurred.
	ep_SliderJoint* CreateSliderJoint(ep_Body* body1, ep_Body* body2, double x1, double y1, double x2a, double y2a, double x2b, double y2b, double rotation);
	/// Destroys a slider joint.
	/// @param railjoint Pointer to the slider joint to destroy.
	void DestroySliderJoint(ep_SliderJoint* sliderjoint);
	/// Finds the slider joint that corresponds to the given id.
	/// @param _id The id of the slider joint.
	/// @return Returns a pointer to the slider joint, or NULL if the slider joint does not exist.
	ep_SliderJoint* FindSliderJoint(unsigned long _id);
	
	//JOINTS//
	
	/// Creates a new view.
	/// @return Returns a pointer to the new view, or NULL if an error occurred.
	ep_View* CreateView();
	/// Destroys a view.
	/// @param world Pointer to the view to destroy.
	void DestroyView(ep_View* view);
	/// Finds the view that corresponds to the given id.
	/// @param id The id of the view.
	/// @return Returns a pointer to the view, or NULL if the world does not exist.
	ep_View* FindView(unsigned long _id);
	
	/// Creates a new water object.
	/// @return Returns a pointer to the new water object, or NULL if an error occurred.
	ep_Water* CreateWater();
	/// Destroys a water object.
	/// @param world Pointer to the water object to destroy.
	void DestroyWater(ep_Water* water);
	/// Finds the water object that corresponds to the given id.
	/// @param id The id of the water object.
	/// @return Returns a pointer to the view, or NULL if the world does not exist.
	ep_Water* FindWater(unsigned long _id);
	
	/// Changes the settings of the world.
	/// @param _timestep The length of one time step. The default value is 1.0.
	/// @param _velocity_iterations The number of iterations used by the velocity solver. Using a higher value increases accuracy. The default value is 20.
	/// @param _position_iterations The number of iterations used by the position solver. Using a higher value increases accuracy. The default value is 10.
	/// @param _contact_threshold The threshold value for collisions. This stops contacts from being destroyed immediately after creation and increases stability.
	/// @param _velocity_threshold The velocity that is lost during collisions. This compensates for the energy collisions add to the system when gravity is used.
	/// @param _baumgarte_factor The bias factor used by the Baumgarte stabilization. Using a lower value will increase stability but decrease accuracy. The default value is 0.1.
	/// @param _mass_bias The bias factor used to calculate the mass of constraints. The best value depends on the complexity of the simulation. The default value is 0.5.
	/// @param _position_factor The bias factor used by the position solver. Using a lower value will sometimes increase stability. The default value is 1.0.
	void SetSettings(double _timestep, unsigned long _velocity_iterations, unsigned long _position_iterations, double _contact_threshold, double _velocity_threshold, double _baumgarte_factor, double _mass_bias, double _position_factor);
	/// Changes the primary axis of the world.
	/// This doesn't change the results but the simulation will be faster if set correctly.
	/// Horizontal scanning is usually faster if there are lots of shapes with similar y-values.
	/// This is often the case when (vertical) gravity is used.
	/// @param _horizontal Indicates whether the world is mostly horizontal. The default value is true.
	void SetPrimaryAxis(bool _horizontal);
	/// Changes the sleeping settings of the world.
	/// @param _enable_sleeping Indicates whether sleeping should be enabled.
	/// @param _time_stable The time objects have to be stable before going to sleep. Set this to 0.0 to disable sleeping for stable objects.
	/// @param _time_outofview The time objects have to be out of view before going to sleep. Set this to 0.0 to disable sleeping for objects that are out of view.
	/// @param _stable_maxvel Bodies with a velocity lower than this value are considered stable.
	/// @param _stable_maxrotvel Bodies with a rotational velocity lower than this value are considered stable.
	void SetSleeping(bool _enable_sleeping, double _time_stable, double _time_outofview, double _stable_maxvel, double _stable_maxrotvel);
	/// Searches collisions and updates contacts.
	/// @return Returns whether successful.
	bool UpdateContacts();
	/// Simulates one time step. Remember to call UpdateContacts first.
	/// @return Returns whether successful.
	bool SimulateStep();
	
	/// Performs a collision test with a 'virtual' box shape.
	/// You can use ep_World::GetCollisionShape to get a pointer to the shapes that were found.
	/// @param w The width of the box.
	/// @param h The height of the box.
	/// @param _x The x coordinate of the box.
	/// @param _y The y coordinate of the box.
	/// @param _rot The rotation of the box.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @param collidemask1 The first collision mask.
	/// @param collidemask2 The second collision mask.
	/// @param group The group of the shape. Use zero for no group.
	/// @return Returns the number of collisions found.
	unsigned long CollisionTestBox(double w, double h, double _x, double _y, double _rot, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group);
	/// Performs a collision test with a 'virtual' line shape.
	/// You can use ep_World::GetCollisionShape to get a pointer to the shapes that were found.
	/// @param x1 The x coordinate of the first point of the line.
	/// @param y1 The x coordinate of the first point of the line.
	/// @param x2 The x coordinate of the second point of the line.
	/// @param y2 The y coordinate of the second point of the line.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @param collidemask1 The first collision mask.
	/// @param collidemask2 The second collision mask.
	/// @param group The group of the shape. Use zero for no group.
	/// @return Returns the number of collisions found.
	unsigned long CollisionTestLine(double x1, double y1, double x2, double y2, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group);
	/// Performs a collision test with a 'virtual' circle shape.
	/// You can use ep_World::GetCollisionShape to get a pointer to the shapes that were found.
	/// @param r The radius of the circle.
	/// @param _x The x coordinate of the circle.
	/// @param _y The y coordinate of the circle.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @param collidemask1 The first collision mask.
	/// @param collidemask2 The second collision mask.
	/// @param group The group of the shape. Use zero for no group.
	/// @return Returns the number of collisions found.
	unsigned long CollisionTestCircle(double r, double _x, double _y, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group);
	/// Performs a collision test with a 'virtual' polygon shape.
	/// You can use ep_World::GetCollisionShape to get a pointer to the shape that were found.
	/// @param polygon The polygon.
	/// @param _x The x coordinate of the polygon.
	/// @param _y The y coordinate of the polygon.
	/// @param _rot The rotation of the polygon.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @param collidemask1 The first collision mask.
	/// @param collidemask2 The second collision mask.
	/// @param group The group of the shape. Use zero for no group.
	/// @return Returns the number of collisions found.
	unsigned long CollisionTestPolygon(ep_Polygon* polygon, double _x, double _y, double _rot, double contact_threshold, unsigned long collidemask1, unsigned long collidemask2, unsigned long group);
	/// Performs a ray cast.
	/// You can use ep_World::GetCollisionShape to get a pointer to the shape that was found.
	/// @param _x The x coordinate of the starting point of the ray.
	/// @param _y The y coordinate of the starting point of the ray.
	/// @param vx The x component of the direction vector of the ray.
	/// @param vy The y component of the direction vector of the ray.
	/// @param collidemask1 The first collision mask.
	/// @param collidemask2 The second collision mask.
	/// @param group The group of the ray. Use zero for no group.
	/// @return Returns the distance to the point of intersection, or -1.0 if there was no intersection.
	double RayCast(double _x, double _y, double vx, double vy, unsigned long collidemask1, unsigned long collidemask2, unsigned long group);
	
	/// Returns a pointer to the shape that was found by a collision/raycast function.
	ep_Shape* GetCollisionShape(unsigned long index);
	
#if EP_COMPILE_DEBUGDRAW
	
	/// Draws all bodies in this world using the callback class.
	void DebugDrawBodies();
	/// Draws all pairs (contacts and joints) in this world using the callback class.
	void DebugDrawLinks();
	/// Draws all views in this world using the callback class.
	void DebugDrawViews();
	/// Draws the velocity of all bodies in this world using the callback class.
	void DebugDrawVelocity();
	/// Draws all forces (including contacts and joints) in this world using the callback class.
	void DebugDrawForces();
	/// Draws all points (contacts and joints) in this world using the callback class.
	void DebugDrawConstraints();
	
#endif
	
#if EP_COMPILE_MULTIPOLY
	
	private:
	bool Multipoly_Decompose(bool showerrors);
	void Multipoly_CancelDecompose();
	int Multipoly_MakeTask(ep_Multipoly_Task* result, ep_Multipoly_Cut* cuts, unsigned long first, unsigned long last, ep_Polygon** p, bool showerrors);
	bool Multipoly_IsValidCut(unsigned long i, unsigned long j);
	void Multipoly_CalculateRating(ep_Multipoly_Cut* c);
	
	public:
	/// Begins a new multipolygon
	bool MultipolyBegin(unsigned long vertexcount);
	/// Ends a multipolygon and creates the polygons.
	bool MultipolyEnd(bool showerrors);
	/// Sets the coordinates of a vertex of the multipolygon.
	bool MultipolySetVertex(unsigned long index, double x, double y);
	/// Returns a pointer to the first polygon.
	inline ep_Polygon* MultipolyGetFirst() { return multipoly_first_polygon; }
	/// Returns a pointer to the last polygon.
	inline ep_Polygon* MultipolyGetLast() { return multipoly_last_polygon; }
	
#endif // EP_COMPILE_MULTIPOLY
	
#if EP_COMPILE_SERIALIZE
	
	public:
	// Serializes the world. The state of this world and all related objects (polygons, bodies, contacts, joints, ...) is saved.
	bool Serialize();
	// Frees the serialized data. This is done automatically when the world is destroyed.
	void FreeSerializeData();
	// Unserializes the world.
	bool Unserialize(const char* data, unsigned long length);
	// Returns the serialized data.
	inline const char* GetSerializeData() { return serializedata; }
	// Returns the length of the serialized data.
	inline unsigned long GetSerializeDataLength() { return serializedata_length; }
	
#endif // EP_COMPILE_SERIALIZE
	
	// public inline functions
	public:
	
	/// Returns the id of this world.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this world to speed up the next FindWorld call (if the id matches the id of this world).
	inline void CacheID() { main->current_world = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this world.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the previous world.
	inline ep_World* GetPrevious() { return prev; }
	/// Returns a pointer to the next world.
	inline ep_World* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
	/// Returns a pointer to the first polygon.
	inline ep_Polygon* GetFirstPolygon() { return first_polygon; }
	/// Returns a pointer to the last polygon.
	inline ep_Polygon* GetLastPolygon() { return last_polygon; }
	/// Returns the number of polygons in this world.
	inline unsigned long GetPolygonCount() { return polygoncount; }
	
	/// Returns a pointer to the first body.
	inline ep_Body* GetFirstBody() { return first_body; }
	/// Returns a pointer to the last body.
	inline ep_Body* GetLastBody() { return last_body; }
	/// Returns the number of bodies in this world.
	inline unsigned long GetBodyCount() { return bodycount; }
	
	/// Returns a pointer to the first contact.
	inline ep_Contact* GetFirstContact() { return first_contact; }
	/// Returns a pointer to the last contact.
	inline ep_Contact* GetLastContact() { return last_contact; }
	/// Returns the number of contacts in this world.
	inline unsigned long GetContactCount() { return contactcount; }
	
	/// Returns a pointer to the first hinge joint.
	inline ep_HingeJoint* GetFirstHingeJoint() { return first_hingejoint; }
	/// Returns a pointer to the last hinge joint.
	inline ep_HingeJoint* GetLastHingeJoint() { return last_hingejoint; }
	/// Returns the number of hinge joints in this world.
	inline unsigned long GetHingeJointCount() { return hingejointcount; }
	
	/// Returns a pointer to the first distance joint.
	inline ep_DistanceJoint* GetFirstDistanceJoint() { return first_distancejoint; }
	/// Returns a pointer to the last distance joint.
	inline ep_DistanceJoint* GetLastDistanceJoint() { return last_distancejoint; }
	/// Returns the number of distance joints in this world.
	inline unsigned long GetDistanceJointCount() { return distancejointcount; }
	
	/// Returns a pointer to the first rail joint.
	inline ep_RailJoint* GetFirstRailJoint() { return first_railjoint; }
	/// Returns a pointer to the last rail joint.
	inline ep_RailJoint* GetLastRailJoint() { return last_railjoint; }
	/// Returns the number of rail joints in this world.
	inline unsigned long GetRailJointCount() { return railjointcount; }
	
	/// Returns a pointer to the first slider joint.
	inline ep_SliderJoint* GetFirstSliderJoint() { return first_sliderjoint; }
	/// Returns a pointer to the last slider joint.
	inline ep_SliderJoint* GetLastSliderJoint() { return last_sliderjoint; }
	/// Returns the number of slider joints in this world.
	inline unsigned long GetSliderJointCount() { return sliderjointcount; }
	
	//JOINTS//
	
	/// Returns the number of shapes in this world.
	inline unsigned long GetShapeCount() { return shapecount; }
	/// Returns the number of forces in this world.
	inline unsigned long GetForceCount() { return forcecount; }
	
	/// Returns a pointer to the first view.
	inline ep_View* GetFirstView() { return first_view; }
	/// Returns a pointer to the last view.
	inline ep_View* GetLastView() { return last_view; }
	/// Returns the number of views in this world.
	inline unsigned long GetViewCount() { return viewcount; }
	
	/// Returns a pointer to the first water object.
	inline ep_Water* GetFirstWater() { return first_water; }
	/// Returns a pointer to the last water object.
	inline ep_Water* GetLastWater() { return last_water; }
	/// Returns the number of water objects in this world.
	inline unsigned long GetWaterCount() { return watercount; }
	
};

#endif // EP_WORLD_H

