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
 * File: ep_Body.h
 * Definition of ep_Body.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_BODY_H
#define EP_BODY_H

#if EP_COMPILE_BOXCHAIN
struct ep_BoxChain_Vertex {
	double x, y;
	double len;
	double vx, vy, t;
};
#endif // EP_COMPILE_BOXCHAIN

#if EP_USE_IDHASHTABLE
class ep_Body : private ep_IdHashTableEntry {
#else
class ep_Body {
#endif
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Contact;
	friend class ep_HingeJoint;
	friend class ep_DistanceJoint;
	friend class ep_RailJoint;
	friend class ep_SliderJoint;
	//JOINTS//
	friend class ep_Shape;
	friend class ep_Force;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_Body *prev, *next;
#if !EP_USE_IDHASHTABLE
	unsigned long id;
#endif
	
	unsigned long idcounter_shapes, shapecount;
	ep_Shape *first_shape, *last_shape, *current_shape;
	
	unsigned long idcounter_forces, forcecount;
	ep_Force *first_force, *last_force, *current_force;
	
	bool isstatic, norotation;
	
	double xoff, yoff;
	double mass, invmass;
	double inertia, invinertia;
	
	double x, y, rot, rot_sin, rot_cos;
	double xvel, yvel, rotvel;
	
	double maxvel, maxrotvel;
	double gravity_x, gravity_y;
	double damping, rotdamping, damping_factor, rotdamping_factor;
	
	bool storecontactimpulses, storejointimpulses;
	bool sleepstable, sleepoutofview, issleeping;
	double stabletimer, outofviewtimer;
	
	union {
		ep_Body *nextislandbody;
#if EP_COMPILE_SERIALIZE
		unsigned long nextislandbody_id;
#endif // EP_COMPILE_SERIALIZE
	};
	
	ep_HingeJointLink *first_hingejointlink, *last_hingejointlink;
	ep_DistanceJointLink *first_distancejointlink, *last_distancejointlink;
	ep_RailJointLink *first_railjointlink, *last_railjointlink;
	ep_SliderJointLink *first_sliderjointlink, *last_sliderjointlink;
	//JOINTS//
	
#if EP_COMPILE_BOXCHAIN
	unsigned long boxchain_vertexcount;
	ep_BoxChain_Vertex *boxchain_vertices;
	ep_Shape *boxchain_first_shape, *boxchain_last_shape;
#endif // EP_COMPILE_BOXCHAIN
	
	// private functions
	private:
	
	void Init(ep_World* _world, bool _isstatic, bool _norotation, unsigned long _id);
	void DeInit();
	
	void Awake(bool linkagechange, bool propertychange);
	void Moved();
	
	EP_FORCEINLINE void _ApplyImpulse(double xx, double yy, double xforce, double yforce) {
		xvel += xforce*invmass;
		yvel += yforce*invmass;
		rotvel += (yy*xforce-xx*yforce)*invinertia;
	}
	EP_FORCEINLINE void _ApplyTorque(double torque) {
		rotvel += torque*invinertia;
	}
	EP_FORCEINLINE void _ApplyImpulseTorque(double xx, double yy, double xforce, double yforce, double torque) {
		xvel += xforce*invmass;
		yvel += yforce*invmass;
		rotvel += (yy*xforce-xx*yforce+torque)*invinertia;
	}
	EP_FORCEINLINE void _ApplyPseudoImpulse(double xx, double yy, double xforce, double yforce) {
		x += xforce*invmass;
		y += yforce*invmass;
		rot += (yy*xforce-xx*yforce)*invinertia;
		rot_sin = sin(rot);
		rot_cos = cos(rot);
	}
	EP_FORCEINLINE void _ApplyPseudoTorque(double torque) {
		rot += torque*invinertia;
		rot_sin = sin(rot);
		rot_cos = cos(rot);
	}
	EP_FORCEINLINE void _ApplyPseudoImpulseTorque(double xx, double yy, double xforce, double yforce, double torque) {
		x += xforce*invmass;
		y += yforce*invmass;
		rot += (yy*xforce-xx*yforce+torque)*invinertia;
		rot_sin = sin(rot);
		rot_cos = cos(rot);
	}
	
	// public functions
	public:
	
	/// Creates a new box shape.
	/// @param _w The width of the box.
	/// @param _h The height of the box.
	/// @param _x The x coordinate of the center of the shape relative to the body.
	/// @param _y The y coordinate of the center of the shape relative to the body.
	/// @param _rot The rotation of the shape relative to the body.
	/// @param _density The density of the shape.
	/// @return Returns a pointer to the new shape, or NULL if an error occurred.
	ep_Shape* CreateBoxShape(double _w, double _h, double _x, double _y, double _rot, double _density);
	/// Creates a new line shape. This is actually a rotated box shape with height 0.
	/// @param x1 The x coordinate of the first point of the line.
	/// @param y1 The x coordinate of the first point of the line.
	/// @param x2 The x coordinate of the second point of the line.
	/// @param y2 The y coordinate of the second point of the line.
	/// @param _density The density of the shape.
	/// @return Returns a pointer to the new shape, or NULL if an error occurred.
	ep_Shape* CreateLineShape(double x1, double y1, double x2, double y2, double _density);
	/// Creates a new circle shape.
	/// @param _r The radius of the circle.
	/// @param _x The x coordinate of the center of the shape relative to the body.
	/// @param _y The y coordinate of the center of the shape relative to the body.
	/// @param _rot The rotation of the shape relative to the body.
	/// @param _density The density of the shape.
	/// @return Returns a pointer to the new shape, or NULL if an error occurred.
	ep_Shape* CreateCircleShape(double _r, double _x, double _y, double _rot, double _density);
	/// Creates a new polygon shape.
	/// @param polygon Pointer to the polygon.
	/// @param _x The x coordinate of the center of the shape relative to the body.
	/// @param _y The y coordinate of the center of the shape relative to the body.
	/// @param _rot The rotation of the shape relative to the body.
	/// @param _density The density of the shape.
	/// @return Returns a pointer to the new shape, or NULL if an error occurred.
	ep_Shape* CreatePolygonShape(ep_Polygon* polygon, double _x, double _y, double _rot, double _density);
	/// Destroys a shape.
	/// @param world Pointer to the shape to destroy.
	void DestroyShape(ep_Shape* shape);
	/// Finds the shape that corresponds to the given id.
	/// @param id The id of the shape.
	/// @return Returns a pointer to the shape, or NULL if the shape does not exist.
	ep_Shape* FindShape(unsigned long _id);
	
	/// Creates a new force.
	/// @param _x The x coordinate of the local point.
	/// @param _y The y coordinate of the local point.
	/// @param _local Indicates whether the force is local.
	/// @param _ignoremass Indicates whether the mass of the body should be ignored.
	/// @return Returns a pointer to the new world, or NULL if an error occurred.
	ep_Force* CreateForce(double _x, double _y, bool _local, bool _ignoremass);
	/// Destroys a force.
	/// @param force Pointer to the force to destroy.
	void DestroyForce(ep_Force* force);
	/// Finds the force that corresponds to the given id.
	/// @param id The id of the force.
	/// @return Returns a pointer to the force, or NULL if the force does not exist.
	ep_Force* FindForce(unsigned long _id);
	
	/// Returns a pointer to the first hinge joint connected to this body.
	/// @return Returns a pointer to the hinge joint, or NULL if no hinge joint is found.
	ep_HingeJoint* GetFirstHingeJoint();
	/// Returns a pointer to the last hinge joint connected to this body.
	/// @return Returns a pointer to the hinge joint, or NULL if no hinge joint is found.
	ep_HingeJoint* GetLastHingeJoint();
	/// Returns a pointer to the next hinge joint connected to this body.
	/// @param hingejoint Pointer to the current hinge joint.
	/// @return Returns a pointer to the hinge joint, or NULL if no hinge joint is found.
	ep_HingeJoint* GetPreviousHingeJoint(ep_HingeJoint* hingejoint);
	/// Returns a pointer to the previous hinge joint connected to this body.
	/// @param hingejoint Pointer to the current hinge joint.
	/// @return Returns a pointer to the hinge joint, or NULL if no hinge joint is found.
	ep_HingeJoint* GetNextHingeJoint(ep_HingeJoint* hingejoint);
	
	/// Returns a pointer to the first distance joint connected to this body.
	/// @return Returns a pointer to the distance joint, or NULL if no distance joint is found.
	ep_DistanceJoint* GetFirstDistanceJoint();
	/// Returns a pointer to the last distance joint connected to this body.
	/// @return Returns a pointer to the distance joint, or NULL if no distance joint is found.
	ep_DistanceJoint* GetLastDistanceJoint();
	/// Returns a pointer to the next distance joint connected to this body.
	/// @param distancejoint Pointer to the current distance joint.
	/// @return Returns a pointer to the distance joint, or NULL if no distance joint is found.
	ep_DistanceJoint* GetPreviousDistanceJoint(ep_DistanceJoint* distancejoint);
	/// Returns a pointer to the previous distance joint connected to this body.
	/// @param distancejoint Pointer to the current distance joint.
	/// @return Returns a pointer to the distance joint, or NULL if no distance joint is found.
	ep_DistanceJoint* GetNextDistanceJoint(ep_DistanceJoint* distancejoint);
	
	/// Returns a pointer to the first rail joint connected to this body.
	/// @return Returns a pointer to the rail joint, or NULL if no rail joint is found.
	ep_RailJoint* GetFirstRailJoint();
	/// Returns a pointer to the last rail joint connected to this body.
	/// @return Returns a pointer to the rail joint, or NULL if no rail joint is found.
	ep_RailJoint* GetLastRailJoint();
	/// Returns a pointer to the next rail joint connected to this body.
	/// @param railjoint Pointer to the current rail joint.
	/// @return Returns a pointer to the rail joint, or NULL if no rail joint is found.
	ep_RailJoint* GetPreviousRailJoint(ep_RailJoint* railjoint);
	/// Returns a pointer to the previous rail joint connected to this body.
	/// @param railjoint Pointer to the current rail joint.
	/// @return Returns a pointer to the rail joint, or NULL if no rail joint is found.
	ep_RailJoint* GetNextRailJoint(ep_RailJoint* railjoint);
	
	/// Returns a pointer to the first slider joint connected to this body.
	/// @return Returns a pointer to the slider joint, or NULL if no slider joint is found.
	ep_SliderJoint* GetFirstSliderJoint();
	/// Returns a pointer to the last slider joint connected to this body.
	/// @return Returns a pointer to the slider joint, or NULL if no slider joint is found.
	ep_SliderJoint* GetLastSliderJoint();
	/// Returns a pointer to the next slider joint connected to this body.
	/// @param sliderjoint Pointer to the current slider joint.
	/// @return Returns a pointer to the slider joint, or NULL if no slider joint is found.
	ep_SliderJoint* GetPreviousSliderJoint(ep_SliderJoint* sliderjoint);
	/// Returns a pointer to the previous slider joint connected to this body.
	/// @param sliderjoint Pointer to the current slider joint.
	/// @return Returns a pointer to the slider joint, or NULL if no slider joint is found.
	ep_SliderJoint* GetNextSliderJoint(ep_SliderJoint* sliderjoint);
	
	//JOINTS//
	
	/// Calculates the mass, moment of inertia and center of mass of this body based on its shapes.
	/// @return Returns whether successful.
	bool CalculateMass();
	/// Changes the mass of the body.
	/// @param _mass The new mass.
	/// @return Returns whether successful.
	bool SetMass(double _mass);
	/// Changes the moment of inertia of the body.
	/// @param _mass The new moment of inertia.
	/// @return Returns whether successful.
	bool SetInertia(double _inertia);
	/// Changes the center of mass of the body.
	/// @param localx The x coordinate of the new center of mass.
	/// @param localy The y coordinate of the new center of mass.
	/// @param updateinertia Indicates whether to update the moment of inertia (using the parallel axis theorem).
	/// @return Returns whether successful.
	bool SetCenter(double localx, double localy, bool updateinertia);
	/// Changes the position of the body using the origin of the body as the reference.
	/// @param _x The x coordinate of the new position.
	/// @param _y The y coordinate of the new position.
	/// @param _rot The new rotation of the body.
	void SetPosition(double _x, double _y, double _rot);
	/// Changes the position of the body using the center of mass of the body as the reference.
	/// @param _x The x coordinate of the new position.
	/// @param _y The y coordinate of the new position.
	/// @param _rot The new rotation of the body.
	void SetPositionCenter(double _x, double _y, double _rot);
	/// Changes the position of the body using a local point of the body as the reference.
	/// @param _x The x coordinate of the new position.
	/// @param _y The y coordinate of the new position.
	/// @param _rot The new rotation of the body.
	/// @param localx The x coordinate of the reference point (local).
	/// @param localy The y coordinate of the reference point (local).
	void SetPositionLocalPoint(double _x, double _y, double _rot, double localx, double localy);
	/// Changes the velocity of the body using the center of mass of the body as the reference.
	/// @param _xvel The x component of the new velocity of the body.
	/// @param _yvel The y component of the new velocity of the body.
	/// @param _rotvel The new rotational velocity of the body.
	bool SetVelocityCenter(double _xvel, double _yvel, double _rotvel);
	/// Changes the velocity of the body using a local point of the body as the reference.
	/// @param _xvel The x component of the new velocity of the body.
	/// @param _yvel The new y velocity of the body.
	/// @param _rotvel The new rotational velocity of the body.
	/// @param localx The x coordinate of the reference point (local).
	/// @param localy The y coordinate of the reference point (local).
	bool SetVelocityLocalPoint(double _xvel, double _yvel, double _rotvel, double localx, double localy);
	
	/// Changes the maximum velocity of the body.
	/// @param _maxvel The new maximum velocity. Use 0.0 for no limit.
	/// @param _maxrotvel The new maximum rotational velocity. Use 0.0 for no limit.
	bool SetMaxVelocity(double _maxvel, double _maxrotvel);
	/// Changes the gravity of the body, ignoring the mass.
	/// @param _gravity_x The x component of the new gravity of the body.
	/// @param _gravity_y The y component of the new gravity of the body.
	bool SetGravity(double _gravity_x, double _gravity_y);
	/// Changes the damping factors of the body.
	/// @param _damping The new damping factor of the body.
	/// @param _rotdamping The new rotational damping factor of the body.
	bool SetDamping(double _damping, double _rotdamping);
	/// Changes whether impulses are stored for the body.
	/// Impulses are stored only when this setting is set to true for both bodies.
	/// @param _storecontactimpulses Indicates whether contact impulses are stored. The default value is true.
	/// @param _storejointimpulses Indicates whether joint impulses are stored. The default value is true.
	void StoreImpulses(bool _storecontactimpulses, bool _storejointimpulses);
	/// Changes the sleeping settings of the body.
	/// These settings are ignored if sleeping when stable or sleeping out of view is disabled for the world.
	/// @param _sleepstable Indicates whether the body should sleep when it is stable. The default value is true.
	/// @param _sleepoutofview Indicates whether the body should sleep when it is out of view. The default value is true.
	bool SetSleeping(bool _sleepstable, bool _sleepoutofview);
	
	/// Performs a collision test with a 'virtual' box shape.
	/// You can use ep_World::GetCollisionShape to get a pointer to the shape that was found.
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
	/// You can use ep_World::GetCollisionShape to get a pointer to the shape that was found.
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
	/// You can use ep_World::GetCollisionShape to get a pointer to the shape that was found.
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
	/// You can use ep_World::GetCollisionShape to get a pointer to the shape that was found.
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
	
#if EP_COMPILE_BOXCHAIN
	/// Begins a new box chain.
	/// @param vertexcount The number of vertices.
	bool BoxChainBegin(unsigned long vertexcount);
	/// Ends the box chain and creates the box shapes.
	/// @param circular Indicates whether this is a circular box chain.
	/// @param ignorefirstlast Indicates whether the first and last boxes should be ignored. This is not used for circular box chains.
	/// @param width_top The width of the top of the box chain when the box chain is defined left-to-right. This can be negative.
	/// @param width_bottom The width of the bottom of the box chain when the box chain is defined left-to-right. This can be negative.
	/// @param density The density of the shapes.
	bool BoxChainEnd(bool circular, bool ignorefirstlast, double width_top, double width_bottom, double density);
	/// Sets the coordinates of a vertex of the box chain.
	bool BoxChainSetVertex(unsigned long index, double x, double y);
	/// Returns a pointer to the first box chain shape.
	inline ep_Shape* BoxChainGetFirst() { return boxchain_first_shape; }
	/// Returns a pointer to the last box chain shape.
	inline ep_Shape* BoxChainGetLast() { return boxchain_last_shape; }
#endif // EP_COMPILE_BOXCHAIN
	
	// public inline functions
	public:
	
	/// Returns the id of this body.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this body to speed up the next FindBody call (if the id matches the id of this body).
	inline void CacheID() { world->current_body = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this body.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this body.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous body.
	inline ep_Body* GetPrevious() { return prev; }
	/// Returns a pointer to the next body.
	inline ep_Body* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
	/// Returns a pointer to the first shape.
	inline ep_Shape* GetFirstShape() { return first_shape; }
	/// Returns a pointer to the last shape.
	inline ep_Shape* GetLastShape() { return last_shape; }
	/// Returns the number of shapes for this body.
	inline unsigned long GetShapeCount() { return shapecount; }
	
	/// Returns a pointer to the first polygon.
	inline ep_Force* GetFirstForce() { return first_force; }
	/// Returns a pointer to the last polygon.
	inline ep_Force* GetLastForce() { return last_force; }
	/// Returns the number of forces for this body.
	inline unsigned long GetForceCount() { return forcecount; }
	
	/// Returns the mass of the body.
	inline double GetMass() { return mass; }
	/// Returns the moment of inertia of the body.
	inline double GetInertia() { return inertia; }
	/// Returns the x component of the center of mass of the body.
	inline double GetCenterOfMassX() { return xoff; }
	/// Returns the y component of the center of mass of the body.
	inline double GetCenterOfMassY() { return yoff; }
	
	/// Returns the x coordinate of the body using the origin as the reference.
	inline double GetX() { return x-ep_transform_x(rot_sin, rot_cos, xoff, yoff); }
	/// Returns the y coordinate of the body using the origin as the reference.
	inline double GetY() { return y-ep_transform_y(rot_sin, rot_cos, xoff, yoff); }
	/// Returns the x coordinate of the body using the center of mass as the reference.
	inline double GetXCenter() { return x; }
	/// Returns the y coordinate of the body using the center of mass as the reference.
	inline double GetYCenter() { return y; }
	/// Returns the rotation of the body.
	inline double GetRot() { return rot; }
	
	/// Returns the x velocity of the body using the origin as the reference.
	inline double GetXVelCenter() { return xvel; }
	/// Returns the y velocity of the body using the origin as the reference.
	inline double GetYVelCenter() { return yvel; }
	/// Returns the x coordinate of the body using a local point as the reference.
	inline double GetXVelLocalPoint(double localx, double localy) { return xvel+ep_transform_y(rot_sin, rot_cos, localx-xoff, localy-yoff)*rotvel; }
	/// Returns the y coordinate of the body using a local point as the reference.
	inline double GetYVelLocalPoint(double localx, double localy) { return yvel-ep_transform_x(rot_sin, rot_cos, localx-xoff, localy-yoff)*rotvel; }
	/// Returns the rotational velocity of the body.
	inline double GetRotVel() { return rotvel; }
	
	/// Returns whether the body is static.
	inline bool IsStatic() { return isstatic; }
	/// Returns whether the body can't rotate.
	inline bool IsNoRotation() { return norotation; }
	/// Returns whether the body is sleeping.
	inline bool IsSleeping() { return issleeping; }
	/// Returns the time this body has been stable.
	inline double StableTimer() { return stabletimer; }
	/// Returns the time this body has been out of view.
	inline double OutOfViewTimer() { return outofviewtimer; }
	
	/// Coordinate and vector conversions.
	inline double CoordLocalToWorldX(double localx, double localy) { return x+ep_transform_x(rot_sin, rot_cos, localx-xoff, localy-yoff); }
	inline double CoordLocalToWorldY(double localx, double localy) { return y+ep_transform_y(rot_sin, rot_cos, localx-xoff, localy-yoff); }
	inline double CoordWorldToLocalX(double worldx, double worldy) { return xoff+ep_invtransform_x(rot_sin, rot_cos, worldx-x, worldy-y); }
	inline double CoordWorldToLocalY(double worldx, double worldy) { return yoff+ep_invtransform_y(rot_sin, rot_cos, worldx-x, worldy-y); }
	inline double VectLocalToWorldX(double vx, double vy) { return ep_transform_x(rot_sin, rot_cos, vx, vy); }
	inline double VectLocalToWorldY(double vx, double vy) { return ep_transform_y(rot_sin, rot_cos, vx, vy); }
	inline double VectWorldToLocalX(double vx, double vy) { return ep_invtransform_x(rot_sin, rot_cos, vx, vy); }
	inline double VectWorldToLocalY(double vx, double vy) { return ep_invtransform_y(rot_sin, rot_cos, vx, vy); }
	
	/// Applies an impulse.
	/// @param localx The x coordinate of the point of application (local coordinates).
	/// @param localy The y coordinate of the point of application (local coordinates).
	/// @param xforce The x component of the force.
	/// @param yforce The y component of the force.
	/// @param torque The torque.
	/// @param local Indicates whether the force is local.
	/// @param ignoremass Indicates whether the mass and moment of inertia of the body should be ignored.
	/// @param awake Indicates whether the body should be awoken.
	inline void ApplyImpulse(double localx, double localy, double xforce, double yforce, double torque, bool local, bool ignoremass, bool awake) {
		if(!awake && issleeping) return;
		if(local) {
			xvel += ep_transform_x(rot_sin, rot_cos, xforce, yforce)*((ignoremass)? 1.0 : invmass);
			yvel += ep_transform_y(rot_sin, rot_cos, xforce, yforce)*((ignoremass)? 1.0 : invmass);
			rotvel +=
			  ((localy-yoff)*xforce-(localx-xoff)*yforce)*invinertia*((ignoremass)? mass : 1.0)
			  +torque*((ignoremass)? 1.0 : invinertia);
		} else {
			xvel += xforce*((ignoremass)? 1.0 : invmass);
			yvel += yforce*((ignoremass)? 1.0 : invmass);
			rotvel +=
			  ((localy-yoff)*ep_invtransform_x(rot_sin, rot_cos, xforce, yforce)
			  -(localx-xoff)*ep_invtransform_y(rot_sin, rot_cos, xforce, yforce))*invinertia*((ignoremass)? mass : 1.0)
			  +torque*((ignoremass)? 1.0 : invinertia);
		}
		if(awake) Awake(false, false);
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Applied impulse to body %lu in world %lu.", id, world->id);
#endif
	}
	
	/// Applies an impulse to a point relative to the center of mass.
	/// @param relativex The x component of the point of application relative to the center of mass.
	/// @param relativey The y component of the point of application relative to the center of mass.
	/// @param xforce The x component of the force.
	/// @param yforce The y component of the force.
	/// @param torque The torque.
	/// @param ignoremass Indicates whether the mass and moment of inertia of the body should be ignored.
	/// @param awake Indicates whether the body should be awoken.
	inline void ApplyImpulseRelative(double relativex, double relativey, double xforce, double yforce, double torque, bool ignoremass, bool awake) {
		if(!awake && issleeping) return;
		xvel += xforce*((ignoremass)? 1.0 : invmass);
		yvel += yforce*((ignoremass)? 1.0 : invmass);
		rotvel +=
		  (relativey*xforce-relativex*yforce)*invinertia*((ignoremass)? mass : 1.0)
		  +torque*((ignoremass)? 1.0 : invinertia);
		if(awake) Awake(false, false);
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Applied impulse to body %lu in world %lu.", id, world->id);
#endif
	}
	
};

#endif // EP_BODY_H

