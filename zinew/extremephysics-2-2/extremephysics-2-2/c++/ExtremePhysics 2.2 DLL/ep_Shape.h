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
 * File: ep_Shape.h
 * Definition of ep_Shape.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_SHAPE_H
#define EP_SHAPE_H

class ep_Shape {
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Body;
	friend class ep_Contact;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_Body *body;
	ep_Shape *prev, *next;
	unsigned long id;
	
	int shapetype;
	union {
		struct {
			double w, h;
		} box;
		struct {
			double r;
		} circle;
		struct {
			ep_Polygon *polygon;
		} polygon;
	} shapedata;
	
	double x, y, rot, rot_sin, rot_cos, density;
	
	float restitution, friction;
	double normalvelocity, tangentvelocity;
	unsigned long collidemask1, collidemask2, group;
	
	bool updateaabb, updatecontacts;
	double aabb_x1, aabb_y1, aabb_x2, aabb_y2;
	ep_uint32 aabbul_x1, aabbul_y1, aabbul_x2, aabbul_y2;
	double x_t, y_t, sin_t, cos_t;
	ep_Contact *temp_c;
	
	ep_ContactLink *first_contactlink, *last_contactlink;
	
	// private functions
	private:
	
	void Init(ep_Body* _body, int _shapetype, double _x, double _y, double _rot, double _density, double arg1, double arg2, ep_Polygon* arg3, unsigned long _id);
	void DeInit();
	
	void UpdateTransformedCoordinates();
	void UpdateAABB();
	void UpdateAABBInternal();
	void InitVirtualBox(double w, double h, double _x, double _y, double rot);
	void InitVirtualLine(double x1, double y1, double x2, double y2);
	void InitVirtualCircle(double r, double _x, double _y);
	bool InitVirtualPolygon(ep_Polygon* polygon, double _x, double _y, double rot);
	
	// public functions
	public:
	
	/// Returns a pointer to the first contact connected to this shape.
	/// @return Returns a pointer to the contact, or NULL if no contact is found.
	ep_Contact* GetFirstContact();
	/// Returns a pointer to the last contact connected to this shape.
	/// @return Returns a pointer to the contact, or NULL if no contact is found.
	ep_Contact* GetLastContact();
	/// Returns a pointer to the next contact connected to this shape.
	/// @param hingejoint Pointer to the current contact.
	/// @return Returns a pointer to the contact, or NULL if no contact is found.
	ep_Contact* GetPreviousContact(ep_Contact* contact);
	/// Returns a pointer to the previous contact connected to this shape.
	/// @param hingejoint Pointer to the current contact.
	/// @return Returns a pointer to the contact, or NULL if no contact is found.
	ep_Contact* GetNextContact(ep_Contact* contact);
	
	/// Changes the material of the shape.
	/// @param _restitution The coefficient of restitution.
	/// @param _friction The coefficient of friction.
	/// @param _normalvelocity The normal velocity of the surface.
	/// @param _tangentvelocity The tangent velocity of the surface.
	void SetMaterial(float _restitution, float _friction, double _normalvelocity, double _tangentvelocity);
	/// Changes the collision settings of the shape.
	/// @param _collidemask1 The first collision mask.
	/// @param _collidemask2 The second collision mask.
	/// @param _group The group of the shape. Use zero for no group.
	void SetCollision(unsigned long _collidemask1, unsigned long _collidemask2, unsigned long _group);
	
	/// Performs a collision test with a 'virtual' box shape.
	/// @param w The width of the box.
	/// @param h The height of the box.
	/// @param _x The x coordinate of the box.
	/// @param _y The y coordinate of the box.
	/// @param _rot The rotation of the box.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @return Returns whether a collision was found.
	bool CollisionTestBox(double w, double h, double _x, double _y, double _rot, double contact_threshold);
	/// Performs a collision test with a 'virtual' line shape.
	/// @param x1 The x coordinate of the first point of the line.
	/// @param y1 The x coordinate of the first point of the line.
	/// @param x2 The x coordinate of the second point of the line.
	/// @param y2 The y coordinate of the second point of the line.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @return Returns whether a collision was found.
	bool CollisionTestLine(double x1, double y1, double x2, double y2, double contact_threshold);
	/// Performs a collision test with a 'virtual' circle shape.
	/// @param r The radius of the circle.
	/// @param _x The x coordinate of the circle.
	/// @param _y The y coordinate of the circle.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @return Returns whether a collision was found.
	bool CollisionTestCircle(double r, double _x, double _y, double contact_threshold);
	/// Performs a collision test with a 'virtual' polygon shape.
	/// @param polygon The polygon.
	/// @param _x The x coordinate of the polygon.
	/// @param _y The y coordinate of the polygon.
	/// @param _rot The rotation of the polygon.
	/// @param contact_threshold The contact threshold used in the collision test.
	/// @return Returns whether a collision was found.
	bool CollisionTestPolygon(ep_Polygon* polygon, double _x, double _y, double _rot, double contact_threshold);
	/// Performs a ray cast.
	/// @param _x The x coordinate of the starting point of the ray.
	/// @param _y The y coordinate of the starting point of the ray.
	/// @param vx The x component of the direction vector of the ray.
	/// @param vy The y component of the direction vector of the ray.
	/// @return Returns the distance to the point of intersection, or -1.0 if there was no intersection.
	double RayCast(double _x, double _y, double vx, double vy);
	
	// inline functions
	public:
	
	/// Returns the id of this shape.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this shape to speed up the next FindShape call (if the id matches the id of this shape).
	inline void CacheID() { body->current_shape = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this shape.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this shape.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the body associated with this shape.
	inline ep_Body* GetBody() { return body; }
	/// Returns a pointer to the previous shape.
	inline ep_Shape* GetPrevious() { return prev; }
	/// Returns a pointer to the next shape.
	inline ep_Shape* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
};

#endif // EP_SHAPE_H

