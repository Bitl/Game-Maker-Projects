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
 * File: ep_Contact.h
 * Definition of ep_Contact.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_CONTACT_H
#define EP_CONTACT_H

struct ep_ContactLink {
	ep_Contact *contact;
	ep_Shape *other_shape;
	ep_ContactLink *prev, *next;
};

struct ep_ContactPoint {
	bool active;
	double x1, y1, x2, y2;
	double xx1, yy1, xx2, yy2;
	double normaltargetvel, tangenttargetvel;
	double normalmass, tangentmass;
	double normalforce, tangentforce;
	double normalveldelta, tangentveldelta;
	double normalpseudoforce;
};

#if EP_USE_IDHASHTABLE
class ep_Contact : private ep_IdHashTableEntry {
#else
class ep_Contact {
#endif
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Shape;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_Contact *prev, *next;
#if !EP_USE_IDHASHTABLE
	unsigned long id;
#endif
	
	ep_Body *body1, *body2;
	ep_Shape *shape1, *shape2;
	ep_ContactLink link1, link2;
	
	double nx, ny;
	double restitution, friction;
	
	ep_ContactPoint contactpoints[2];
	bool useblocksolver;
	double cpfactor[2];
	double massmatrix11, massmatrix12, massmatrix22;
	
	bool destroy;
	ep_Contact *nextconstraint;
	
	// private functions
	private:
	
	void Init(ep_World* _world, ep_Shape* _shape1, ep_Shape* _shape2, unsigned long _id, bool awake);
	void DeInit();
	
	void Update(ep_CollisionData* data);
	
	void InitCPVelocityConstraints(ep_ContactPoint* cp, double* r1, double* r2, double* factor);
	void InitVelocityConstraints();
	void ApplyCPImpulses(ep_ContactPoint* cp);
	void ApplyImpulses();
	void BlockSolveCPNormalVelocityConstraints();
	void SolveCPNormalVelocityConstraint(ep_ContactPoint* cp);
	void SolveCPTangentVelocityConstraint(ep_ContactPoint* cp);
	void SolveVelocityConstraints();
	
	void InitPositionConstraints();
	void SolveCPNormalPositionConstraint(ep_ContactPoint* cp);
	void SolvePositionConstraints();
	
	// public functions
	public:
	
	// inline functions
	public:
	
	/// Returns the id of this contact.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this contact to speed up the next FindContact call (if the id matches the id of this contact).
	inline void CacheID() { world->current_contact = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this contact.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this contact.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous contact.
	inline ep_Contact* GetPrevious() { return prev; }
	/// Returns a pointer to the next contact.
	inline ep_Contact* GetNext() { return next; }
	
	/// Returns a pointer to the first body associated with this contact.
	inline ep_Body* GetBody1() { return body1; }
	/// Returns a pointer to the second body associated with this contact.
	inline ep_Body* GetBody2() { return body2; }
	/// Returns a pointer to the first shape associated with this contact.
	inline ep_Shape* GetShape1() { return shape1; }
	/// Returns a pointer to the second shape associated with this contact.
	inline ep_Shape* GetShape2() { return shape2; }
	
	/// Returns the x component of the normal vector in world coordinates.
	inline double GetNormalX() { return nx; }
	/// Returns the y component of the normal vector in world coordinates.
	inline double GetNormalY() { return ny; }
	
	/// Returns whether the first contact point is active.
	inline bool GetPoint1Active() { return contactpoints[0].active; }
	/// Returns whether the second contact point is active.
	inline bool GetPoint2Active() { return contactpoints[1].active; }
	
	/// Returns the x component of the position of the first contact point (in world coordinates).
	inline double GetPoint1X() {
		if(!contactpoints[0].active) return 0.0;
		return (
		  body1->x+ep_transform_x(body1->rot_sin, body1->rot_cos, contactpoints[0].x1-body1->xoff, contactpoints[0].y1-body1->yoff)+
		  body2->x+ep_transform_x(body2->rot_sin, body2->rot_cos, contactpoints[0].x2-body2->xoff, contactpoints[0].y2-body2->yoff))*0.5;
	}
	/// Returns the y component of the position of the first contact point (in world coordinates).
	inline double GetPoint1Y() {
		if(!contactpoints[0].active) return 0.0;
		return (
		  body1->y+ep_transform_y(body1->rot_sin, body1->rot_cos, contactpoints[0].x1-body1->xoff, contactpoints[0].y1-body1->yoff)+
		  body2->y+ep_transform_y(body2->rot_sin, body2->rot_cos, contactpoints[0].x2-body2->xoff, contactpoints[0].y2-body2->yoff))*0.5;
	}
	/// Returns the x component of the position of the second contact point (in world coordinates).
	inline double GetPoint2X() {
		if(!contactpoints[1].active) return 0.0;
		return (
		  body1->x+ep_transform_x(body1->rot_sin, body1->rot_cos, contactpoints[1].x1-body1->xoff, contactpoints[1].y1-body1->yoff)+
		  body2->x+ep_transform_x(body2->rot_sin, body2->rot_cos, contactpoints[1].x2-body2->xoff, contactpoints[1].y2-body2->yoff))*0.5;
	}
	/// Returns the y component of the position of the second contact point (in world coordinates).
	inline double GetPoint2Y() {
		if(!contactpoints[1].active) return 0.0;
		return (
		  body1->y+ep_transform_y(body1->rot_sin, body1->rot_cos, contactpoints[1].x1-body1->xoff, contactpoints[1].y1-body1->yoff)+
		  body2->y+ep_transform_y(body2->rot_sin, body2->rot_cos, contactpoints[1].x2-body2->xoff, contactpoints[1].y2-body2->yoff))*0.5;
	}
	
	/// Returns the separation of the first contact point.
	inline double GetPoint1Separation() {
		if(!contactpoints[0].active) return 0.0;
		return
		  ((body1->x+ep_transform_x(body1->rot_sin, body1->rot_cos, contactpoints[0].x1-body1->xoff, contactpoints[0].y1-body1->yoff))
		  -(body2->x+ep_transform_x(body2->rot_sin, body2->rot_cos, contactpoints[0].x2-body2->xoff, contactpoints[0].y2-body2->yoff)))*nx+
		  ((body1->y+ep_transform_y(body1->rot_sin, body1->rot_cos, contactpoints[0].x1-body1->xoff, contactpoints[0].y1-body1->yoff))
		  -(body2->y+ep_transform_y(body2->rot_sin, body2->rot_cos, contactpoints[0].x2-body2->xoff, contactpoints[0].y2-body2->yoff)))*ny;
	}
	/// Returns the separation of the second contact point.
	inline double GetPoint2Separation() {
		if(!contactpoints[1].active) return 0.0;
		return
		  ((body1->x+ep_transform_x(body1->rot_sin, body1->rot_cos, contactpoints[1].x1-body1->xoff, contactpoints[1].y1-body1->yoff))
		  -(body2->x+ep_transform_x(body2->rot_sin, body2->rot_cos, contactpoints[1].x2-body2->xoff, contactpoints[1].y2-body2->yoff)))*nx+
		  ((body1->y+ep_transform_y(body1->rot_sin, body1->rot_cos, contactpoints[1].x1-body1->xoff, contactpoints[1].y1-body1->yoff))
		  -(body2->y+ep_transform_y(body2->rot_sin, body2->rot_cos, contactpoints[1].x2-body2->xoff, contactpoints[1].y2-body2->yoff)))*ny;
	}
	
	/// Returns the normal force of the first contact point.
	inline double GetPoint1NormalForce() {
		if(!contactpoints[0].active) return 0.0;
		return contactpoints[0].normalforce;
	}
	/// Returns the tangent force of the first contact point.
	inline double GetPoint1TangentForce() {
		if(!contactpoints[0].active) return 0.0;
		return contactpoints[0].tangentforce;
	}
	/// Returns the normal force of the second contact point.
	inline double GetPoint2NormalForce() {
		if(!contactpoints[1].active) return 0.0;
		return contactpoints[1].normalforce;
	}
	/// Returns the tangent force of the second contact point.
	inline double GetPoint2TangentForce() {
		if(!contactpoints[1].active) return 0.0;
		return contactpoints[1].tangentforce;
	}
	
	/// Returns the normal velocity difference of the first contact point.
	inline double GetPoint1NormalVelDelta() {
		if(!contactpoints[0].active) return 0.0;
		return contactpoints[0].normalveldelta;
	}
	/// Returns the tangent velocity difference of the first contact point.
	inline double GetPoint1TangentVelDelta() {
		if(!contactpoints[0].active) return 0.0;
		return contactpoints[0].tangentveldelta;
	}
	/// Returns the normal velocity difference of the second contact point.
	inline double GetPoint2NormalVelDelta() {
		if(!contactpoints[1].active) return 0.0;
		return contactpoints[1].normalveldelta;
	}
	/// Returns the tangent velocity difference of the second contact point.
	inline double GetPoint2TangentVelDelta() {
		if(!contactpoints[1].active) return 0.0;
		return contactpoints[1].tangentveldelta;
	}
	
};

#endif // EP_CONTACT_H

