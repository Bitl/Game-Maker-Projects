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
 * File: ep_DistanceJoint.h
 * Definition of ep_DistanceJoint.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_DISTANCEJOINT_H
#define EP_DISTANCEJOINT_H

struct ep_DistanceJointLink {
	ep_DistanceJoint *distancejoint;
	ep_Body *other_body;
	ep_DistanceJointLink *prev, *next;
};

#if EP_USE_IDHASHTABLE
class ep_DistanceJoint : private ep_IdHashTableEntry {
#else
class ep_DistanceJoint {
#endif
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Body;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_DistanceJoint *prev, *next;
#if !EP_USE_IDHASHTABLE
	unsigned long id;
#endif
	
	ep_Body *body1, *body2;
	ep_DistanceJointLink link1, link2;
	
	double x1, y1, x2, y2;
	
	double maxmotorforce, motorvel;
	
	double limit1_maxforce, limit1_distance, limit1_restitution, limit1_velocity;
	double limit2_maxforce, limit2_distance, limit2_restitution, limit2_velocity;
	double limit_contact_threshold, limit_velocity_threshold;
	double spring1_k, spring1_distance, spring1_damping;
	double spring2_k, spring2_distance, spring2_damping;
	
	double xx1, yy1, xx2, yy2;
	double vx, vy, distance;
	double targetlimitvel, minlimitforce, maxlimitforce;
	double motormass;
	
	double motorforce, limitforce;
	double pseudolimitforce;
	
	ep_DistanceJoint *nextconstraint;
	
	// private functions
	private:
	
	void Init(ep_World* _world, ep_Body* _body1, ep_Body* _body2, double _x1, double _y1, double _x2, double _y2, unsigned long _id, bool awake);
	void DeInit();
	
	void IntegrateVelocity();
	
	void InitVelocityConstraints();
	void ApplyImpulses();
	void SolveVelocityConstraints();
	
	void InitPositionConstraints();
	void SolvePositionConstraints();
	
	// public functions
	public:
	
	/// Sets the motor force and velocity of this distance joint
	/// @param _maxmotorforce The new maximum motor force. Use 0.0 to disable the motor.
	/// @param _motorvel The new motor velocity.
	void SetMotor(double _maxmotorforce, double _motorvel);
	/// Changes the limit settings of the distance joint.
	/// @param contact_threshold The new contact threshold.
	/// @param velocity_threshold The new velocity threshold.
	void SetLimitSettings(double contact_threshold, double velocity_threshold);
	/// Sets the lower limit of the distance joint.
	/// @param maxlimitforce The maximum limit force. Use 0.0 to disable the limit.
	/// @param distance The lower limit.
	/// @param restitution The coefficient of restitution.
	/// @param velocity The velocity of the limit.
	void SetLowerLimit(double maxlimitforce, double distance, double restitution, double velocity);
	/// Sets the upper limit of the distance joint.
	/// @param maxlimitforce The maximum limit force. Use 0.0 to disable the limit.
	/// @param distance The upper limit.
	/// @param restitution The coefficient of restitution.
	/// @param velocity The velocity of the limit.
	void SetUpperLimit(double maxlimitforce, double distance, double restitution, double velocity);
	/// Sets the lower spring of the distance joint.
	/// @param k The spring constant. Use 0.0 to disable the spring.
	/// @param distance The lower limit.
	/// @param damping The damping force of the spring.
	void SetLowerSpring(double k, double distance, double damping);
	/// Sets the upper spring of the distance joint.
	/// @param k The spring constant. Use 0.0 to disable the spring.
	/// @param distance The lower limit.
	/// @param damping The damping force of the spring.
	void SetUpperSpring(double k, double distance, double damping);
	
	// inline functions
	public:
	
	/// Returns the id of this distance joint.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this distance joint to speed up the next FindDistanceJoint call (if the id matches the id of this distance joint).
	inline void CacheID() { world->current_distancejoint = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this distance joint.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this distance joint.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous distance joint.
	inline ep_DistanceJoint* GetPrevious() { return prev; }
	/// Returns a pointer to the next distance joint.
	inline ep_DistanceJoint* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
	/// Returns a pointer to the first body associated with this distance joint.
	inline ep_Body* GetBody1() { return body1; }
	/// Returns a pointer to the second body associated with this distance joint.
	inline ep_Body* GetBody2() { return body2; }
	
	/// Returns the current distance of the distance joint.
	inline double GetDistance() {
		double _vx, _vy;
		_vx = ((body1->x+ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff))-(body2->x+ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff)));
		_vy = ((body1->y+ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff))-(body2->y+ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff)));
		return sqrt(ep_sqr(vx)+ep_sqr(vy));
	}
	/// Returns the motor force applied during the last step.
	inline double GetMotorForce() { return motorforce; };
	/// Returns the limit force applied during the last step.
	inline double GetLimitForce() { return limitforce; };
	
};

#endif // EP_DISTANCEJOINT_H

