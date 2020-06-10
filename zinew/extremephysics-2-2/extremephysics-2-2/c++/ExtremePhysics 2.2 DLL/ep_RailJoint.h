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
 * File: ep_RailJoint.h
 * Definition of ep_RailJoint.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_RAILJOINT_H
#define EP_RAILJOINT_H

struct ep_RailJointLink {
	ep_RailJoint *railjoint;
	ep_Body *other_body;
	ep_RailJointLink *prev, *next;
};

#if EP_USE_IDHASHTABLE
class ep_RailJoint : private ep_IdHashTableEntry {
#else
class ep_RailJoint {
#endif
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Body;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_RailJoint *prev, *next;
#if !EP_USE_IDHASHTABLE
	unsigned long id;
#endif
	
	ep_Body *body1, *body2;
	ep_RailJointLink link1, link2;
	
	double x1, y1, x2, y2;
	double vx, vy, length;
	
	double maxnormalforce;
	double maxmotorforce, motorvel;
	
	double limit1_maxforce, limit1_position, limit1_restitution, limit1_velocity;
	double limit2_maxforce, limit2_position, limit2_restitution, limit2_velocity;
	double limit_contact_threshold, limit_velocity_threshold;
	double spring1_k, spring1_position, spring1_damping;
	double spring2_k, spring2_position, spring2_damping;
	
	double xx1, yy1, xx2, yy2;
	double vxx, vyy, position, targetnormalvel;
	double targetlimitvel, minlimitforce, maxlimitforce;
	double normalmass, motormass;
	
	double normalforce, motorforce, limitforce;
	double pseudonormalforce, pseudolimitforce;
	
	ep_RailJoint *nextconstraint;
	
	// private functions
	private:
	
	void Init(ep_World* _world, ep_Body* _body1, ep_Body* _body2, double _x1, double _y1, double _x2, double _y2, double _vx, double _vy, double _length, unsigned long _id, bool awake);
	void DeInit();
	
	void IntegrateVelocity();
	
	void InitVelocityConstraints();
	void ApplyImpulses();
	void SolveVelocityConstraints();
	
	void InitPositionConstraints();
	void SolvePositionConstraints();
	
	// public functions
	public:
	
	/// Sets the maximum normal force of this rail joint
	/// @param _maxnormalforce The new maximum normal force. Use 0.0 for no limit.
	void SetMaxNormalForce(double _maxnormalforce);
	/// Sets the motor force and velocity of this rail joint
	/// @param _maxmotorforce The new maximum motor force. Use 0.0 to disable the motor.
	/// @param _motorvel The new motor velocity.
	void SetMotor(double _maxmotorforce, double _motorvel);
	/// Changes the limit settings of the hinge joint.
	/// @param contact_threshold The new contact threshold.
	/// @param velocity_threshold The new velocity threshold.
	void SetLimitSettings(double contact_threshold, double velocity_threshold);
	/// Sets the lower limit of the rail joint.
	/// @param maxlimitforce The maximum limit force. Use 0.0 to disable the limit.
	/// @param position The lower limit.
	/// @param restitution The coefficient of restitution.
	/// @param velocity The velocity of the limit.
	void SetLowerLimit(double maxlimitforce, double position, double restitution, double velocity);
	/// Sets the upper limit of the rail joint.
	/// @param maxlimitforce The maximum limit force. Use 0.0 to disable the limit.
	/// @param position The upper limit.
	/// @param restitution The coefficient of restitution.
	/// @param velocity The velocity of the limit.
	void SetUpperLimit(double maxlimitforce, double position, double restitution, double velocity);
	/// Sets the lower spring of the rail joint.
	/// @param k The spring constant. Use 0.0 to disable the spring.
	/// @param position The lower limit.
	/// @param damping The damping force of the spring.
	void SetLowerSpring(double k, double position, double damping);
	/// Sets the upper spring of the rail joint.
	/// @param k The spring constant. Use 0.0 to disable the spring.
	/// @param position The lower limit.
	/// @param damping The damping force of the spring.
	void SetUpperSpring(double k, double position, double damping);
	
	// inline functions
	public:
	
	/// Returns the id of this rail joint.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this rail joint to speed up the next FindRailJoint call (if the id matches the id of this rail joint).
	inline void CacheID() { world->current_railjoint = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this rail joint.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this rail joint.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous rail joint.
	inline ep_RailJoint* GetPrevious() { return prev; }
	/// Returns a pointer to the next rail joint.
	inline ep_RailJoint* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
	/// Returns a pointer to the first body associated with this rail joint.
	inline ep_Body* GetBody1() { return body1; }
	/// Returns a pointer to the second body associated with this rail joint.
	inline ep_Body* GetBody2() { return body2; }
	
	/// Returns the current position of the rail joint.
	inline double GetPosition() {
		return
		  (ep_transform_x(body2->rot_sin, body2->rot_cos, vx, vy)
		   *(body1->x-body2->x
		     +ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff)
		     -ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff))
		  +ep_transform_y(body2->rot_sin, body2->rot_cos, vx, vy)
		   *(body1->y-body2->y
		     +ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff)
		     -ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff)))/length;
	}
	/// Returns the normal force applied during the last step.
	inline double GetNormalForce() { return normalforce; };
	/// Returns the motor force applied during the last step.
	inline double GetMotorForce() { return motorforce; };
	/// Returns the limit force applied during the last step.
	inline double GetLimitForce() { return limitforce; };
	
};

#endif // EP_RAILJOINT_H

