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
 * File: ep_HingeJoint.h
 * Definition of ep_HingeJoint.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_HINGEJOINT_H
#define EP_HINGEJOINT_H

/*
I did not use inheritance for joints. All joints are completely separate classes.
Using inheritance and virtual functions would probably result in a big performance
penalty because of the number of virtual function calls made by the solver (these
functions are now inlined because it caused a noticeable performance boost).
It might be a good idea to create a separate class for links (contact links + joint links),
this would make the sleeping system more maintainable but it will make the functions that
depend on the separate link lists more complicated and less efficient.
*/

struct ep_HingeJointLink {
	ep_HingeJoint *hingejoint;
	ep_Body *other_body;
	ep_HingeJointLink *prev, *next;
};

#if EP_USE_IDHASHTABLE
class ep_HingeJoint : private ep_IdHashTableEntry {
#else
class ep_HingeJoint {
#endif
	
	friend class ep_Main;
	friend class ep_World;
	friend class ep_Body;
	
	// variables
	private:
	
	ep_Main *main;
	ep_World *world;
	ep_HingeJoint *prev, *next;
#if !EP_USE_IDHASHTABLE
	unsigned long id;
#endif
	
	ep_Body *body1, *body2;
	ep_HingeJointLink link1, link2;
	
	double x1, y1, x2, y2;
	double referencerotation;
	
	double maxforce;
	double maxmotortorque, motorvel;
	
	double limit1_maxtorque, limit1_rot, limit1_restitution, limit1_velocity;
	double limit2_maxtorque, limit2_rot, limit2_restitution, limit2_velocity;
	double limit_contact_threshold, limit_velocity_threshold;
	double spring1_k, spring1_rotation, spring1_damping;
	double spring2_k, spring2_rotation, spring2_damping;
	
	double xx1, yy1, xx2, yy2, targetvel_x, targetvel_y;
	double targetlimitvel, minlimittorque, maxlimittorque;
	double massmatrix11, massmatrix12, massmatrix22, motormass;
	
	double xforce, yforce, motortorque, limittorque;
	double pseudoxforce, pseudoyforce, pseudolimittorque;
	
	ep_HingeJoint *nextconstraint;
	
	// private functions
	private:
	
	void Init(ep_World* _world, ep_Body* _body1, ep_Body* _body2, double _x1, double _y1, double _x2, double _y2, double _referencerotation, unsigned long _id, bool awake);
	void DeInit();
	
	void IntegrateVelocity();
	
	void InitVelocityConstraints();
	void ApplyImpulses();
	void SolveVelocityConstraints();
	
	void InitPositionConstraints();
	void SolvePositionConstraints();
	
	// public functions
	public:
	
	/// Sets the maximum force of this hinge joint
	/// @param _maxforce The new maximum force. Use 0.0 for no limit.
	void SetMaxForce(double _maxforce);
	/// Sets the motor torque and velocity of this hinge joint
	/// @param _maxmotortorque The new maximum motor torque. Use 0.0 to disable the motor.
	/// @param _motorvel The new motor velocity.
	bool SetMotor(double _maxmotortorque, double _motorvel);
	/// Changes the limit settings of the hinge joint.
	/// @param contact_threshold The new contact threshold.
	/// @param velocity_threshold The new velocity threshold.
	void SetLimitSettings(double contact_threshold, double velocity_threshold);
	/// Sets the lower limit of the hinge joint.
	/// @param maxlimittorque The maximum limit torque. Use 0.0 to disable the limit.
	/// @param rotation The lower limit of the rotation.
	/// @param restitution The coefficient of restitution.
	/// @param velocity The velocity of the limit.
	bool SetLowerLimit(double maxlimittorque, double rotation, double restitution, double velocity);
	/// Sets the upper limit of the hinge joint.
	/// @param maxlimittorque The maximum limit torque. Use 0.0 to disable the limit.
	/// @param rotation The upper limit of the rotation.
	/// @param restitution The coefficient of restitution.
	/// @param velocity The velocity of the limit.
	bool SetUpperLimit(double maxlimittorque, double rotation, double restitution, double velocity);
	/// Sets the lower spring of the hinge joint.
	/// @param k The spring constant. Use 0.0 to disable the spring.
	/// @param distance The lower limit.
	/// @param damping The damping force of the spring.
	bool SetLowerSpring(double k, double rotation, double damping);
	/// Sets the upper spring of the hinge joint.
	/// @param k The spring constant. Use 0.0 to disable the spring.
	/// @param distance The lower limit.
	/// @param damping The damping force of the spring.
	bool SetUpperSpring(double k, double rotation, double damping);
	
	// inline functions
	public:
	
	/// Returns the id of this hinge joint.
	inline unsigned long GetID() { return id; }
	/// Caches the id of this hinge joint to speed up the next FindHingeJoint call (if the id matches the id of this hinge joint).
	inline void CacheID() { world->current_hingejoint = this; }
	/// Returns a pointer to the main ExtremePhysics object associated with this hinge joint.
	inline ep_Main* GetMain() { return main; }
	/// Returns a pointer to the world associated with this hinge joint.
	inline ep_World* GetWorld() { return world; }
	/// Returns a pointer to the previous hinge joint.
	inline ep_HingeJoint* GetPrevious() { return prev; }
	/// Returns a pointer to the next hinge joint.
	inline ep_HingeJoint* GetNext() { return next; }
	/// Returns a pointer to the userdata memory block.
	inline void* GetUserData() { return (void*)(this+1); }
	
	/// Returns a pointer to the first body associated with this hinge joint.
	inline ep_Body* GetBody1() { return body1; }
	/// Returns a pointer to the second body associated with this hinge joint.
	inline ep_Body* GetBody2() { return body2; }
	
	/// Returns the current rotation of the hinge joint.
	inline double GetRotation() { return body1->rot-body2->rot-referencerotation; };
	/// Returns the x component of the force applied during the last step.
	inline double GetXForce() { return xforce; };
	/// Returns the y component of the force applied during the last step.
	inline double GetYForce() { return yforce; };
	/// Returns the motor torque applied during the last step.
	inline double GetMotorTorque() { return motortorque; };
	/// Returns the limit torque applied during the last step.
	inline double GetLimitTorque() { return limittorque; };
	
};

#endif // EP_HINGEJOINT_H

