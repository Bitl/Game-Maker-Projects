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
 * File: ep_HingeJoint_VelocityConstraints.h
 * Solves hinge joint velocity constraints.
 * Included in ep_World_SolveVelocityConstraints.cpp.
 */

#ifndef EP_HINGEJOINT_VELOCITYCONSTRAINTS_H
#define EP_HINGEJOINT_VELOCITYCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_HingeJoint::InitVelocityConstraints() {
	
	// limits
	double rot = body1->rot-body2->rot-referencerotation;
	if(limit1_maxtorque!=0.0 && limit2_maxtorque!=0.0 && limit1_rot>=limit2_rot) {
		targetlimitvel = (limit1_velocity+limit2_velocity)*0.5-world->baumgarte_factor/world->timestep*(rot-limit1_rot);
		maxlimittorque = +limit1_maxtorque;
		minlimittorque = -limit2_maxtorque;
	} else {
		double currrotvel = body1->rotvel-body2->rotvel;
		if(limit1_maxtorque!=0.0 && rot<limit1_rot+limit_contact_threshold) {
			maxlimittorque = +limit1_maxtorque;
			targetlimitvel = limit1_velocity-world->baumgarte_factor/world->timestep*(rot-limit1_rot)-ep_min(0.0, currrotvel+limit_velocity_threshold)*limit1_restitution;
		} else {
			maxlimittorque = 0.0;
		}
		if(limit2_maxtorque!=0.0 && rot>limit2_rot-limit_contact_threshold) {
			minlimittorque = -limit2_maxtorque;
			targetlimitvel = limit2_velocity-world->baumgarte_factor/world->timestep*(rot-limit2_rot)+ep_min(0.0, -currrotvel+limit_velocity_threshold)*limit2_restitution;
		} else {
			minlimittorque = 0.0;
		}
	}
	
	// calculate mass matrix
	{
		
		// The derivation is at the bottom of the file.
		double aa, bb, dd;
		aa = body1->invmass+body2->invmass
		  +body1->invinertia*yy1*yy1
		  +body2->invinertia*yy2*yy2;
		bb =
		  -body1->invinertia*yy1*xx1
		  -body2->invinertia*yy2*xx2;
		dd = body1->invmass+body2->invmass
		  +body1->invinertia*xx1*xx1
		  +body2->invinertia*xx2*xx2;
		
		// invert (create mass matrix)
		double d = (1.0+world->mass_bias)/(aa*dd-bb*bb);
		massmatrix11 = d*dd;
		massmatrix12 = -d*bb;
		massmatrix22 = d*aa;
		
	}
	
}

EP_FORCEINLINE void ep_HingeJoint::ApplyImpulses() {
	
	if(body1->storejointimpulses && body2->storejointimpulses) {
		
		// clamp stored impulse (maybe maxforce was changed)
		if(maxforce!=0.0) {
			double d = ep_sqr(xforce)+ep_sqr(yforce);
			if(d>ep_sqr(maxforce)) {
				d = sqrt(d);
				xforce *= maxforce/d;
				yforce *= maxforce/d;
			}
		}
		
		// clamp stored torque
		motortorque = ep_clamp(-maxmotortorque, +maxmotortorque, motortorque);
		limittorque = ep_clamp(minlimittorque,  maxlimittorque, limittorque);
		
		body1->_ApplyImpulse(xx1, yy1, +xforce, +yforce);
		body2->_ApplyImpulse(xx2, yy2, -xforce, -yforce);
		body1->_ApplyTorque(+motortorque+limittorque);
		body2->_ApplyTorque(-motortorque-limittorque);
		
	} else {
		xforce = 0.0;
		yforce = 0.0;
		motortorque = 0.0;
		limittorque = 0.0;
	}
	
}

EP_FORCEINLINE void ep_HingeJoint::SolveVelocityConstraints() {
	
	// motor/limits
	if(maxmotortorque!=0.0 || minlimittorque!=maxlimittorque) {
		
		double currrotvel = body1->rotvel-body2->rotvel;
		double combinedtorque;
		
		if(maxmotortorque!=0.0) {
			
			combinedtorque = (motorvel-currrotvel)*motormass;
			
			// always clamp the accumulated impulse
			if(motortorque+combinedtorque<-maxmotortorque) {
				combinedtorque = -maxmotortorque-motortorque;
				motortorque = -maxmotortorque;
			} else if(motortorque+combinedtorque>maxmotortorque) {
				combinedtorque = +maxmotortorque-motortorque;
				motortorque = +maxmotortorque;
			} else {
				motortorque += combinedtorque;
			}
			
			currrotvel += combinedtorque/motormass;
			
		} else {
			combinedtorque = 0.0;
		}
		
		if(minlimittorque!=maxlimittorque) {
			
			double f = (targetlimitvel-currrotvel)*motormass;
			
			// always clamp the accumulated impulse
			if(limittorque+f<minlimittorque) {
				f = minlimittorque-limittorque;
				limittorque = minlimittorque;
			} else if(limittorque+f>maxlimittorque) {
				f = maxlimittorque-limittorque;
				limittorque = maxlimittorque;
			} else {
				limittorque += f;
			}
			
			combinedtorque += f;
			
		}
		
		if(combinedtorque!=0.0) {
			body1->_ApplyTorque(+combinedtorque);
			body2->_ApplyTorque(-combinedtorque);
		}
		
	}
	
	// hinge joint constraint
	{
		double xx = targetvel_x-((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel));
		double yy = targetvel_y-((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel));
		double fx = xx*massmatrix11+yy*massmatrix12;
		double fy = xx*massmatrix12+yy*massmatrix22;
		
		// always clamp the accumulated impulse
		if(maxforce==0.0) {
			xforce += fx;
			yforce += fy;
		} else {
			double fxx = xforce+fx;
			double fyy = yforce+fy;
			double d = ep_sqr(fxx)+ep_sqr(fyy);
			if(d>ep_sqr(maxforce)) {
				d = sqrt(d);
				fxx *= maxforce/d;
				fyy *= maxforce/d;
				fx = fxx-xforce;
				fy = fyy-yforce;
				xforce = fxx;
				yforce = fyy;
			} else {
				xforce += fx;
				yforce += fy;
			}
		}
		
		body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
		body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
	}
	
}

/*
Hinge joint constraint derivation notes:

constraint: relative speed at joint anchor is always zero

delta_xspeed = fx/mass1+fx/mass2+( yy1)*(fx*yy1-fy*xx1)/inertia1+( yy2)*(fx*yy2-fy*xx2)/inertia2;
delta_yspeed = fy/mass1+fy/mass2+(-xx1)*(fx*yy1-fy*xx1)/inertia1+(-xx2)*(fx*yy2-fy*xx2)/inertia2;

delta_xspeed = 
fx*(1/mass1)                + 0                           +
fx*(1/mass2)                + 0                           +
fx*yy1*yy1*(1/inertia1)     + -fy*yy1*xx1*(1/inertia1)   +
fx*yy2*yy2*(1/inertia2)     + -fy*yy2*xx2*(1/inertia2)   ;

delta_yspeed = 
0                           + fy*(1/mass1)                +
0                           + fy*(1/mass2)                +
-fx*xx1*yy1*(1/inertia1)    + fy*xx1*xx1*(1/inertia1)    +
-fx*xx2*yy2*(1/inertia2)    + fy*xx2*xx2*(1/inertia2)    ;

matrix:
  [ (1/mass1) , 0         ] + [ (1/mass2) , 0         ]
  [ 0         , (1/mass1) ]   [ 0         , (1/mass2) ]
+ [  yy1*yy1*(1/inertia1) , -yy1*xx1*(1/inertia1) ]
  [ -xx1*yy1*(1/inertia1) ,  xx1*xx1*(1/inertia1) ]
+ [  yy2*yy2*(1/inertia2) , -yy2*xx2*(1/inertia2) ]
  [ -xx2*yy2*(1/inertia2) ,  xx2*xx2*(1/inertia2) ]
the determinant is always positive if 1/mass1+1/mass2>=0
*/

#endif // EP_HINGEJOINT_VELOCITYCONSTRAINTS_H

