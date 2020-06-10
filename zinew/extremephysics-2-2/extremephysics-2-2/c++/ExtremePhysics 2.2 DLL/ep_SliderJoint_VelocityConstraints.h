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
 * File: ep_SliderJoint_VelocityConstraints.h
 * Solves slider joint velocity constraints.
 * Included in ep_World_SolveVelocityConstraints.cpp.
 */

#ifndef EP_SLIDERJOINT_VELOCITYCONSTRAINTS_H
#define EP_SLIDERJOINT_VELOCITYCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_SliderJoint::InitVelocityConstraints() {
	
	// limits
	if(limit1_maxforce!=0.0 || limit2_maxforce!=0.0) {
		
		if(limit1_maxforce!=0.0 && limit2_maxforce!=0.0 && limit1_position>=limit2_position) {
			targetlimitvel = (limit1_velocity+limit2_velocity)*0.5-world->baumgarte_factor/world->timestep*(position-limit1_position);
			maxlimitforce = +limit1_maxforce;
			minlimitforce = -limit2_maxforce;
		} else {
			double currvel =
			   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vxx
			  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vyy;
			if(limit1_maxforce!=0.0 && position<limit1_position+limit_contact_threshold) {
				maxlimitforce = +limit1_maxforce;
				targetlimitvel = limit1_velocity-world->baumgarte_factor/world->timestep*(position-limit1_position)-ep_min(0.0, currvel+limit_velocity_threshold)*limit1_restitution;
			} else {
				maxlimitforce = 0.0;
			}
			if(limit2_maxforce!=0.0 && position>limit2_position-limit_contact_threshold) {
				minlimitforce = -limit2_maxforce;
				targetlimitvel = limit2_velocity-world->baumgarte_factor/world->timestep*(position-limit2_position)+ep_min(0.0, -currvel+limit_velocity_threshold)*limit2_restitution;
			} else {
				minlimitforce = 0.0;
			}
		}
		
	} else {
		minlimitforce = 0.0;
		maxlimitforce = 0.0;
	}
	
	// calculate mass
	/*{
		double r1 = yy1*vyy+xx1*vxx;
		double r2 = yy2*vyy+xx2*vxx;
		normalmass = (1.0+world->mass_bias)/(
		  body1->invmass+body2->invmass
		  +ep_sqr(r1)*body1->invinertia
		  +ep_sqr(r2)*body2->invinertia);
	}*/
	// calculate mass matrix
	{
		
		// The derivation is at the bottom of the file.
		// nx = vyy, ny = -vxx
		double r1 = yy1*vyy+xx1*vxx;
		double r2 = yy2*vyy+xx2*vxx;
		double aa, bb, dd;
		aa = body1->invmass+body2->invmass
		  +body1->invinertia*ep_sqr(r1)
		  +body2->invinertia*ep_sqr(r2);
		bb = body1->invinertia*r1
		    +body2->invinertia*r2;
		dd = body1->invinertia+body2->invinertia;
		
		// invert (create mass matrix)
		double d = (1.0+world->mass_bias)/(aa*dd-bb*bb);
		massmatrix11 = d*dd;
		massmatrix12 = -d*bb;
		massmatrix22 = d*aa;
		
	}
	
}

EP_FORCEINLINE void ep_SliderJoint::ApplyImpulses() {
	
	if(body1->storejointimpulses && body2->storejointimpulses) {
		
		// clamp stored force
		if(maxnormalforce!=0.0) {
			double d = (torqueradius==0.0)? fabs(normalforce) : fabs(normalforce)+fabs(torque)/torqueradius;
			if(d>maxnormalforce) {
				normalforce *= maxnormalforce/d;
				torque *= maxnormalforce/d;
			}
		}
		motorforce = ep_clamp(-maxmotorforce, +maxmotorforce, motorforce);
		limitforce = ep_clamp(minlimitforce,  maxlimitforce, limitforce);
		
		double f = motorforce+limitforce;
		double fx = f*vxx+normalforce*vyy;
		double fy = f*vyy-normalforce*vxx;
		body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
		body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
		body1->_ApplyTorque(+torque);
		body2->_ApplyTorque(-torque);
		
	} else {
		normalforce = 0.0;
		torque = 0.0;
		motorforce = 0.0;
		limitforce = 0.0;
	}
	
}

EP_FORCEINLINE void ep_SliderJoint::SolveVelocityConstraints() {
	
	// motor/limits
	{
		double currvel =
		   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vxx
		  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vyy;
		double combinedforce;
		
		if(maxmotorforce!=0.0) {
			
			combinedforce = (motorvel-currvel)*motormass;
			
			// always clamp the accumulated impulse
			if(motorforce+combinedforce<-maxmotorforce) {
				combinedforce = -maxmotorforce-motorforce;
				motorforce = -maxmotorforce;
			} else if(motorforce+combinedforce>maxmotorforce) {
				combinedforce = +maxmotorforce-motorforce;
				motorforce = +maxmotorforce;
			} else {
				motorforce += combinedforce;
			}
			
			currvel += combinedforce/motormass;
			
		} else {
			combinedforce = 0.0;
		}
		
		if(minlimitforce!=maxlimitforce) {
			
			double f = (targetlimitvel-currvel)*motormass;
			
			// always clamp the accumulated impulse
			if(limitforce+f<minlimitforce) {
				f = minlimitforce-limitforce;
				limitforce = minlimitforce;
			} else if(limitforce+f>maxlimitforce) {
				f = maxlimitforce-limitforce;
				limitforce = maxlimitforce;
			} else {
				limitforce += f;
			}
			
			combinedforce += f;
			
		}
		
		if(combinedforce!=0.0) {
			double fx = vxx*combinedforce;
			double fy = vyy*combinedforce;
			body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
			body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
		}
	}
	
	// normal constraint
	double currnormalvel =
		((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vyy
		-((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vxx;
	double currrotvel = body1->rotvel-body2->rotvel;
	double nv = targetnormalvel-currnormalvel;
	double rv = targetrotvel-currrotvel;
	double f = nv*massmatrix11+rv*massmatrix12;
	double t = nv*massmatrix12+rv*massmatrix22;
	
	// always clamp the accumulated impulse
	if(maxnormalforce==0.0) {
		normalforce += f;
		torque += t;
	} else {
		double ff = normalforce+f;
		double tt = torque+t;
		double d = (torqueradius==0.0)? fabs(ff) : fabs(ff)+fabs(tt)/torqueradius;
		if(d>maxnormalforce) {
			ff *= maxnormalforce/d;
			tt *= maxnormalforce/d;
			f = ff-normalforce;
			t = tt-torque;
			normalforce = ff;
			torque = tt;
		} else {
			normalforce += f;
			torque += t;
		}
	}
	
	double fx = +f*vyy;
	double fy = -f*vxx;
	body1->_ApplyImpulseTorque(xx1, yy1, +fx, +fy, +t);
	body2->_ApplyImpulseTorque(xx2, yy2, -fx, -fy, -t);
	
}

/*
Slider joint constraint derivation notes:

constraint: relative speed at joint anchor <dot> normal is always zero + relative rotational velocity is always zero

fx = f*nx;
fy = f*ny;

delta_xspeed = fx/mass1+fx/mass2+( yy1)*(fx*yy1-fy*xx1)/inertia1+( yy2)*(fx*yy2-fy*xx2)/inertia2+( yy1)*torque/inertia1+( yy2)*torque/inertia2;
delta_yspeed = fy/mass1+fy/mass2+(-xx1)*(fx*yy1-fy*xx1)/inertia1+(-xx2)*(fx*yy2-fy*xx2)/inertia2+(-xx1)*torque/inertia1+(-xx2)*torque/inertia2;

delta_xspeed = f*nx/mass1+f*nx/mass2+( yy1)*(f*nx*yy1-f*ny*xx1)/inertia1+( yy2)*(f*nx*yy2-f*ny*xx2)/inertia2+( yy1)*torque/inertia1+( yy2)*torque/inertia2;
delta_yspeed = f*ny/mass1+f*ny/mass2+(-xx1)*(f*nx*yy1-f*ny*xx1)/inertia1+(-xx2)*(f*nx*yy2-f*ny*xx2)/inertia2+(-xx1)*torque/inertia1+(-xx2)*torque/inertia2;

delta_xspeed =
f*(nx*(1/mass1+1/mass2)+
  ( yy1)*(nx*yy1-ny*xx1)/inertia1+( yy2)*(nx*yy2-ny*xx2)/inertia2)+
torque*(( yy1)/inertia1+( yy2)/inertia2);

delta_yspeed =
f*(ny*(1/mass1+1/mass2)+
  +(-xx1)*(nx*yy1-ny*xx1)/inertia1+(-xx2)*(nx*yy2-ny*xx2)/inertia2)+
torque*((-xx1)/inertia1+(-xx2)/inertia2);

delta_normalspeed = nx*delta_xspeed+ny*delta_yspeed;
delta_normalspeed = f*(
  nx*nx*(1/mass1+1/mass2)+
  ny*ny*(1/mass1+1/mass2)+
  nx*( yy1)*(nx*yy1-ny*xx1)/inertia1+nx*( yy2)*(nx*yy2-ny*xx2)/inertia2+
  ny*(-xx1)*(nx*yy1-ny*xx1)/inertia1+ny*(-xx2)*(nx*yy2-ny*xx2)/inertia2)
+torque*(nx*( yy1)/inertia1+nx*( yy2)/inertia2)
         ny*(-xx1)/inertia1+ny*(-xx2)/inertia2);
delta_normalspeed = f*(
  (nx*nx+ny*ny)*(1/mass1+1/mass2)+
  (( yy1)*(nx*nx*yy1-nx*ny*xx1)+(xx1)*(ny*ny*xx1-ny*nx*yy1))/inertia1+
  (( yy2)*(nx*nx*yy2-nx*ny*xx2)+(xx2)*(ny*ny*xx2-ny*nx*yy2))/inertia2)
+torque*((nx*yy1-ny*xx1)/inertia1)
         (nx*yy2-ny*xx2)/inertia2);

nx*nx+ny*ny is always 1

delta_normalspeed = f*(
  1/mass1+1/mass2+
  (nx*nx*yy1*yy1+ny*ny*xx1*xx1-2*ny*nx*xx1*yy1)/inertia1+
  (nx*nx*yy2*yy2+ny*ny*xx2*xx2-2*ny*nx*xx2*yy2)/inertia2)
+torque*((nx*yy1-ny*xx1)/inertia1)
         (nx*yy2-ny*xx2)/inertia2);
delta_normalspeed = f*(
  1/mass1+1/mass2+
  (nx*yy1-ny*xx1)^2/inertia1+
  (nx*yy2-ny*xx2)^2/inertia2)
+torque*((nx*yy1-ny*xx1)/inertia1)
         (nx*yy2-ny*xx2)/inertia2);

delta_rotspeed =
f*((nx*yy1-ny*xx1)/inertia1+
   (nx*yy2-ny*xx2)/inertia2)
+torque*(1/inertia1+1/inertia2)

matrix:
  [ (1/mass1) , 0            ] + [ (1/mass2) , 0            ]
  [ 0         , (1/inertia1) ]   [ 0         , (1/inertia2) ]
+ [  (nx*yy1-ny*xx1)^2*(1/inertia1) , (nx*yy1-ny*xx1)  *(1/inertia1) ]
  [  (nx*yy1-ny*xx1)  *(1/inertia1) , 0                              ]
+ [  (nx*yy2-ny*xx2)^2*(1/inertia2) , (nx*yy2-ny*xx2)  *(1/inertia2) ]
  [  (nx*yy2-ny*xx2)  *(1/inertia2) , 0                              ]
the determinant is always positive if
1/mass1>=0 and 1/inertia1>=0, or
1/mass2>=0 and 1/inertia2>=0
*/

#endif // EP_SLIDERJOINT_VELOCITYCONSTRAINTS_H

