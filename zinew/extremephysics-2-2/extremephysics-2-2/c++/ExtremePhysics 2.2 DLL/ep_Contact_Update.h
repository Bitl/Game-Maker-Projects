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
 * File: ep_Contact_Update.h
 * Updates contacts.
 * Included in ep_World_UpdateContacts.cpp.
 */

#ifndef EP_CONTACT_UPDATE_H
#define EP_CONTACT_UPDATE_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_Contact::Update(ep_CollisionData* data) {
	// This function is called every step
	// except when the bodies didn't move.
	
	// if the contact point wasn't active during the
	// previous step the value is incorrect
	// copy the forces if the contact points are swapped
	if(data->cp_active[0] && !contactpoints[0].active) {
		if(!data->cp_active[1] && contactpoints[1].active) {
			contactpoints[0].normalforce = contactpoints[1].normalforce;
			contactpoints[0].tangentforce = contactpoints[1].tangentforce;
		} else {
			contactpoints[0].normalforce = 0.0;
			contactpoints[0].tangentforce = 0.0;
		}
	}
	if(data->cp_active[1] && !contactpoints[1].active) {
		if(!data->cp_active[0] && contactpoints[0].active) {
			contactpoints[1].normalforce = contactpoints[0].normalforce;
			contactpoints[1].tangentforce = contactpoints[0].tangentforce;
		} else {
			contactpoints[1].normalforce = 0.0;
			contactpoints[1].tangentforce = 0.0;
		}
	}
	
	// the normal vector points to body1
	nx = data->nx;
	ny = data->ny;
	contactpoints[0].active = data->cp_active[0];
	if(data->cp_active[0]) {
		contactpoints[0].xx1 = data->cp_x[0][0]-body1->x;
		contactpoints[0].yy1 = data->cp_y[0][0]-body1->y;
		contactpoints[0].xx2 = data->cp_x[0][1]-body2->x;
		contactpoints[0].yy2 = data->cp_y[0][1]-body2->y;
		contactpoints[0].x1 = body1->xoff+ep_invtransform_x(body1->rot_sin, body1->rot_cos, contactpoints[0].xx1, contactpoints[0].yy1);
		contactpoints[0].y1 = body1->yoff+ep_invtransform_y(body1->rot_sin, body1->rot_cos, contactpoints[0].xx1, contactpoints[0].yy1);
		contactpoints[0].x2 = body2->xoff+ep_invtransform_x(body2->rot_sin, body2->rot_cos, contactpoints[0].xx2, contactpoints[0].yy2);
		contactpoints[0].y2 = body2->yoff+ep_invtransform_y(body2->rot_sin, body2->rot_cos, contactpoints[0].xx2, contactpoints[0].yy2);
	}
	contactpoints[1].active = data->cp_active[1];
	if(data->cp_active[1]) {
		contactpoints[1].xx1 = data->cp_x[1][0]-body1->x;
		contactpoints[1].yy1 = data->cp_y[1][0]-body1->y;
		contactpoints[1].xx2 = data->cp_x[1][1]-body2->x;
		contactpoints[1].yy2 = data->cp_y[1][1]-body2->y;
		contactpoints[1].x1 = body1->xoff+ep_invtransform_x(body1->rot_sin, body1->rot_cos, contactpoints[1].xx1, contactpoints[1].yy1);
		contactpoints[1].y1 = body1->yoff+ep_invtransform_y(body1->rot_sin, body1->rot_cos, contactpoints[1].xx1, contactpoints[1].yy1);
		contactpoints[1].x2 = body2->xoff+ep_invtransform_x(body2->rot_sin, body2->rot_cos, contactpoints[1].xx2, contactpoints[1].yy2);
		contactpoints[1].y2 = body2->yoff+ep_invtransform_y(body2->rot_sin, body2->rot_cos, contactpoints[1].xx2, contactpoints[1].yy2);
	}
	
}

#endif // EP_CONTACT_UPDATE_H

