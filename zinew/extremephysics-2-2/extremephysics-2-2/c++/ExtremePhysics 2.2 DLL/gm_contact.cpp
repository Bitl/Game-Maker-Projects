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
 * File: gm_contact.cpp
 * Wrapper for ep_Contact.
 */

#include "gm.h"

gmexport double ep_contact_destroy(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_destroy: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	world->DestroyContact(contact);
	return 1;
}

gmexport double ep_contact_exists(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	return (world->FindContact(gm_cast<unsigned long>(contact_id))!=NULL)? 1 : 0;
}

gmexport double ep_contact_get_body1(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_body1: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_body1: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	if(contact->GetBody1()==NULL) {
		return 0;
	}
	contact->GetBody1()->CacheID();
	return contact->GetBody1()->GetID();
}

gmexport double ep_contact_get_body2(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_body2: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_body2: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	if(contact->GetBody2()==NULL) {
		return 0;
	}
	contact->GetBody2()->CacheID();
	return contact->GetBody2()->GetID();
}

gmexport double ep_contact_get_shape1(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_shape1: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_shape1: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	if(contact->GetShape1()==NULL) {
		return 0;
	}
	contact->GetShape1()->CacheID();
	return contact->GetShape1()->GetID();
}

gmexport double ep_contact_get_shape2(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_shape2: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_shape2: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	if(contact->GetShape2()==NULL) {
		return 0;
	}
	contact->GetShape2()->CacheID();
	return contact->GetShape2()->GetID();
}

gmexport double ep_contact_get_normal_x(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_normal_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_normal_x: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetNormalX();
}

gmexport double ep_contact_get_normal_y(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_normal_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_normal_y: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetNormalY();
}

gmexport double ep_contact_get_point1_active(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_active: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_active: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return (contact->GetPoint1Active())? 1 : 0;
}

gmexport double ep_contact_get_point2_active(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_active: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_active: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return (contact->GetPoint2Active())? 1 : 0;
}

gmexport double ep_contact_get_point1_x(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_x: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1X();
}

gmexport double ep_contact_get_point1_y(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_y: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1Y();
}

gmexport double ep_contact_get_point2_x(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_x: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_x: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2X();
}

gmexport double ep_contact_get_point2_y(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_y: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_y: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2Y();
}

gmexport double ep_contact_get_point1_separation(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_separation: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_separation: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1Separation();
}

gmexport double ep_contact_get_point2_separation(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_separation: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_separation: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2Separation();
}

gmexport double ep_contact_get_point1_normalforce(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_normalforce: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_normalforce: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1NormalForce();
}

gmexport double ep_contact_get_point1_tangentforce(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_tangentforce: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_tangentforce: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1TangentForce();
}

gmexport double ep_contact_get_point2_normalforce(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_normalforce: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_normalforce: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2NormalForce();
}

gmexport double ep_contact_get_point2_tangentforce(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_tangentforce: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_tangentforce: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2TangentForce();
}

gmexport double ep_contact_get_point1_normalveldelta(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_normalveldelta: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_normalveldelta: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1NormalVelDelta();
}

gmexport double ep_contact_get_point1_tangentveldelta(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_tangentveldelta: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point1_tangentveldelta: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint1TangentVelDelta();
}

gmexport double ep_contact_get_point2_normalveldelta(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_normalveldelta: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_normalveldelta: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2NormalVelDelta();
}

gmexport double ep_contact_get_point2_tangentveldelta(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_tangentveldelta: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_get_point2_tangentveldelta: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	return contact->GetPoint2TangentVelDelta();
}

gmexport double ep_contact_previous(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_previous: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	if(contact->GetPrevious()==NULL) {
		return 0;
	}
	contact->GetPrevious()->CacheID();
	return contact->GetPrevious()->GetID();
}

gmexport double ep_contact_next(double world_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_contact_next: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	if(contact->GetNext()==NULL) {
		return 0;
	}
	contact->GetNext()->CacheID();
	return contact->GetNext()->GetID();
}

