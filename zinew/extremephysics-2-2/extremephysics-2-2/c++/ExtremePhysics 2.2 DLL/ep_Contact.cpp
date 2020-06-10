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
 * File: ep_Contact.cpp
 * Implementation of ep_Contact.
 */

#include "ExtremePhysics.h"

void ep_Contact::Init(ep_World* _world, ep_Shape* _shape1, ep_Shape* _shape2, unsigned long _id, bool awake) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_contact;
	next = NULL;
	if(prev==NULL) world->first_contact = this;
	else prev->next = this;
	world->last_contact = this;
	
	// initialize variables
	id = _id;
#if EP_USE_IDHASHTABLE
	world->idhashtable_contacts.Insert(this);
#endif
	
	body1 = _shape1->body;
	body2 = _shape2->body;
	shape1 = _shape1;
	shape2 = _shape2;
	
	contactpoints[0].active = false;
	contactpoints[1].active = false;
	
	// initialize links
	link1.contact = this;
	link2.contact = this;
	link1.other_shape = shape2;
	link2.other_shape = shape1;
	
	// add link1 to shape1's contact list
	link1.prev = shape1->last_contactlink;
	link1.next = NULL;
	if(link1.prev==NULL) shape1->first_contactlink = &link1;
	else link1.prev->next = &link1;
	shape1->last_contactlink = &link1;
	
	// add link2 to shape2's contact list
	link2.prev = shape2->last_contactlink;
	link2.next = NULL;
	if(link2.prev==NULL) shape2->first_contactlink = &link2;
	else link2.prev->next = &link2;
	shape2->last_contactlink = &link2;
	
	// initialize some variables
	contactpoints[0].normalveldelta = 0.0;
	contactpoints[0].tangentveldelta = 0.0;
	contactpoints[1].normalveldelta = 0.0;
	contactpoints[1].tangentveldelta = 0.0;
	
	// awake connected bodies
	if(awake) {
		body1->Awake(true, false);
		body2->Awake(true, false);
	}
	
	++world->contactcount;
	
}

void ep_Contact::DeInit() {
	
	// remove from linked list
	if(prev==NULL) world->first_contact = next;
	else prev->next = next;
	if(next==NULL) world->last_contact = prev;
	else next->prev = prev;
	if(world->current_contact==this) world->current_contact = NULL;
#if EP_USE_IDHASHTABLE
	world->idhashtable_contacts.Remove(this);
#endif
	
	// remove link1 from body1's joint list
	if(link1.prev==NULL) shape1->first_contactlink = link1.next;
	else link1.prev->next = link1.next;
	if(link1.next==NULL) shape1->last_contactlink = link1.prev;
	else link1.next->prev = link1.prev;
	
	// remove link2 from body2's joint list
	if(link2.prev==NULL) shape2->first_contactlink = link2.next;
	else link2.prev->next = link2.next;
	if(link2.next==NULL) shape2->last_contactlink = link2.prev;
	else link2.next->prev = link2.prev;
	
	// awake connected bodies
	body1->Awake(true, false);
	body2->Awake(true, false);
	
	--world->contactcount;
	
}

