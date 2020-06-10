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
 * File: gm_shape.cpp
 * Wrapper for ep_Shape.
 */

#include "gm.h"

gmexport double ep_shape_create_box(double world_id, double body_id, double w, double h, double x, double y, double rot, double density) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_box: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_box: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->CreateBoxShape(w, h, x, y, rot, density))==NULL) {
		return 0;
	}
	((gmuserdata*)(shape->GetUserData()))->Clear();
	shape->CacheID();
	return shape->GetID();
}

gmexport double ep_shape_create_line(double world_id, double body_id, double x1, double y1, double x2, double y2, double density) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_box: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_box: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->CreateLineShape(x1, y1, x2, y2, density))==NULL) {
		return 0;
	}
	((gmuserdata*)(shape->GetUserData()))->Clear();
	shape->CacheID();
	return shape->GetID();
}

gmexport double ep_shape_create_circle(double world_id, double body_id, double r, double x, double y, double rot, double density) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_circle: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_circle: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->CreateCircleShape(r, x, y, rot, density))==NULL) {
		return 0;
	}
	((gmuserdata*)(shape->GetUserData()))->Clear();
	shape->CacheID();
	return shape->GetID();
}

gmexport double ep_shape_create_polygon(double world_id, double body_id, double polygon_id, double x, double y, double rot, double density) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_polygon: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_create_polygon: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->CreatePolygonShape(polygon, x, y, rot, density))==NULL) {
		return 0;
	}
	((gmuserdata*)(shape->GetUserData()))->Clear();
	shape->CacheID();
	return shape->GetID();
}

gmexport double ep_shape_destroy(double world_id, double body_id, double shape_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_destroy: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_destroy: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_destroy: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	body->DestroyShape(shape);
	return 1;
}

gmexport double ep_shape_exists(double world_id, double body_id, double shape_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_exists: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_exists: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	return (body->FindShape(gm_cast<unsigned long>(shape_id))!=NULL)? 1 : 0;
}

gmexport double ep_shape_get_first_contact(double world_id, double body_id, double shape_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_first_contact: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_first_contact: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_first_contact: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	ep_Contact *contact = shape->GetFirstContact();
	if(contact==NULL) {
		return 0;
	}
	contact->CacheID();
	return contact->GetID();
}

gmexport double ep_shape_get_last_contact(double world_id, double body_id, double shape_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_last_contact: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_last_contact: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_last_contact: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	ep_Contact *contact = shape->GetLastContact();
	if(contact==NULL) {
		return 0;
	}
	contact->CacheID();
	return contact->GetID();
}

gmexport double ep_shape_get_previous_contact(double world_id, double body_id, double shape_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_previous_contact: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_previous_contact: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_previous_contact: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_previous_contact: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	contact = shape->GetPreviousContact(contact);
	if(contact==NULL) {
		return 0;
	}
	contact->CacheID();
	return contact->GetID();
}

gmexport double ep_shape_get_next_contact(double world_id, double body_id, double shape_id, double contact_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_next_contact: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_next_contact: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_next_contact: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	ep_Contact *contact;
	if((contact = world->FindContact(gm_cast<unsigned long>(contact_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_next_contact: Contact %lu doesn't exist in world %lu.", gm_cast<unsigned long>(contact_id), world->GetID());
		return 0;
	}
	contact = shape->GetNextContact(contact);
	if(contact==NULL) {
		return 0;
	}
	contact->CacheID();
	return contact->GetID();
}

gmexport double ep_shape_set_material(double world_id, double body_id, double shape_id, double restitution, double friction, double normalvelocity, double tangentvelocity) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_material: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_material: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_material: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	shape->SetMaterial(gm_cast<float>(restitution), gm_cast<float>(friction), normalvelocity, tangentvelocity);
	return 1;
}

gmexport double ep_shape_set_collision(double world_id, double body_id, double shape_id, double collidemask1, double collidemask2, double group) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_collision: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_collision: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_collision: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	shape->SetCollision(gm_cast<unsigned long>(collidemask1), gm_cast<unsigned long>(collidemask2), gm_cast<unsigned long>(group));
	return 1;
}

gmexport double ep_shape_collision_test_box(double world_id, double body_id, double shape_id, double w, double h, double x, double y, double rot, double contact_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_box: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_box: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_box: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	return (shape->CollisionTestBox(w, h, x, y, rot, contact_threshold))? 1 : 0;
}

gmexport double ep_shape_collision_test_line(double world_id, double body_id, double shape_id, double x1, double y1, double x2, double y2, double contact_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_line: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_line: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_line: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	return (shape->CollisionTestLine(x1, y1, x2, y2, contact_threshold))? 1 : 0;
}

gmexport double ep_shape_collision_test_circle(double world_id, double body_id, double shape_id, double r, double x, double y, double contact_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_circle: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_circle: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_circle: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	return (shape->CollisionTestCircle(r, x, y, contact_threshold))? 1 : 0;
}

gmexport double ep_shape_collision_test_polygon(double world_id, double body_id, double shape_id, double polygon_id, double x, double y, double rot, double contact_threshold) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_polygon: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_polygon: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_polygon: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	ep_Polygon *polygon;
	if((polygon = world->FindPolygon(gm_cast<unsigned long>(polygon_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_collision_test_polygon: Polygon %lu doesn't exist in world %lu.", gm_cast<unsigned long>(polygon_id), world->GetID());
		return 0;
	}
	return (shape->CollisionTestPolygon(polygon, x, y, rot, contact_threshold))? 1 : 0;
}

gmexport double ep_shape_ray_cast(double world_id, double body_id, double shape_id, double x, double y, double vx, double vy) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_ray_cast: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_ray_cast: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_ray_cast: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	return shape->RayCast(x, y, vx, vy);
}

gmexport double ep_shape_previous(double world_id, double body_id, double shape_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_previous: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_previous: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_previous: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	if(shape->GetPrevious()==NULL) {
		return 0;
	}
	shape->GetPrevious()->CacheID();
	return shape->GetPrevious()->GetID();
}

gmexport double ep_shape_next(double world_id, double body_id, double shape_id) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_next: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_next: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_next: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	if(shape->GetNext()==NULL) {
		return 0;
	}
	shape->GetNext()->CacheID();
	return shape->GetNext()->GetID();
}

gmexport double ep_shape_set_uservar(double world_id, double body_id, double shape_id, double index, double value) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_uservar: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_set_uservar: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't set user variable of shape %lu of body %lu in world %lu, index is out of range.", shape->GetID(), body->GetID(), world->GetID());
		return 0;
	}
	((gmuserdata*)(shape->GetUserData()))->var[i] = value;
	return 1;
}

gmexport double ep_shape_get_uservar(double world_id, double body_id, double shape_id, double index) {
	ep_World *world;
	if((world = epmain.FindWorld(gm_cast<unsigned long>(world_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_uservar: World %lu doesn't exist.", gm_cast<unsigned long>(world_id));
		return 0;
	}
	ep_Body *body;
	if((body = world->FindBody(gm_cast<unsigned long>(body_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_uservar: Body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(body_id), world->GetID());
		return 0;
	}
	ep_Shape *shape;
	if((shape = body->FindShape(gm_cast<unsigned long>(shape_id)))==NULL) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "ep_shape_get_uservar: Shape %lu of body %lu doesn't exist in world %lu.", gm_cast<unsigned long>(shape_id), body->GetID(), world->GetID());
		return 0;
	}
	int i = gm_cast<int>(index);
	if(i<0 || i>=GM_USERVARS) {
		epmain.Message(EP_MESSAGELEVEL_ERROR, "Can't get user variable of shape %lu of body %lu in world %lu, index is out of range.", shape->GetID(), body->GetID(), world->GetID());
		return 0;
	}
	return ((gmuserdata*)(shape->GetUserData()))->var[i];
}

