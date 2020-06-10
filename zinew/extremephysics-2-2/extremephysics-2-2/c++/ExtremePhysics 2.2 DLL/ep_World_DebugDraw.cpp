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
 * File: ep_World.cpp
 * Implementation of ep_World.
 */

#include "ExtremePhysics.h"

#if EP_COMPILE_DEBUGDRAW

void ep_World::DebugDrawBodies() {
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		
		main->callback->DrawBody(body->isstatic, body->issleeping, body->x, body->y);
		
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			
			double xx = body->x+ep_transform_x(body->rot_sin, body->rot_cos, shape->x-body->xoff, shape->y-body->yoff),
			       yy = body->y+ep_transform_y(body->rot_sin, body->rot_cos, shape->x-body->xoff, shape->y-body->yoff);
			double sin_t = ep_totaltransform_sin(body->rot_sin, body->rot_cos, shape->rot_sin, shape->rot_cos),
			       cos_t = ep_totaltransform_cos(body->rot_sin, body->rot_cos, shape->rot_sin, shape->rot_cos);
			
			if(shape->shapetype==EP_SHAPETYPE_BOX) {
				double vx[4], vy[4];
				vx[0] = xx+ep_transform_x(sin_t, cos_t, -shape->shapedata.box.w, -shape->shapedata.box.h);
				vy[0] = yy+ep_transform_y(sin_t, cos_t, -shape->shapedata.box.w, -shape->shapedata.box.h);
				vx[1] = xx+ep_transform_x(sin_t, cos_t, +shape->shapedata.box.w, -shape->shapedata.box.h);
				vy[1] = yy+ep_transform_y(sin_t, cos_t, +shape->shapedata.box.w, -shape->shapedata.box.h);
				vx[2] = xx+ep_transform_x(sin_t, cos_t, +shape->shapedata.box.w, +shape->shapedata.box.h);
				vy[2] = yy+ep_transform_y(sin_t, cos_t, +shape->shapedata.box.w, +shape->shapedata.box.h);
				vx[3] = xx+ep_transform_x(sin_t, cos_t, -shape->shapedata.box.w, +shape->shapedata.box.h);
				vy[3] = yy+ep_transform_y(sin_t, cos_t, -shape->shapedata.box.w, +shape->shapedata.box.h);
				main->callback->DrawSegment(vx[0], vy[0], vx[1], vy[1]);
				main->callback->DrawSegment(vx[1], vy[1], vx[2], vy[2]);
				main->callback->DrawSegment(vx[2], vy[2], vx[3], vy[3]);
				main->callback->DrawSegment(vx[3], vy[3], vx[0], vy[0]);
			}
			if(shape->shapetype==EP_SHAPETYPE_CIRCLE) {
				main->callback->DrawCircle(xx, yy, shape->shapedata.circle.r);
				main->callback->DrawSegment(
				  xx,
				  yy,
				  xx+ep_transform_x(sin_t, cos_t, shape->shapedata.circle.r, 0.0),
				  yy+ep_transform_y(sin_t, cos_t, shape->shapedata.circle.r, 0.0));
			}
			if(shape->shapetype==EP_SHAPETYPE_POLYGON) { // poly
				ep_PolygonVertex *vertices = shape->shapedata.polygon.polygon->vertices;
				unsigned long vc = shape->shapedata.polygon.polygon->vertexcount;
				for(unsigned long i = 0; i<vc; ++i) {
					main->callback->DrawSegment(
					  xx+ep_transform_x(sin_t, cos_t, vertices[i].x, vertices[i].y),
					  yy+ep_transform_y(sin_t, cos_t, vertices[i].x, vertices[i].y),
					  xx+ep_transform_x(sin_t, cos_t, vertices[(i+1)%vc].x, vertices[(i+1)%vc].y),
					  yy+ep_transform_y(sin_t, cos_t, vertices[(i+1)%vc].x, vertices[(i+1)%vc].y));
				}
			}
			
		}
		
	}
	
}

void ep_World::DebugDrawLinks() {
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			main->callback->DrawShapeLink(
			  body->isstatic,
			  body->x,
			  body->y,
			  body->x+ep_transform_x(body->rot_sin, body->rot_cos, shape->x-body->xoff, shape->y-body->yoff),
			  body->y+ep_transform_y(body->rot_sin, body->rot_cos, shape->x-body->xoff, shape->y-body->yoff));
		}
	}
	
	for(ep_Contact *contact = first_contact; contact!=NULL; contact = contact->next) {
		main->callback->DrawContactLink(
		  contact->body1->isstatic || contact->body2->isstatic,
		  contact->body1->x+ep_transform_x(contact->body1->rot_sin, contact->body1->rot_cos, contact->shape1->x-contact->body1->xoff, contact->shape1->y-contact->body1->yoff),
		  contact->body1->y+ep_transform_y(contact->body1->rot_sin, contact->body1->rot_cos, contact->shape1->x-contact->body1->xoff, contact->shape1->y-contact->body1->yoff),
		  contact->body2->x+ep_transform_x(contact->body2->rot_sin, contact->body2->rot_cos, contact->shape2->x-contact->body2->xoff, contact->shape2->y-contact->body2->yoff),
		  contact->body2->y+ep_transform_y(contact->body2->rot_sin, contact->body2->rot_cos, contact->shape2->x-contact->body2->xoff, contact->shape2->y-contact->body2->yoff));
	}
	
	for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
		main->callback->DrawJointLink(
		  hingejoint->body1->isstatic || hingejoint->body2->isstatic,
		  hingejoint->body1->x,
		  hingejoint->body1->y,
		  hingejoint->body2->x,
		  hingejoint->body2->y);
	}
	
	for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
		main->callback->DrawJointLink(
		  distancejoint->body1->isstatic || distancejoint->body2->isstatic,
		  distancejoint->body1->x,
		  distancejoint->body1->y,
		  distancejoint->body2->x,
		  distancejoint->body2->y);
	}
	
	for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
		main->callback->DrawJointLink(
		  railjoint->body1->isstatic || railjoint->body2->isstatic,
		  railjoint->body1->x,
		  railjoint->body1->y,
		  railjoint->body2->x,
		  railjoint->body2->y);
	}
	
	for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
		main->callback->DrawJointLink(
		  sliderjoint->body1->isstatic || sliderjoint->body2->isstatic,
		  sliderjoint->body1->x,
		  sliderjoint->body1->y,
		  sliderjoint->body2->x,
		  sliderjoint->body2->y);
	}
	
	//JOINTS//
	
}

void ep_World::DebugDrawViews() {
	
	for(ep_View *view = first_view; view!=NULL; view = view->next) {
		main->callback->DrawView(view->x1, view->y1, view->x2, view->y2);
	}
	
}

void ep_World::DebugDrawVelocity() {
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		main->callback->DrawVelocity(body->x, body->y, body->xvel, body->yvel, body->rotvel);
	}
	
}

void ep_World::DebugDrawForces() {
	
	for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
		double xx = (
		  hingejoint->body1->x+ep_transform_x(hingejoint->body1->rot_sin, hingejoint->body1->rot_cos, hingejoint->x1-hingejoint->body1->xoff, hingejoint->y1-hingejoint->body1->yoff)+
		  hingejoint->body2->x+ep_transform_x(hingejoint->body2->rot_sin, hingejoint->body2->rot_cos, hingejoint->x2-hingejoint->body2->xoff, hingejoint->y2-hingejoint->body2->yoff))*0.5;
		double yy = (
		  hingejoint->body1->y+ep_transform_y(hingejoint->body1->rot_sin, hingejoint->body1->rot_cos, hingejoint->x1-hingejoint->body1->xoff, hingejoint->y1-hingejoint->body1->yoff)+
		  hingejoint->body2->y+ep_transform_y(hingejoint->body2->rot_sin, hingejoint->body2->rot_cos, hingejoint->x2-hingejoint->body2->xoff, hingejoint->y2-hingejoint->body2->yoff))*0.5;
		main->callback->DrawForce(xx, yy, hingejoint->xforce, hingejoint->yforce, hingejoint->motortorque+hingejoint->limittorque);
	}
	
	for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
		double xx1 = distancejoint->body1->x+ep_transform_x(distancejoint->body1->rot_sin, distancejoint->body1->rot_cos, distancejoint->x1-distancejoint->body1->xoff, distancejoint->y1-distancejoint->body1->yoff);
		double xx2 = distancejoint->body2->x+ep_transform_x(distancejoint->body2->rot_sin, distancejoint->body2->rot_cos, distancejoint->x2-distancejoint->body2->xoff, distancejoint->y2-distancejoint->body2->yoff);
		double yy1 = distancejoint->body1->y+ep_transform_y(distancejoint->body1->rot_sin, distancejoint->body1->rot_cos, distancejoint->x1-distancejoint->body1->xoff, distancejoint->y1-distancejoint->body1->yoff);
		double yy2 = distancejoint->body2->y+ep_transform_y(distancejoint->body2->rot_sin, distancejoint->body2->rot_cos, distancejoint->x2-distancejoint->body2->xoff, distancejoint->y2-distancejoint->body2->yoff);
		double fx, fy;
		fx = xx1-xx2;
		fy = yy1-yy2;
		double d = sqrt(ep_sqr(fx)+ep_sqr(fy));
		if(d<EP_EPSILON_MINNORMALVECTOR) {
			fx = 1.0;
			fy = 0.0;
		} else {
			fx /= d;
			fy /= d;
		}
		fx *= distancejoint->motorforce+distancejoint->limitforce;
		fy *= distancejoint->motorforce+distancejoint->limitforce;
		main->callback->DrawForce(xx1, yy1, +fx, +fy, 0.0);
		main->callback->DrawForce(xx2, yy2, -fx, -fy, 0.0);
	}
	
	for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
		double xx, yy;
		xx = railjoint->body1->x+ep_transform_x(railjoint->body1->rot_sin, railjoint->body1->rot_cos, railjoint->x1-railjoint->body1->xoff, railjoint->y1-railjoint->body1->yoff);
		yy = railjoint->body1->y+ep_transform_y(railjoint->body1->rot_sin, railjoint->body1->rot_cos, railjoint->x1-railjoint->body1->xoff, railjoint->y1-railjoint->body1->yoff);
		double vxx, vyy;
		vxx = ep_transform_x(railjoint->body2->rot_sin, railjoint->body2->rot_cos, railjoint->vx, railjoint->vy);
		vyy = ep_transform_y(railjoint->body2->rot_sin, railjoint->body2->rot_cos, railjoint->vx, railjoint->vy);
		double fx, fy;
		fx = vxx*(railjoint->motorforce+railjoint->limitforce)+vyy*railjoint->normalforce;
		fy = vyy*(railjoint->motorforce+railjoint->limitforce)-vxx*railjoint->normalforce;
		main->callback->DrawForce(xx, yy, fx, fy, 0.0);
	}
	
	for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
		double xx, yy;
		xx = sliderjoint->body1->x+ep_transform_x(sliderjoint->body1->rot_sin, sliderjoint->body1->rot_cos, sliderjoint->x1-sliderjoint->body1->xoff, sliderjoint->y1-sliderjoint->body1->yoff);
		yy = sliderjoint->body1->y+ep_transform_y(sliderjoint->body1->rot_sin, sliderjoint->body1->rot_cos, sliderjoint->x1-sliderjoint->body1->xoff, sliderjoint->y1-sliderjoint->body1->yoff);
		double vxx, vyy;
		vxx = ep_transform_x(sliderjoint->body2->rot_sin, sliderjoint->body2->rot_cos, sliderjoint->vx, sliderjoint->vy);
		vyy = ep_transform_y(sliderjoint->body2->rot_sin, sliderjoint->body2->rot_cos, sliderjoint->vx, sliderjoint->vy);
		double fx, fy;
		fx = vxx*(sliderjoint->motorforce+sliderjoint->limitforce)+vyy*sliderjoint->normalforce;
		fy = vyy*(sliderjoint->motorforce+sliderjoint->limitforce)-vxx*sliderjoint->normalforce;
		main->callback->DrawForce(xx, yy, fx, fy, sliderjoint->torque);
	}
	
	//JOINTS//
	
	for(ep_Contact *contact = first_contact; contact!=NULL; contact = contact->next) {
		for(int i = 0; i<2; ++i) {
			ep_ContactPoint *cp = contact->contactpoints+i;
			if(cp->active) {
				double xx = (
				  contact->body1->x+ep_transform_x(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff)+
				  contact->body2->x+ep_transform_x(contact->body2->rot_sin, contact->body2->rot_cos, cp->x2-contact->body2->xoff, cp->y2-contact->body2->yoff))*0.5;
				double yy = (
				  contact->body1->y+ep_transform_y(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff)+
				  contact->body2->y+ep_transform_y(contact->body2->rot_sin, contact->body2->rot_cos, cp->x2-contact->body2->xoff, cp->y2-contact->body2->yoff))*0.5;
				main->callback->DrawForce(xx, yy, cp->normalforce*contact->nx, cp->normalforce*contact->ny, 0.0);
				main->callback->DrawForce(xx, yy, +cp->tangentforce*contact->ny, -cp->tangentforce*contact->nx, 0.0);
			}
		}
	}
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Force *force = body->first_force; force!=NULL; force = force->next) {
			if(force->local) {
				main->callback->DrawForce(
				  body->x+ep_transform_x(body->rot_sin, body->rot_cos, force->x-body->xoff, force->y-body->yoff),
				  body->y+ep_transform_y(body->rot_sin, body->rot_cos, force->x-body->xoff, force->y-body->yoff),
				  ep_transform_x(body->rot_sin, body->rot_cos, force->xforce, force->yforce)*((force->ignoremass)? body->mass : 1.0),
				  ep_transform_y(body->rot_sin, body->rot_cos, force->xforce, force->yforce)*((force->ignoremass)? body->mass : 1.0),
				  force->torque*((force->ignoremass)? body->mass : 1.0));
			} else {
				main->callback->DrawForce(
				  body->x+ep_transform_x(body->rot_sin, body->rot_cos, force->x-body->xoff, force->y-body->yoff),
				  body->y+ep_transform_y(body->rot_sin, body->rot_cos, force->x-body->xoff, force->y-body->yoff),
				  force->xforce*((force->ignoremass)? body->mass : 1.0),
				  force->yforce*((force->ignoremass)? body->mass : 1.0),
				  force->torque*((force->ignoremass)? body->mass : 1.0));
			}
		}
	}
	
}

void ep_World::DebugDrawConstraints() {
	
	for(ep_Contact *contact = first_contact; contact!=NULL; contact = contact->next) {
		for(int i = 0; i<2; ++i) {
			ep_ContactPoint *cp = contact->contactpoints+i;
			if(cp->active) {
				main->callback->DrawContactPoint(
				  contact->body1->x+ep_transform_x(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff),
				  contact->body1->y+ep_transform_y(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff),
				  contact->body2->x+ep_transform_x(contact->body2->rot_sin, contact->body2->rot_cos, cp->x2-contact->body2->xoff, cp->y2-contact->body2->yoff),
				  contact->body2->y+ep_transform_y(contact->body2->rot_sin, contact->body2->rot_cos, cp->x2-contact->body2->xoff, cp->y2-contact->body2->yoff));
				
				/*
				GM_draw_line(
				  contact->body1->x,
				  contact->body1->y,
				  contact->body1->x+ep_transform_x(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff),
				  contact->body1->y+ep_transform_y(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff));
				GM_draw_line(
				  contact->body2->x,
				  contact->body2->y,
				  contact->body2->x+ep_transform_x(contact->body2->rot_sin, contact->body2->rot_cos, cp->x2-contact->body2->xoff, cp->y2-contact->body2->yoff),
				  contact->body2->y+ep_transform_y(contact->body2->rot_sin, contact->body2->rot_cos, cp->x2-contact->body2->xoff, cp->y2-contact->body2->yoff));
				GM_draw_set_color(255);
				GM_draw_line(
				  contact->body1->x+ep_transform_x(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff)-contact->nx*10.0,
				  contact->body1->y+ep_transform_y(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff)-contact->ny*10.0,
				  contact->body1->x+ep_transform_x(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff)+contact->nx*50.0,
				  contact->body1->y+ep_transform_y(contact->body1->rot_sin, contact->body1->rot_cos, cp->x1-contact->body1->xoff, cp->y1-contact->body1->yoff)+contact->ny*50.0);
				*/
				
			}
		}
	}
	for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
		main->callback->DrawHingeJoint(
		  hingejoint->body1->x+ep_transform_x(hingejoint->body1->rot_sin, hingejoint->body1->rot_cos, hingejoint->x1-hingejoint->body1->xoff, hingejoint->y1-hingejoint->body1->yoff),
		  hingejoint->body1->y+ep_transform_y(hingejoint->body1->rot_sin, hingejoint->body1->rot_cos, hingejoint->x1-hingejoint->body1->xoff, hingejoint->y1-hingejoint->body1->yoff),
		  hingejoint->body2->x+ep_transform_x(hingejoint->body2->rot_sin, hingejoint->body2->rot_cos, hingejoint->x2-hingejoint->body2->xoff, hingejoint->y2-hingejoint->body2->yoff),
		  hingejoint->body2->y+ep_transform_y(hingejoint->body2->rot_sin, hingejoint->body2->rot_cos, hingejoint->x2-hingejoint->body2->xoff, hingejoint->y2-hingejoint->body2->yoff));
	}
	for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
		main->callback->DrawDistanceJoint(
		  distancejoint->body1->x+ep_transform_x(distancejoint->body1->rot_sin, distancejoint->body1->rot_cos, distancejoint->x1-distancejoint->body1->xoff, distancejoint->y1-distancejoint->body1->yoff),
		  distancejoint->body1->y+ep_transform_y(distancejoint->body1->rot_sin, distancejoint->body1->rot_cos, distancejoint->x1-distancejoint->body1->xoff, distancejoint->y1-distancejoint->body1->yoff),
		  distancejoint->body2->x+ep_transform_x(distancejoint->body2->rot_sin, distancejoint->body2->rot_cos, distancejoint->x2-distancejoint->body2->xoff, distancejoint->y2-distancejoint->body2->yoff),
		  distancejoint->body2->y+ep_transform_y(distancejoint->body2->rot_sin, distancejoint->body2->rot_cos, distancejoint->x2-distancejoint->body2->xoff, distancejoint->y2-distancejoint->body2->yoff));
	}
	for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
		main->callback->DrawRailJoint(
		  railjoint->body1->x+ep_transform_x(railjoint->body1->rot_sin, railjoint->body1->rot_cos, railjoint->x1-railjoint->body1->xoff, railjoint->y1-railjoint->body1->yoff),
		  railjoint->body1->y+ep_transform_y(railjoint->body1->rot_sin, railjoint->body1->rot_cos, railjoint->x1-railjoint->body1->xoff, railjoint->y1-railjoint->body1->yoff),
		  railjoint->body2->x+ep_transform_x(railjoint->body2->rot_sin, railjoint->body2->rot_cos, railjoint->x2-railjoint->body2->xoff, railjoint->y2-railjoint->body2->yoff),
		  railjoint->body2->y+ep_transform_y(railjoint->body2->rot_sin, railjoint->body2->rot_cos, railjoint->x2-railjoint->body2->xoff, railjoint->y2-railjoint->body2->yoff),
		  railjoint->body2->x+ep_transform_x(railjoint->body2->rot_sin, railjoint->body2->rot_cos, railjoint->x2+railjoint->vx*railjoint->length-railjoint->body2->xoff, railjoint->y2+railjoint->vy*railjoint->length-railjoint->body2->yoff),
		  railjoint->body2->y+ep_transform_y(railjoint->body2->rot_sin, railjoint->body2->rot_cos, railjoint->x2+railjoint->vx*railjoint->length-railjoint->body2->xoff, railjoint->y2+railjoint->vy*railjoint->length-railjoint->body2->yoff));
	}
	for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
		main->callback->DrawSliderJoint(
		  sliderjoint->body1->x+ep_transform_x(sliderjoint->body1->rot_sin, sliderjoint->body1->rot_cos, sliderjoint->x1-sliderjoint->body1->xoff, sliderjoint->y1-sliderjoint->body1->yoff),
		  sliderjoint->body1->y+ep_transform_y(sliderjoint->body1->rot_sin, sliderjoint->body1->rot_cos, sliderjoint->x1-sliderjoint->body1->xoff, sliderjoint->y1-sliderjoint->body1->yoff),
		  sliderjoint->body2->x+ep_transform_x(sliderjoint->body2->rot_sin, sliderjoint->body2->rot_cos, sliderjoint->x2-sliderjoint->body2->xoff, sliderjoint->y2-sliderjoint->body2->yoff),
		  sliderjoint->body2->y+ep_transform_y(sliderjoint->body2->rot_sin, sliderjoint->body2->rot_cos, sliderjoint->x2-sliderjoint->body2->xoff, sliderjoint->y2-sliderjoint->body2->yoff),
		  sliderjoint->body2->x+ep_transform_x(sliderjoint->body2->rot_sin, sliderjoint->body2->rot_cos, sliderjoint->x2+sliderjoint->vx*sliderjoint->length-sliderjoint->body2->xoff, sliderjoint->y2+sliderjoint->vy*sliderjoint->length-sliderjoint->body2->yoff),
		  sliderjoint->body2->y+ep_transform_y(sliderjoint->body2->rot_sin, sliderjoint->body2->rot_cos, sliderjoint->x2+sliderjoint->vx*sliderjoint->length-sliderjoint->body2->xoff, sliderjoint->y2+sliderjoint->vy*sliderjoint->length-sliderjoint->body2->yoff));
	}
	
	//JOINTS//
	
}

#endif

