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
 * File: ep_World_SimulateWater.cpp
 * Simulates simple water (buoyancy and drag).
 */

#include "ExtremePhysics.h"

void ep_World::SimulateWater() {
	
	for(ep_Water *water = first_water; water!=NULL; water = water->next) {
		bool usedrag = (water->lineardrag!=0.0 || water->quadraticdrag!=0.0);
		for(ep_Body *body = first_body; body!=NULL; body = body->next) {
			if(body->issleeping || body->isstatic) continue;
			for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
				
				if(shape->updateaabb) {
					shape->UpdateTransformedCoordinates();
					shape->UpdateAABB();
				}
				if(shape->aabb_x1<=water->x2 && shape->aabb_x2>=water->x1 && shape->aabb_y1<=water->y2 && shape->aabb_y2>=water->y1) {
				
					// calculate area and center of mass
					double area, inertia, center_x, center_y;
#ifdef __GNUC__
					inertia = 0.0;
#endif
					if(shape->shapetype==EP_SHAPETYPE_CIRCLE) {
						double r = shape->shapedata.circle.r;
						if(r==0.0) continue;
						double d = water->y1-shape->y_t;
						if(d>=r) {
							continue;
						} else if(d<=-r) {
							area = M_PI*ep_sqr(r);
							inertia = area*ep_sqr(r)*0.5;
							center_x = 0.0;
							center_y = 0.0;
						} else {
							double r2 = ep_sqr(r);
							double d2 = ep_sqr(d);
							double e = r2-d2;
							double sqrte = sqrt(e);
							double e32 = e*sqrte;
							double ac = acos(d/r);
							area = ac*r2-d*sqrte;
							if(area<EP_EPSILON_MINPOLYMASS) {
								continue;
							}
							double offset = (2.0/3.0)*e32/area;
							if(usedrag) {
								double offset2 = ep_sqr(offset);
								inertia = (d-4.0*offset)*e32/3.0+(0.5*r2+offset2)*(r2*ac-d*sqrte);
							}
							center_x = 0.0;
							center_y = offset;
						}
					} else {
						double d = water->y1-shape->y_t;
						ep_PolygonVertex *vertices;
						unsigned long vc;
						ep_PolygonVertex boxvertices[4];
						if(shape->shapetype==EP_SHAPETYPE_BOX) {
							if(shape->shapedata.box.w==0.0 || shape->shapedata.box.h==0.0) continue;
							boxvertices[0].x = -shape->shapedata.box.w;
							boxvertices[0].y = -shape->shapedata.box.h;
							boxvertices[1].x = +shape->shapedata.box.w;
							boxvertices[1].y = -shape->shapedata.box.h;
							boxvertices[2].x = +shape->shapedata.box.w;
							boxvertices[2].y = +shape->shapedata.box.h;
							boxvertices[3].x = -shape->shapedata.box.w;
							boxvertices[3].y = +shape->shapedata.box.h;
							vertices = boxvertices;
							vc = 4;
							area = shape->shapedata.box.w*shape->shapedata.box.h*4.0;
							inertia = area*(ep_sqr(shape->shapedata.box.w)+ep_sqr(shape->shapedata.box.h))/3.0;
							center_x = 0.0;
							center_y = 0.0;
						} else {
							vertices = shape->shapedata.polygon.polygon->vertices;
							vc = shape->shapedata.polygon.polygon->vertexcount;
							area = shape->shapedata.polygon.polygon->mass;
							inertia = shape->shapedata.polygon.polygon->inertia;
							center_x = ep_transform_x(shape->sin_t, shape->cos_t, shape->shapedata.polygon.polygon->xoff, shape->shapedata.polygon.polygon->yoff);
							center_y = ep_transform_y(shape->sin_t, shape->cos_t, shape->shapedata.polygon.polygon->xoff, shape->shapedata.polygon.polygon->yoff);
						}
						unsigned long j = vc-1;
						double prev_vy = ep_transform_y(shape->sin_t, shape->cos_t, vertices[j].x, vertices[j].y);
						unsigned long i1, i2, counter = 0;
						double x1, x2;
#ifdef __GNUC__
						i1 = 0;
						i2 = 0;
						x1 = 0.0;
						x2 = 0.0;
#endif
						for(unsigned long i = 0; i<vc; ++i) {
							double vy = ep_transform_y(shape->sin_t, shape->cos_t, vertices[i].x, vertices[i].y);
							if(vy>d && prev_vy<=d) {
								double vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[i].x, vertices[i].y);
								double prev_vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[j].x, vertices[j].y);
								double dist = vy-prev_vy;
								if(dist<EP_EPSILON_MINCONTACTPOINTS) {
									x1 = prev_vx;
								} else {
									x1 = prev_vx+(vx-prev_vx)*(d-prev_vy)/dist;
								}
								i1 = i;
								if(++counter>=2) break;
							}
							if(vy<=d && prev_vy>d) {
								double vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[i].x, vertices[i].y);
								double prev_vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[j].x, vertices[j].y);
								double dist = prev_vy-vy;
								if(dist<EP_EPSILON_MINCONTACTPOINTS) {
									x2 = vx;
								} else {
									x2 = vx+(prev_vx-vx)*(d-vy)/dist;
								}
								i2 = j;
								if(++counter>=2) break;
							}
							j = i;
							prev_vy = vy;
						}
						if(counter==0) {
							if(prev_vy<=d) {
								continue;
							}
						} else {
							area = 0.0;
							{
								double xoff = 0.0, yoff = 0.0;
								unsigned long i = i1;
								while(i!=i2) {
									j = (i+1)%vc;
									double dd = vertices[i].x*vertices[j].y-vertices[i].y*vertices[j].x;
									area += dd;
									xoff += dd*(vertices[i].x+vertices[j].x)/3.0;
									yoff += dd*(vertices[i].y+vertices[j].y)/3.0;
									i = j;
								}
								center_x = ep_transform_x(shape->sin_t, shape->cos_t, xoff, yoff);
								center_y = ep_transform_y(shape->sin_t, shape->cos_t, xoff, yoff);
							}
							{
								double dd, vx, vy;
								dd = (x2-x1)*d;
								area += dd;
								center_x += dd*(x2+x1)/3.0;
								center_y += dd*d*(2.0/3.0);
								vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[i1].x, vertices[i1].y);
								vy = ep_transform_y(shape->sin_t, shape->cos_t, vertices[i1].x, vertices[i1].y);
								dd = x1*vy-d*vx;
								area += dd;
								center_x += dd*(x1+vx)/3.0;
								center_y += dd*(d+vy)/3.0;
								vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[i2].x, vertices[i2].y);
								vy = ep_transform_y(shape->sin_t, shape->cos_t, vertices[i2].x, vertices[i2].y);
								dd = vx*d-vy*x2;
								area += dd;
								center_x += dd*(vx+x2)/3.0;
								center_y += dd*(vy+d)/3.0;
							}
							if(area<EP_EPSILON_MINPOLYMASS) {
								continue;
							}
							center_x /= area;
							center_y /= area;
							area *= 0.5;
							if(usedrag) {
								// calculate moment of inertia
								inertia = 0.0;
								{
									double xoff = ep_invtransform_x(shape->sin_t, shape->cos_t, center_x, center_y);
									double yoff = ep_invtransform_y(shape->sin_t, shape->cos_t, center_x, center_y);
									unsigned long i = i1;
									while(i!=i2) {
										j = (i+1)%vc;
										double vx1 = vertices[i].x-xoff, vy1 = vertices[i].y-yoff;
										double vx2 = vertices[j].x-xoff, vy2 = vertices[j].y-yoff;
										inertia += (vx1*vy2-vy1*vx2)*(ep_sqr(vx1)+ep_sqr(vy1)+ep_sqr(vx2)+ep_sqr(vy2)+(vx1)*(vx2)+(vy1)*(vy2));
										i = j;
									}
								}
								{
									double vx, vy;
									double vx1, vy1, vx2, vy2;
									vx1 = x2-center_x; vy1 = d-center_y;
									vx2 = x1-center_x; vy2 = d-center_y;
									inertia += (vx1*vy2-vy1*vx2)*(ep_sqr(vx1)+ep_sqr(vy1)+ep_sqr(vx2)+ep_sqr(vy2)+(vx1)*(vx2)+(vy1)*(vy2));
									vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[i1].x, vertices[i1].y);
									vy = ep_transform_y(shape->sin_t, shape->cos_t, vertices[i1].x, vertices[i1].y);
									vx1 = x1-center_x; vy1 = d-center_y;
									vx2 = vx-center_x; vy2 = vy-center_y;
									inertia += (vx1*vy2-vy1*vx2)*(ep_sqr(vx1)+ep_sqr(vy1)+ep_sqr(vx2)+ep_sqr(vy2)+(vx1)*(vx2)+(vy1)*(vy2));
									vx = ep_transform_x(shape->sin_t, shape->cos_t, vertices[i2].x, vertices[i2].y);
									vy = ep_transform_y(shape->sin_t, shape->cos_t, vertices[i2].x, vertices[i2].y);
									vx1 = vx-center_x; vy1 = vy-center_y;
									vx2 = x2-center_x; vy2 = d-center_y;
									inertia += (vx1*vy2-vy1*vx2)*(ep_sqr(vx1)+ep_sqr(vy1)+ep_sqr(vx2)+ep_sqr(vy2)+(vx1)*(vx2)+(vy1)*(vy2));
								}
								inertia /= 12.0;
							}
						}
					}
				
					if(shape->x_t+center_x>water->x1 && shape->x_t+center_x<water->x2 && shape->y_t+center_y<water->y2) {
					
						center_x += shape->x_t-body->x;
						center_y += shape->y_t-body->y;
						double f = -area*water->density*timestep;
						double fx = water->gravity_x*f;
						double fy = water->gravity_y*f;
					
						if(usedrag) {
							/*
							Simulating real quadratic drag is very hard, so I'm using
							a much simpler method here. It's not very accurate but it looks
							realistic enough in most situations.
							*/
							double delta_xvel = water->xvel-(body->xvel+center_y*body->rotvel);
							double delta_yvel = water->yvel-(body->yvel-center_x*body->rotvel);
							double v = sqrt(ep_sqr(delta_xvel)+ep_sqr(delta_yvel));
							double ld = water->lineardrag*timestep/shape->density;
							if(ld>1.0) ld = 1.0;
							double drag = area*shape->density*(1.0-(1.0-ld)*exp(-v*water->quadraticdrag*timestep/shape->density));
							fx += delta_xvel*drag;
							fy += delta_yvel*drag;
							double delta_rotvel = -body->rotvel;
							double equivalent_radius = sqrt(sqrt(inertia/(M_PI*0.5)));
							ld = water->lineardrag*timestep/shape->density;
							if(ld>1.0) ld = 1.0;
							drag = inertia*shape->density*(1.0-(1.0-ld)*exp(-fabs(delta_rotvel)*equivalent_radius*water->quadraticdrag*timestep/shape->density));
							body->_ApplyTorque(delta_rotvel*drag);
						}
					
						body->_ApplyImpulse(center_x, center_y, fx, fy);
					
					}
				
				}
			
			}
		}
	}
	
}

