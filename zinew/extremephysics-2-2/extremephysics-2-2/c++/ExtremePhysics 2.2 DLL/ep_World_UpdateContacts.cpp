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
 * File: ep_World_UpdateContacts.cpp
 * Updates contacts for the simulation.
 */

#include "ExtremePhysics.h"

#include "ep_Contact_Update.h"

/*
Portability warning:
This code assumes sizeof(double)==8.
*/

EP_FORCEINLINE void RadixSort(const ep_uint32 values[], unsigned long indices[], unsigned long temp[], unsigned long count) {
	// A very fast linear time sorting algorithm for 32-bit unsigned integers
	// The algorithm sorts the list once for every radix (byte)
	// using counting sort (a stable sorting algorithm).
	// The array of values will not be sorted, only the indices
	// (and temp) are changed by the algorithm.
	
	unsigned long counters[4*256]; // 4kb
	unsigned long offsets[256]; // 1kb
	
	// erase counters
	for(unsigned long *p = counters; p!=counters+4*256;) {
		*(p++) = 0;
	}
	
	// fill counters
	{
		for(unsigned char *radix = (unsigned char*)(values), *stop = (unsigned char*)(values+count); radix!=stop;) {
			++counters[*(radix++)+256*0];
			++counters[*(radix++)+256*1];
			++counters[*(radix++)+256*2];
			++counters[*(radix++)+256*3];
		}
	}
	
	// radix sort
	for(unsigned int j = 0; j<4; ++j) {
		
		// fill offsets
		unsigned long *counter = counters+256*j;
		unsigned long cumsum = 0;
		offsets[0] = 0;
		for(unsigned long *p = offsets+1; p!=offsets+256; ) {
			cumsum += *(counter++);
			*(p++) = cumsum;
		}
		
		// sort
		unsigned char *input = (unsigned char*)(values)+j;
		for(unsigned long *ind = indices, *stop = indices+count; ind!=stop; ) {
			unsigned long i = *(ind++);
			temp[offsets[input[4*i]]++] = i;
		}
		
		// swap indices
		{
			unsigned long *t;
			t = indices;
			indices = temp;
			temp = t;
		}
		
	}
	
}

bool ep_World::_UpdateContacts() {
	// Based on Pierre Terdiman's Radix Sweep-And-Prune (Radix SAP).
	// http://www.codercorner.com/SweepAndPrune.htm
	// It's basically the same as SAP, but it's not incremental.
	// When there are lots of small, fast moving bodies (e.g. particles)
	// this is a lot better (normal SAP will waste time looking
	// for coherence even when there is no coherence). When there
	// are no moving bodies normal SAP is about 50% faster.
	
	// update and count shapes
	unsigned long shapecount = 0;
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			if(shape->collidemask1==0 && shape->collidemask2==0) continue;
			if(shape->updateaabb) {
				shape->UpdateTransformedCoordinates();
				shape->UpdateAABB();
			}
			++shapecount;
		}
	}
	if(shapecount<2) {
		return true;
	}
	
	// set 'destroy' flag
	for(ep_Contact *contact = first_contact; contact!=NULL; contact = contact->next) {
		contact->destroy = (contact->shape1->updatecontacts || contact->shape2->updatecontacts);
	}
	
	// create temporary array for sorting
	void *temparray = ep_malloc(shapecount*sizeof(ep_Shape*)+shapecount*sizeof(ep_uint32)+2*shapecount*sizeof(unsigned long)+shapecount*sizeof(unsigned long*));
	if(temparray==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not initialize simulation in world %lu, memory allocation of temporary AABB array failed.", id);
#endif
		return false;
	}
	ep_Shape **shapes = (ep_Shape**)(temparray);
	ep_uint32 *x1_array = (ep_uint32*)(shapes+shapecount);
	unsigned long *sorted = (unsigned long*)(x1_array+shapecount);
	unsigned long *temp = (unsigned long*)(sorted+shapecount);
	unsigned long **nextupdate = (unsigned long**)(temp+shapecount);
	
	// fill temporary array
	{
		unsigned long i = 0;
		for(ep_Body *body = first_body; body!=NULL; body = body->next) {
			for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
				if(shape->collidemask1==0 && shape->collidemask2==0) continue;
				// This method uses a surjection from double to ep_uint32.
				// Precision is not important here and radix sort uses ep_uint32.
				// Originally I used "(f<0)? f^0xffffffff : f^0x80000000"
				// but the method described at
				// http://www.stereopsis.com/radix.html
				// is probably faster.
				double x1, y1, x2, y2;
				ep_uint32 f;
				if(horizontal) {
					x1 = shape->aabb_x1;
					y1 = shape->aabb_y1;
					x2 = shape->aabb_x2+contact_threshold;
					y2 = shape->aabb_y2+contact_threshold;
				} else {
					x1 = shape->aabb_y1;
					y1 = shape->aabb_x1;
					x2 = shape->aabb_y2+contact_threshold;
					y2 = shape->aabb_x2+contact_threshold;
				}
				f = *((ep_uint32*)(&x1)+1);
				shape->aabbul_x1 = f^((-(ep_int32)(f>>31))|0x80000000);
				f = *((ep_uint32*)(&y1)+1);
				shape->aabbul_y1 = f^((-(ep_int32)(f>>31))|0x80000000);
				f = *((ep_uint32*)(&x2)+1);
				shape->aabbul_x2 = f^((-(ep_int32)(f>>31))|0x80000000);
				f = *((ep_uint32*)(&y2)+1);
				shape->aabbul_y2 = f^((-(ep_int32)(f>>31))|0x80000000);
				shapes[i] = shape;
				x1_array[i] = shape->aabbul_x1;
				sorted[i] = i;
				++i;
			}
		}
	}
	
	// sort
	RadixSort(x1_array, sorted, temp, shapecount);
	
	// create list of next update pointers
	{
		unsigned long *current_nextupdate = sorted+shapecount;
		unsigned long *stop = sorted;
		for(unsigned long *pos = current_nextupdate; pos!=stop; ) {
			--pos;
			nextupdate[*pos] = current_nextupdate;
			if(shapes[*pos]->updatecontacts) {
				current_nextupdate = pos;
			}
		}
	}
	
	// SAP (Sweep And Prune)
	{
		unsigned long *stop = sorted+shapecount;
		for(unsigned long *pos1 = sorted; pos1!=stop; ++pos1) {
			ep_uint32 x2 = shapes[*pos1]->aabbul_x2;
			ep_uint32 y1 = shapes[*pos1]->aabbul_y1;
			ep_uint32 y2 = shapes[*pos1]->aabbul_y2;
			
			if(shapes[*pos1]->updatecontacts) { // update vs all
				
				// first pass (clear temp_c)
				unsigned long *pos2 = pos1+1;
				if(pos2!=stop && x1_array[*pos2]<=x2) {
					shapes[*pos2]->temp_c = NULL;
					for(++pos2; pos2!=stop && x1_array[*pos2]<=x2; ++pos2) {
						shapes[*pos2]->temp_c = NULL;
					}
				} else {
					continue;
				}
				
				// find existing contacts
				for(ep_ContactLink *cl = shapes[*pos1]->first_contactlink; cl!=NULL; cl = cl->next) {
					cl->other_shape->temp_c = cl->contact;
				}
				
				// second pass (update contacts)
				for(unsigned long *pos2 = pos1+1; pos2!=stop && x1_array[*pos2]<=x2; ++pos2) {
					if(shapes[*pos2]->aabbul_y1>y2 || y1>shapes[*pos2]->aabbul_y2) continue;
					
					// flip shapes?
					ep_Shape *shape1, *shape2;
					if((*pos1)>(*pos2)) {
						shape1 = shapes[*pos2];
						shape2 = shapes[*pos1];
					} else {
						shape1 = shapes[*pos1];
						shape2 = shapes[*pos2];
					}
					
					if(shape1->body==shape2->body || (shape1->body->isstatic && shape2->body->isstatic)) { //  || (!shape1->updatecontacts && !shape2->updatecontacts)
						continue;
					}
					if(!CheckCollisionMasks(shape1->collidemask1, shape1->collidemask2, shape1->group, shape2->collidemask1, shape2->collidemask2, shape2->group)) {
						continue;
					}
					
					ep_Contact *contact = shapes[*pos2]->temp_c;
					ep_CollisionData cd;
					if(Collision(&cd, shape1, shape2, (contact==NULL)? -contact_threshold : contact_threshold)) {
						if(contact==NULL) {
#if EP_USE_EXCEPTIONS
							try {
#endif
							contact = CreateContact(shape1, shape2); // awakes the bodies
#if EP_USE_EXCEPTIONS
							}
							catch(...) {
								ep_free(temparray);
								throw;
							}
#endif
							if(contact==NULL) {
								ep_free(temparray);
								return false;
							}
						}
						contact->Update(&cd);
						contact->destroy = false;
					}
					
				}
				
			} else { // noupdate vs update
				
				// first pass (clear temp_c)
				unsigned long *pos2 = nextupdate[*pos1];
				if(pos2!=stop && x1_array[*pos2]<=x2) {
					shapes[*pos2]->temp_c = NULL;
					for(pos2 = nextupdate[*pos2]; pos2!=stop && x1_array[*pos2]<=x2; pos2 = nextupdate[*pos2]) {
						shapes[*pos2]->temp_c = NULL;
					}
				} else {
					continue;
				}
				
				// find existing contacts
				for(ep_ContactLink *cl = shapes[*pos1]->first_contactlink; cl!=NULL; cl = cl->next) {
					cl->other_shape->temp_c = cl->contact;
				}
				
				// second pass (update contacts)
				for(unsigned long *pos2 = nextupdate[*pos1]; pos2!=stop && x1_array[*pos2]<=x2; pos2 = nextupdate[*pos2]) {
					if(shapes[*pos2]->aabbul_y1>y2 || y1>shapes[*pos2]->aabbul_y2) continue;
					
					// flip shapes?
					ep_Shape *shape1, *shape2;
					if((*pos1)>(*pos2)) {
						shape1 = shapes[*pos2];
						shape2 = shapes[*pos1];
					} else {
						shape1 = shapes[*pos1];
						shape2 = shapes[*pos2];
					}
					
					if(shape1->body==shape2->body || (shape1->body->isstatic && shape2->body->isstatic)) {
						continue;
					}
					if(!CheckCollisionMasks(shape1->collidemask1, shape1->collidemask2, shape1->group, shape2->collidemask1, shape2->collidemask2, shape2->group)) {
						continue;
					}
					
					ep_Contact *contact = shapes[*pos2]->temp_c;
					ep_CollisionData cd;
					if(Collision(&cd, shape1, shape2, (contact==NULL)? -contact_threshold : contact_threshold)) {
						if(contact==NULL) {
#if EP_USE_EXCEPTIONS
							try {
#endif
							contact = CreateContact(shape1, shape2); // awakes the bodies
#if EP_USE_EXCEPTIONS
							}
							catch(...) {
								ep_free(temparray);
								throw;
							}
#endif
							if(contact==NULL) {
								ep_free(temparray);
								return false;
							}
						}
						contact->Update(&cd);
						contact->destroy = false;
					}
					
				}
				
			}
			
		}
	}
	
	ep_free(temparray);
	
	// destroy invalid contacts
	for(ep_Contact *contact = first_contact, *t; contact!=NULL; contact = t) {
		t = contact->next;
		if(contact->destroy) {
			DestroyContact(contact); // this will awake the bodies
		}
	}
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
			shape->updatecontacts = false;
		}
	}
	
	return true;
	
}

