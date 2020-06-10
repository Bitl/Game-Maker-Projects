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
 * File: ep_IdHashTable.h
 * This is just a very simple hash table to speed up id lookups.
 * Included in ExtremePhysics.h.
 */

#ifndef EP_IDHASHTABLE_H
#define EP_IDHASHTABLE_H

#if EP_USE_IDHASHTABLE
#define EP_IDHASHTABLE_BUCKETS (1<<EP_IDHASHTABLE_BUCKETS_BITS)
#define EP_IDHASHTABLE_BUCKETS_MASK (EP_IDHASHTABLE_BUCKETS-1)
class ep_IdHashTableEntry {
	
	friend class ep_IdHashTable;
	
	public:
	unsigned long id;
	
	private:
	ep_IdHashTableEntry *hashentry_prev, *hashentry_next;
	
};
class ep_IdHashTable {
	
	private:
	ep_IdHashTableEntry *buckets[EP_IDHASHTABLE_BUCKETS];
	
	private:
	EP_FORCEINLINE unsigned int GetBucket(unsigned long id) { return id&EP_IDHASHTABLE_BUCKETS_MASK; }
	
	public:
	EP_FORCEINLINE void Init() {
		for(unsigned int b = 0; b<EP_IDHASHTABLE_BUCKETS; ++b) {
			buckets[b] = NULL;
		}
	}
	EP_FORCEINLINE void Insert(ep_IdHashTableEntry* e) {
		unsigned int b = GetBucket(e->id);
		e->hashentry_prev = NULL;
		if(buckets[b]!=NULL) {
			buckets[b]->hashentry_prev = e;
		}
		e->hashentry_next = buckets[b];
		buckets[b] = e;
	}
	EP_FORCEINLINE void Remove(ep_IdHashTableEntry* e) {
		unsigned int b = GetBucket(e->id);
		if(e->hashentry_prev!=NULL) {
			e->hashentry_prev->hashentry_next = e->hashentry_next;
		} else {
			buckets[b] = e->hashentry_next;
		}
		if(e->hashentry_next!=NULL) {
			e->hashentry_next->hashentry_prev = e->hashentry_prev;
		}
	}
	EP_FORCEINLINE ep_IdHashTableEntry* Find(unsigned long id) {
		unsigned int b = GetBucket(id);
		for(ep_IdHashTableEntry* e = buckets[b]; e!=NULL; e = e->hashentry_next) {
			if(e->id==id) return e;
		}
		return NULL;
	}
	
};
#endif // EP_USE_IDHASHTABLE

#endif // EP_IDHASHTABLE_H

