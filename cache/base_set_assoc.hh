/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASESETASSOC_HH__
#define __MEM_CACHE_TAGS_BASESETASSOC_HH__

#include <cassert>
#include <cstring>
#include <list>

#include "mem/cache/base.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/cacheset.hh"
#include "mem/packet.hh"
#include "params/BaseSetAssoc.hh"

/**
 * A BaseSetAssoc cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc tags provide a base, as well as the functionality
 * common to any set associative tags. Any derived class must implement
 * the methods related to the specifics of the actual replacment policy.
 * These are:
 *
 * BlkType* accessBlock();
 * BlkType* findVictim();
 * void insertBlock();
 * void invalidate();
 */
class BaseSetAssoc : public BaseTags
{
  public:
    /** Typedef the block type used in this tag store. */
    typedef CacheBlk BlkType;
    /** Typedef for a list of pointers to the local block class. */
    typedef std::list<BlkType*> BlkList;
    /** Typedef the set type used in this tag store. */
    typedef CacheSet<CacheBlk> SetType;
    uint64_t *tag_array_cpu1 ;  // pointer to our tag store for cpu 1
    uint64_t *tag_array_cpu2 ; //  pointer to our tag store for cpu 2
    uint64_t *counter_array_cpu1 ; // pointer to our array of hit counters for CPU 1
    uint64_t *counter_array_cpu2 ; // pointer to our array of hit counters for CPU 2 
    bool *valid_array_cpu1 ;
    bool *valid_array_cpu2 ;
    uint16_t *position_array_cpu1 ; // pointer to position arrays to calculate LRU positions 
    uint16_t *position_array_cpu2 ;
    bool is_l2 ; // another variable to check whether l2 cache or not 
    uint64_t num_hits ;
    uint16_t *barrier ; 
  protected:
    /** The associativity of the cache. */
    const unsigned assoc;
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;
    /** The number of sets in the cache. */
    const unsigned numSets;
    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;
   
    /** The cache sets. */
    SetType *sets;

    /** The cache blocks. */
    BlkType *blks;
    /** The data blocks, 1 per cache block. */
    uint8_t *dataBlks;

    /** The amount to shift the address to get the set. */
    int setShift;
    /** The amount to shift the address to get the tag. */
    int tagShift;
    /** Mask out all bits that aren't part of the set index. */
    unsigned setMask;
    /** Mask out all bits that aren't part of the block offset. */
    unsigned blkMask;

public:

    /** Convenience typedef. */
     typedef BaseSetAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    BaseSetAssoc(const Params *p);

    /**
     * Destructor
     */
    virtual ~BaseSetAssoc();

    /**
     * Return the block size.
     * @return the block size.
     */
    unsigned
    getBlockSize() const
    {
        return blkSize;
    }

    /**
     * Return the subblock size. In the case of BaseSetAssoc it is always
     * the block size.
     * @return The block size.
     */
    unsigned
    getSubBlockSize() const
    {
        return blkSize;
    }

    /**
     * Return the number of sets this cache has
     * @return The number of sets.
     */
    unsigned
    getNumSets() const override
    {
        return numSets;
    }

    /**
     * Return the number of ways this cache has
     * @return The number of ways.
     */
    unsigned
    getNumWays() const override
    {
        return assoc;
    }

    /**
     * Find the cache block given set and way
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The cache block.
     */
    CacheBlk *findBlockBySetAndWay(int set, int way) const override;

    /**
     * Invalidate the given block.
     * @param blk The block to invalidate.
     */
    
    // Function to find the way a given tag is located in . Depending on id checks the required tag store and set . Returns -1 if not found . 

   
  
   int find_lru_pos ( int way , int set , int id )
   {
      uint16_t *position_array = (id==0) ? position_array_cpu1 : position_array_cpu2 ;
      int pos = -1 ;
      
      for (int i = 0; i<assoc ; i++)
      {
         if(position_array[set *assoc + i] == way)
         {
           pos = i ;
           break ; 
         }
       }
       return pos ; 
         
   }

  void move_pos_to_MRU ( int pos,int set,int id)
   {
      uint16_t *position_array = (id==0)?position_array_cpu1:position_array_cpu2 ; 
      
      for(int i = find_lru_pos(pos , set , id) ; i>=1 ; i--)
      {
         position_array[set *assoc + i ] = position_array[set*assoc + i -1] ;
       }
       position_array[set*assoc] = pos ;
   }

  void move_pos_to_LRU ( int pos , int set , int id)
   {
      uint16_t *position_array = (id==0)?position_array_cpu1 : position_array_cpu2 ; 
      for (int i = find_lru_pos (pos , set ,id) ; i<assoc - 1 ; i++)
          position_array[set*assoc + i] = position_array[set *assoc + i+ 1];
      position_array[set * assoc + assoc -1 ] = pos ;
   }
      

  

  void update_counter (int pos , int set , int id)
  {
    int lru_pos = find_lru_pos (pos ,set ,id) ;
    if (id==0)
    {
       counter_array_cpu1[set*assoc + lru_pos] ++ ;
    }
    else
    {
      counter_array_cpu2[set*assoc + lru_pos]++;
    }
  }


   int find_tag ( Addr tag , int set , int id)  
   {
     uint64_t* tag_array = (id==0) ? tag_array_cpu1 : tag_array_cpu2 ;
     bool *valid_array = (id==0) ? valid_array_cpu1 : valid_array_cpu2 ;
     int pos = -1 ;
     
     for (int i = 0 ; i<assoc ;i++)
     {
        if ((tag_array[set*assoc +i] ==tag) && valid_array[set *assoc + i]){
           pos =i ;
           break ;
        }
     }
     return pos ;
 
    
   }
       	    
          
    void invalidateTag (Addr tag , int set , int id) 
    {
     bool *valid_array = (id==0)? valid_array_cpu1 : valid_array_cpu2 ; 
     int pos = find_tag (tag ,set , id) ;
     if(pos!=-1) valid_array[set * assoc + pos] = false ; 
     move_pos_to_LRU (pos , set , id) ; 
    } 

    int insertTag (Addr tag , int set , int id )   // insert tag in LRU position
    {
      uint16_t* position_array = (id ==0) ? position_array_cpu1 : position_array_cpu2 ;
      bool *valid_array = (id ==0)? valid_array_cpu1 : valid_array_cpu2 ;
      uint16_t pos_to_insert = position_array[set * assoc + assoc -1 ] ;
      uint64_t* tag_array = (id ==0) ? tag_array_cpu1 : tag_array_cpu2 ; 
      tag_array[set *assoc + pos_to_insert] = tag ; 
      valid_array [set*assoc + pos_to_insert] = true ;
      return (int)pos_to_insert ;
    }  
        

    void invalidate(CacheBlk *blk) override
    {
        assert(blk);
        assert(blk->isValid());
        if((blk->thread_context != -1)&& is_l2 ) // if valid context ID
           invalidateTag (blk->tag , blk->set ,blk->thread_context) ; // call invalidate tag 
        tagsInUse--;
        assert(blk->srcMasterId < cache->system->maxMasters());
        occupancies[blk->srcMasterId]--;
        blk->srcMasterId = Request::invldMasterId;
        blk->task_id = ContextSwitchTaskId::Unknown;
        blk->tickInserted = curTick();
    }

     void accessTag(Addr tag , int set , int id)  // Function to access a tag in a given tag store depending on the context ID  . 
   {
        int pos = find_tag (tag , set,id);  // function to find our tag
        if (pos >=0)   //  
        {
           update_counter(pos,set ,id) ;  // update counters for the LRU position corresponding to that way
           move_pos_to_MRU (pos ,set , id) ; // position corresponds to the way  . Move to head makes the tag most recently used .  
           
        }
       else 
        {
          int way = insertTag ( tag , set , id) ;  // insert the tag into tag store , replacing the LRU pos . function returns the way in which the tag has been inserted . 
           move_pos_to_MRU(way,set,id) ;     // move it to head
        }
    }

     uint64_t get_total_utility (int i , int id)
    {
       uint64_t sum = 0 ;
       uint64_t *counter_array = (id==0)? counter_array_cpu1:counter_array_cpu2 ;
       for (int j1 = 0 ; j1 < numSets ; j1++)
       {
         for (int j2 = 1; j2 < i ; j2++)
         {
            sum += counter_array[j1*assoc + j2] ;
         }
        }
       return sum ;
   
     }

   uint16_t get_optimal_utility_set (int set)  // returns the max utility barrier value for a single set
   {
      uint64_t max_utility = -1;
      uint16_t choice = assoc/2 ; 
      for (int i = 1 ; i< assoc ; i++)
      {
        uint64_t local_sum = 0 ;
        uint64_t sum_cpu1 = 0;
        uint64_t sum_cpu2 = 0; 
        for ( int j = 1 ; j< i ; j++)  // get utility when increasing from 1 to i ways for application 1 
          sum_cpu1 += counter_array_cpu1[set *assoc + j] ;
        for (int j1 = 1 ; j1 < assoc - i ; j1++)
          sum_cpu2 += counter_array_cpu2[set *assoc + j1 ] ;
        local_sum = sum_cpu1 + sum_cpu2 ; 
        if (local_sum > max_utility)
        {
           choice = i ;
           max_utility = local_sum ;
        }
      }
      printf("max %d\n",(int)max_utility);
      return choice ; 
} 
        
   

   void repartition()
   {
    int i ,j;
    for (i =0 ; i< numSets ; i++)
    {
       barrier[i] = get_optimal_utility_set (i) ;
    }
    
     
      
       for ( i =0 ; i<numSets ; i++)
       {
         for ( j =0 ; j < assoc ; j++)
        {
           if(sets[i].blks[j]->way < barrier[i])
           {
             if(sets[i].blks[j]->thread_context != 0)
                 sets[i].blks[j]->is_interchanged = true ; 
             else if (sets[i].blks[j]-> is_interchanged == true )
                 sets[i].blks[j]->is_interchanged = false ; 
             sets[i].blks[j]->owner_id = 0 ;
           }
          else if (sets[i].blks[j]->way >= barrier[i])
           {
              if(sets[i].blks[j]->thread_context!=1)
                sets[i].blks[j]->is_interchanged = true ;
              else if ( sets[i].blks[j]->is_interchanged ==true)
                sets[i].blks[j]->is_interchanged = false ; 
              sets[i].blks[j]->owner_id = 1 ;
           }
         }
        }
         
        for ( i = 0 ; i<numSets ; i++ )
        {
           for ( j =0 ; j < assoc ; j++)
           {
             if(sets[i].blks[j]->is_interchanged == true)
             {
                sets[i].moveToTail(sets[i].blks[j] , barrier[i] , sets[i].blks[j]->owner_id ,is_l2) ;
             }
            }
         }

        for ( i = 0 ; i<numSets ; i++ )                   // move invalid blocks to the tail again since we give them less priority than interchanged blocks
        {
           for ( j =0 ; j < assoc ; j++)
           {
             if(!sets[i].blks[j]->isValid())
             {
                sets[i].moveToTail(sets[i].blks[j] , barrier[i] , sets[i].blks[j]->owner_id ,is_l2) ;
             }
            }
         }
    
     // barrier = choice ;
    printf("Reallocating %d\n" , barrier[0]);
     
   
}        
 
      

       
      
        
      
       
        
        
       

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache
     * access and should only be used as such. Returns the access latency as a
     * side effect.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @param lat The access latency.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat,
                          int context_src) override
    {
        Addr tag = extractTag(addr);
        int set = extractSet(addr);
        if((context_src!=-1)&& is_l2)                      // call access tag only if valid context id 
          accessTag (tag , set , context_src) ;
        BlkType *blk = sets[set].findBlk(tag, is_secure);
        lat = accessLatency;;

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                dataAccesses += 1;
            }
        } else {
            dataAccesses += allocAssoc;
        }

        if (blk != nullptr) {
            if (blk->whenReady > curTick()
                && cache->ticksToCycles(blk->whenReady - curTick())
                > accessLatency) {
                lat = cache->ticksToCycles(blk->whenReady - curTick());
            }
            blk->refCount += 1;
        }
    if(blk!=NULL) num_hits ++ ;
    if (num_hits !=0 && num_hits%2000 ==0 &&is_l2)
       repartition();

        return blk;
    }

    /**
     * Finds the given address in the cache, do not update replacement data.
     * i.e. This is a no-side-effect find of a block.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    /**
     * Find an invalid block to evict for the address provided.
     * If there are no invalid blocks, this will return the block
     * in the least-recently-used position.
     * @param addr The addr to a find a replacement candidate for.
     * @return The candidate block.
     */
    CacheBlk* findVictim(Addr addr , int id) override
    {
        BlkType *blk = nullptr;
        int set = extractSet(addr);
        if(is_l2)
        {
          int ll = (id ==0) ? 0 : barrier[set] ;
          int ul = (id ==0 ) ? barrier[set] -1 : assoc -1 ;
          for (int i =0 ; i < assoc ; i++)
          {
              if ( (sets[set].blks[i]->way >=ll) && (sets[set].blks[i]->way <=ul))
              {
                 blk = sets[set].blks[i] ;
                 if (!blk->isValid() )
                    break ;
               }
           }
           return blk ;
         }
              

        // prefer to evict an invalid block
        for (int i = 0; i < allocAssoc; ++i) {
            blk = sets[set].blks[i];
            if (!blk->isValid())
                break;
        }

        return blk;
    }

  

    /**
     * Insert the new block into the cache.
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
     void insertBlock(PacketPtr pkt, CacheBlk *blk) override
     {
         Addr addr = pkt->getAddr();
         MasterID master_id = pkt->req->masterId();
         uint32_t task_id = pkt->req->taskId();

         if (!blk->isTouched) {
             tagsInUse++;
             blk->isTouched = true;
             if (!warmedUp && tagsInUse.value() >= warmupBound) {
                 warmedUp = true;
                 warmupCycle = curTick();
             }
         }

         // If we're replacing a block that was previously valid update
         // stats for it. This can't be done in findBlock() because a
         // found block might not actually be replaced there if the
         // coherence protocol says it can't be.
         if (blk->isValid()) {
             replacements[0]++;
             totalRefs += blk->refCount;
             ++sampledRefs;
             blk->refCount = 0;

             // deal with evicted block
             assert(blk->srcMasterId < cache->system->maxMasters());
             occupancies[blk->srcMasterId]--;

             blk->invalidate();
         }

         blk->isTouched = true;

         // Set tag for new block.  Caller is responsible for setting status.
         blk->tag = extractTag(addr);

         // deal with what we are bringing in
         assert(master_id < cache->system->maxMasters());
         occupancies[master_id]++;
         blk->srcMasterId = master_id;
         blk->task_id = task_id;
         blk->tickInserted = curTick();
         if(pkt->req->hasContextId())
         blk->thread_context = pkt->req->contextId() ;  // store the context ID of the request in the block 
         else
         blk->thread_context = -1 ;  // if no context ID store -1 
         blk->is_interchanged = false ; 
         blk->owner_id = blk->thread_context ;

         // We only need to write into one tag and one data block.
         tagAccesses += 1;
         dataAccesses += 1;
     }

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Generate the tag from the given address.
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(Addr addr) const override
    {
        return (addr >> tagShift);
    }

    /**
     * Calculate the set index from the address.
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const override
    {
        return ((addr >> setShift) & setMask);
    }

    /**
     * Align an address to the block size.
     * @param addr the address to align.
     * @return The block address.
     */
    Addr blkAlign(Addr addr) const
    {
        return (addr & ~(Addr)blkMask);
    }

    /**
     * Regenerate the block address from the tag.
     * @param tag The tag of the block.
     * @param set The set of the block.
     * @return The block address.
     */
    Addr regenerateBlkAddr(Addr tag, unsigned set) const override
    {
        return ((tag << tagShift) | ((Addr)set << setShift));
    }

    /**
     * Called at end of simulation to complete average block reference stats.
     */
    void cleanupRefs() override;

    /**
     * Print all tags used
     */
    std::string print() const override;

    /**
     * Called prior to dumping stats to compute task occupancy
     */
    void computeStats() override;

    /**
     * Visit each block in the tag store and apply a visitor to the
     * block.
     *
     * The visitor should be a function (or object that behaves like a
     * function) that takes a cache block reference as its parameter
     * and returns a bool. A visitor can request the traversal to be
     * stopped by returning false, returning true causes it to be
     * called for the next block in the tag store.
     *
     * \param visitor Visitor to call on each block.
     */
    void forEachBlk(CacheBlkVisitor &visitor) override {
        for (unsigned i = 0; i < numSets * assoc; ++i) {
            if (!visitor(blks[i]))
                return;
        }
    }
};

#endif // __MEM_CACHE_TAGS_BASESETASSOC_HH__
