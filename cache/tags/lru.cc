/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Definitions of a LRU tag store.
 */

#include "mem/cache/tags/lru.hh"

#include "debug/CacheRepl.hh"
#include "debug/Umon.hh"
#include "mem/cache/base.hh"
#include <vector>

#define VERBOSE 1
#define UmonInvalidContextID 4

LRU::LRU(const Params *p)
    : BaseSetAssoc(p)
{
}


/* UMON Functions  */
void
LRU::updateUmon(Addr addr, int tid){
    
    int set = extractSet(addr);
    bool found = false;
    int foundWay = 0;
    Addr tag = extractTag(addr);
    
    //searching for tag match
    for(unsigned j = 0; j < assoc; j ++){
        if(tag == sets[set].umon[tid][j].tag){
            found = true;
            foundWay = j;
            break;
        }    
    }
    
    //This function is only called after the block has been found in the cache
    if(!found){
        DPRINTF(Umon,"Update.Could not find block in umon structure. Set = %d tid = %d\n", set, tid);
        //fatal("Invalid cache montitoring state. Could note update Umon after a cache access");
    }
    
    
    //Update lru information
    for(unsigned i = 0; i < assoc; i ++){
        if(i == foundWay && sets[set].umon[tid][i].empty == false){
            sets[set].umon[tid][i].hitCounter++;
            sets[set].umon[tid][i].lru = 1;
        }
        else if (sets[set].umon[tid][i].empty == false){
            if(sets[set].umon[tid][i].lru > assoc - 1)
            {}
            else{
                sets[set].umon[tid][i].lru++;
            }
        }
    }
}


CacheBlk*
LRU::findLRUBlk(Addr addr, int tid){
    int set = extractSet(addr);
    Addr lruBlkTag = 0;
    CacheBlk * lruBlk = nullptr;
    int highestLRU = 0;

    DPRINTF(Umon,"highest lru %d \n", highestLRU);

    /* Finding the least recently used for a given set among all processes */
    for(unsigned i = 0; i < assoc; i++){
        DPRINTF(Umon,"way(%d) set(%d) tag(%d) lru(%d) empty(%d) \n", i, set, sets[set].umon[tid][i].tag, sets[set].umon[tid][i].lru, sets[set].umon[tid][i].empty);
        if(sets[set].umon[tid][i].lru > highestLRU && sets[set].umon[tid][i].empty == false ){
            lruBlkTag = sets[set].umon[tid][i].tag;
            highestLRU = sets[set].umon[tid][i].lru;
        }
    }
    
    DPRINTF(Umon,"midqy tag %d , lru %d\n", lruBlkTag, highestLRU);
    
    /* Finding corresponding Blk */
    for (unsigned i = 0; i < assoc; i++) {
        BlkType *b = sets[set].blks[i];
        DPRINTF(Umon,"way(%d) set(%d) blktag(%d)\n", i, set, b->tag );
        if (b->tag == lruBlkTag) {
            lruBlk = b;
            break;
        }
    }
    
    if(lruBlk == nullptr){
        DPRINTF(Umon,"findLRUBLK.Could not find block in set. Set = %d tid = %d\n", set, tid);
        //fatal("nope");
    }
    else{
        DPRINTF(Umon,"findLRUBLK.Found block in set. Set = %d tid = %d\n", set, tid);
        //fatal("yay");
    }
    
    return lruBlk;
}

CacheBlk*
LRU::findUmonVictim(Addr addr, int tid){
    if (tid == -1)
        return nullptr;
        //fatal("Trying to find victim for bad tid");
    
    
    int set = extractSet(addr);
    bool found = false;
    CacheBlk * victim = nullptr;
    
    /* Checking empty blocks */ 
    for (unsigned i = 0; i < assoc; i++) {
        BlkType *b = sets[set].blks[i];
        if (b->isTouched == false) {
            victim = b;
            found = true;
            DPRINTF(Umon,"Found empty block for victim \n");
            return victim;
        }
    }    
    
    /* Checking over utilized ways */
    bool foundOverUtilizedThread = false;
    int overUtilizedTread = 0;
    if(!found){
        foundOverUtilizedThread = false;
        overUtilizedTread = 0;
        for(unsigned i = 0; i < numCpus; i++){
            if(sets[set].umonPartitionInfo[i].allocatedWays < sets[set].umonPartitionInfo[i].waysInUse){
                foundOverUtilizedThread = true;
                overUtilizedTread = i;
                break;
            }
        }
        
        if(foundOverUtilizedThread){
            victim = findLRUBlk(addr, overUtilizedTread);
            
            if(victim != nullptr)
            {
                found = true;
                DPRINTF(Umon,"Victim.Found over utilized thread for victim. Tid= %d\n", overUtilizedTread);
            }
        }
    }
    
    /* Finding the corresponding cache block */
    if(!found){
        victim = findLRUBlk(addr, tid);
        
        if(victim != nullptr){
            found = true;
            DPRINTF(Umon,"Victim.Found victim from the asking tid. Set = %d tid = %d\n", set, tid);
        }
    }
    
    if(!found)
    {
        fatal("Could not find victim");
    }
    else{
        /* removing from Umon*/
        removeUmon(set, victim->tag);
    }
    
    return victim;
}

void
LRU::insertUmon(Addr addr, int tid){
    
    int set = extractSet(addr);
    bool added = false;
    Addr tag = extractTag(addr);
    
    //updating umon info
    sets[set].umonPartitionInfo[tid].waysInUse++;
    
    //adding to umon
    DPRINTF(Umon,"Insert. Inserting entry tid = %d, assoc = %d\n", tid, assoc);
    for (unsigned i = 0; i < assoc ; i++){
        if (sets[set].umon[tid][i].empty == true){
            sets[set].umon[tid][i].tag = tag;
            sets[set].umon[tid][i].tid = tid;
            sets[set].umon[tid][i].lru = 1; 
            sets[set].umon[tid][i].empty = false;
            added = true;
            break;
        }
    }
    if(!added){
        fatal("Could not add a Umon entry. Set = %d tid = %d\n", set, tid);
    }
}

void
LRU::removeUmon(int set, Addr tag){
    bool found = false;
    int victimTid = numCpus -1;
    
    //removing from umon
    for(unsigned j = 0; j < numCpus; j++){
        for (unsigned i = 0; i < assoc ; i++){
            DPRINTF(Umon,"current loop i=%d and j=%d\n", i,j);
            if (sets[set].umon[j][i].tag == tag){
                DPRINTF(Umon,"in loop\n");
                sets[set].umon[j][i].tag = 0;
                sets[set].umon[j][i].tid = 0;
                sets[set].umon[j][i].lru = assoc - 1; 
                sets[set].umon[j][i].empty = true;
                victimTid = j;
                found = true;
                break;
            }
        }
        if(found)
            break;
    }
    
    if(!found){
        fatal("Remove.Could not find Umon entry to remove. Set = %d\n", set);
    }
    
    //updating umon info
    sets[set].umonPartitionInfo[victimTid].waysInUse--;
}

void
LRU::repartitionUmonHitStatic(Addr addr){
    if(assoc != 8){
        fatal("Assuming 8 ways");
    }
    
    /*
    int set = extractSet(addr);
    std::vector<int> cpuPriority;
    int count = numCpus;
    int highestPriority = -1;
    while(count != 0){
        for(unsigned i = 0; i < numCpus; i++){
            bool inSet = false;
            for()
            
            if()
        }
        
        
        count--;
    }
    
    */
    /*Assigning priority based on highest hit counters. Distribution is static*/
    
    return;
}

void
LRU::repartitionUmonHitDyn(Addr addr){
    int set = extractSet(addr);
    /*Assigning priority based on highest hit counters. Distribution is dynamic*/
    
    return;
}

void
LRU::repartitionUmonTagStatic(Addr addr){
    int set = extractSet(addr);
    /*Assigning priority based on highest tag utilization. Distribution is dynamic*/
    
    return;
}

void
LRU::repartitionUmonTagDyn(Addr addr){
    int set = extractSet(addr);
    /*Assigning priority based on highest tag utilization. Distribution is dynamic*/
    
    return;
}


/* end */

CacheBlk*
LRU::accessBlock(Addr addr, bool is_secure, Cycles &lat, int master_id)
{   
    
    if(isL2 == false){
        CacheBlk *blk = BaseSetAssoc::accessBlock(addr, is_secure, lat, master_id);

        if (blk != nullptr) {
            // move this block to head of the MRU list
            sets[blk->set].moveToHead(blk); 
            DPRINTF(CacheRepl, "set %x: moving blk %x (%s) to MRU\n",
                    blk->set, regenerateBlkAddr(blk->tag, blk->set),
                    is_secure ? "s" : "ns");
        }

        return blk;
    }
    else{
        #if(VERBOSE)
            DPRINTF(Umon,"Beginning access with master id %d\n", master_id);
        #endif

        int set = extractSet(addr);        
        if(sets[set].umonAccesses >= sets[set].umonThreshold){
            DPRINTF(Umon, "Beginning Umon repartitioning for set %d",set);
            repartitionUmon(addr);
            DPRINTF(Umon, "End repartition");
        }
        
        CacheBlk *blk = BaseSetAssoc::accessBlock(addr, is_secure, lat, master_id);
        
        if (blk != nullptr) {
            updateUmon(addr, master_id);
            DPRINTF(CacheRepl, "set %x: moving blk %x (%s) to MRU\n",
                    blk->set, regenerateBlkAddr(blk->tag, blk->set),
                    is_secure ? "s" : "ns");
        }       
        
        #if(VERBOSE)
            DPRINTF(Umon,"Ending access\n");
        #endif
        return blk;
    }
    return NULL;
}

CacheBlk*
LRU::findVictim(Addr addr, int tid)
{
    int set = extractSet(addr);
    if( isL2 == false){
        set = extractSet(addr);
        // grab a replacement candidate
        BlkType *blk = nullptr;
        for (int i = assoc - 1; i >= 0; i--) {
            BlkType *b = sets[set].blks[i];
            if (b->way < allocAssoc) {
                blk = b;
                break;
            }
        }
        assert(!blk || blk->way < allocAssoc);

        if (blk && blk->isValid()) {
            DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n",
                    set, regenerateBlkAddr(blk->tag, set));
        }

        return blk;
    }
    else{
        #if(VERBOSE)
            DPRINTF(Umon,"Beginning find victim\n");
        #endif
        
        BlkType *blk = findUmonVictim(addr, tid);
        
        if (blk && blk->isValid()) {
            DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n",
                    set, regenerateBlkAddr(blk->tag, set));
        }
        
        #if(VERBOSE)
            DPRINTF(Umon,"Ending find victim\n");
        #endif
        return blk;
    }
    return NULL;
}

void
LRU::insertBlock(PacketPtr pkt, BlkType *blk)
{
    if(isL2 == false){
        BaseSetAssoc::insertBlock(pkt, blk);

        int set = extractSet(pkt->getAddr());
        sets[set].moveToHead(blk);
    }
    else{
        #if(VERBOSE)
            DPRINTF(Umon,"Beginning insert\n");
        #endif
        
        BaseSetAssoc::insertBlock(pkt, blk);
        
        /*ISSUE should be fixed adding the new 5th core*/
        if(pkt->req->hasContextId()){
            insertUmon(pkt->getAddr(), pkt->req->contextId());
        }
        else{
            insertUmon(pkt->getAddr(), UmonInvalidContextID);
        }
        
        #if(VERBOSE)
            DPRINTF(Umon,"Ending insert\n");
        #endif
    }
}

void
LRU::invalidate(CacheBlk *blk)
{
    if(isL2 == false){
        BaseSetAssoc::invalidate(blk);

        // should be evicted before valid blocks
        int set = blk->set;
        sets[set].moveToTail(blk);
    }
    else{
        #if(VERBOSE)
            DPRINTF(Umon,"Beginning invalidate\n");
        #endif
            
        BaseSetAssoc::invalidate(blk);
        removeUmon(blk->set ,blk->tag);      
        
        #if(VERBOSE)
            DPRINTF(Umon,"Ending invalidate\n");            
        #endif
    }
}

LRU*
LRUParams::create()
{
    return new LRU(this);
}
