/*
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 */

#include "mem/ruby/structures/CacheMemory.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubyResourceStalls.hh"
#include "debug/RubyStats.hh"
#include "mem/dram_ctrl.hh"
#include "mem/pim.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/system/WeightedLRUPolicy.hh"

using namespace std;

ostream&
operator<<(ostream& out, const CacheMemory& obj)
{
    obj.print(out);
    out << flush;
    return out;
}

CacheMemory *
RubyCacheParams::create()
{
    return new CacheMemory(this);
}

CacheMemory::CacheMemory(const Params *p)
    : SimObject(p),
    dataArray(p->dataArrayBanks, p->dataAccessLatency,
              p->start_index_bit, p->ruby_system),
    tagArray(p->tagArrayBanks, p->tagAccessLatency,
             p->start_index_bit, p->ruby_system),
    m_ruby_system(p->ruby_system),
    scopeCache(p),
    noScopeBitvector(p->noScopeBitvector)
{
    m_cache_size = p->size;
    m_cache_assoc = p->assoc;
    m_replacementPolicy_ptr = p->replacement_policy;
    m_replacementPolicy_ptr->setCache(this);
    m_start_index_bit = p->start_index_bit;
    m_is_instruction_only_cache = p->is_icache;
    m_resource_stalls = p->resourceStalls;
    m_block_size = p->block_size;  // may be 0 at this point. Updated in init()
    traverse_idx = 0; // added by BenP
}

void
CacheMemory::init()
{
    if (m_block_size == 0) {
        m_block_size = RubySystem::getBlockSizeBytes();
    }
    m_cache_num_sets = (m_cache_size / m_cache_assoc) / m_block_size;
    assert(m_cache_num_sets > 1);
    m_cache_num_set_bits = floorLog2(m_cache_num_sets);
    assert(m_cache_num_set_bits > 0);

    m_cache.resize(m_cache_num_sets,
                    std::vector<AbstractCacheEntry*>(m_cache_assoc, nullptr));
    traverse_idx = 0; // added by BenP
    scopeCache.init();
    scopeBitvector.resize(m_cache_num_sets,false);
    m_PIM_mem_ctrl = m_ruby_system->m_PIM_mem_ctrl;
    if (m_PIM_mem_ctrl == NULL) {
        DPRINTF(RubyCache,"memory controll for PIM is NULL\n");
        if (m_ruby_system == NULL) {
            DPRINTF(RubyCache,"ruby system pointer is allso NULL\n");
        }
    } else {
        DPRINTF(RubyCache,"memory controll for PIM is non-NULL : 0x%x\n"
                            ,(uint64_t)m_PIM_mem_ctrl);
    }
}

CacheMemory::~CacheMemory()
{
    if (m_replacementPolicy_ptr)
        delete m_replacementPolicy_ptr;
    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            delete m_cache[i][j];
        }
    }
}

// convert a Address to its location in the cache
int64_t
CacheMemory::addressToCacheSet(Addr address) const
{
    assert(address == makeLineAddress(address));
    return bitSelect(address, m_start_index_bit,
                     m_start_index_bit + m_cache_num_set_bits - 1);
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
int
CacheMemory::findTagInSet(int64_t cacheSet, Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end())
        if (m_cache[cacheSet][it->second]->m_Permission !=
            AccessPermission_NotPresent)
            return it->second;
    return -1; // Not found
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
int
CacheMemory::findTagInSetIgnorePermissions(int64_t cacheSet,
                                           Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end())
        return it->second;
    return -1; // Not found
}

// Given an unique cache block identifier (idx): return the valid address
// stored by the cache block.  If the block is invalid/notpresent, the
// function returns the 0 address
Addr
CacheMemory::getAddressAtIdx(int idx) const
{
    Addr tmp(0);

    int set = idx / m_cache_assoc;
    assert(set < m_cache_num_sets);

    int way = idx - set * m_cache_assoc;
    assert (way < m_cache_assoc);

    AbstractCacheEntry* entry = m_cache[set][way];
    if (entry == NULL ||
        entry->m_Permission == AccessPermission_Invalid ||
        entry->m_Permission == AccessPermission_NotPresent) {
        return tmp;
    }
    return entry->m_Address;
}

bool
CacheMemory::tryCacheAccess(Addr address, RubyRequestType type,
                            DataBlock*& data_ptr)
{
    assert(address == makeLineAddress(address));
    DPRINTF(RubyCache, "address: %#x\n", address);
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc != -1) {
        // Do we even have a tag match?
        AbstractCacheEntry* entry = m_cache[cacheSet][loc];
        m_replacementPolicy_ptr->touch(cacheSet, loc, curTick());
        data_ptr = &(entry->getDataBlk());

        if (entry->m_Permission == AccessPermission_Read_Write) {
            return true;
        }
        if ((entry->m_Permission == AccessPermission_Read_Only) &&
            (type == RubyRequestType_LD || type == RubyRequestType_IFETCH)) {
            return true;
        }
        // The line must not be accessible
    }
    data_ptr = NULL;
    return false;
}

bool
CacheMemory::testCacheAccess(Addr address, RubyRequestType type,
                             DataBlock*& data_ptr)
{
    assert(address == makeLineAddress(address));
    DPRINTF(RubyCache, "address: %#x\n", address);
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);

    if (loc != -1) {
        // Do we even have a tag match?
        AbstractCacheEntry* entry = m_cache[cacheSet][loc];
        m_replacementPolicy_ptr->touch(cacheSet, loc, curTick());
        data_ptr = &(entry->getDataBlk());

        return m_cache[cacheSet][loc]->m_Permission !=
            AccessPermission_NotPresent;
    }

    data_ptr = NULL;
    return false;
}

// tests to see if an address is present in the cache
bool
CacheMemory::isTagPresent(Addr address) const
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);

    if (loc == -1) {
        // We didn't find the tag
        DPRINTF(RubyCache, "No tag match for address: %#x\n", address);
        return false;
    }
    DPRINTF(RubyCache, "address: %#x found\n", address);
    return true;
}

// Returns true if there is:
//   a) a tag match on this address or there is
//   b) an unused line in the same cache "way"
bool
CacheMemory::cacheAvail(Addr address) const
{
    assert(address == makeLineAddress(address));

    int64_t cacheSet = addressToCacheSet(address);

    for (int i = 0; i < m_cache_assoc; i++) {
        AbstractCacheEntry* entry = m_cache[cacheSet][i];
        if (entry != NULL) {
            if (entry->m_Address == address ||
                entry->m_Permission == AccessPermission_NotPresent) {
                // Already in the cache or we found an empty entry
                return true;
            }
        } else {
            return true;
        }
    }
    return false;
}

AbstractCacheEntry*
CacheMemory::allocate(Addr address, AbstractCacheEntry *entry, bool touch)
{
    assert(address == makeLineAddress(address));
    assert(!isTagPresent(address));
    assert(cacheAvail(address));
    DPRINTF(RubyCache, "address: %#x\n", address);

    // Find the first open slot
    int64_t cacheSet = addressToCacheSet(address);
    std::vector<AbstractCacheEntry*> &set = m_cache[cacheSet];
    for (int i = 0; i < m_cache_assoc; i++) {
        if (!set[i] || set[i]->m_Permission == AccessPermission_NotPresent) {
            if (set[i] && (set[i] != entry)) {
                warn_once("This protocol contains a cache entry handling bug: "
                    "Entries in the cache should never be NotPresent! If\n"
                    "this entry (%#x) is not tracked elsewhere, it will "
                    "memory leak here. Fix your protocol to eliminate these!",
                    address);
            }
            set[i] = entry;  // Init entry
            set[i]->m_Address = address;
            set[i]->m_Permission = AccessPermission_Invalid;
            DPRINTF(RubyCache, "Allocate clearing lock for addr: %x\n",
                    address);
            set[i]->m_locked = -1;
            m_tag_index[address] = i;
            entry->setSetIndex(cacheSet);
            entry->setWayIndex(i);

            if (touch) {
                m_replacementPolicy_ptr->touch(cacheSet, i, curTick());
            }

            //if (isAddrPIM(address)){
            //    scopeBitvector[cacheSet] = true;
            //}
            return entry;
        }
    }
    panic("Allocate didn't find an available entry");
}

void
CacheMemory::scopeBitvectorInsert(Addr address){
    if (isAddrPIM(address)){
        // Find the set
        int64_t cacheSet = addressToCacheSet(address);
        DPRINTF(RubyCache, "set scopeBitvector[%ld] for addr: %x\n",
                    cacheSet,address);
        scopeBitvector[cacheSet] = true;
    }
}

void
CacheMemory::scopeBitvectorRemove(Addr address){
    if (isAddrPIM(address)){
        // Find the set
        int64_t cacheSet = addressToCacheSet(address);
        DPRINTF(RubyCache, "Remove scopeBitvector[%ld] for addr: %x\n",
                    cacheSet,address);
        bool scopeBit = false;
        for (int way = 0 ; way < m_cache_assoc ; way++){
            if ((m_cache[cacheSet][way] != NULL) &&
                (m_cache[cacheSet][way]->m_Address != address) &&
                (m_cache[cacheSet][way]->m_Permission !=
                        AccessPermission_Invalid) &&
                (m_cache[cacheSet][way]->m_Permission !=
                        AccessPermission_NotPresent)){ // not null
                DPRINTF(RubyCache, "set contain addr: 0x%lX, isPIM: %d\n",
                    m_cache[cacheSet][way]->m_Address,
                    isAddrPIM(m_cache[cacheSet][way]->m_Address));
                scopeBit |= isAddrPIM(m_cache[cacheSet][way]->m_Address);
            }
        }
        scopeBitvector[cacheSet] = scopeBit;
        DPRINTF(RubyCache, "Set scopeBitvector[%d] = %d\n",
                        cacheSet,scopeBit);
    }
}


void
CacheMemory::deallocate(Addr address)
{
    assert(address == makeLineAddress(address));
    assert(isTagPresent(address));
    DPRINTF(RubyCache, "address: %#x\n", address);
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc != -1) {
        delete m_cache[cacheSet][loc];
        m_cache[cacheSet][loc] = NULL;
        m_tag_index.erase(address);
        // if we are removing a PIM address, then check if we need
        // to remove the set bit in the scopeBitvector
        bool scopeBit = false;
        if (isAddrPIM(address)){
            for (int way = 0 ; way < m_cache_assoc ; way++){
                if ((m_cache[cacheSet][way] != NULL) &&
                    (m_cache[cacheSet][way]->m_Permission !=
                            AccessPermission_Invalid) &&
                    (m_cache[cacheSet][way]->m_Permission !=
                            AccessPermission_NotPresent)){ // not null
                    DPRINTF(RubyCache, "set contain addr: 0x%lX, isPIM: %d\n",
                        m_cache[cacheSet][way]->m_Address,
                        isAddrPIM(m_cache[cacheSet][way]->m_Address));
                    scopeBit |= isAddrPIM(m_cache[cacheSet][way]->m_Address);
                }
            }
            scopeBitvector[cacheSet] = scopeBit;
            DPRINTF(RubyCache, "Set scopeBitvector[%d] = %d\n",
                            cacheSet,scopeBit);
        }
        //scopeBitvectorRemove(address);
    }
}

// Returns with the physical address of the conflicting cache line
Addr
CacheMemory::cacheProbe(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(!cacheAvail(address));

    int64_t cacheSet = addressToCacheSet(address);
    return m_cache[cacheSet][m_replacementPolicy_ptr->getVictim(cacheSet)]->
        m_Address;
}

// looks an address up in the cache
AbstractCacheEntry*
CacheMemory::lookup(Addr address)
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc == -1) return NULL;
    return m_cache[cacheSet][loc];
}

// looks an address up in the cache
const AbstractCacheEntry*
CacheMemory::lookup(Addr address) const
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc == -1) return NULL;
    return m_cache[cacheSet][loc];
}

// Sets the most recently used bit for a cache block
void
CacheMemory::setMRU(Addr address)
{
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);

    if (loc != -1)
        m_replacementPolicy_ptr->touch(cacheSet, loc, curTick());
}

void
CacheMemory::setMRU(const AbstractCacheEntry *e)
{
    uint32_t cacheSet = e->getSetIndex();
    uint32_t loc = e->getWayIndex();
    m_replacementPolicy_ptr->touch(cacheSet, loc, curTick());
}

void
CacheMemory::setMRU(Addr address, int occupancy)
{
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);

    if (loc != -1) {
        if (m_replacementPolicy_ptr->useOccupancy()) {
            (static_cast<WeightedLRUPolicy*>(m_replacementPolicy_ptr))->
                touch(cacheSet, loc, curTick(), occupancy);
        } else {
            m_replacementPolicy_ptr->
                touch(cacheSet, loc, curTick());
        }
    }
}

int
CacheMemory::getReplacementWeight(int64_t set, int64_t loc)
{
    assert(set < m_cache_num_sets);
    assert(loc < m_cache_assoc);
    int ret = 0;
    if (m_cache[set][loc] != NULL) {
        ret = m_cache[set][loc]->getNumValidBlocks();
        assert(ret >= 0);
    }

    return ret;
}

void
CacheMemory::recordCacheContents(int cntrl, CacheRecorder* tr) const
{
    uint64_t warmedUpBlocks = 0;
    uint64_t totalBlocks M5_VAR_USED = (uint64_t)m_cache_num_sets *
                                       (uint64_t)m_cache_assoc;

    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            if (m_cache[i][j] != NULL) {
                AccessPermission perm = m_cache[i][j]->m_Permission;
                RubyRequestType request_type = RubyRequestType_NULL;
                if (perm == AccessPermission_Read_Only) {
                    if (m_is_instruction_only_cache) {
                        request_type = RubyRequestType_IFETCH;
                    } else {
                        request_type = RubyRequestType_LD;
                    }
                } else if (perm == AccessPermission_Read_Write) {
                    request_type = RubyRequestType_ST;
                }

                if (request_type != RubyRequestType_NULL) {
                    tr->addRecord(cntrl, m_cache[i][j]->m_Address,
                                  0, request_type,
                                  m_replacementPolicy_ptr->getLastAccess(i, j),
                                  m_cache[i][j]->getDataBlk());
                    warmedUpBlocks++;
                }
            }
        }
    }

    DPRINTF(RubyCacheTrace, "%s: %lli blocks of %lli total blocks"
            "recorded %.2f%% \n", name().c_str(), warmedUpBlocks,
            totalBlocks, (float(warmedUpBlocks) / float(totalBlocks)) * 100.0);
}

void
CacheMemory::print(ostream& out) const
{
    out << "Cache dump: " << name() << endl;
    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            if (m_cache[i][j] != NULL) {
                out << "  Index: " << i
                    << " way: " << j
                    << " entry: " << *m_cache[i][j] << endl;
            } else {
                out << "  Index: " << i
                    << " way: " << j
                    << " entry: NULL" << endl;
            }
        }
    }
}

void
CacheMemory::printData(ostream& out) const
{
    out << "printData() not supported" << endl;
}

void
CacheMemory::setLocked(Addr address, int context)
{
    DPRINTF(RubyCache, "Setting Lock for addr: %#x to %d\n", address, context);
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    assert(loc != -1);
    m_cache[cacheSet][loc]->setLocked(context);
}

void
CacheMemory::clearLocked(Addr address)
{
    DPRINTF(RubyCache, "Clear Lock for addr: %#x\n", address);
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    assert(loc != -1);
    m_cache[cacheSet][loc]->clearLocked();
}

bool
CacheMemory::isLocked(Addr address, int context)
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    assert(loc != -1);
    DPRINTF(RubyCache, "Testing Lock for addr: %#llx cur %d con %d\n",
            address, m_cache[cacheSet][loc]->m_locked, context);
    return m_cache[cacheSet][loc]->isLocked(context);
}

void
CacheMemory::regStats()
{
    SimObject::regStats();
    scopeCache.regStats();

    m_demand_hits
        .name(name() + ".demand_hits")
        .desc("Number of cache demand hits")
        ;

    m_demand_misses
        .name(name() + ".demand_misses")
        .desc("Number of cache demand misses")
        ;

    m_demand_accesses
        .name(name() + ".demand_accesses")
        .desc("Number of cache demand accesses")
        ;

    m_demand_accesses = m_demand_hits + m_demand_misses;

    // added by BenP
    m_pim_ops
            .name(name() + ".pim_ops")
        .desc("Number of cache pim ops")
        ;
    // end added section

    m_sw_prefetches
        .name(name() + ".total_sw_prefetches")
        .desc("Number of software prefetches")
        .flags(Stats::nozero)
        ;

    m_hw_prefetches
        .name(name() + ".total_hw_prefetches")
        .desc("Number of hardware prefetches")
        .flags(Stats::nozero)
        ;

    m_prefetches
        .name(name() + ".total_prefetches")
        .desc("Number of prefetches")
        .flags(Stats::nozero)
        ;

    m_prefetches = m_sw_prefetches + m_hw_prefetches;

    m_accessModeType
        .init(RubyRequestType_NUM)
        .name(name() + ".access_mode")
        .flags(Stats::pdf | Stats::total)
        ;
    for (int i = 0; i < RubyAccessMode_NUM; i++) {
        m_accessModeType
            .subname(i, RubyAccessMode_to_string(RubyAccessMode(i)))
            .flags(Stats::nozero)
            ;
    }

    numDataArrayReads
        .name(name() + ".num_data_array_reads")
        .desc("number of data array reads")
        .flags(Stats::nozero)
        ;

    numDataArrayWrites
        .name(name() + ".num_data_array_writes")
        .desc("number of data array writes")
        .flags(Stats::nozero)
        ;

    numTagArrayReads
        .name(name() + ".num_tag_array_reads")
        .desc("number of tag array reads")
        .flags(Stats::nozero)
        ;

    numTagArrayWrites
        .name(name() + ".num_tag_array_writes")
        .desc("number of tag array writes")
        .flags(Stats::nozero)
        ;

    numTagArrayStalls
        .name(name() + ".num_tag_array_stalls")
        .desc("number of stalls caused by tag array")
        .flags(Stats::nozero)
        ;

    numDataArrayStalls
        .name(name() + ".num_data_array_stalls")
        .desc("number of stalls caused by data array")
        .flags(Stats::nozero)
        ;

    pimHoldCycles
        // this init number a magic number,
        // I don't know how large this should be
        .init(2000)
         .name(name() + ".pimHoldCycles")
         .desc("The cycles that a pim request stays at the cache.")
         .flags(Stats::nozero)
         ;
    pimNonZeroHoldCycles
        // this init number a magic number,
        // I don't know how large this should be
        .init(2000)
         .name(name() + ".pimNonZeroHoldCycles")
         .desc("The cycles that a pim request stays at "
                "the cache for non-zero values.")
         .flags(Stats::nozero)
         ;
    pimZeroHoldCycles
         .name(name() + ".pimZeroHoldCycles")
        .desc("number of pim requests with zero hold time.");

    pimScanCycles
        // this init number a magic number,
        // I don't know how large this should be
         .init(2000)
         .name(name() + ".pimScanCycles")
         .desc("The cycles that a pim request perform the scan.")
         .flags(Stats::nozero)
         ;
    pimNonZeroScanCycles
        // this init number a magic number,
        // I don't know how large this should be
         .init(2000)
         .name(name() + ".pimNonZeroScanCycles")
         .desc("The cycles that a pim request perform "
                "the scan for non zero values.")
         .flags(Stats::nozero)
         ;
    pimZeroScanCycles
        .name(name() + ".pimZeroScanCycles")
        .desc("number of pim requests with zero scan cycles.");

    pimScopeCacheCountOnArrival
         // we assume this is enough init.
         // Otherwise, the number should be raised.
         .init(m_cache_num_sets)
         .name(name() + ".pimScopeCacheCountOnArrival")
         .desc("The count encountered by pim requests.")
         .flags(Stats::nozero)
         ;
    pimNonZeroScopeCacheCountOnArrival
        // we assume this is enough init.
         // Otherwise, the number should be raised.
         .init(m_cache_num_sets)
         .name(name() + ".pimNonZeroScopeCacheCountOnArrival")
         .desc("The count encountered by pim requests for positive values.")
         .flags(Stats::nozero)
         ;
    pimZeroScopeCacheCountOnArrival
        .name(name() + ".pimZeroScopeCacheCountOnArrival")
        .desc("number of pim requests encountering zero at the scope cache.");
    pimNoneScopeCacheCountOnArrival
        .name(name() + ".pimNoneScopeCacheCountOnArrival")
        .desc("number of pim requests not present at the scope cache.");

    scopeBitvectorSkipSetRate
        // this init number a magic number,
        // I don't know how large this should be
         .init(0,1,200)
         .name(name() + ".scopeBitvectorSkipSetRate")
         .desc("The presentage of sets that were skipped "
                "suring a scan due to the scopeBitvector.")
         .flags(Stats::nozero)
         ;
    scanResourceStall
        // this init number a magic number,
        // I don't know how large this should be
         .init(2000)
         .name(name() + ".scanResourceStall")
         .desc("The cycles that a pim request scan is "
                "stalled due to resource contation.")
         .flags(Stats::nozero)
         ;
    scanResourceStallRate
        // this init number a magic number,
        // I don't know how large this should be
         .init(0,1,200)
         .name(name() + ".scanResourceStallRate")
         .desc("The cycles that a pim request scan is "
                "stalled due to resource contation divided"
                "by the scan cycles.")
         .flags(Stats::nozero)
         ;
    scanLineFlush
        // this init number a magic number,
        // I don't know how large this should be
         .init(2000)
         .name(name() + ".scanLineFlush")
         .desc("The number of cache flushed performed by a PIM op scan.")
         .flags(Stats::nozero)
         ;
}

void
CacheMemory::ScopeCache::regStats()
{
    SimObject::regStats();

    setInsertions
        // this init number a magic number,
        // I don't know how large this should be
         .init(m_num_sets)
         .name(name() + ".setInsertions")
         .desc("Number of inserts per set in the scopeCache.")
         ;
}
// assumption: SLICC generated files will only call this function
// once **all** resources are granted
void
CacheMemory::recordRequestType(CacheRequestType requestType, Addr addr)
{
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            CacheRequestType_to_string(requestType));
    switch(requestType) {
    case CacheRequestType_DataArrayRead:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        numDataArrayReads++;
        return;
    case CacheRequestType_DataArrayWrite:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        numDataArrayWrites++;
        return;
    case CacheRequestType_TagArrayRead:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        numTagArrayReads++;
        return;
    case CacheRequestType_TagArrayWrite:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        numTagArrayWrites++;
        return;
    default:
        warn("CacheMemory access_type not found: %s",
             CacheRequestType_to_string(requestType));
    }
}

bool
CacheMemory::checkResourceAvailable(CacheResourceType res, Addr addr)
{
    if (!m_resource_stalls) {
        return true;
    }

    if (res == CacheResourceType_TagArray) {
        if (tagArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Tag array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            numTagArrayStalls++;
            return false;
        }
    } else if (res == CacheResourceType_DataArray) {
        if (dataArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Data array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            numDataArrayStalls++;
            return false;
        }
    } else {
        panic("Unrecognized cache resource type.");
    }
}

bool
CacheMemory::isBlockInvalid(int64_t cache_set, int64_t loc)
{
  return (m_cache[cache_set][loc]->m_Permission == AccessPermission_Invalid);
}

bool
CacheMemory::isBlockNotBusy(int64_t cache_set, int64_t loc)
{
  return (m_cache[cache_set][loc]->m_Permission != AccessPermission_Busy);
}

// added by BenP
// implement cyclic traversing of the cache data, returning a valid entries
AbstractCacheEntry*
CacheMemory::getNextNoninvalidEntry()
{
    int start_idx = traverse_idx; // start where we previously stopped
    int set;
    int way;
    AbstractCacheEntry* entry;
    bool traverse = true;
    while (traverse) {
        // find set and way
        set = traverse_idx / m_cache_assoc;
        assert(set < m_cache_num_sets);
        way = traverse_idx - set * m_cache_assoc;
        assert (way < m_cache_assoc);
        // get entry
        entry = m_cache[set][way];
        // if the entry is not invalid then return the address
        if (entry == NULL ||
            entry->m_Permission == AccessPermission_Invalid ||
            entry->m_Permission == AccessPermission_NotPresent) {
            // increment the index for the next search
            traverse_idx = (traverse_idx+1) % (m_cache_size/m_block_size);
            // stop if we reached the index we started with
            traverse = (traverse_idx != start_idx);
        } else {
            // increment the index for the next search
            traverse_idx = (traverse_idx+1) % (m_cache_size/m_block_size);
            return entry;
        }
    }
    // if we reached here then we haven't found anything
    return NULL;
}


bool CacheMemory::isAddrPIM(Addr addr)
{
    if (m_PIM_mem_ctrl == NULL) {
        return false;
    } else {
        return m_PIM_mem_ctrl->isAddrPIM(addr);
    }
}


CacheMemory::ScopeCache::ScopeCache(const RubyCacheParams *p)
    : SimObject(p),
    Counter(p->Counter)
{
    m_num_sets = p->scopeCache_sets;
    m_assoc = p->scopeCache_assoc;
    // This is highly specific, should be carfule
    //if (p->size < 2<<20) { // L1
    //    m_num_sets = 16;
    //    m_assoc = 1;
    //} else { // L2
    //    m_num_sets = 64;
    //    m_assoc = 4;
    //}

    setMask = m_num_sets-1;
}

void
CacheMemory::ScopeCache::init(){
    m_scopeCache.resize(m_num_sets,
                    std::vector<scopeEntry>(m_assoc));
}

bool
CacheMemory::ScopeCache::lookup(Addr addr, Tick time, bool touch){
    Addr tag = getTag(addr);
    if (m_scope_tag_index.count(tag)){
        scopeSet cache_set = m_scopeCache[getSet(addr)];
        int way = m_scope_tag_index[tag];
        if (cache_set[way].valid && (cache_set[way].tag == tag)){
            if (touch)
                cache_set[way].time_stemp = time;
            DPRINTF(RubyCache, "ScopeCache: found page : 0x%lX (addr : 0x%lX) "
                    "in set : %d way: %d\n",tag,addr,getSet(addr),way);
            return true;
        }
        panic("Scope Cache lookup found tag in tag array"
                " not matching with entry.");
    }
    return false;
}

int
CacheMemory::ScopeCache::lookup_count(Addr addr, Tick time, bool touch){
    Addr tag = getTag(addr);
    if (m_scope_tag_index.count(tag)){
        scopeSet cache_set = m_scopeCache[getSet(addr)];
        int way = m_scope_tag_index[tag];
        if (cache_set[way].valid && (cache_set[way].tag == tag)){
            if (touch)
                cache_set[way].time_stemp = time;
            DPRINTF(RubyCache, "ScopeCache: found page : 0x%lX (addr : 0x%lX) "
                    "in set : %d way: %d (count : %d)\n",
                    tag,addr,getSet(addr),way,cache_set[way].count);
            assert(cache_set[way].count > -1);
            return cache_set[way].count;
        }
        panic("Scope Cache lookup found tag in tag array"
                " not matching with entry.");
    }
    //DPRINTF(RubyCache, "ScopeCache: count returning -1\n");
    return -1;
}

void
CacheMemory::ScopeCache::insert(Addr addr,Tick time){
    unsigned way;
    Addr tag = getTag(addr);
    Addr set = getSet(addr);
    bool found = m_scope_tag_index.count(tag);
    if (!found){
        // find the location
        way = getVictimLRU(set);
        // get the values of the existing entry
        Addr old_tag = m_scopeCache[set][way].tag;
        bool old_valid = m_scopeCache[set][way].valid;
        //erase old entry and insert the new one
        if (old_valid) {
            m_scope_tag_index.erase(old_tag);
        }
        m_scopeCache[set][way].valid = true;
        m_scopeCache[set][way].time_stemp = time;
        m_scopeCache[set][way].tag = tag;
        m_scopeCache[set][way].count = 0;
        m_scope_tag_index[tag] = way;
        DPRINTF(RubyCache, "ScopeCache: Inserted page: 0x%x (addr : 0x%x) at "
                            "time %ld , set : %d way : %d. "
                            "Replaced page - valid: %d, page:0x%x\n",
                            tag,addr,time,set,way,old_valid,old_tag);
        setInsertions[set]++;
    } else {
        // if the scope exist, and we are trying to insert it,
        // then we must have cleaned the cache from all scope blocks.
        // so we should have a zero count here.
        way = m_scope_tag_index[tag];
        assert(m_scopeCache[set][way].count == 0);
    }
}

void
CacheMemory::ScopeCache::incdec_count(Addr addr, bool inc){
    Addr tag = getTag(addr);
    unsigned exists = m_scope_tag_index.count(tag);//lookup(addr,0,false);
    //DPRINTF(RubyCache, "ScopeCache: incdec count tag : 0x%lX ,"
    //" found : %d\n",tag,exists);
    if (exists){
        Addr set = getSet(addr);
        unsigned way = m_scope_tag_index[tag];
        //for (unsigned way = 0 ; way < m_assoc ; way++) {
        if (m_scopeCache[set][way].valid && (m_scopeCache[set][way].tag == tag)){
            if (!Counter) {
                int erased = m_scope_tag_index.erase(tag);
                assert(erased);
                m_scopeCache[set][way].valid = false;
                m_scopeCache[set][way].tag = 0;
                m_scopeCache[set][way].count = -1;
            } else if (inc) {
                m_scopeCache[set][way].count += 1;
                DPRINTF(RubyCache, "ScopeCache: inc page : 0x%x "
                               "(addr : 0x%x), new count : %d ,"
                               " set : %d way : %d\n",
                    tag,addr,m_scopeCache[set][way].count,set,way);
            } else {
                assert(m_scopeCache[set][way].count > 0);
                m_scopeCache[set][way].count -= 1;
                DPRINTF(RubyCache, "ScopeCache: dec page : 0x%x "
                               "(addr : 0x%x), new count : %d ,"
                               " set : %d way : %d\n",
                    tag,addr,m_scopeCache[set][way].count,set,way);
            }
            return ;
        }
        //}
        panic("Scope Cache found tag in tag array but not in set.");
    }
}

// implement LRU
Tick
CacheMemory::ScopeCache::getVictimLRU(Addr set) const {
    scopeSet cache_set = m_scopeCache[set];
    Tick minTime = MaxTick;
    Tick minTime_way = 0;
    for (unsigned way = 0 ; way < m_assoc ; way++) {
        if (cache_set[way].valid){
            if (minTime >= cache_set[way].time_stemp){
                minTime = cache_set[way].time_stemp;
                minTime_way = way;
            }
        } else {
            return way;
        }
    }
    return minTime_way;
}

void
CacheMemory::scopeBitvector_scanReset(){
    sbv_word_idx = 0;
    sbv_wordbit_idx = 0;
    sbv_scan_way = 0;
    scopeBitvsctoreSkip = 0;
    scanSets_count = 0;
    scanResourceStall_count = 0;
    scanLineFlush_count = 0;
}

AbstractCacheEntry*
CacheMemory::scopeBitvector_scanGetEntry(Addr pim_addr){
    assert(sbv_wordbit_idx < SBV_WORD_LEN);

    AbstractCacheEntry* returned_entry = NULL;
    int way = sbv_scan_way;
    int started_bit = sbv_wordbit_idx;
    int bit_idx_it = sbv_wordbit_idx;
    unsigned set = sbv_word_idx*SBV_WORD_LEN + sbv_wordbit_idx;
    bool empty_marked_set = true;
    // if we are at the end of the bit vector then just return -1 (not found)
    if (!scopeBitvector_isScanDone()) {
        DPRINTF(RubyCache, "scopeBitVector scan not done\n");
        DPRINTF(RubyCache, "go over word %d, starting at bit %d way %d\n",
                            sbv_word_idx,bit_idx_it,way);
        // go over the bits of the current word
        for (;bit_idx_it < SBV_WORD_LEN; bit_idx_it++) {
            // save the current bit
            way = (bit_idx_it == started_bit) ? sbv_scan_way : 0;
            int started_way = way;
            sbv_wordbit_idx = bit_idx_it;
            set = sbv_word_idx*SBV_WORD_LEN + bit_idx_it;
            // if the bit is high then go over the ways of this set
            DPRINTF(RubyCache, "scopeBitvector[%d] = %d\n",
                                    set,scopeBitvector[set]);
            // count the set only at the first go
            if (way == 0)
                scanSets_count++;
            if (scopeBitvector[set] || noScopeBitvector){
                for (; way < m_cache_assoc; way++){
                    // go to the next way (or bit or word)
                    sbv_scan_way = way;
                    // if entry is valid, non-empty entry,
                    // and the address match than return it
                    AbstractCacheEntry* cur_entry = m_cache[set][way];
                    if ((cur_entry != NULL) &&
                        (cur_entry->m_Permission !=
                            AccessPermission_NotPresent) &&
                        (cur_entry->m_Permission !=
                            AccessPermission_Invalid)) {
                        empty_marked_set = false;
                        if (pim::addr2PIMaddr(cur_entry->m_Address) ==
                                    pim::addr2PIMaddr(pim_addr)){
                            returned_entry = cur_entry;
                            DPRINTF(RubyCache, "scopeBitVector scan found"
                                    " entry set %d way %d\n",set,way);
                            break;
                        } else {
                            DPRINTF(RubyCache, "scopeBitVector scan entry at"
                                " set %d way %d not right address: 0x%lX\n",
                                set,way,cur_entry->m_Address);
                        }
                    }
                }

                if ((!noScopeBitvector) && empty_marked_set &&
                    (started_way==0))
                    panic("started searching the set due to raised scopeBit,"
                           " but there is no valid block in set!\n");

                // if we didn't found anything then zero the way count
                //if (!returned_entry)
                    //way = 0;
                // if we reach here then break, we do only one set at a time
                break;
            } else if (way == 0) {
                // if this is the first time we try this set
                // and the bit is off then mark it as a Skip
                // due to the scopeBitvector
                scopeBitvsctoreSkip++;
            }
        }

        // advance the scan indexes
        // if we did not found anything
        if (!returned_entry) {
            // did we finished the word?
            if (sbv_wordbit_idx == SBV_WORD_LEN - 1){
                // go to the next word
                sbv_word_idx ++;
                sbv_wordbit_idx = 0;
            } else {
                // we didn't finished a word.
                // so we scanned a set without founding anything.
                // advance the wordbit
                sbv_wordbit_idx ++;
            }
            // in either option we start a new set
            sbv_scan_way = 0;
        }/* we move the case of a found entry to be handled by a
            different function since we want to prograss the scan
            even if there are no TBEs avilable, we will advance
            the scan when we evict the found block
        }*/
    }

    return returned_entry;
}

void
CacheMemory::scopeBitvector_advanceScan(){
    // we assume the scan found something
    // is it at the end of the set?
    if (sbv_scan_way == m_cache_assoc - 1){
        // go to the next word if this is the last bit
        // in the word, or all the rest of the bits are
        // zeros
        bool next_word = true;
        DPRINTF(RubyCache,
            "scanning the rest of the scopeBitvector word\n");
        for (int bit = sbv_wordbit_idx + 1 ;
                        bit < SBV_WORD_LEN ;
                        bit ++){
            DPRINTF(RubyCache, "scopeBitvector[%d] = %d\n",
                    sbv_word_idx*SBV_WORD_LEN + bit,
                    scopeBitvector[sbv_word_idx*SBV_WORD_LEN + bit]);
            if (scopeBitvector[sbv_word_idx*SBV_WORD_LEN + bit]){
                next_word = false;
                break;
            }
        }
        if (next_word) {
            sbv_word_idx ++;
            sbv_wordbit_idx = 0;
        } else {
            sbv_wordbit_idx++;
        }
        // since we are starting a new set
        sbv_scan_way = 0;
    } else {
        // we are not at the end of the set, continue to the next way
        sbv_scan_way++;
    }
}

bool
CacheMemory::scopeBitvector_isScanDone(){
    return (sbv_word_idx >= scopeBitvector.size()/SBV_WORD_LEN);
}

// This method is here to update the scopeBitvector when inserting a
// new PIM page. Since we mark an existing page as PIM, we don't
// know if there are alredy existing cache lines with blocks from that
// page, and since these blocks reached the cache while being non-PIM
// they are unmarked by the scopeBitvector. Here we mark them at the
// scope bitvectore. Since the page is not in the scopeCache, the first
// PIM operations will have to scan the scope bitvectore and will deal
// with these blocks.
// In a real system, the page is either marked as PIM at it's allocation,
// which make this whole scenario immposible, or marked as PIM during
// operation. The marking as PIM during operation (by the OS) will require
// the OS to flush the cache (WBINVD). This is like marking a page as
// UNCACHABLE at allocation or during operation. As marking pages as PIM
// is a hack in our simulation and is not measured for performance, this
// method should be performed in zero simultation time as it comes to bring
// the simulation to its intended state.
// This method should be called whanever a page is marked as PIM.
void
CacheMemory::scopeBitvector_updateOnPIMpage(Addr pim_addr){
    Addr pim_page_addr = pim::addr2PIMaddr(pim_addr);
    for (uint32_t set = 0 ; set < m_cache_num_sets ; set++){
        // look into the set only if it is not already marked
        if (!scopeBitvector[set]) {
            for (uint32_t way = 0 ; way < m_cache_assoc ; way++) {
                scopeBitvector[set] = scopeBitvector[set] ||
                         ((m_cache[set][way] != NULL) &&
                         (pim::addr2PIMaddr(m_cache[set][way]->m_Address)
                                                == pim_page_addr));
            }
        }
    }
}

AbstractCacheEntry*
CacheMemory::cacheAccess(unsigned set, unsigned way){
    return m_cache[set][way];
}
