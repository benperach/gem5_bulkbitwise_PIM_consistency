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

#ifndef __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__
#define __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__

#include <string>
#include <unordered_map>
#include <vector>

#include "base/statistics.hh"
#include "debug/RubyCacheWarnings.hh"
#include "mem/dram_ctrl.hh"
#include "mem/pim.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/protocol/CacheRequestType.hh"
#include "mem/ruby/protocol/CacheResourceType.hh"
#include "mem/ruby/protocol/RubyRequest.hh"
#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/structures/AbstractReplacementPolicy.hh"
#include "mem/ruby/structures/BankedArray.hh"
#include "mem/ruby/system/CacheRecorder.hh"
#include "params/RubyCache.hh"
#include "sim/sim_object.hh"

class CacheMemory : public SimObject
{
  public:
    typedef RubyCacheParams Params;
    CacheMemory(const Params *p);
    ~CacheMemory();

    void init();

    class ScopeCache : public SimObject
    {
        public:
            struct scopeEntry {
                bool valid;
                Tick time_stemp;
                Addr tag;
                int count;
                scopeEntry():
                    valid(false),
                    time_stemp(0),
                    tag(0),
                    count(0)
                {}
            };
            typedef std::vector<scopeEntry> scopeSet;
            ScopeCache(const RubyCacheParams *p);
            // ~ScopeCache(); use default distructor

            void init();

            // return a hit or miss
            bool lookup(Addr addr, Tick time, bool touch = true);
            // return a hit and count or miss
            int lookup_count(Addr addr, Tick time, bool touch = true);
            // insert to cache, possebly removing other lines
            void insert(Addr addr, Tick time);
            // increment/decrement count for a cached address if exist
            // (if not, do nothing)
            void incdec_count(Addr addr, bool inc);

            Stats::Vector setInsertions;
            void regStats();
        private:
            // The first index is the # of cache lines.
            // The second index is the the amount associativity.
            std::unordered_map<Addr, int> m_scope_tag_index;
            std::vector<scopeSet> m_scopeCache;

            Tick getVictimLRU(Addr set) const;

            unsigned m_num_sets;       /** total number of sets */
            unsigned m_assoc;          /** set associativity */
            Addr setMask;

            inline Addr getSet(Addr addr)
                        {return ((addr >> pim::LOG_PAGE_SIZE) & setMask);};
            inline Addr getTag(Addr addr)
                        {return (addr >> pim::LOG_PAGE_SIZE);};
                        
            bool Counter;

    };

    // Public Methods
    // perform a cache access and see if we hit or not.  Return true on a hit.
    bool tryCacheAccess(Addr address, RubyRequestType type,
                        DataBlock*& data_ptr);

    // similar to above, but doesn't require full access check
    bool testCacheAccess(Addr address, RubyRequestType type,
                         DataBlock*& data_ptr);

    // tests to see if an address is present in the cache
    bool isTagPresent(Addr address) const;

    // Returns true if there is:
    //   a) a tag match on this address or there is
    //   b) an unused line in the same cache "way"
    bool cacheAvail(Addr address) const;

    // find an unused entry and sets the tag appropriate for the address
    AbstractCacheEntry* allocate(Addr address,
                                 AbstractCacheEntry* new_entry, bool touch);
    AbstractCacheEntry* allocate(Addr address, AbstractCacheEntry* new_entry)
    {
        return allocate(address, new_entry, true);
    }
    void allocateVoid(Addr address, AbstractCacheEntry* new_entry)
    {
        allocate(address, new_entry, true);
    }

    void scopeBitvectorInsert(Addr address);
    void scopeBitvectorRemove(Addr address);

    // Explicitly free up this address
    void deallocate(Addr address);

    // Returns with the physical address of the conflicting cache line
    Addr cacheProbe(Addr address) const;

    // looks an address up in the cache
    AbstractCacheEntry* lookup(Addr address);
    AbstractCacheEntry* cacheAccess(unsigned set, unsigned way);
    const AbstractCacheEntry* lookup(Addr address) const;

    Cycles getTagLatency() const { return tagArray.getLatency(); }
    Cycles getDataLatency() const { return dataArray.getLatency(); }

    bool isBlockInvalid(int64_t cache_set, int64_t loc);
    bool isBlockNotBusy(int64_t cache_set, int64_t loc);

    // Hook for checkpointing the contents of the cache
    void recordCacheContents(int cntrl, CacheRecorder* tr) const;

    // Set this address to most recently used
    void setMRU(Addr address);
    void setMRU(Addr addr, int occupancy);
    int getReplacementWeight(int64_t set, int64_t loc);
    void setMRU(const AbstractCacheEntry *e);

    // Functions for locking and unlocking cache lines corresponding to the
    // provided address.  These are required for supporting atomic memory
    // accesses.  These are to be used when only the address of the cache entry
    // is available.  In case the entry itself is available. use the functions
    // provided by the AbstractCacheEntry class.
    void setLocked (Addr addr, int context);
    void clearLocked (Addr addr);
    bool isLocked (Addr addr, int context);

    // Print cache contents
    void print(std::ostream& out) const;
    void printData(std::ostream& out) const;

    void regStats();
    bool checkResourceAvailable(CacheResourceType res, Addr addr);
    void recordRequestType(CacheRequestType requestType, Addr addr);

    AbstractCacheEntry* getNextNoninvalidEntry();

    bool scopeCacheLookup(Addr addr, Tick time)
                    {return scopeCache.lookup(addr,time);};
    int scopeCacheLookupCount(Addr addr, Tick time, bool touch = true)
                    {return scopeCache.lookup_count(addr,time,touch);};
    void scopeCacheInsert(Addr addr, Tick time)
                    {scopeCache.insert(addr,time);};
    void scopeCacheIncrementCount(Addr addr)
                {scopeCache.incdec_count(addr,true);};
    void scopeCacheDecrementCount(Addr addr)
                {scopeCache.incdec_count(addr,false);};
    bool isAddrPIM(Addr addr);

    Addr addr2PIMpage(Addr addr) {return (addr & pim::PAGE_MASK);}

    AbstractCacheEntry* scopeBitvector_scanGetEntry(Addr pim_addr);
    void scopeBitvector_advanceScan();
    void scopeBitvector_scanReset();
    bool scopeBitvector_isScanDone();
    void scopeBitvector_updateOnPIMpage(Addr pim_addr);

  public:
    Stats::Scalar m_demand_hits;
    Stats::Scalar m_demand_misses;
    Stats::Formula m_demand_accesses;

    Stats::Scalar m_pim_ops; // added by BenP

    Stats::Scalar m_sw_prefetches;
    Stats::Scalar m_hw_prefetches;
    Stats::Formula m_prefetches;

    Stats::Vector m_accessModeType;

    Stats::Scalar numDataArrayReads;
    Stats::Scalar numDataArrayWrites;
    Stats::Scalar numTagArrayReads;
    Stats::Scalar numTagArrayWrites;

    Stats::Scalar numTagArrayStalls;
    Stats::Scalar numDataArrayStalls;

    Stats::Histogram pimHoldCycles;
    Stats::Histogram pimNonZeroHoldCycles;
    Stats::Scalar pimZeroHoldCycles;
    Stats::Histogram pimScanCycles;
    Stats::Histogram pimNonZeroScanCycles;
    Stats::Scalar pimZeroScanCycles;
    Stats::Histogram pimScopeCacheCountOnArrival;
    Stats::Histogram pimNonZeroScopeCacheCountOnArrival;
    Stats::Scalar pimZeroScopeCacheCountOnArrival;
    Stats::Scalar pimNoneScopeCacheCountOnArrival;

    Stats::Distribution scopeBitvectorSkipSetRate;
    Stats::Histogram scanResourceStall;
    Stats::Distribution scanResourceStallRate;
    Stats::Histogram scanLineFlush;
    
    inline void pimHoldCyclesSample(Cycles count, Addr addr){
        pimHoldCycles.sample(uint64_t(count));
        if (count == Cycles(0)){
            pimZeroHoldCycles++;
        } else {
            pimNonZeroHoldCycles.sample(uint64_t(count));
            if (scanSets_count > 0) {
                float stallRate =
                    ((float)scanResourceStall_count)/((float)scanSets_count);
                scanResourceStallRate.sample(stallRate);
            } else {
                scanResourceStallRate.sample(0);
            }

            scanResourceStall.sample(scanResourceStall_count);
            if (scanResourceStall_count > 2){
                DPRINTF(RubyCacheWarnings,"WARNING: large resource stall on addr 0x%x\n",addr);
            }
        }
        //assert(count < 10000);
        if (count > 10000){
            DPRINTF(RubyCacheWarnings,"WARNING: large hold time on pim address 0x%x\n",addr);
        }
    }

    //inline void pimScanCyclesSample(int count){
    //    pimScanCycles.sample(count);
    //    if (count == 0){
    //        pimZeroScanCycles++;
    //    } else {
    //        pimNonZeroScanCycles.sample(count);
    //    }
    //}

    inline void pimScopeCacheCountOnArrivalSample(int count){
        if (count > 0) {
            pimNonZeroScopeCacheCountOnArrival.sample(count);
            pimScopeCacheCountOnArrival.sample(count);
        } else if (count == 0) {
            pimZeroScopeCacheCountOnArrival++;
            pimScopeCacheCountOnArrival.sample(count);
        } else {
            pimNoneScopeCacheCountOnArrival++;
        }
    }

    inline void sampelScanStats(uint64_t Scan_cycles, Addr addr){
        pimScanCycles.sample(Scan_cycles);
        if (Scan_cycles > 0) {

            pimNonZeroScanCycles.sample(Scan_cycles);

            if (scanSets_count > 0) {
                float skipRate =
                    ((float)scopeBitvsctoreSkip)/((float)scanSets_count);
                scopeBitvectorSkipSetRate.sample(skipRate);
            } else {
                scopeBitvectorSkipSetRate.sample(0);
            }

            scanLineFlush.sample(scanLineFlush_count);
        } else {
            pimZeroScanCycles++;
        }
    }

    inline void scanResourceStall_inc(){
        DPRINTF(RubyCacheWarnings,"Resource stall during scan\n");
        scanResourceStall_count++;
    }
    inline void scanLineFlush_inc(){
        scanLineFlush_count++;
    }

    int getCacheSize() const { return m_cache_size; }
    int getCacheAssoc() const { return m_cache_assoc; }
    int getNumBlocks() const { return m_cache_num_sets * m_cache_assoc; }
    Addr getAddressAtIdx(int idx) const;

  private:
    // convert a Address to its location in the cache
    int64_t addressToCacheSet(Addr address) const;

    // Given a cache tag: returns the index of the tag in a set.
    // returns -1 if the tag is not found.
    int findTagInSet(int64_t line, Addr tag) const;
    int findTagInSetIgnorePermissions(int64_t cacheSet, Addr tag) const;

    // Private copy constructor and assignment operator
    CacheMemory(const CacheMemory& obj);
    CacheMemory& operator=(const CacheMemory& obj);

  private:
    // Data Members (m_prefix)
    bool m_is_instruction_only_cache;

    // The first index is the # of cache lines.
    // The second index is the the amount associativity.
    std::unordered_map<Addr, int> m_tag_index;
    std::vector<std::vector<AbstractCacheEntry*> > m_cache;

    AbstractReplacementPolicy *m_replacementPolicy_ptr;

    BankedArray dataArray;
    BankedArray tagArray;

    int m_cache_size;
    int m_cache_num_sets;
    int m_cache_num_set_bits;
    int m_cache_assoc;
    int m_start_index_bit;
    bool m_resource_stalls;
    int m_block_size;

    RubySystem *m_ruby_system; // added by BenP
    DRAMCtrl *m_PIM_mem_ctrl; // added by BenP
    int traverse_idx; // added by BenP
    ScopeCache scopeCache;

    std::vector<bool> scopeBitvector;
    const unsigned SBV_WORD_LEN = 32;
    unsigned sbv_word_idx;
    unsigned sbv_wordbit_idx;
    unsigned sbv_scan_way;
    uint64_t scopeBitvsctoreSkip;
    uint64_t scanSets_count;
    uint64_t scanResourceStall_count;
    uint64_t scanLineFlush_count;
    bool noScopeBitvector;
};

std::ostream& operator<<(std::ostream& out, const CacheMemory& obj);

#endif // __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__
