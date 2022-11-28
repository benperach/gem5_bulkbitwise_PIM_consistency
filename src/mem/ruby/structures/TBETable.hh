/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_STRUCTURES_TBETABLE_HH__
#define __MEM_RUBY_STRUCTURES_TBETABLE_HH__

#include <iostream>
#include <iterator>     // std::next
#include <unordered_map>

#include "debug/RubyCache.hh"
#include "mem/pim.hh"
#include "mem/ruby/common/Address.hh"

template<class ENTRY>
class TBETable
{
  public:
    TBETable(int number_of_TBEs)
        : pim_entrys(0),
          m_number_of_TBEs(number_of_TBEs)
    {
    }

    bool isPresent(Addr address) const;
    void allocate(Addr address);
    void deallocate(Addr address);
    void allocatePIM();
    void deallocatePIM();
    bool
    areNSlotsAvailable(int n, Tick current_time) const
    {
        return (m_number_of_TBEs - (m_map.size() + pim_entrys) >= n);
    }

    ENTRY *lookup(Addr address);
    ENTRY *lookup_next();
    // Print cache contents
    void print(std::ostream& out) const;

    inline Addr realAddr2TBEpim(Addr addr) {
        //adding the MSb for distinguishing
        return (pim::addr2PIMaddr(addr) | (1UL << 63));
    }

    //inline Addr TBEpim2realAddr(Addr addr){
    //    return addr & ((1UL << 63) - 1);
    //}

    int size() {return (m_map.size() + pim_entrys);}

    void reset_TBE_scan();
    ENTRY* scan_get_next(Addr pim_addr);

  private:
    // Private copy constructor and assignment operator
    TBETable(const TBETable& obj);
    TBETable& operator=(const TBETable& obj);

    // Data Members (m_prefix)
    std::unordered_map<Addr, ENTRY> m_map;
    
    // The PIM entries here are just to compansate
    // for the infinite buffer size at between the
    // Ruby system and the memory controller.
    // By allocating tbe entries for PIM requests,
    // and releasing them when receiving an ack,
    // we mimic a finit capacity between the L2 and
    // the memory controller.
    // The pim_entrys counter is for the basic design,
    // where no order is maintain, so we only keep trak
    // of how many on-the-fly operations there are,
    // we don't want to associate a request with an address
    // so there won't be a mix of addresses and requests
    // will be held because they are to the same address.
    // This is basically a token base protocol.
    // For the designs with some order control (i.e.,
    // consistency model) PIM requests are assigned a
    // TBE, so there is no need for the pim_entry counter.
    unsigned pim_entrys;

    Addr addr_pntr = 0;

    typename std::unordered_map<Addr, ENTRY>::iterator scan_it;
    bool scan_it_inc;

  private:
    int m_number_of_TBEs;
};

template<class ENTRY>
inline std::ostream&
operator<<(std::ostream& out, const TBETable<ENTRY>& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

template<class ENTRY>
inline bool
TBETable<ENTRY>::isPresent(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(m_map.size() + pim_entrys <= m_number_of_TBEs);
    return !!m_map.count(address);
}

template<class ENTRY>
inline void
TBETable<ENTRY>::allocate(Addr address)
{
    assert(!isPresent(address));
    DPRINTF(RubyCache, "TBE: alloc reg TBE. reg (new) : %d, PIM : %d\n",(m_map.size()+1),pim_entrys);
    assert(m_map.size() + pim_entrys < m_number_of_TBEs);
    m_map[address] = ENTRY();
}

template<class ENTRY>
inline void
TBETable<ENTRY>::allocatePIM()
{
    DPRINTF(RubyCache, "TBE: alloc PIM TBE. reg : %d, PIM (new) : %d\n",m_map.size(),(pim_entrys+1));
    assert(m_map.size() + pim_entrys < m_number_of_TBEs);
    pim_entrys++;
}

template<class ENTRY>
inline void
TBETable<ENTRY>::deallocatePIM()
{
    DPRINTF(RubyCache, "TBE: remove PIM TBE. reg : %d, PIM (new) : %d\n",m_map.size(),(pim_entrys-1));
    assert(pim_entrys > 0);
    pim_entrys--;
}

template<class ENTRY>
inline void
TBETable<ENTRY>::deallocate(Addr address)
{
    assert(isPresent(address));
    DPRINTF(RubyCache, "TBE: remove reg TBE. reg (new) : %d, PIM : %d\n",(m_map.size()-1),pim_entrys);
    assert(m_map.size() > 0);
    // if the deallocate is the current scanned TBE
    // then adjust the TBE
    bool set_scan_it_begin = false;
    if (scan_it == m_map.find(address)) {
        // if this is the begining then we can't just
        // decrement the iterator, we need to set it as the
        // new begin after the erasure
        if (scan_it == m_map.begin()) {
            set_scan_it_begin = true;
        } else {
            // scan_it is a forward iterator, so we need
            // to find the previous iterator manually
            auto it = m_map.begin();
            for( ; it != m_map.end() ; it++){
                if (std::next(it,1) == scan_it)
                    break;
            }
            assert(it != m_map.end());
            scan_it = it;
        }
    }
    
    m_map.erase(address);
    
    if (set_scan_it_begin){
        scan_it = m_map.begin();
    }
}

// looks an address up in the cache
template<class ENTRY>
inline ENTRY*
TBETable<ENTRY>::lookup(Addr address)
{
  if (m_map.find(address) != m_map.end())
    return &(m_map.find(address)->second);
  return NULL;
}

// looks the next entry in the TBE table.
// It is assumed that TBE contain only non-empty entries.
template<class ENTRY>
inline ENTRY*
TBETable<ENTRY>::lookup_next()
{
    if (m_map.empty())
        return NULL;
    // check if the last pointed address is still in the TBE.
    // if not, point to the begining
    auto it = m_map.find(addr_pntr);
    //DPRINTF(RubyCache, "TBE: find pointer for addr: %#x\n", addr_pntr);
    if (it == m_map.end()){
        it = m_map.begin();
        DPRINTF(RubyCache, "TBE: no addr, go to begin, recived addr: %#x\n",
                                it->first);
        // the TBE is not empty, so 'it' should not be end
        assert(it != m_map.end());
    }
    // here 'it' points to a valid entry
    // if the next entry is the end then go to begin (which is not the end)
    if (std::next(it,1) != m_map.end())
        addr_pntr = std::next(it,1)->first;
    else
        addr_pntr = m_map.begin()->first;
    return &(it->second); // return the entry
}

template<class ENTRY>
void
TBETable<ENTRY>::reset_TBE_scan()
{
    scan_it = m_map.begin();
    scan_it_inc = false;
}

template<class ENTRY>
inline ENTRY*
TBETable<ENTRY>::scan_get_next(Addr pim_addr)
{
    if (scan_it_inc){
        // we increment only on the start to make sure
        // the end() iterator hasn't changed
        scan_it++;
        scan_it_inc = false;
    }
    while (scan_it != m_map.end()) {
        // check if this is an address to the PIM page
        // and not a PIM operations itself
        if ((realAddr2TBEpim(scan_it->first) == pim_addr) &&
            (scan_it->first != pim_addr)){
            scan_it_inc = true;
            return &(scan_it->second);
        }
        scan_it++;
    }
    return NULL;
}


template<class ENTRY>
inline void
TBETable<ENTRY>::print(std::ostream& out) const
{
}

#endif // __MEM_RUBY_STRUCTURES_TBETABLE_HH__
