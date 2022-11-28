/*
 * Copyright (c) 2019 ARM Limited
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

#include "mem/ruby/system/Sequencer.hh"

#include "arch/x86/ldstflags.hh"
#include "base/logging.hh"
#include "base/str.hh"
#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/MemoryAccess.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubySequencer.hh"
#include "debug/RubyStats.hh"
#include "mem/pim.hh"
#include "mem/packet.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/protocol/PrefetchBit.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/system.hh"

using namespace std;

Sequencer *
RubySequencerParams::create()
{
    return new Sequencer(this);
}

Sequencer::Sequencer(const Params *p)
    : RubyPort(p), m_IncompleteTimes(MachineType_NUM),
      deadlockCheckEvent([this]{ wakeup(); }, "Sequencer deadlock check")
{
    m_outstanding_count = 0;

    m_instCache_ptr = p->icache;
    m_dataCache_ptr = p->dcache;
    m_max_outstanding_requests = p->max_outstanding_requests;
    m_deadlock_threshold = p->deadlock_threshold;

    holding_wbinvd = false;
    holding_pim_atomic = false;

    m_coreId = p->coreid; // for tracking the two CorePair sequencers
    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);
    assert(m_instCache_ptr != NULL);
    assert(m_dataCache_ptr != NULL);

    if (p->coherence_protocol == "MESI_Two_Level_pim_atomic"){
        PIM_coherence = PIM_coherence_type::ATOMIC;
    } else if (p->coherence_protocol == "MESI_Two_Level_pim_write"){
        PIM_coherence = PIM_coherence_type::WRITE;
    } else if (p->coherence_protocol == "MESI_Two_Level_pim_scope"){
        PIM_coherence = PIM_coherence_type::SCOPE;
    } else if (p->coherence_protocol == "MESI_Two_Level_pim_scope_relax"){
        PIM_coherence = PIM_coherence_type::SCOPE_RELAX;
    } else if (p->coherence_protocol == "MESI_Two_Level_pim_uc"){
        PIM_coherence = PIM_coherence_type::UC;
    } else {
        PIM_coherence = PIM_coherence_type::NONE;
    }
    DPRINTF(RubySequencer,
        "sequancer for core id %d recognized protocol %s, protocol ID %d\n",
        p->coreid,p->coherence_protocol,PIM_coherence);
    m_runningGarnetStandalone = p->garnet_standalone;
        dummy_datablock = new DataBlock();
}

Sequencer::~Sequencer()
{
}

void
Sequencer::wakeup()
{
    assert(drainState() != DrainState::Draining);

    // Check for deadlock of any of the requests
    Cycles current_time = curCycle();

    // Check across all outstanding requests
    int total_outstanding = 0;

    for (const auto &table_entry : m_RequestTable) {
        for (const auto seq_req : table_entry.second) {
            if (current_time - seq_req.issue_time < m_deadlock_threshold)
                continue;

            panic("Possible Deadlock detected. Aborting!\n version: %d "
                  "request.paddr: 0x%x m_readRequestTable: %d current time: "
                  "%u issue_time: %d difference: %d\n", m_version,
                  seq_req.pkt->getAddr(), table_entry.second.size(),
                  current_time * clockPeriod(), seq_req.issue_time
                  * clockPeriod(), (current_time * clockPeriod())
                  - (seq_req.issue_time * clockPeriod()));
        }
        total_outstanding += table_entry.second.size();
    }
    // also go over the additional structurs in for the different PIM coherence
    if (PIM_coherence == PIM_coherence_type::ATOMIC) {
        for (const auto &seq_req : pim_atomic_rqst) {
            if (current_time - seq_req.issue_time < m_deadlock_threshold)
                continue;

            panic("Possible Deadlock detected. Aborting!\n version: %d "
                  "AtomicPIM, request.paddr: 0x%x, current time: "
                  "%u issue_time: %d difference: %d\n", m_version,
                  seq_req.pkt->getAddr(),
                  current_time * clockPeriod(), seq_req.issue_time
                  * clockPeriod(), (current_time * clockPeriod())
                  - (seq_req.issue_time * clockPeriod()));
        }
        total_outstanding += pim_atomic_rqst.size();
    } else if ((PIM_coherence == PIM_coherence_type::WRITE) ||
              (PIM_coherence == PIM_coherence_type::SCOPE) ||
              (PIM_coherence == PIM_coherence_type::SCOPE_RELAX)) {
        for (const auto &table_entry : m_RequestPIMTable) {
            for (const auto seq_req : table_entry.second) {
                if (current_time - seq_req.issue_time < m_deadlock_threshold)
                    continue;

                panic("Possible Deadlock detected. Aborting!\n version: %d "
                      "request.paddr: 0x%x m_readRequestTable:"
                      " %d current time: "
                      "%u issue_time: %d difference: %d\n", m_version,
                      seq_req.pkt_copy.getAddr(), table_entry.second.size(),
                      current_time * clockPeriod(), seq_req.issue_time
                      * clockPeriod(), (current_time * clockPeriod())
                      - (seq_req.issue_time * clockPeriod()));
            }
            total_outstanding += table_entry.second.size();
        }
    }

    assert(m_outstanding_count == total_outstanding);

    if (m_outstanding_count > 0) {
        // If there are still outstanding requests, keep checking
        schedule(deadlockCheckEvent, clockEdge(m_deadlock_threshold));
    }
}

int
Sequencer::functionalWrite(Packet *func_pkt)
{
    int num_written = RubyPort::functionalWrite(func_pkt);

    for (const auto &table_entry : m_RequestTable) {
        for (const auto& seq_req : table_entry.second) {
            if (seq_req.functionalWrite(func_pkt))
                ++num_written;
        }
    }

    return num_written;
}


void Sequencer::resetStats()
{
    m_outstandReqHist.reset();
    m_latencyHist.reset();
    m_latencyPIMHist.reset();
    m_hitLatencyHist.reset();
    m_missLatencyHist.reset();
    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist[i]->reset();
        m_hitTypeLatencyHist[i]->reset();
        m_missTypeLatencyHist[i]->reset();
        for (int j = 0; j < MachineType_NUM; j++) {
            m_hitTypeMachLatencyHist[i][j]->reset();
            m_missTypeMachLatencyHist[i][j]->reset();
        }
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        m_missMachLatencyHist[i]->reset();
        m_hitMachLatencyHist[i]->reset();

        m_IssueToInitialDelayHist[i]->reset();
        m_InitialToForwardDelayHist[i]->reset();
        m_ForwardToFirstResponseDelayHist[i]->reset();
        m_FirstResponseToCompletionDelayHist[i]->reset();

        m_IncompleteTimes[i] = 0;
    }
}

// Insert the request in the request table. Return RequestStatus_Aliased
// if the entry was already present.
RequestStatus
Sequencer::insertRequest(PacketPtr pkt, RubyRequestType primary_type,
                         RubyRequestType secondary_type)
{

    bool PIM_alaiased = false;
    // See if we should schedule a deadlock check
    if (!deadlockCheckEvent.scheduled() &&
        drainState() != DrainState::Draining) {
        schedule(deadlockCheckEvent, clockEdge(m_deadlock_threshold));
    }

    Addr line_addr;
    // for uncachable bring only the required data
    if ((PIM_coherence == PIM_coherence_type::UC) && (isAddrPIM(pkt->getAddr()))) {
        line_addr = pkt->getAddr();
    } else {
        line_addr = makeLineAddress(pkt->getAddr());
    }

    // Check if there is any outstanding request for the same cache line.
    auto &seq_req_list = m_RequestTable[line_addr];
    // Create a default entry
    seq_req_list.emplace_back(pkt, primary_type, secondary_type, curCycle());
    m_outstanding_count++;
        DPRINTF(RubySequencer, "increase m_outstanding_count %d\n",
                        m_outstanding_count);

    if ((PIM_coherence == PIM_coherence_type::WRITE) ||
        (PIM_coherence == PIM_coherence_type::SCOPE) ||
        (PIM_coherence == PIM_coherence_type::SCOPE_RELAX)) {
        //check if there is a dependedncy on a PIM operation.
        // We search if the PIM request queue exist, since we don't want
        // to do this for non-PIM pages.
        Addr pim_addr = pim::addr2PIMaddr(pkt->getAddr());
        auto seq_PIMreq_list = m_RequestPIMTable.find(pim_addr);
        if (seq_PIMreq_list != m_RequestPIMTable.end()) {
            assert(!seq_PIMreq_list->second.empty());
            seq_req_list.back().dependedOnCount++;
            seq_PIMreq_list->second.back().dependedByVec.push_back(
                                &(seq_req_list.back()));
            PIM_alaiased = true;
        }
    }

    if ((seq_req_list.size() > 1) || PIM_alaiased) {
        return RequestStatus_Aliased;
    }

    m_outstandReqHist.sample(m_outstanding_count);

    return RequestStatus_Ready;
}

void
Sequencer::markRemoved()
{
    m_outstanding_count--;
        DPRINTF(RubySequencer, "decrease m_outstanding_count %d\n",
                        m_outstanding_count);
}

void
Sequencer::invalidateSC(Addr address)
{
    AbstractCacheEntry *e = m_dataCache_ptr->lookup(address);
    // The controller has lost the coherence permissions, hence the lock
    // on the cache line maintained by the cache should be cleared.
    if (e && e->isLocked(m_version)) {
        e->clearLocked();
    }
}

bool
Sequencer::handleLlsc(Addr address, SequencerRequest* request)
{
    AbstractCacheEntry *e = m_dataCache_ptr->lookup(address);
    if (!e)
        return true;

    // The success flag indicates whether the LLSC operation was successful.
    // LL ops will always succeed, but SC may fail if the cache line is no
    // longer locked.
    bool success = true;
    if (request->m_type == RubyRequestType_Store_Conditional) {
        if (!e->isLocked(m_version)) {
            //
            // For failed SC requests, indicate the failure to the cpu by
            // setting the extra data to zero.
            //
            request->pkt->req->setExtraData(0);
            success = false;
        } else {
            //
            // For successful SC requests, indicate the success to the cpu by
            // setting the extra data to one.
            //
            request->pkt->req->setExtraData(1);
        }
        //
        // Independent of success, all SC operations must clear the lock
        //
        e->clearLocked();
    } else if (request->m_type == RubyRequestType_Load_Linked) {
        //
        // Note: To fully follow Alpha LLSC semantics, should the LL clear any
        // previously locked cache lines?
        //
        e->setLocked(m_version);
    } else if (e->isLocked(m_version)) {
        //
        // Normal writes should clear the locked address
        //
        e->clearLocked();
    }
    return success;
}

void
Sequencer::recordMissLatency(SequencerRequest* srequest, bool llscSuccess,
                             const MachineType respondingMach,
                             bool isExternalHit, Cycles initialRequestTime,
                             Cycles forwardRequestTime,
                             Cycles firstResponseTime)
{
    RubyRequestType type = srequest->m_type;
    Cycles issued_time = srequest->issue_time;
    Cycles completion_time = curCycle();

    assert(curCycle() >= issued_time);
    Cycles total_lat = completion_time - issued_time;

    if (initialRequestTime < issued_time) {
        // if the request was combined in the protocol with an earlier request
        // for the same address, it is possible that it will return an
        // initialRequestTime corresponding the earlier request.  Since Cycles
        // is unsigned, we can't let this request get profiled below.

        total_lat = Cycles(0);
    }

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %d cycles\n",
             curTick(), m_version, "Seq", llscSuccess ? "Done" : "SC_Failed",
             "", "", printAddress(srequest->pkt->getAddr()), total_lat);

    m_latencyHist.sample(total_lat);
    m_typeLatencyHist[type]->sample(total_lat);

    if (isExternalHit) {
         m_missLatencyHist.sample(total_lat);
        m_missTypeLatencyHist[type]->sample(total_lat);

        if (respondingMach != MachineType_NUM) {
            m_missMachLatencyHist[respondingMach]->sample(total_lat);
            m_missTypeMachLatencyHist[type][respondingMach]->sample(total_lat);

            if ((issued_time <= initialRequestTime) &&
                (initialRequestTime <= forwardRequestTime) &&
                (forwardRequestTime <= firstResponseTime) &&
                (firstResponseTime <= completion_time)) {

                m_IssueToInitialDelayHist[respondingMach]->sample(
                    initialRequestTime - issued_time);
                m_InitialToForwardDelayHist[respondingMach]->sample(
                    forwardRequestTime - initialRequestTime);
                m_ForwardToFirstResponseDelayHist[respondingMach]->sample(
                    firstResponseTime - forwardRequestTime);
                m_FirstResponseToCompletionDelayHist[respondingMach]->sample(
                    completion_time - firstResponseTime);
            } else {
                m_IncompleteTimes[respondingMach]++;
            }
        }
    } else {
        m_hitLatencyHist.sample(total_lat);
        m_hitTypeLatencyHist[type]->sample(total_lat);

        if (respondingMach != MachineType_NUM) {
            m_hitMachLatencyHist[respondingMach]->sample(total_lat);
            m_hitTypeMachLatencyHist[type][respondingMach]->sample(total_lat);
        }
    }
}

void
Sequencer::writeCallback(Addr address, DataBlock& data,
                         const bool externalHit, const MachineType mach,
                         const Cycles initialRequestTime,
                         const Cycles forwardRequestTime,
                         const Cycles firstResponseTime,
                         bool uncacheable)
{
    DPRINTF(RubySequencer, "Rubyrequester write callback from addr 0x%X "
                            "m_outstanding_count %d\n",
                        address,m_outstanding_count);
        //
    // Free the whole list as we assume we have had the exclusive access
    // to this cache line when response for the write comes back
    //
    assert(uncacheable || (address == makeLineAddress(address)));
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];

    // Perform hitCallback on every cpu request made to this cache block while
    // ruby request was outstanding. Since only 1 ruby request was made,
    // profile the ruby latency once.
    bool ruby_request = true;
    int aliased_stores = 0;
    int aliased_loads = 0;
    while (!seq_req_list.empty()) {
        SequencerRequest &seq_req = seq_req_list.front();
        if (ruby_request) {
            assert(seq_req.m_type != RubyRequestType_LD);
            assert(seq_req.m_type != RubyRequestType_IFETCH);
        }

        if ((seq_req.m_type != RubyRequestType_LD) &&
                    (seq_req.m_type != RubyRequestType_IFETCH)) {

            // added by BenP: Stop going over accumulate
            // requests when encounter flush, as we do
            // want to flush the block handle write request
            if (seq_req.pkt->isFlush()){
                    issueRequest(seq_req.pkt,
                            seq_req.m_second_type);
                    break;
            }
            // added by BenP: Stop going over accumulate
            // requests when encounter a depended request
            if (seq_req.dependedOnCount > 0){
                // a sanity check, a regular request can
                // be depended only on one PIM request
                assert(seq_req.dependedOnCount == 1);
                break;
            }
            //
            // For Alpha, properly handle LL, SC, and
            // write requests with respect to locked
            // cache blocks.
            //
            // Not valid for Garnet_standalone protocl
            //
            bool success = true;
            if (!m_runningGarnetStandalone && !uncacheable)
                            success = handleLlsc(address,
                                &seq_req);

            // Handle SLICC block_on behavior for
            // Locked_RMW accesses. NOTE: the address
            // variable here is assumed to be a line
            // address, so when blocking buffers, must
            // check line addresses.
            if (seq_req.m_type ==
                            RubyRequestType_Locked_RMW_Read) {
                    // blockOnQueue blocks all first-level
                    // cache controller queues waiting on
                    // memory accesses for the specified
                    // address that go to the specified queue.
                    // In this case, a Locked_RMW_Write must
                    // go to the mandatory_q before unblocking
                    // the first-level controller. This will
                    // block standard loads, stores, ifetches,
                    // etc.
                    m_controller->blockOnQueue(address,
                                            m_mandatory_q_ptr);
            } else if (seq_req.m_type ==
                    RubyRequestType_Locked_RMW_Write) {
                    m_controller->unblock(address);
            }

            if (ruby_request) {
                    recordMissLatency(&seq_req, success,
                        mach, externalHit,
                        initialRequestTime,
                        forwardRequestTime,
                        firstResponseTime);
            } else {
                            aliased_stores++;
            }
            markRemoved();
            ruby_request = false;
            hitCallback(&seq_req, data, success,
                    mach, externalHit,
                    initialRequestTime,
                    forwardRequestTime,
                    firstResponseTime);
        } else {
            // handle read request
            assert(!ruby_request);
            markRemoved();
            ruby_request = false;
            aliased_loads++;
            hitCallback(&seq_req, data, true, mach, externalHit,
                        initialRequestTime, forwardRequestTime,
                        firstResponseTime);
        }
        // release dependedncies
        while (!seq_req.dependedByVec.empty()){
            SequencerRequestPIM* pim_req =
                    static_cast<SequencerRequestPIM *>
                    (seq_req.dependedByVec.back());
            pim_req->dependedOnCount--;
            DPRINTF(RubySequencer,
                        "Freeing PIM request dependedcy,"
                        " PIM addr 0x%X dependedOnCount %d\n",
                        pim_req->pkt->getAddr(),
                        pim_req->dependedOnCount);
            // if the depended PIM request has all it's dependencies
            // removed and it is in the front of it's queue,
            // then issue it
            if ((pim_req->dependedOnCount == 0) &&
                    (pim_req ==
                        &m_RequestPIMTable[
                        pim::addr2PIMaddr(pim_req->pkt_copy.getAddr())
                        ].front())){
                DPRINTF(RubySequencer, "Issuing PIM request\n");
                issueRequest(&(pim_req->pkt_copy),
                                pim_req->m_second_type);
            }
            seq_req.dependedByVec.pop_back();
        }
        seq_req_list.pop_front();
        // if this is uncacheable write request, then only
        // the requesting packet is allowed to participate.
        // this is because we don't support a bitvectore for
        // byte valid in the ruby cach, only offset and length,
        // so we can't support a non-concecutive data in the
        // block
        if (uncacheable){
            if (!seq_req_list.empty()){
                // issue the next request if it is possible with dependedncies
                SequencerRequest &next_seq_req = seq_req_list.front();
                if (next_seq_req.dependedOnCount == 0) {
                    DPRINTF(RubySequencer, "Issuing next request in sequencer\n");
                    issueRequest(next_seq_req.pkt,
                                    next_seq_req.m_second_type);
                }
            }
            break;
        }
    }

    m_aliasStores.sample(aliased_stores);
    m_aliasLoads_from_store.sample(aliased_loads);
     // free all outstanding requests corresponding to this address
    if (seq_req_list.empty()) {
        m_RequestTable.erase(address);
    }
    // if there are no more requests, and there is a wbinvd requests,
    // then we can issue the wbinvd
    if (m_RequestTable.empty() && holding_wbinvd){
        SequencerRequest &wbinvd_req = wbinvd_rqst.front();
        issueRequest(wbinvd_req.pkt, wbinvd_req.m_second_type);
    }
    // The same as WBINVD, just for PIM atomic operations
    if (m_RequestTable.empty() && holding_pim_atomic){
        SequencerRequest &pim_atomic_req = pim_atomic_rqst.front();
        issueRequest(pim_atomic_req.pkt, pim_atomic_req.m_second_type);
    }
}

void
Sequencer::readCallback(Addr address, DataBlock& data,
                        bool externalHit, bool uncacheable,
                        const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
    DPRINTF(RubySequencer, "Rubyrequester read callback from addr 0x%X "
                                "m_outstanding_count %d\n",
                        address,m_outstanding_count);
        //
    // Free up read requests until we hit the first Write request
    // or end of the corresponding list.
    //
    assert(uncacheable || (address == makeLineAddress(address)));
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];

    // Perform hitCallback on every cpu request made to this cache block while
    // ruby request was outstanding. Since only 1 ruby request was made,
    // profile the ruby latency once.
    bool ruby_request = true;
    int aliased_loads = 0;
    while (!seq_req_list.empty()) {
        SequencerRequest &seq_req = seq_req_list.front();
        if (ruby_request) {
            assert((seq_req.m_type == RubyRequestType_LD) ||
                   (seq_req.m_type == RubyRequestType_IFETCH));
        } else if (!uncacheable) {
            aliased_loads++;
        }
        if (((!ruby_request) && uncacheable) || 
            ((seq_req.m_type != RubyRequestType_LD) &&
            (seq_req.m_type != RubyRequestType_IFETCH))) {
            // Write request: reissue request to the cache hierarchy
            issueRequest(seq_req.pkt, seq_req.m_second_type);
            break;
        }
        if (ruby_request) {
            recordMissLatency(&seq_req, true, mach, externalHit,
                              initialRequestTime, forwardRequestTime,
                              firstResponseTime);
        }
        markRemoved();
        ruby_request = false;
        hitCallback(&seq_req, data, true, mach, externalHit,
                    initialRequestTime, forwardRequestTime,
                    firstResponseTime,uncacheable);
        // release dependedncies
        while (!seq_req.dependedByVec.empty()){
            SequencerRequestPIM* pim_req =
                        static_cast<SequencerRequestPIM *>
                            (seq_req.dependedByVec.back());
            pim_req->dependedOnCount--;
            DPRINTF(RubySequencer, "Freeing PIM request dependedcy,"
                            " PIM addr 0x%X dependedOnCount %d\n",
                        pim_req->pkt_copy.getAddr(),
                        pim_req->dependedOnCount);
            // if the depended PIM request has all it's
            // dependencies removed and it is in the front
            // of it's queue, then issue it
            if ((pim_req->dependedOnCount == 0) &&
                    (pim_req ==
                        &m_RequestPIMTable[
                        pim::addr2PIMaddr(pim_req->pkt_copy.getAddr())
                        ].front())){
                DPRINTF(RubySequencer, "Issuing PIM request\n");
                issueRequest(&(pim_req->pkt_copy),
                                pim_req->m_second_type);
            }
            seq_req.dependedByVec.pop_back();
        }
        seq_req_list.pop_front();
    }

    m_aliasLoads_from_load.sample(aliased_loads);
    // free all outstanding requests corresponding to this address
    if (seq_req_list.empty()) {
        m_RequestTable.erase(address);
    }
    // if there are no more requests, and there is a wbinvd requests,
    // then we can issue the wbinvd
    if (m_RequestTable.empty() && holding_wbinvd){
        SequencerRequest &wbinvd_req = wbinvd_rqst.front();
        issueRequest(wbinvd_req.pkt, wbinvd_req.m_second_type);
    }
    // The same as WBINVD, just for PIM atomic operations
    if (m_RequestTable.empty() && holding_pim_atomic){
        SequencerRequest &pim_atomic_req = pim_atomic_rqst.front();
        issueRequest(pim_atomic_req.pkt, pim_atomic_req.m_second_type);
    }
}

void
Sequencer::flushCallback(Addr address,
                        bool externalHit, const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
    DPRINTF(RubySequencer, "Rubyrequester flush callback from addr 0x%X"
                                " m_outstanding_count %d\n",
                    address, m_outstanding_count);
    //
    // Free up the flush request.
    // If there is a request afterwards then issue it.
    //
    assert(address == makeLineAddress(address));
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];

        bool ruby_request = true;
        while (!seq_req_list.empty()) {
            SequencerRequest &seq_req = seq_req_list.front();
            // if this is a flush request then clear it up
            if (seq_req.pkt->isFlush()){
                if (ruby_request) {
                        recordMissLatency(&seq_req, true, mach,
                            externalHit,
                            initialRequestTime,
                            forwardRequestTime,
                            firstResponseTime);
                }
                markRemoved();
                ruby_request = false;
                hitCallback(&seq_req, *dummy_datablock,
                                true, mach, externalHit,
                                initialRequestTime,
                                forwardRequestTime,
                                firstResponseTime);
                // release dependedncies
                while (!seq_req.dependedByVec.empty()){
                    SequencerRequestPIM* pim_req =
                        static_cast<SequencerRequestPIM *>
                        (seq_req.dependedByVec.back());
                    pim_req->dependedOnCount--;
                    DPRINTF(RubySequencer,
                                "Freeing PIM request dependedcy,"
                                " PIM addr 0x%X dependedOnCount"
                                " %d\n",
                                pim_req->pkt_copy.getAddr(),
                                pim_req->dependedOnCount);
                    // if the depended PIM request has all it's
                    // dependencies removed and it is in the front
                    // of it's queue, then issue it
                    if ((pim_req->dependedOnCount == 0) &&
                            (pim_req ==
                            &(m_RequestPIMTable[
                            pim::addr2PIMaddr(pim_req->pkt_copy.getAddr())
                            ].front()))){
                        DPRINTF(RubySequencer, "Issuing PIM request\n");
                        issueRequest(&(pim_req->pkt_copy),
                                        pim_req->m_second_type);
                    }
                    seq_req.dependedByVec.pop_back();
                }
                seq_req_list.pop_front();
            } else { // if there is a non flush request then issue it
                    issueRequest(seq_req.pkt, seq_req.m_second_type);
                    break;
            }
        }

        // free all outstanding requests corresponding to this address
    if (seq_req_list.empty()) {
        m_RequestTable.erase(address);
    }
    // if there are no more requests, and there is a wbinvd requests,
    // then we can issue the wbinvd
    if (m_RequestTable.empty() && holding_wbinvd){
        SequencerRequest &wbinvd_req = wbinvd_rqst.front();
        issueRequest(wbinvd_req.pkt, wbinvd_req.m_second_type);
    }
    // The same as WBINVD, just for PIM atomic operations
    if (m_RequestTable.empty() && holding_pim_atomic){
        SequencerRequest &pim_atomic_req = pim_atomic_rqst.front();
        issueRequest(pim_atomic_req.pkt, pim_atomic_req.m_second_type);
    }
}

void
Sequencer::wbinvdCallback(Addr address,
                        bool externalHit, const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
        DPRINTF(RubySequencer, "Rubyrequester wbinvd callback from addr 0x%X"
                                    " m_outstanding_count %d\n",
                        address, m_outstanding_count);
        //
    // Free up the wbinvd request.
    // If there is a request afterwards then issue it.
    //
    // the wbinvd request should be at the top here
    assert(!wbinvd_rqst.empty());
    SequencerRequest &seq_req = wbinvd_rqst.front();
    assert(seq_req.pkt->isWBINVD_BP());
    // clear it up
    holding_wbinvd = false;
    markRemoved();
    hitCallback(&seq_req, *dummy_datablock,
                    true, mach, externalHit,
                    initialRequestTime,
                    forwardRequestTime,
                    firstResponseTime);
    wbinvd_rqst.pop_front();
}

void
Sequencer::pimAtomicCallback(Addr address,
                        bool externalHit, const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
    DPRINTF(RubySequencer, "Rubyrequester PIM atomic callback from"
                               " addr 0x%X m_outstanding_count %d\n",
                        address, m_outstanding_count);
    assert(!pim_atomic_rqst.empty());
    SequencerRequest &seq_req = pim_atomic_rqst.front();
    assert(seq_req.pkt->isPIM());

    Cycles completion_time = curCycle();
    Cycles issued_time = seq_req.issue_time;
    Cycles total_lat = completion_time - issued_time;
    m_latencyPIMHist.sample(total_lat);

    recordMissLatency(&seq_req, true, mach,
                            externalHit,
                            initialRequestTime,
                            forwardRequestTime,
                            firstResponseTime);
    holding_pim_atomic = false;
    markRemoved();
    hitCallback(&seq_req, *dummy_datablock,
                    true, mach, externalHit,
                    initialRequestTime,
                    forwardRequestTime,
                    firstResponseTime);
    // clear it up
    pim_atomic_rqst.pop_front();
}

void
Sequencer::pimCallback(Addr address,
                        bool externalHit, const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
    DPRINTF(RubySequencer, "Rubyrequester PIM callback from"
                               " addr 0x%X m_outstanding_count %d\n",
                        address, m_outstanding_count);
    SequencerRequest* seq_req;
    if (PIM_coherence == PIM_coherence_type::ATOMIC) {
        assert(!pim_atomic_rqst.empty());
        seq_req = &pim_atomic_rqst.front();
        holding_pim_atomic = false;
        assert(seq_req->pkt->isPIM());
    } else {// if (PIM_coherence == PIM_coherence_type::WRITE) {
        auto &pim_seq_req_list =
                m_RequestPIMTable[pim::addr2PIMaddr(address)];
        assert(!pim_seq_req_list.empty());
        SequencerRequestPIM* seqPIM_req = &pim_seq_req_list.front();
        assert(seqPIM_req->pkt_copy.isPIM());
        seq_req = seqPIM_req;
    }

    Cycles completion_time = curCycle();
    Cycles issued_time = seq_req->issue_time;
    Cycles total_lat = completion_time - issued_time;
    m_latencyPIMHist.sample(total_lat);

    recordMissLatency(seq_req, true, mach,
                            externalHit,
                            initialRequestTime,
                            forwardRequestTime,
                            firstResponseTime);
    markRemoved();
    // perform the hitCallback (informing the core) for the
    // the stricter protocols, the rest of the protocols suppose
    // to notify the processor erlier.
    if ((PIM_coherence == PIM_coherence_type::ATOMIC) ||
        (PIM_coherence == PIM_coherence_type::WRITE)) {
            hitCallback(seq_req, *dummy_datablock,
                    true, mach, externalHit,
                    initialRequestTime,
                    forwardRequestTime,
                    firstResponseTime);
    }
    if (PIM_coherence == PIM_coherence_type::ATOMIC) {
        // clear it up
        pim_atomic_rqst.pop_front();
    } else if ((PIM_coherence == PIM_coherence_type::WRITE) ||
            (PIM_coherence == PIM_coherence_type::SCOPE) ||
            (PIM_coherence == PIM_coherence_type::SCOPE_RELAX)) {
        // release dependedncies
        while (!seq_req->dependedByVec.empty()){
            SequencerRequest* req = seq_req->dependedByVec.back();
            req->dependedOnCount--;
            DPRINTF(RubySequencer,
                    "Freeing regular request dependedcy, addr 0x%X "
                    "dependedOnCount %d\n",
                        req->pkt->getAddr(),req->dependedOnCount);
            // if the depended  request has all it's dependencies
            // removed and itis in the front of it's queue,
            // then issue it
            if ((req->dependedOnCount == 0) &&
                    (req->pkt ==
                        m_RequestTable[
                            makeLineAddress(req->pkt->getAddr())
                            ].front().pkt)){
                DPRINTF(RubySequencer, "Issuing request\n");
                issueRequest(req->pkt,
                                req->m_second_type);
            }
            seq_req->dependedByVec.pop_back();
        }
        // clear it up or send next PIM request
        auto &pim_seq_req_list =
                    m_RequestPIMTable[pim::addr2PIMaddr(address)];
        pim_seq_req_list.pop_front();
        if (pim_seq_req_list.empty()) {
            m_RequestPIMTable.erase(pim::addr2PIMaddr(address));
        } else {
            auto &pim_next_req = pim_seq_req_list.front();
            if (pim_next_req.dependedOnCount == 0){
                DPRINTF(RubySequencer, "Issuing next PIM request\n");
                issueRequest(&pim_next_req.pkt_copy,
                                pim_next_req.m_second_type);
            }
        }
        // wake up stalled operation in the core
        trySendRetries();
    }
}

void
Sequencer::getPIMAtomicData(DataBlock& data){
    SequencerRequest &seq_req = pim_atomic_rqst.front();
    PacketPtr pkt = seq_req.pkt;
    assert(pkt->isPIM());
    data.setData(pkt->getConstPtr<uint8_t>(),
                         0, pkt->getSize());
}

void
Sequencer::getPIMData(DataBlock& data, Addr address){
    if (PIM_coherence == PIM_coherence_type::ATOMIC) {
        SequencerRequest &seq_req = pim_atomic_rqst.front();
        PacketPtr pkt = seq_req.pkt;
        assert(pkt->isPIM());
        data.setData(pkt->getConstPtr<uint8_t>(),
                         0, pkt->getSize());
    } else {// if (PIM_coherence == PIM_coherence_type::WRITE) {
        auto &pim_seq_req_list =
                            m_RequestPIMTable[pim::addr2PIMaddr(address)];
        SequencerRequestPIM &seqPIM_req = pim_seq_req_list.front();
        //data.setData((uint8_t*)seqPIM_req.pkt_copy.getConstPtr<uint8_t>(),
        //            0, seqPIM_req.pkt_copy.getSize());
        data.setData((uint8_t*)seqPIM_req.data.getData(0,
                    seqPIM_req.pkt_copy.getSize()),
                    0, seqPIM_req.pkt_copy.getSize());
    }
}

void
Sequencer::getUncacheableData(Addr address, DataBlock& data){
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];
    SequencerRequest &seq_req = seq_req_list.front();
    PacketPtr pkt = seq_req.pkt;
    Addr request_address(pkt->getAddr());
    data.setData(pkt->getConstPtr<uint8_t>(),
                    getOffset(request_address), pkt->getSize());
}

void
Sequencer::hitCallback(SequencerRequest* srequest, DataBlock& data,
                       bool llscSuccess,
                       const MachineType mach, const bool externalHit,
                       const Cycles initialRequestTime,
                       const Cycles forwardRequestTime,
                       const Cycles firstResponseTime,
                       bool uncacheable)
{
    warn_once("Replacement policy updates recently became the responsibility "
              "of SLICC state machines. Make sure to setMRU() near callbacks "
              "in .sm files!");

    PacketPtr pkt = srequest->pkt;
    Addr request_address(pkt->getAddr());
    RubyRequestType type = srequest->m_type;

    // update the data unless it is a non-data-carrying flush
    if (RubySystem::getWarmupEnabled()) {
        data.setData(pkt->getConstPtr<uint8_t>(),
                     getOffset(request_address), pkt->getSize());
    } else if (!(   pkt->isFlush() ||
                    pkt->isWBINVD_BP())) {
        if ((type == RubyRequestType_LD) ||
            (type == RubyRequestType_IFETCH) ||
            (type == RubyRequestType_RMW_Read) ||
            (type == RubyRequestType_Locked_RMW_Read) ||
            (type == RubyRequestType_Load_Linked)) {
            if (uncacheable && isAddrPIM(pkt->getAddr())){
                // uncacheable reads bring only the required data
                pkt->setData(
                    data.getData(0, pkt->getSize()));
                DPRINTF(RubySequencer, "read uncachable data from addr 0x%X %s\n",
                        pkt->getAddr(),data);
            } else {
                pkt->setData(
                    data.getData(getOffset(request_address), pkt->getSize()));
                DPRINTF(RubySequencer, "read data from addr 0x%X %s\n",
                        pkt->getAddr(),data);
            }
            
        } else if (pkt->req->isSwap()) {
            std::vector<uint8_t> overwrite_val(pkt->getSize());
            pkt->writeData(&overwrite_val[0]);
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
            data.setData(&overwrite_val[0],
                         getOffset(request_address), pkt->getSize());
            DPRINTF(RubySequencer, "swap data %s\n", data);
        } else if (type != RubyRequestType_Store_Conditional || llscSuccess) {
            // Types of stores set the actual data here, apart from
            // failed Store Conditional requests

                        // changed section by BenP. We want to set the data for
                        // PIM requests in offset 0. It is actually meta data
                        // and not a data to write so we want it to be in a
                        // constant offset.
            data.setData(pkt->getConstPtr<uint8_t>(),
                         pkt->isPIM() ?
                         0 : getOffset(request_address), pkt->getSize());
                        // original code:
                        //data.setData(pkt->getConstPtr<uint8_t>(),
                         //getOffset(request_address), pkt->getSize());
            // end changed section

                        DPRINTF(RubySequencer, "set data from addr 0x%X %s\n",
                                    pkt->getAddr(),data);
        }
    }

    // If using the RubyTester, update the RubyTester sender state's
    // subBlock with the recieved data.  The tester will later access
    // this state.
    if (m_usingRubyTester) {
        DPRINTF(RubySequencer, "hitCallback %s 0x%x using RubyTester\n",
                pkt->cmdString(), pkt->getAddr());
        RubyTester::SenderState* testerSenderState =
            pkt->findNextSenderState<RubyTester::SenderState>();
        assert(testerSenderState);
        // for no data flushes don't touch the data
        if (!(  pkt->isFlush() ||
                pkt->isWBINVD_BP() ||
                pkt->isPIM()))
                testerSenderState->subBlock.mergeFrom(data);
    }

    RubySystem *rs = m_ruby_system;
    if (RubySystem::getWarmupEnabled()) {
        assert(pkt->req);
        delete pkt;
        rs->m_cache_recorder->enqueueNextFetchRequest();
    } else if (RubySystem::getCooldownEnabled()) {
        delete pkt;
        rs->m_cache_recorder->enqueueNextFlushRequest();
    } else {
        ruby_hit_callback(pkt);
        testDrainComplete();
    }
}

bool
Sequencer::empty() const
{
    return m_RequestTable.empty();
}

RequestStatus
Sequencer::makeRequest(PacketPtr pkt)
{
    // note that holding_pim_atomic can be True only with the
    // ATOMIC PIM coherency protocol
    if ((m_outstanding_count >= m_max_outstanding_requests) ||
            (holding_wbinvd) || (holding_pim_atomic)) {
        return RequestStatus_BufferFull;
    }

    RubyRequestType primary_type = RubyRequestType_NULL;
    RubyRequestType secondary_type = RubyRequestType_NULL;

    if (pkt->isLLSC()) {
        //
        // Alpha LL/SC instructions need to be handled carefully by the cache
        // coherence protocol to ensure they follow the proper semantics. In
        // particular, by identifying the operations as atomic, the protocol
        // should understand that migratory sharing optimizations should not
        // be performed (i.e. a load between the LL and SC should not steal
        // away exclusive permission).
        //
        if (pkt->isWrite()) {
            DPRINTF(RubySequencer, "Issuing SC\n");
            primary_type = RubyRequestType_Store_Conditional;
        } else {
            DPRINTF(RubySequencer, "Issuing LL\n");
            assert(pkt->isRead());
            primary_type = RubyRequestType_Load_Linked;
        }
        secondary_type = RubyRequestType_ATOMIC;
    } else if (pkt->req->isLockedRMW()) {
        //
        // x86 locked instructions are translated to store cache coherence
        // requests because these requests should always be treated as read
        // exclusive operations and should leverage any migratory sharing
        // optimization built into the protocol.
        //
        if (pkt->isWrite()) {
            DPRINTF(RubySequencer, "Issuing Locked RMW Write\n");
            primary_type = RubyRequestType_Locked_RMW_Write;
        } else {
            DPRINTF(RubySequencer, "Issuing Locked RMW Read\n");
            assert(pkt->isRead());
            primary_type = RubyRequestType_Locked_RMW_Read;
        }
        secondary_type = RubyRequestType_ST;
    } else {
        //
        // To support SwapReq, we need to check isWrite() first: a SwapReq
        // should always be treated like a write, but since a SwapReq implies
        // both isWrite() and isRead() are true, check isWrite() first here.
        //
        if (pkt->isWBINVD_BP()) {
            primary_type = secondary_type = RubyRequestType_WBINVD_BP;
            DPRINTF(RubySequencer, "Issuing WBINVD_BP\n");
            // we don't really need the list and emplace back, but I wanted
            // to be sure to have the same behaivuar
            wbinvd_rqst.emplace_back(pkt, primary_type,
                                secondary_type, curCycle());
            holding_wbinvd = true;
            m_outstanding_count++;
            DPRINTF(RubySequencer, "increase m_outstanding_count %d\n",
                            m_outstanding_count);
            m_outstandReqHist.sample(m_outstanding_count);
            // if there are no operation waiting then issue the wbinvd
            if (m_RequestTable.empty()){
                issueRequest(pkt, secondary_type);
            }
            return RequestStatus_Issued;
        } else if (pkt->isPIM()) {
            // Identifying PIM op for Ruby.
            // We change only the secondary_type, since it is
            // only used to construct the Ruby message in the
            // issueRequest function below.
            primary_type = RubyRequestType_ST;
            // for the scope relax coherency model, we identify
            // if this is a barrier
            if ((*(pkt->getConstPtr<uint8_t>()) & pim::OPCODE_MASK)
                        >> pim::OPCODE_BIT_START
                    == pim::pimOpcode::BARRIER)
                secondary_type = RubyRequestType_PIM_BARRIER;
            else
                secondary_type = RubyRequestType_PIM;
            DPRINTF(RubySequencer, "Is pim, coherence protocol : %d\n",
                        PIM_coherence);
            if (PIM_coherence == PIM_coherence_type::ATOMIC) {
                DPRINTF(RubySequencer, "Issuing atomic pim, address 0x%X\n",
                        pkt->getAddr());
                // we don't really need the list and emplace back, but I wanted
                // to be sure to have the same behaivuar
                pim_atomic_rqst.emplace_back(pkt, primary_type,
                                  secondary_type, curCycle());
                holding_pim_atomic = true;
                m_outstanding_count++;
                DPRINTF(RubySequencer, "increase m_outstanding_count %d\n",
                              m_outstanding_count);
                m_outstandReqHist.sample(m_outstanding_count);
                // if there are no operation waiting then issue the PIM request
                if (m_RequestTable.empty()){
                  issueRequest(pkt, secondary_type);
                }
                return RequestStatus_Issued;
            } else if ((PIM_coherence == PIM_coherence_type::WRITE) ||
                        (PIM_coherence == PIM_coherence_type::SCOPE) ||
                        (PIM_coherence == PIM_coherence_type::SCOPE_RELAX)){
                Addr pim_addr = pim::addr2PIMaddr(pkt->getAddr());
                auto &pim_seq_req = m_RequestPIMTable[pim_addr];
                pim_seq_req.emplace_back(pkt, primary_type,
                                  secondary_type, curCycle());
                DPRINTF(RubySequencer,
                        "Issuing write/scope-consistency pim, address 0x%X"
                        ", place in queue %d\n",
                        pkt->getAddr(),pim_seq_req.size());
                m_outstanding_count++;
                DPRINTF(RubySequencer, "increase m_outstanding_count %d\n",
                              m_outstanding_count);
                // find dependedncies.
                // Since empty lists are erased, there can be at
                // most total_outstanding elements in m_RequestTable
                // (typically 16)
                for (auto &seq_req : m_RequestTable) {
                    if (pim::addr2PIMaddr(seq_req.first) == pim_addr) {
                        // increase the dependedncy count for the
                        // currnt request
                        auto &dpend_req = seq_req.second.back();
                        pim_seq_req.back().dependedOnCount++;
                        // add the dependedncy
                        dpend_req.dependedByVec.push_back(
                                         &(pim_seq_req.back()));
                        DPRINTF(RubySequencer,
                            "add dependedncy on %s to addr 0x%x\n",
                            dpend_req.pkt->cmdString(),
                            dpend_req.pkt->getAddr());
                    }
                }
                // Issue the PIM request only if it is the only
                // one at it's queue and there is no dependedncies.
                // In general, this is not enough for the WRITE
                // protocol, since the protocol require order between
                // all addresses, not just on the same scope. We relay
                // that the core releases the PIM requests at commit,
                // so it is the oldest memory access when it reaches
                // here and block all other write/PIM requests (which
                // is what we want in the WRITE protocol). Hence, we
                // don't enforce here the order with other scopes.
                if ((pim_seq_req.size() == 1) &&
                        (pim_seq_req.back().dependedOnCount == 0))
                    issueRequest(pkt, secondary_type);

                return RequestStatus_Issued;
            } else if ((PIM_coherence != PIM_coherence_type::NONE) &&
                        (PIM_coherence != PIM_coherence_type::UC)){
                panic("encountered unsoppreted coherence protocol at the"
                        " Ruby sequencer");
            }
        } else if (pkt->isWrite()) {
            //
            // Note: M5 packets do not differentiate ST from RMW_Write
            //
            primary_type = secondary_type = RubyRequestType_ST;
        } else if (pkt->isRead()) {
            if (pkt->req->isInstFetch()) {
                primary_type = secondary_type = RubyRequestType_IFETCH;
            } else {
                bool storeCheck = false;
                // only X86 need the store check
                if (system->getArch() == Arch::X86ISA) {
                    uint32_t flags = pkt->req->getFlags();
                    storeCheck = flags &
                        (X86ISA::StoreCheck << X86ISA::FlagShift);
                }
                if (storeCheck) {
                    primary_type = RubyRequestType_RMW_Read;
                    secondary_type = RubyRequestType_ST;
                } else {
                    primary_type = secondary_type = RubyRequestType_LD;
                }
            }
        } else if (pkt->isFlush()) {
          primary_type = secondary_type = RubyRequestType_FLUSH;
        } else {
            panic("Unsupported ruby packet type\n");
        }
    }

    // Check if the line is blocked for a Locked_RMW
    if (m_controller->isBlocked(makeLineAddress(pkt->getAddr())) &&
        (primary_type != RubyRequestType_Locked_RMW_Write)) {
        // Return that this request's cache line address aliases with
        // a prior request that locked the cache line. The request cannot
        // proceed until the cache line is unlocked by a Locked_RMW_Write
        return RequestStatus_Aliased;
    }
    RequestStatus status = insertRequest(pkt, primary_type, secondary_type);

    // It is OK to receive RequestStatus_Aliased, it can be considered Issued
    if (status != RequestStatus_Ready && status != RequestStatus_Aliased)
        return status;
    // non-aliased with any existing request in the request table, just issue
    // to the cache
    if (status != RequestStatus_Aliased)
        issueRequest(pkt, secondary_type);

    // TODO: issue hardware prefetches here
    return RequestStatus_Issued;
}

void
Sequencer::scopeProtocolHitCallback(PacketPtr pkt){
    if (((PIM_coherence == PIM_coherence_type::SCOPE) ||
        (PIM_coherence == PIM_coherence_type::SCOPE_RELAX))
        && (pkt->isPIM())) {
        // If we are here then this packet was the last to be entered to the
        // queue, then get it's request pointer from the queue
        Addr pim_addr = pim::addr2PIMaddr(pkt->getAddr());
        auto &pim_seq_req = m_RequestPIMTable[pim_addr];
        hitCallback(&(pim_seq_req.back()), *dummy_datablock,
            true, MachineType_NUM, false,Cycles(0),Cycles(0),Cycles(0));
            // we don't use the last arguments for hitCallback since
            // they are not neccecery and are not used
    }
}

void
Sequencer::issueRequest(PacketPtr pkt, RubyRequestType secondary_type)
{
    assert(pkt != NULL);
    ContextID proc_id = pkt->req->hasContextId() ?
        pkt->req->contextId() : InvalidContextID;

    ContextID core_id = coreId();

    // If valid, copy the pc to the ruby request
    Addr pc = 0;
    if (pkt->req->hasPC()) {
        pc = pkt->req->getPC();
    }

    // check if the packet has data as for example prefetch and flush
    // requests do not
    std::shared_ptr<RubyRequest> msg =
        std::make_shared<RubyRequest>(clockEdge(), pkt->getAddr(),
                                      (pkt->isFlush() | pkt->isWBINVD_BP())?
                                      nullptr : pkt->getPtr<uint8_t>(),
                                      pkt->getSize(), pc, secondary_type,
                                      RubyAccessMode_Supervisor, pkt,
                                      PrefetchBit_No, proc_id, core_id);

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %#x %s\n",
            curTick(), m_version, "Seq", "Begin", "", "",
            printAddress(msg->getPhysicalAddress()),
            RubyRequestType_to_string(secondary_type));

    Tick latency = cyclesToTicks(
                        m_controller->mandatoryQueueLatency(secondary_type));
    assert(latency > 0);

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
}

template <class KEY, class VALUE>
std::ostream &
operator<<(ostream &out, const std::unordered_map<KEY, VALUE> &map)
{
    for (const auto &table_entry : map) {
        out << "[ " << table_entry.first << " =";
        for (const auto &seq_req : table_entry.second) {
            out << " " << RubyRequestType_to_string(seq_req.m_second_type);
        }
    }
    out << " ]";

    return out;
}

void
Sequencer::print(ostream& out) const
{
    out << "[Sequencer: " << m_version
        << ", outstanding requests: " << m_outstanding_count
        << ", request table: " << m_RequestTable
        << "]";
}

// this can be called from setState whenever coherence permissions are
// upgraded when invoked, coherence violations will be checked for the
// given block
void
Sequencer::checkCoherence(Addr addr)
{
}

void
Sequencer::recordRequestType(SequencerRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            SequencerRequestType_to_string(requestType));
}

void
Sequencer::evictionCallback(Addr address)
{
    ruby_eviction_callback(address);
}

void
Sequencer::regStats()
{
    RubyPort::regStats();

    // These statistical variables are not for display.
    // The profiler will collate these across different
    // sequencers and display those collated statistics.
    m_outstandReqHist.init(10);
    m_latencyHist.init(10);

    m_latencyPIMHist.init(100)
        .name(name() + ".latencyPIMHist")
        .desc("the latency a PIM request encounter.");
    
    m_aliasLoads_from_load.init(32)
        .name(name() + ".m_aliasLoads_from_load")
        .desc("histogram of aliased loads from load.");
    m_aliasLoads_from_store.init(32)
        .name(name() + ".m_aliasLoads_from_store")
        .desc("histogram of aliased loads stores.");
    m_aliasStores.init(32)
        .name(name() + ".m_aliasStores")
        .desc("histogram of aliased stores.");

    m_hitLatencyHist.init(10);
    m_missLatencyHist.init(10);

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist.push_back(new Stats::Histogram());
        m_typeLatencyHist[i]->init(10);

        m_hitTypeLatencyHist.push_back(new Stats::Histogram());
        m_hitTypeLatencyHist[i]->init(10);

        m_missTypeLatencyHist.push_back(new Stats::Histogram());
        m_missTypeLatencyHist[i]->init(10);
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        m_hitMachLatencyHist.push_back(new Stats::Histogram());
        m_hitMachLatencyHist[i]->init(10);

        m_missMachLatencyHist.push_back(new Stats::Histogram());
        m_missMachLatencyHist[i]->init(10);

        m_IssueToInitialDelayHist.push_back(new Stats::Histogram());
        m_IssueToInitialDelayHist[i]->init(10);

        m_InitialToForwardDelayHist.push_back(new Stats::Histogram());
        m_InitialToForwardDelayHist[i]->init(10);

        m_ForwardToFirstResponseDelayHist.push_back(new Stats::Histogram());
        m_ForwardToFirstResponseDelayHist[i]->init(10);

        m_FirstResponseToCompletionDelayHist.push_back(new Stats::Histogram());
        m_FirstResponseToCompletionDelayHist[i]->init(10);
    }

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_hitTypeMachLatencyHist.push_back(std::vector<Stats::Histogram *>());
        m_missTypeMachLatencyHist.push_back(std::vector<Stats::Histogram *>());

        for (int j = 0; j < MachineType_NUM; j++) {
            m_hitTypeMachLatencyHist[i].push_back(new Stats::Histogram());
            m_hitTypeMachLatencyHist[i][j]->init(10);

            m_missTypeMachLatencyHist[i].push_back(new Stats::Histogram());
            m_missTypeMachLatencyHist[i][j]->init(10);
        }
    }
}
