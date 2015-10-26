/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Adrian Sai-wah Tam
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Adrian Sai-wah Tam <adrian.sw.tam@gmail.com>
 */

#define NS_LOG_APPEND_CONTEXT \
  if (m_node) { std::clog << Simulator::Now ().GetSeconds () << " [node " << m_node->GetId () << "] "; }

#include "tcp-cr.h"
#include "ns3/log.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/trace-helper.h"
#include "ns3/string.h"

// For statistic calcluation
#include  "ns3/ap.h"
#include  "ns3/dataanalysis.h"
#include  "ns3/alglibinternal.h"
#include  "ns3/specialfunctions.h"
#include  "ns3/alglibmisc.h"
#include  "ns3/integration.h"
#include  "ns3/linalg.h"
#include  "ns3/optimization.h"
#include  "ns3/solvers.h"
#include  "ns3/statistics.h"

/*
#include  "ap.cpp"
#include  "dataanalysis.cpp"
#include  "alglibinternal.cpp"
#include  "specialfunctions.cpp"
#include  "alglibmisc.cpp"
#include  "integration.cpp"
#include  "linalg.cpp"
#include  "optimization.cpp"
#include  "solvers.cpp"
#include  "statistics.cpp"
*/

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpCR");

NS_OBJECT_ENSURE_REGISTERED(TcpCR);

const int STABLE = 0;
const int UNSTABLE = 1;
const int COMPETEING = 2;

TypeId TcpCR::GetTypeId(void)
{
    static TypeId tid =
            TypeId("ns3::TcpCR").SetParent<TcpSocketBase>()
            .SetGroupName("Internet")
            .AddConstructor<TcpCR>()
            .AddAttribute("ReTxThreshold",
                    "Threshold for fast retransmit",
                    UintegerValue(3),
                    MakeUintegerAccessor(&TcpCR::m_retxThresh),
                    MakeUintegerChecker<uint32_t>())
            .AddAttribute("LimitedTransmit","Enable limited transmit",
                    BooleanValue(false),
                    MakeBooleanAccessor(&TcpCR::m_limitedTx),
                    MakeBooleanChecker())
             .AddAttribute("Folder", "Use this to choose foler to put all results",
                     StringValue("./results/TcpCR/"),
                     MakeStringAccessor(&TcpCR::folder),
                     MakeStringChecker());
    return tid;
}

TcpCR::TcpCR(void) :
        m_retxThresh(3), // mute valgrind, actual value set by the attribute system
        m_inFastRec(false),
        m_limitedTx(false), // mute valgrind, actual value set by the attribute system
        m_ackSeqStartNum(0),
        m_ackSeqEndNum(0),
        m_ackSeqStartTime(0.0),
        m_ackSeqEndTime(0.0),
        m_ackSeqCount(0),
        m_ackSeqLength(1),
        m_ebw(0),
        m_currentTp(0),
        //m_lastTp(0),
        m_firstMinRtt(Time(0)),
        m_minRtt(0.0),
        m_firstSlowStart(true),
        m_currentRTT(0.0),
        m_startCollecting(Time(0)),
        m_nFlows(0),
        m_fast_probing(0),
        //m_currentState(UNSTABLE),
        //m_starveCount(-1),
        m_lastSlowStart(Time(0)),
        m_fastConverge(false),
        m_power(0),
        m_stage(STABLE),
        m_comThresh(0),
        m_sigma(1e-6),
        m_var(0),
        //m_last_var(-1e-6),
        //m_lastVar(0),
        m_src_port(0),
        m_dst_port(0)
{
    NS_LOG_FUNCTION("Dude, I am here");
}

TcpCR::TcpCR(const TcpCR& sock) :
        TcpSocketBase(sock), m_retxThresh(sock.m_retxThresh), m_inFastRec(false), m_limitedTx(
                sock.m_limitedTx)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC("Invoked the copy constructor");
}

TcpCR::~TcpCR(void)
{
}

Ptr<TcpSocketBase> TcpCR::Fork(void)
{
    return CopyObject<TcpCR>(this);
}

int TcpCR::EstimateFlowNumber(double ratio)
{
    int flowNumber;
    for (int i = 1; i <= 4; i++)
    {
        flowNumber = i;
        double lowerBound =  ((int) (100 * (2*i + 1.0)/(2.0*i*(i+1))))/100.0;
        double ratioRound = ceil(ratio * 100)/100.0;
        if (ratioRound >= lowerBound)
        {
            break;
        }
    }
/*    if (ratio > 0.75)
        flowNumber = 1;
    else if (ratio > 0.4)
        flowNumber =  2;
    else if (ratio > 0.28)
        flowNumber =  3;
    else if (ratio > 0.22)
        flowNumber =  4;
    else if (ratio > 0.18)
        flowNumber =  5;
    else if (ratio > 0.15)
        flowNumber =  6;
    else if (ratio > 0.13)
        flowNumber =  7;
    else
        flowNumber = 8;*/


    return flowNumber;
}

//std::cout<< Simulator::Now().GetSeconds() << " "<< m_dst_port << " m_ssThresh: " << m_ssThresh

/*        if (m_dst_port == 4 && (Simulator::Now().GetSeconds() > 210) && (Simulator::Now().GetSeconds() < 230))
 {
 std::cout<< Simulator::Now().GetSeconds() << " CongAvoid "<< m_dst_port << " ssthresh: " << m_ssThresh/1000 << " cwnd: " << m_cWnd/1000 << std::endl;
 }*/

/* New ACK (up to seqnum seq) received. Increase cwnd and call TcpSocketBase::NewAck() */
void TcpCR::NewAck(const SequenceNumber32& seq)
{
    NS_LOG_FUNCTION(this << seq);
    NS_LOG_LOGIC(
            "TcpCR received ACK for seq " << seq << " cwnd " << m_cWnd << " ssthresh " << m_ssThresh);
    // Check for exit condition of fast recovery
    if (m_inFastRec && seq < m_recover)
    { // Partial ACK, partial window deflation (RFC2582 sec.3 bullet #5 paragraph 3)
        m_cWnd += m_segmentSize - (seq - m_txBuffer->HeadSequence());
        NS_LOG_INFO("Partial ACK for seq " << seq << " in fast recovery: cwnd set to " << m_cWnd);
        m_txBuffer->DiscardUpTo(seq);  //Bug 1850:  retransmit before newack
        DoRetransmit(); // Assume the next seq is lost. Retransmit lost packet
        TcpSocketBase::NewAck(seq); // update m_nextTxSequence and send new data if allowed by window
        return;
    }
    else if (m_inFastRec && seq >= m_recover)
    { // Full ACK (RFC2582 sec.3 bullet #5 paragraph 2, option 1)
        m_cWnd = std::min(m_ssThresh.Get(), BytesInFlight() + m_segmentSize);
        m_inFastRec = false;
        NS_LOG_INFO(
                "Received full ACK for seq " << seq <<". Leaving fast recovery with cwnd set to " << m_cWnd);
    }
    // Increase of cwnd based on current phase (slow start or congestion avoidance)
    if (m_cWnd < m_ssThresh)
    {
        double fEBW = 0;
        double BDP = 0;
        if (m_fast_probing == 0)
        {
            // Check whether do we enter into fast probing mode
            // In fast probing mode, the goal is to quickly decide the bandwidth and reset the ssthresh according to flow number
            m_ackSeqCount += 1;
            if ((m_ackSeqStartNum == 0) & (m_cWnd.Get() / m_segmentSize > 1))
            {
                m_ackSeqStartTime = Simulator::Now().GetSeconds();
                m_ackSeqStartNum = m_ackSeqCount;
                // Cannot arbitrarily set m_ackSeqLength to be 1, because the initial cwnd might be other value
                m_ackSeqLength = m_cWnd.Get() / m_segmentSize;
            }
            else if (m_ackSeqCount == m_ackSeqStartNum + m_ackSeqLength - 1)
            {
                m_ackSeqEndNum = m_ackSeqCount;
                m_ackSeqEndTime = Simulator::Now().GetSeconds();
                double last = m_ackSeqEndTime - m_ackSeqStartTime;
                fEBW = ((1054 * (m_ackSeqLength - 1)) / last); // in Bps
                m_ebw = fEBW;
                // Calculate BDP using minimum RTT * EBW
                BDP = m_minRtt.GetSeconds() * fEBW;  // in Byte
                if (m_firstMinRtt == 0)
                    m_firstMinRtt = m_minRtt;
                // Here we need the information senet by the receiver
                if (m_nFlows == 0)
                {
                    m_nFlows = 1;
/*                    if (Simulator::Now().GetSeconds() < 25)
                    {
                        m_nFlows = 1;
                    }
                    else if (Simulator::Now().GetSeconds() < 45)
                    {
                        m_nFlows = 2;
                    }
                    else if (Simulator::Now().GetSeconds() < 65)
                    {
                        m_nFlows = 3;
                    }
                    else if (Simulator::Now().GetSeconds() < 125)
                    {
                        m_nFlows = 4;
                    }
                    else if (Simulator::Now().GetSeconds() < 145)
                    {
                        m_nFlows = 3;
                    }
                    else if (Simulator::Now().GetSeconds() < 165)
                    {
                        m_nFlows = 2;
                    }
                    else
                    {
                        m_nFlows = 1;
                    }*/
                }
                m_ssThresh = BDP / m_nFlows;   // in byte
                // First slow start, and there are other flows, in this case we do two tunes
                // First we do not "trust" the rtt setting
                // Second we slightly increase the initial ssthresh
                // Quickly set the ssthresh, get out of fast probing mode
                m_fast_probing = 1;
                // Need to use the following parameters..so do not touch them
                m_ackSeqCount = 0;
                m_ackSeqStartNum = 0;
                m_ackSeqEndNum = 0;
                m_ackSeqStartTime = 0;
                m_ackSeqEndTime = 0;
            }
        }
        // Updating m_ssThresh if min_rtt changed ..
        m_cWnd += m_segmentSize;
        NS_LOG_INFO(
                "In SlowStart, ACK of seq " << seq << "; update cwnd to " << m_cWnd << "; ssthresh " << m_ssThresh);
    }
    else
    {
        if (m_lastSlowStart == 0)
        {
            m_lastSlowStart = Simulator::Now();
        }
        // Congestion avoidance mode, increase by (segSize*segSize)/cwnd. (RFC2581, sec.3.1)
        // To increase cwnd for one segSize per RTT, it should be (ackBytes*segSize)/cwnd
        double adder = static_cast<double>(m_segmentSize * m_segmentSize) / m_cWnd.Get();
        adder = std::max(1.0, adder);
        // m_var is the varience of RTT, only update cwnd when the RTT is stable
        if (m_stage == STABLE)
        {
            // Revise ssthresh, incase the min rtt measured is not right ....
            if (m_firstSlowStart)
            {
                if (m_firstMinRtt != m_minRtt)
                {
                    double BDP = m_minRtt.GetSeconds() * m_ebw;
                    m_ssThresh = BDP / m_nFlows;
                    m_cWnd = m_ssThresh;
                }
            }
            if ((m_currentRTT < 1.05 * m_minRtt.GetSeconds()))
            {/*
                if ((Simulator::Now().GetSeconds() > 125) && (Simulator::Now().GetSeconds() < 138))
                {
                    std::cout << Simulator::Now().GetSeconds() << " " << m_dst_port
                            << ": Starve state: m_currentRTT: " << m_currentRTT << " m_minRtt "
                            << m_minRtt.GetSeconds() << std::endl;
                }*/

                if (!m_firstSlowStart || m_nFlows == 1)
                {
                    double sinceLastSlowStart = Simulator::Now().GetSeconds()
                            - m_lastSlowStart.GetSeconds();
                    if (sinceLastSlowStart > 2)
                    {
                        // Probably underutilize the bandwidth
                        if (m_fastConverge == false)
                        {
                            // If we just get into situation where RTT falls behind ssthresh
                            m_fastConverge = true;
                            // The speed to converge is propotion to its estimation of flow number
                            // This guarantee the flow with lower throughput will increase faster
                            m_power = m_nFlows;
                            m_lastIncrease = Simulator::Now();
                        }
                        else
                        {
                            double timeDifference = Simulator::Now().GetSeconds()
                                    - m_lastIncrease.GetSeconds();
                            // If after 0.5s we still got starving, double the increase rate
                            if (timeDifference > 1)
                            {
                                m_power *= 2;
                                m_lastIncrease = Simulator::Now();
                            }
                        }
                        m_cWnd += static_cast<uint32_t>(m_power * adder);
                    }
                }
            }
            else if (m_currentRTT > 1.2 * m_minRtt.GetSeconds())
            {
                // In congsetion state, two possibilities:
                // (1) The estimate flow number is not right, reset ssthresh
                // (2) The estimate flow number is fine, but send a little bit too aggressive, reduce cwnd gracefully
                if ((Simulator::Now().GetSeconds() > 125) && (Simulator::Now().GetSeconds() < 138))
                {
                    std::cout << Simulator::Now().GetSeconds() << " " << m_dst_port
                            << ": Full state: m_currentRTT: " << m_currentRTT << " m_minRtt "
                            << m_minRtt.GetSeconds() << std::endl;
                }
                m_fastConverge = false;
                if (m_currentTp != 0)
                {
                    // We need estimation of throughput to estimate fair share and flow number
                    double fixRatio = m_currentTp.Get() / (double) m_ebw;
                    uint32_t estimateFlows = EstimateFlowNumber(fixRatio);
                    double BDP = (double) m_ebw * m_minRtt.GetSeconds();
                    // If every flow is as aggressive as me, then the total delay should not execeed ?
                    // If some bad fish is not obey the game rule, the diff between current rtt and maximum rtt should be larger than
                    // the maximin total delay
                    if (estimateFlows > m_nFlows)
                    {
                        // The estimation of flow number is either too aggressive or too conservative
                        double sinceLastReduce = Simulator::Now().GetSeconds()
                                - m_lastReduce.GetSeconds();
                        double sinceLastSlowStart = Simulator::Now().GetSeconds()
                                - m_lastSlowStart.GetSeconds();
                        // If it has been over 0.5s since last fast reduce
                        // Why this has not to be first slow start ?

/*                        if ((m_dst_port == 3) &&(Simulator::Now().GetSeconds() > 45)
                                && (Simulator::Now().GetSeconds() < 47))
                        {
                            std::cout << Simulator::Now().GetSeconds() << " " << m_dst_port
                                    << ": Full state: m_lastSlowStart: " << m_lastSlowStart.GetSeconds()
                                    << " sinceLastSlowStart " << sinceLastSlowStart << std::endl;
                        }*/
                        if (sinceLastSlowStart > 2)
                        {
                            if (m_lastReduce == 0 || sinceLastReduce > 1)
                            {
                                // Reduce one flow at one time, maximum
                                m_nFlows = estimateFlows;
                                m_ssThresh = BDP / m_nFlows;
                                m_cWnd = m_ssThresh;
                                m_lastReduce = Simulator::Now();
                            }
                        }
                    }
                    else if (estimateFlows == m_nFlows)
                    {
                        m_cWnd -= adder;
                    }
                }
            }
            else
            {
                // In good state. Reallocate fair share
                if (m_currentTp != 0)
                {
                    double fixRatio = m_currentTp.Get() / (double) m_ebw;
                    uint32_t estimateFlows = EstimateFlowNumber(fixRatio);
                    if (estimateFlows < m_nFlows)
                    {
                        m_nFlows = estimateFlows;
                        double BDP = (double) m_ebw * m_minRtt.GetSeconds();
                        m_ssThresh = BDP / m_nFlows;
                        m_cWnd = std::max(m_cWnd, m_ssThresh);
                    }
                }
            }
        }
        else if (m_stage == COMPETEING)
        {
            m_power = std::max(1, (int) m_nFlows - 1);
            m_cWnd += static_cast<uint32_t>(m_nFlows * adder);
            double fixRatio = m_currentTp.Get() / (double) m_ebw;
            uint32_t estimateFlows = EstimateFlowNumber(fixRatio);
            m_nFlows = estimateFlows;
        }
        // Do not do anything, collecting RTT samples
        NS_LOG_FUNCTION(this);
        if (m_startCollecting == 0)
        {
            m_startCollecting = Simulator::Now();
        }
        double timeDifference = Simulator::Now().GetSeconds() - m_startCollecting.GetSeconds();
        // Save rtt samples, compute varience of RTT
        if (timeDifference < 0.5)
        {
            m_time_samples.push_back(Simulator::Now().GetSeconds());  // in s
            m_rtt_samples.push_back(m_currentRTT * 1000);  // in ms
        }
        else
        {
            double slope = GetSlope();
            m_time_samples.clear();
            m_rtt_samples.clear();
            m_startCollecting = Simulator::Now();
            double standard = 1 / m_currentRTT;
            double lowBound = standard / m_nFlows;
            if (slope > lowBound)
            {
                m_comThresh += 1;
                if (m_comThresh == 3)
                {
                    m_stage = COMPETEING;
                }
            }
            else
            {
                m_comThresh = 0;
                m_stage = STABLE;
            }
        }
        EstimateTp();
        NS_LOG_INFO("In CongAvoid, updated to cwnd " << m_cWnd/1000 << " ssthresh " << m_ssThresh);
    }
    // Complete newAck processing
    TcpSocketBase::NewAck(seq);
    *stream_var->GetStream() << Simulator::Now().GetSeconds() << " " << m_var << std::endl;
}

double TcpCR::GetSlope()
{
    long int npoints = m_time_samples.size(); //points
    alglib::real_2d_array xy;
    xy.setlength(npoints, npoints);
    // Fill up the matrix. First column is x, second column is y
    for (int x = 0; x < npoints; x++)
    {
        for (int x = 0; x < npoints; x++)
        {
            xy(x, 0) = m_time_samples[x];
            xy(x, 1) = m_rtt_samples[x];
        }
    }
    long int nvars = 1;
    long int info = 0;
    alglib::linearmodel lm;
    alglib::lrreport report;
    alglib::lrbuild(xy, npoints, nvars, info, lm, report);
    alglib::real_1d_array v;
    lrunpack(lm, v, nvars);
    return v[0];
}


void TcpCR::EstimateTp()
{
    NS_LOG_FUNCTION(this);
    // Estimate throughput every 0.5 seconds
    m_ackSeqCount += 1;
    if (m_ackSeqStartTime == 0)
        m_ackSeqStartTime = Simulator::Now().GetSeconds();
    double timeDifference = Simulator::Now().GetSeconds() - m_ackSeqStartTime;
    if (timeDifference >= 1.0)
    {
        // get a new m_currentTp samples
        m_currentTp = m_ackSeqCount * m_segmentSize / timeDifference; // in bps
        m_ackSeqStartTime = Simulator::Now().GetSeconds();
        m_ackSeqCount = 0;
    }
    *stream_tp->GetStream() << Simulator::Now().GetSeconds() << " " << m_currentTp / 125000
            << std::endl;
}

double TcpCR::GetAverage(const std::list<double> linkedlist)
{
    std::list<double>::const_iterator it;
    double sum = 0;
    for (it = linkedlist.begin(); it != linkedlist.end(); ++it)
    {
        sum += *it;
    }
    return sum / linkedlist.size();
}

double TcpCR::GetVarience(const std::list<double> linkedlist)
{
    std::list<double>::const_iterator it;
    double sum = 0;
    double avg = 0;
    for (it = linkedlist.begin(); it != linkedlist.end(); ++it)
    {
        sum += *it;
    }
    avg = sum / linkedlist.size();
    double sumsquare = 0;
    for (it = linkedlist.begin(); it != linkedlist.end(); ++it)
    {
        sumsquare += (*it - avg) * (*it - avg);
    }
    return sumsquare / linkedlist.size();
}

int TcpCR::CountAck(const TcpHeader& tcpHeader)
{
    NS_LOG_FUNCTION(this);

    // Calculate the number of acknowledged segments based on the received ACK number
    int cumul_ack = (tcpHeader.GetAckNumber() - m_prevAckNo) / m_segmentSize;

    if (cumul_ack == 0)
    {                // A DUPACK counts for 1 segment delivered successfully
        m_accountedFor++;
        cumul_ack = 1;
    }
    if (cumul_ack > 1)
    {                // A delayed ACK or a cumulative ACK after a retransmission
                     // Check how much new data it ACKs
        if (m_accountedFor >= cumul_ack)
        {
            m_accountedFor -= cumul_ack;
            cumul_ack = 1;
        }
        else if (m_accountedFor < cumul_ack)
        {
            cumul_ack -= m_accountedFor;
            m_accountedFor = 0;
        }
    }

    // Update the previous ACK number
    m_prevAckNo = tcpHeader.GetAckNumber();

    return cumul_ack;
}

template<typename T>
std::string NumberToString(T Number)
{
    std::stringstream ss;
    ss << Number;
    return ss.str();
}

void TcpCR::ReceivedAck(Ptr<Packet> packet, const TcpHeader& tcpHeader)
{
    NS_LOG_FUNCTION(this);
    if (m_src_port == 0)
    {
        m_src_port = tcpHeader.GetDestinationPort();
        m_dst_port = tcpHeader.GetSourcePort();
    }
    AsciiTraceHelper ascii;
    if (!stream_tp)
    {
        AsciiTraceHelper ascii;
        stream_tp = ascii.CreateFileStream(
                folder + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".throughput");
    }
    if (!stream_rtt)
    {
        AsciiTraceHelper ascii;
        stream_rtt = ascii.CreateFileStream(
                folder + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".rtt");
    }
    if (!stream_var)
    {
        AsciiTraceHelper ascii;
        stream_var = ascii.CreateFileStream(
                folder + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".var");
    }
    if (!stream_other)
    {
        AsciiTraceHelper ascii;
        stream_other = ascii.CreateFileStream(
                folder + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".other");
    }
    if ((0 != (tcpHeader.GetFlags() & TcpHeader::ACK)))
    {
        // Only do it for ACK
        // Calculate m_lastRtt
        TcpSocketBase::EstimateRtt(tcpHeader);
        // Update minRtt
        if (m_minRtt == Time(0))
        {
            m_minRtt = m_lastRtt;
        }
        else
        {
            if (m_lastRtt < m_minRtt)
            {
                m_minRtt = m_lastRtt;
            }
        }
        m_currentRTT = m_lastRtt.Get().ToDouble(Time::S);
        FilteringRTT();
        *stream_rtt->GetStream() << Simulator::Now().GetSeconds() << " " << m_currentRTT
                << std::endl;
    }
    // Complete ack processing
    TcpSocketBase::ReceivedAck(packet, tcpHeader);
}


/*void TcpCR::FilteringVarience()
{
    NS_LOG_FUNCTION(this);

    double alpha = 0.9;
    double sample_varience = m_currentVar;
    m_currentVar = (alpha * m_lastVar) + ((1 - alpha) * ((sample_varience + m_lastSampleVar) / 2));
    m_lastSampleVar = sample_varience;
    m_lastVar = m_currentVar;
}*/

/*void TcpCR::FilteringTP()
{
    NS_LOG_FUNCTION(this);

    double alpha = 0.9;
    double sample_bwe = m_currentTp;
    m_currentTp = (alpha * m_lastTp) + ((1 - alpha) * ((sample_bwe + m_lastSampleBW) / 2));
    m_lastSampleBW = sample_bwe;
    m_lastTp = m_currentTp;
}*/

void TcpCR::FilteringRTT()
{
    NS_LOG_FUNCTION(this);
    double alpha = 0.8;
    double sample_rtt = m_currentRTT;
    m_currentRTT = (alpha * m_lastRttAvg) + ((1 - alpha) * ((sample_rtt + m_lastSampleRTT) / 2));
    m_lastSampleRTT = sample_rtt;
    m_lastRttAvg = m_currentRTT;
}

/* Cut cwnd and enter fast recovery mode upon triple dupack */
void TcpCR::DupAck(const TcpHeader& t, uint32_t count)
{
    NS_LOG_FUNCTION(this << count);
    if (count == m_retxThresh && !m_inFastRec)
    { // triple duplicate ack triggers fast retransmit (RFC2582 sec.3 bullet #1)
        m_ssThresh = std::max(2 * m_segmentSize, BytesInFlight() / 2);
        //m_ssThresh = 80000;
        m_cWnd = m_ssThresh + 3 * m_segmentSize;

        m_recover = m_highTxMark;
        m_inFastRec = true;
        NS_LOG_INFO(
                "Triple dupack. Enter fast recovery mode. Reset cwnd to " << m_cWnd << ", ssthresh to " << m_ssThresh << " at fast recovery seqnum " << m_recover);
        DoRetransmit();
    }
    else if (m_inFastRec)
    { // Increase cwnd for every additional dupack (RFC2582, sec.3 bullet #3)
        m_cWnd += m_segmentSize;
        NS_LOG_INFO("Dupack in fast recovery mode. Increase cwnd to " << m_cWnd);
        if (!m_sendPendingDataEvent.IsRunning())
        {
            SendPendingData(m_connected);
        }
    }
    else if (!m_inFastRec && m_limitedTx && m_txBuffer->SizeFromSequence(m_nextTxSequence) > 0)
    { // RFC3042 Limited transmit: Send a new packet for each duplicated ACK before fast retransmit
        NS_LOG_INFO("Limited transmit");
        uint32_t sz = SendDataPacket(m_nextTxSequence, m_segmentSize, true);
        m_nextTxSequence += sz;                    // Advance next tx sequence
    };
}

void TcpCR::Reset(void)
{
    m_ackSeqCount = 0;
    m_ackSeqLength = 1;
    m_ackSeqStartNum = 0;
    m_ackSeqEndNum = 0;
    m_ackSeqStartTime = 0;
    m_ackSeqEndTime = 0;
    m_fast_probing = 0;
    m_currentTp = 0;
    m_lastReduce = Time(0);
    m_lastIncrease = Time(0);
    m_rtt_samples.clear();
    //m_nFlows = 1;
    m_var = 0;
    m_fastConverge = false;
    m_power = 0;
    m_lastSlowStart = Time(0);
    m_startCollecting = Time(0);
    m_firstSlowStart = false;
    m_stage = STABLE;
    m_comThresh = 0;
}

/* Retransmit timeout */
void TcpCR::Retransmit(void)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_LOGIC(this << " ReTxTimeout Expired at time " << Simulator::Now ().GetSeconds ());
    m_inFastRec = false;

    // If erroneous timeout in closed/timed-wait state, just return
    if (m_state == CLOSED || m_state == TIME_WAIT)
        return;
    // If all data are received (non-closing socket and nothing to send), just return
    if (m_state <= ESTABLISHED && m_txBuffer->HeadSequence() >= m_highTxMark)
        return;

    // According to RFC2581 sec.3.1, upon RTO, ssthresh is set to half of flight
    // size and cwnd is set to 1*MSS, then the lost packet is retransmitted and
    // TCP back to slow start
    m_ssThresh = std::max(10 * m_segmentSize, BytesInFlight() / 2);
    //m_ssThresh = 4*125000;
    m_cWnd = m_segmentSize;
    m_nextTxSequence = m_txBuffer->HeadSequence(); // Restart from highest Ack
    NS_LOG_INFO(
            "RTO. Reset cwnd to " << m_cWnd << ", ssthresh to " << m_ssThresh << ", restart from seqnum " << m_nextTxSequence);
    // Be prefared to estimate bandwidth in slow start
    Reset();
    DoRetransmit();                          // Retransmit the packet
}

} // namespace ns3
