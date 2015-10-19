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

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpCR");

NS_OBJECT_ENSURE_REGISTERED(TcpCR);

const int              CONGESTION = 1;
const int              GOOD = 2;
const int              STARVE = 3;

TypeId TcpCR::GetTypeId(void)
{
    static TypeId tid =
            TypeId("ns3::TcpCR").SetParent<TcpSocketBase>().SetGroupName("Internet").AddConstructor<
                    TcpCR>().AddAttribute("ReTxThreshold", "Threshold for fast retransmit",
                    UintegerValue(3), MakeUintegerAccessor(&TcpCR::m_retxThresh),
                    MakeUintegerChecker<uint32_t>()).AddAttribute("LimitedTransmit",
                    "Enable limited transmit", BooleanValue(false),
                    MakeBooleanAccessor(&TcpCR::m_limitedTx), MakeBooleanChecker());
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
        m_ackSeqCount( 0),
        m_ackSeqLength(1),
        m_ebw(0),
        m_currentTp(0),
        m_lastTp(0),
        m_minRtt(0.0),
        m_nFlows(1),
        m_fast_probing(0),
        m_currentState(STARVE),
        m_sigma(1e-6),
        m_var(0),
        m_lastVar(0),
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
    if (ratio > 0.75)
        return 1;
    else if (ratio > 0.4)
        return 2;
    else if (ratio > 0.28)
        return 3;
    else
        return 4;
}

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
    { // Slow start mode, add one segSize to cWnd. Default m_ssThresh is 65535. (RFC2001, sec.1)
        if (m_fast_probing == 0)
        {
            //std::cout<<"m_ackSeqStartNum: "<<m_ackSeqStartNum<<" m_ackSeqLength: " << m_ackSeqLength<<std::endl;
            m_ackSeqCount += 1;
            if ((m_ackSeqStartNum == 0) & (m_cWnd.Get() / m_segmentSize > 1))
            {
                m_ackSeqStartTime = Simulator::Now().GetSeconds();
                m_ackSeqStartNum = m_ackSeqCount;
                m_ackSeqLength = m_cWnd.Get() / m_segmentSize;
            }
            else if (m_ackSeqCount == m_ackSeqStartNum + m_ackSeqLength - 1)
            {
                m_ackSeqEndNum = m_ackSeqCount;
                m_ackSeqEndTime = Simulator::Now().GetSeconds();
                double last = m_ackSeqEndTime - m_ackSeqStartTime;
                double fEBW = ((1054 * (m_ackSeqLength - 1)) / last); // in Bps
                m_ebw = fEBW;
                // Calculate BDP using minimum RTT * EBW
                double BDP = m_minRtt.GetSeconds() * fEBW;  // in Byte
                m_ssThresh = BDP;   // in byte

                m_fast_probing = 1;
                // Get one throughput sample, update status
                m_ackSeqCount = 0;
                m_ackSeqStartNum = 0;
                m_ackSeqEndNum = 0;
                m_ackSeqStartTime = 0;
                m_ackSeqEndTime = 0;
            }
        }
        m_cWnd += m_segmentSize;
        if (m_dst_port == 4 && (Simulator::Now().GetSeconds() > 210) && (Simulator::Now().GetSeconds() < 230))
                {
                    std::cout<< Simulator::Now().GetSeconds() << "SlowStart "<< m_dst_port << " ssthresh: " << m_ssThresh/1000 << " cwnd: " << m_cWnd/1000 << std::endl;
                }
        NS_LOG_INFO(
                "In SlowStart, ACK of seq " << seq << "; update cwnd to " << m_cWnd << "; ssthresh " << m_ssThresh);
    }
    else
    {
        // Congestion avoidance mode, increase by (segSize*segSize)/cwnd. (RFC2581, sec.3.1)
        // To increase cwnd for one segSize per RTT, it should be (ackBytes*segSize)/cwnd
        double adder = static_cast<double>(m_segmentSize * m_segmentSize) / m_cWnd.Get();
        adder = std::max(1.0, adder);
        if (m_var < m_sigma)
        {
            // Stable stage
            //*stream_var->GetStream() << Simulator::Now().GetSeconds() << " " << 1
            //                           << std::endl;

            m_lastState = m_currentState;
            if ((m_currentRTT < 1.2 * m_minRtt.GetSeconds())
                    & (m_currentRTT > 1.05 * m_minRtt.GetSeconds()))
            {
                // In good state
                m_currentState = GOOD;
            }
            else if (m_currentRTT < 1.05 * m_minRtt.GetSeconds())
            {
                // in starve state
                m_currentState = STARVE;
                if (true)
                {
                    m_cWnd += static_cast<uint32_t>(adder);
                }
                else
                {
                }
            }
            else if (m_currentRTT > 1.2 * m_minRtt.GetSeconds())
            {
                m_currentState = CONGESTION;
                // in possible congsetion state
                //std::cout<< Simulator::Now().GetSeconds() << " m_currentTp: " << m_currentTp.Get()/125000 << " m_ebw: " << m_ebw/125000<<std::endl;
                if (m_currentTp != 0)
                {
                    //std::cout<< Simulator::Now().GetSeconds() << " m_currentTp: " << m_currentTp.Get()/125000 << " m_ebw: " << m_ebw/125000<<std::endl;
                    double fixRatio = m_currentTp.Get() / (double) m_ebw;
                    uint32_t estimateFlows = EstimateFlowNumber(fixRatio);
                    if ((Simulator::Now().GetSeconds() > 21)
                            && (Simulator::Now().GetSeconds() < 22))
                    {
                        std::cout<< Simulator::Now().GetSeconds() << " "<< m_dst_port << " fixRatio: " << fixRatio << " Estimate number of flows: " << m_nFlows<<std::endl;
                    }
                    if (estimateFlows > m_nFlows)
                    {
                        int lastNFlow = m_nFlows;
                        m_nFlows = estimateFlows;
                        //std::cout<< Simulator::Now().GetSeconds() << " "<< m_dst_port << " fixRatio: " << fixRatio << " Estimate number of flows: " << m_nFlows<<std::endl;
                        double timeDifference = Simulator::Now().GetSeconds()
                                - m_lastReduce.GetSeconds();
                        if (m_lastReduce == 0 || timeDifference > m_currentRTT)
                        {
                            m_ssThresh = m_ssThresh * lastNFlow/ (double) m_nFlows;
                            m_cWnd = m_ssThresh;
                            m_lastReduce = Simulator::Now();
                        }
                    }
                    //std::cout<< Simulator::Now().GetSeconds() << " fixRatio: " << fixRatio << " Estimate number of flows: " << m_nFlows<<std::endl;
                    else if (estimateFlows == m_nFlows)
                    {
                        m_cWnd -= static_cast<uint32_t>(adder);
                        m_cWnd = std::max(m_segmentSize, m_cWnd.Get());
                        m_ssThresh = m_cWnd;
                    }
                    else
                    {
                        //std::cout<< Simulator::Now().GetSeconds() << " estimateFlows: " << estimateFlows << " m_nFlows: " << m_nFlows<<std::endl;
                    }
                }
            }
        }
        // Save rtt samples
        m_rtt_samples.push_back(m_currentRTT);
        // Make sure only collect RTT samples in one RTT
        if (m_rtt_samples.size() > 200)
        {
            m_rtt_samples.pop_front();
        }
        //std::cout<<m_rtt_samples.size()<<std::endl;
/*        if ((Simulator::Now().GetSeconds() > 13) && (Simulator::Now().GetSeconds() < 14))
        {
            std::list<double>::const_iterator it;
            double sum = 0;
            for (it = m_rtt_samples.begin(); it != m_rtt_samples.end(); ++it)
            {
                sum += *it;
                *stream_var->GetStream() << Simulator::Now().GetSeconds() << " " << *it
                                   << std::endl;
            }
            *stream_var->GetStream() <<  "****************" << std::endl;
        }*/
        m_var = GetVarience(m_rtt_samples);
        m_currentVar = m_var;


        //FilteringVarience();
        *stream_var->GetStream() << Simulator::Now().GetSeconds() << " " << m_var
                            << std::endl;
        //std::cout << Simulator::Now().GetSeconds() << " " << m_currentVar << std::endl;
        // Estimate current throughput
        //m_rtt_sum += m_lastRtt.Get().ToDouble(Time::S);
        m_ackSeqCount += 1;
        if (m_ackSeqStartTime == 0)
            m_ackSeqStartTime = Simulator::Now().GetSeconds();
        double timeDifference = Simulator::Now().GetSeconds() - m_ackSeqStartTime;
        if (timeDifference >= 0.5)
        {
            // get a new m_currentTp samples
            m_currentTp = m_ackSeqCount * m_segmentSize / timeDifference; // in bps
/*            if ((Simulator::Now().GetSeconds() > 15) && (Simulator::Now().GetSeconds() < 16))
            {
                std::cout << Simulator::Now().GetSeconds() << " " << m_dst_port << " m_currentTp: "
                        << m_currentTp << " m_ackSeqCount: " << m_ackSeqCount << " timeDifference: " << timeDifference
                        << std::endl;
            }*/
            m_ackSeqStartTime = Simulator::Now().GetSeconds();
            m_ackSeqCount = 0;
        }
        *stream_tp->GetStream() << Simulator::Now().GetSeconds() << " " << m_currentTp / 125000
                << std::endl;
        m_rtt_avg_last = m_rtt_avg;
        m_rtt_sum = 0;

/*        if (m_ackSeqCount == 1)
        {
            m_ackSeqLength = BytesInFlight() / m_segmentSize;
            m_ackSeqStartTime = Simulator::Now().GetSeconds();
        }
        else if (m_ackSeqCount == m_ackSeqLength)
        {
            // almost finish an RTT
            m_ackSeqEndTime = Simulator::Now().GetSeconds();
            double last = m_ackSeqEndTime - m_ackSeqStartTime;
            //m_rtt_avg = m_rtt_sum / m_ackSeqCount;
            //std::cout << Simulator::Now().GetSeconds() << " " << last << std::endl;
            m_currentTp = m_ackSeqLength * m_segmentSize / last; // in bps
            if ((Simulator::Now().GetSeconds() > 15) && (Simulator::Now().GetSeconds() < 16))
            {
                std::cout<< Simulator::Now().GetSeconds() << " "<< m_dst_port << " m_currentTp: " << m_currentTp << " m_ackSeqLength: "
                        << m_ackSeqLength<< " last: " << last <<  std::endl;
            }
            // fix the problem in bw estimating
            if (m_lastTp == 0)
            {
                m_lastTp = m_currentTp;
            }
            //FilteringBW();
            //std::cout << Simulator::Now().GetSeconds() << " " << m_currentTp/125000 << std::endl;
        }*/
        NS_LOG_INFO("In CongAvoid, updated to cwnd " << m_cWnd/1000 << " ssthresh " << m_ssThresh);

/*        if (m_dst_port == 4 && (Simulator::Now().GetSeconds() > 210) && (Simulator::Now().GetSeconds() < 230))
        {
            std::cout<< Simulator::Now().GetSeconds() << " CongAvoid "<< m_dst_port << " ssthresh: " << m_ssThresh/1000 << " cwnd: " << m_cWnd/1000 << std::endl;
        }*/
    }
    // Complete newAck processing
    TcpSocketBase::NewAck(seq);
}

double TcpCR::GetAverage(const std::list<double> linkedlist)
{
    std::list<double>::const_iterator it;
    double sum = 0;
    for (it = linkedlist.begin(); it!=linkedlist.end(); ++it)
    {
        sum += *it;
    }
    return sum/linkedlist.size();
}


double TcpCR::GetVarience(const std::list<double> linkedlist)
{
    std::list<double>::const_iterator it;
    double sum = 0;
    double avg = 0;
    for (it = linkedlist.begin(); it!=linkedlist.end(); ++it)
    {
        sum += *it;
    }
    avg = sum/linkedlist.size();
    double sumsquare = 0;
    for (it = linkedlist.begin(); it!=linkedlist.end(); ++it)
    {
        sumsquare += (*it - avg)*(*it - avg);
    }
    return sumsquare/linkedlist.size();
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
    std::string fileNameRoot = "./results/TcpCR/";
    AsciiTraceHelper ascii;
    if (!stream_tp)
    {
        AsciiTraceHelper ascii;
        stream_tp = ascii.CreateFileStream(
                fileNameRoot + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".throughput");
    }
    if (!stream_rtt)
    {
        AsciiTraceHelper ascii;
        stream_rtt = ascii.CreateFileStream(
                fileNameRoot + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".rtt");
    }
    if (!stream_var)
    {
        AsciiTraceHelper ascii;
        stream_var = ascii.CreateFileStream(
                fileNameRoot + NumberToString(m_src_port) + "-" + NumberToString(m_dst_port)
                        + ".var");
    }
    int acked = 0;
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
    if ((0 != (tcpHeader.GetFlags() & TcpHeader::ACK)) && tcpHeader.GetAckNumber() >= m_prevAckNo)
    {
        acked = CountAck(tcpHeader);
        m_ackedSegments += acked;
        //EstimateBW (acked, tcpHeader, Time(0));
    }
    // Complete ack processing
    TcpSocketBase::ReceivedAck(packet, tcpHeader);
}

/*void TcpCR::EstimateBW(int acked, const TcpHeader& tcpHeader, Time rtt)
{
    NS_LOG_FUNCTION(this);
    // Get the time when the current ACK is received
    double currentAck = static_cast<double>(Simulator::Now().GetSeconds());
    // Calculate the BW
    m_currentTp = acked * m_segmentSize / (currentAck - m_lastAck);
    // Update the last ACK time
    m_lastAck = currentAck;
    // Filter the BW sample
    FilteringBW();
    //std::cout<<Simulator::Now().GetSeconds() << " " <<m_currentTp * 8 / 1000000<<std::endl;
}*/

void TcpCR::FilteringVarience()
{
    NS_LOG_FUNCTION(this);

    double alpha = 0.9;
    double sample_varience = m_currentVar;
    m_currentVar = (alpha * m_lastVar) + ((1 - alpha) * ((sample_varience + m_lastSampleVar)/2));
    m_lastSampleVar = sample_varience;
    m_lastVar = m_currentVar;
}


void TcpCR::FilteringTP()
{
    NS_LOG_FUNCTION(this);

    double alpha = 0.9;
    double sample_bwe = m_currentTp;
    m_currentTp = (alpha * m_lastTp) + ((1 - alpha) * ((sample_bwe + m_lastSampleBW) / 2));
    m_lastSampleBW = sample_bwe;
    m_lastTp = m_currentTp;
}

void TcpCR::FilteringRTT()
{
    NS_LOG_FUNCTION(this);
    double alpha = 0.8;
    double sample_rtt = m_currentRTT;
    m_currentRTT = (alpha * m_lastRttAvg) + ((1 - alpha) * ((sample_rtt + m_lastSampleRTT) / 2));
    //std::cout<<Simulator::Now().GetSeconds() << " " <<m_currentRTT<<std::endl;
    m_lastSampleRTT = sample_rtt;
    m_lastRttAvg = m_currentRTT;
    //std::cout<<Simulator::Now().GetSeconds() << " " <<m_currentRTT.GetSeconds()<<std::endl;
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

    if (m_dst_port == 4 && (Simulator::Now().GetSeconds() > 218) && (Simulator::Now().GetSeconds() < 219))
        {
            std::cout<< Simulator::Now().GetSeconds() << "RTO "<< m_dst_port << " ssthresh: " << m_ssThresh/1000 << " cwnd: " << m_cWnd/1000 << std::endl;
        }
    // Be prefare to estimate bandwidth in slow start
    m_rtt_sum = 0;
    m_ackSeqCount = 0;
    m_ackSeqLength = 1;
    m_ackSeqStartNum = 0;
    m_ackSeqEndNum = 0;
    m_ackSeqStartTime = 0;
    m_ackSeqEndTime = 0;
    m_fast_probing = 0;
    m_currentTp = 0;
    m_lastTp = 0;
    m_lastReduce = Time(0);
    m_lastIncrease = Time(0);
    m_rtt_samples.clear();
    m_nFlows = 1;
    DoRetransmit();                          // Retransmit the packet
}

} // namespace ns3
