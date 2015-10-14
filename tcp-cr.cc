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

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpCR");

NS_OBJECT_ENSURE_REGISTERED (TcpCR);

TypeId
TcpCR::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpCR")
    .SetParent<TcpSocketBase> ()
    .SetGroupName ("Internet")
    .AddConstructor<TcpCR> ()
    .AddAttribute ("ReTxThreshold", "Threshold for fast retransmit",
                    UintegerValue (3),
                    MakeUintegerAccessor (&TcpCR::m_retxThresh),
                    MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("LimitedTransmit", "Enable limited transmit",
                   BooleanValue (false),
                   MakeBooleanAccessor (&TcpCR::m_limitedTx),
                   MakeBooleanChecker ())
 ;
  return tid;
}

TcpCR::TcpCR (void)
  : m_retxThresh (3), // mute valgrind, actual value set by the attribute system
    m_inFastRec (false),
    m_limitedTx (false), // mute valgrind, actual value set by the attribute system
    m_ackSeqLength(1),
    test(0)
{
  NS_LOG_FUNCTION (this);
}

TcpCR::TcpCR (const TcpCR& sock)
  : TcpSocketBase (sock),
    m_retxThresh (sock.m_retxThresh),
    m_inFastRec (false),
    m_limitedTx (sock.m_limitedTx)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_LOGIC ("Invoked the copy constructor");
}

TcpCR::~TcpCR (void)
{
}

Ptr<TcpSocketBase>
TcpCR::Fork (void)
{
  return CopyObject<TcpCR> (this);
}


/* New ACK (up to seqnum seq) received. Increase cwnd and call TcpSocketBase::NewAck() */
void
TcpCR::NewAck (const SequenceNumber32& seq)
{
  NS_LOG_FUNCTION (this << seq);
  NS_LOG_LOGIC ("TcpCR received ACK for seq " << seq <<
                " cwnd " << m_cWnd <<
                " ssthresh " << m_ssThresh);

  // Check for exit condition of fast recovery
  if (m_inFastRec && seq < m_recover)
    { // Partial ACK, partial window deflation (RFC2582 sec.3 bullet #5 paragraph 3)
      m_cWnd += m_segmentSize - (seq - m_txBuffer->HeadSequence ());
      NS_LOG_INFO ("Partial ACK for seq " << seq << " in fast recovery: cwnd set to " << m_cWnd);
      m_txBuffer->DiscardUpTo(seq);  //Bug 1850:  retransmit before newack
      DoRetransmit (); // Assume the next seq is lost. Retransmit lost packet
      TcpSocketBase::NewAck (seq); // update m_nextTxSequence and send new data if allowed by window
      return;
    }
  else if (m_inFastRec && seq >= m_recover)
    { // Full ACK (RFC2582 sec.3 bullet #5 paragraph 2, option 1)
      m_cWnd = std::min (m_ssThresh.Get (), BytesInFlight () + m_segmentSize);
      m_inFastRec = false;
      NS_LOG_INFO ("Received full ACK for seq " << seq <<". Leaving fast recovery with cwnd set to " << m_cWnd);
    }

  // Increase of cwnd based on current phase (slow start or congestion avoidance)
  if (m_cWnd < m_ssThresh)
    { // Slow start mode, add one segSize to cWnd. Default m_ssThresh is 65535. (RFC2001, sec.1)
      // Only did so if m_segmentSize > 0
      // Rerursively change the value of ssThreash
      // Keep recording the ACK' arrival time
      // keep recording of ack index
      // 1 ack: ignore
      // 2 ack: remember its time,set as start time, update sequence number 2
      // 3 ack: get its time, subtract it with start time, get first throughput sample
      // 4 ack: remember its time and index, set start time: update sequence number 2: 4
      // ...
      // 7 ack: get its time, subtract it with start time, get second throughput sample
      // organize: so what should you remember? The sequence number, seqStartTime, seqEndTime

      // Algorithm 2
      // If (m_ackSeqStartNum == 0) & cwnd > 1
      // set start time and num, length = cwnd
      m_ackSeqCount += 1;
      //std::cout<< m_ackSeqCount << " " << Simulator::Now ().GetSeconds () << std::endl;
      if (test == 0)
      {
      if ((m_ackSeqStartNum == 0) & (m_cWnd.Get()/m_segmentSize > 1))
      {
          m_ackSeqStartTime = Simulator::Now ().GetSeconds ();
          m_ackSeqStartNum = m_ackSeqCount;
          m_ackSeqLength = m_cWnd.Get()/m_segmentSize;
          //std::cout<<"cwnd: "<<m_cWnd<<" m_ackSeqLength: " << m_ackSeqLength<<std::endl;
      }
      else if (m_ackSeqCount == m_ackSeqStartNum + m_ackSeqLength - 1)
        {
            m_ackSeqEndNum = m_ackSeqCount;
            m_ackSeqEndTime = Simulator::Now().GetSeconds();
            double last = m_ackSeqEndTime - m_ackSeqStartTime;
            double fEBW = ((1054 * (m_ackSeqLength - 1)) / last); // in Bps
            std::cout << Simulator::Now().GetSeconds() << " " << fEBW * 8/1000000 << std::endl;
            // Calculate BDP using minimum RTT * EBW
            double BDP = m_minRtt.GetSeconds() * fEBW;  // in Byte
            //double BDP = 0.1 * fEBW;  // in Byte
            //std::cout << Simulator::Now().GetSeconds() << " " << BDP/1000 << std::endl;
            //std::cout << Simulator::Now().GetSeconds() << " " << fEBW * 8/1000000 << std::endl;
            m_ssThresh = BDP/2;
            test = 1;
            // Get one throughput sample, update status
            m_ackSeqStartNum = 0;
            m_ackSeqEndNum = 0;
            m_ackSeqStartTime = 0;
            m_ackSeqEndTime = 0;
        }
      }
      m_cWnd +=  m_segmentSize;
      NS_LOG_INFO ("In SlowStart, ACK of seq " << seq << "; update cwnd to " << m_cWnd << "; ssthresh " << m_ssThresh);
    }
  else
    { // Congestion avoidance mode, increase by (segSize*segSize)/cwnd. (RFC2581, sec.3.1)
      // To increase cwnd for one segSize per RTT, it should be (ackBytes*segSize)/cwnd
      double adder = static_cast<double> (m_segmentSize * m_segmentSize) / m_cWnd.Get ();
      adder = std::max (1.0, adder);
      m_cWnd += static_cast<uint32_t> (adder);
      NS_LOG_INFO ("In CongAvoid, updated to cwnd " << m_cWnd << " ssthresh " << m_ssThresh);
    }

  // Complete newAck processing
  TcpSocketBase::NewAck (seq);
}


void
TcpCR::ReceivedAck(Ptr<Packet> packet, const TcpHeader& tcpHeader)
{
    NS_LOG_FUNCTION(this);
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
    }
    // Complete ack processing
    TcpSocketBase::ReceivedAck(packet, tcpHeader);
}


/* Cut cwnd and enter fast recovery mode upon triple dupack */
void
TcpCR::DupAck (const TcpHeader& t, uint32_t count)
{
  NS_LOG_FUNCTION (this << count);
  if (count == m_retxThresh && !m_inFastRec)
    { // triple duplicate ack triggers fast retransmit (RFC2582 sec.3 bullet #1)
      m_ssThresh = std::max (2 * m_segmentSize, BytesInFlight () / 2);
      //m_ssThresh = 80000;
      m_cWnd = m_ssThresh + 3 * m_segmentSize;
      m_recover = m_highTxMark;
      m_inFastRec = true;
      NS_LOG_INFO ("Triple dupack. Enter fast recovery mode. Reset cwnd to " << m_cWnd <<
                   ", ssthresh to " << m_ssThresh << " at fast recovery seqnum " << m_recover);
      DoRetransmit ();
    }
  else if (m_inFastRec)
    { // Increase cwnd for every additional dupack (RFC2582, sec.3 bullet #3)
      m_cWnd += m_segmentSize;
      NS_LOG_INFO ("Dupack in fast recovery mode. Increase cwnd to " << m_cWnd);
      if (!m_sendPendingDataEvent.IsRunning ())
        {
          SendPendingData (m_connected);
        }
    }
  else if (!m_inFastRec && m_limitedTx && m_txBuffer->SizeFromSequence (m_nextTxSequence) > 0)
    { // RFC3042 Limited transmit: Send a new packet for each duplicated ACK before fast retransmit
      NS_LOG_INFO ("Limited transmit");
      uint32_t sz = SendDataPacket (m_nextTxSequence, m_segmentSize, true);
      m_nextTxSequence += sz;                    // Advance next tx sequence
    };
}

/* Retransmit timeout */
void
TcpCR::Retransmit (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_LOGIC (this << " ReTxTimeout Expired at time " << Simulator::Now ().GetSeconds ());
  m_inFastRec = false;

  // If erroneous timeout in closed/timed-wait state, just return
  if (m_state == CLOSED || m_state == TIME_WAIT) return;
  // If all data are received (non-closing socket and nothing to send), just return
  if (m_state <= ESTABLISHED && m_txBuffer->HeadSequence () >= m_highTxMark) return;

  // According to RFC2581 sec.3.1, upon RTO, ssthresh is set to half of flight
  // size and cwnd is set to 1*MSS, then the lost packet is retransmitted and
  // TCP back to slow start
  m_ssThresh = std::max (2 * m_segmentSize, BytesInFlight () / 2);
  //m_ssThresh = 4*125000;
  m_cWnd = m_segmentSize;
  m_nextTxSequence = m_txBuffer->HeadSequence (); // Restart from highest Ack
  NS_LOG_INFO ("RTO. Reset cwnd to " << m_cWnd <<
               ", ssthresh to " << m_ssThresh << ", restart from seqnum " << m_nextTxSequence);

  // Be prefare to estimate bandwidth in slow start
  m_ackSeqCount = 0;
  m_ackSeqLength = 1;
  m_ackSeqStartNum = 0;
  m_ackSeqEndNum = 0;
  m_ackSeqStartTime = 0;
  m_ackSeqEndTime = 0;
  test = 0;
  DoRetransmit ();                          // Retransmit the packet
}

} // namespace ns3
