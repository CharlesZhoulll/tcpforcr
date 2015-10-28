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

#ifndef TCP_CR_H
#define TCP_CR_H

#include "tcp-socket-base.h"
#include "ns3/output-stream-wrapper.h"

// For statistic calcluation
/*
#include  "ap.h"
#include  "dataanalysis.h"
#include  "alglibinternal.h"
#include  "specialfunctions.h"
#include  "alglibmisc.h"
#include  "integration.h"
#include  "linalg.h"
#include  "optimization.h"
#include  "solvers.h"
#include  "statistics.h"

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

namespace ns3 {

/**
 * \ingroup socket
 * \ingroup tcp
 *
 * \brief An implementation of a stream socket using TCP.
 *
 * This class contains the NewReno implementation of TCP, as of \RFC{2582}.
 */
class TcpCR : public TcpSocketBase
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  /**
   * Create an unbound tcp socket.
   */
  TcpCR (void);
  /**
   * \brief Copy constructor
   * \param sock the object to copy
   */
  TcpCR (const TcpCR& sock);
  virtual ~TcpCR (void);

protected:
  virtual Ptr<TcpSocketBase> Fork (void); // Call CopyObject<TcpCR> to clone me
  virtual void NewAck (SequenceNumber32 const& seq); // Inc cwnd and call NewAck() of parent
  virtual void DupAck (const TcpHeader& t, uint32_t count);  // Halving cwnd and reset nextTxSequence
  virtual void Retransmit (void); // Exit fast recovery upon retransmit timeout
  virtual void ReceivedAck (Ptr<Packet> packet, const TcpHeader& tcpHeader); // Process received ACK
  void GetFlowNumber (void);
  void EstimateRttTrend (void);
  //virtual void EstimateBW (int acked, const TcpHeader& tcpHeader, Time rtt); // Process received ACK
  double GetAverage (const std::list<double>);
  double GetVarience (const std::list<double>);
  void EstimateTp (void);
  double GetSlope (void);
  int EstimateFlowNumber (double ratio);
  //void FilteringTP (void);
  //void FilteringBW (void);
  void Reset (void);
  void FilteringRTT (void);
  //void FilteringVarience (void);
  int CountAck (const TcpHeader& tcpHeader);
  void UpdateAckedSegments (int acked);

protected:
  SequenceNumber32       m_recover;      //!< Previous highest Tx seqnum for fast recovery
  uint32_t               m_retxThresh;   //!< Fast Retransmit threshold
  bool                   m_inFastRec;    //!< currently in fast recovery
  bool                   m_limitedTx;    //!< perform limited transmit

  // For estimate bandwidth
  uint32_t               m_ackSeqStartNum;
  uint32_t               m_ackSeqEndNum;
  double                 m_ackSeqStartTime;
  double                 m_ackSeqEndTime;
  uint32_t               m_ackSeqCount;
  uint32_t               m_ackSeqLength;
  double                 m_ebw;
  TracedValue<double>    m_currentTp;              //!< Current value of the estimated throughput
  //double                 m_lastTp;                 //!< Last bandwidth sample after being filtered

  // For RTT estimation
  Time                   m_firstMinRtt;
  Time                   m_minRtt;
  bool                   m_firstSlowStart;
  //double                 m_rtt_sum;
  //double                 m_rtt_avg;
  //double                 m_rtt_avg_last;
  double                 m_currentRTT;   // in seconds
  double                 m_lastSampleRTT;
  double                 m_lastRttAvg;    // last moving average of RTT
  Time                   m_startCollecting;
  std::vector<double>      m_rtt_samples;
  std::vector<double>      m_time_samples;

  // For fast state transition
  uint32_t               m_nFlows;
  bool                   m_fast_probing;
  bool                   m_fast_reduce;
  bool                   m_fast_increase;
  Time                   m_lastReduce;
  Time                   m_lastIncrease;
  Time                   m_lastSlowStart;
  //uint32_t               m_currentState;
  //uint32_t               m_lastState;
  //int                    m_starveCount;
  bool                   m_fastConverge;
  uint32_t               m_increaseRate;

  uint32_t                   m_stage;
  uint32_t                   m_increaseCount;
  uint32_t                   m_increaseThresh;

  // Judging whether stable or not
  double                 m_sigma;
  double                 m_var;
  //double                 m_last_var;
  //double                 m_currentVar;
  //double                 m_lastVar;
  //double                 m_lastSampleVar;


  // Added by westwood
  int                    m_accountedFor;           //!< The number of received DUPACKs

  double                 m_lastSampleBW;           //!< Last bandwidth sample
  double                 m_lastAck;                //!< The time last ACK was received
  SequenceNumber32       m_prevAckNo;              //!< Previously received ACK number
  bool                   m_IsCount;                //!< Start keeping track of m_ackedSegments for Westwood+ if TRUE
  EventId                m_bwEstimateEvent;        //!< The BW estimation event for Westwood+
  int                    m_ackedSegments;          //!< The number of segments ACKed between RTTs




  // For debugging
  std::string                 folder;
  uint32_t                    m_src_port;
  uint32_t                    m_dst_port;
  Ptr<OutputStreamWrapper>    stream_tp;
  Ptr<OutputStreamWrapper>    stream_rtt;
  Ptr<OutputStreamWrapper>    stream_var;
  Ptr<OutputStreamWrapper>    stream_other;
};

} // namespace ns3

#endif /* TCP_NEWRENO_H */
