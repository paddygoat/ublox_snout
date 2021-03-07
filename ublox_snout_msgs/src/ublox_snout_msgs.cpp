//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#include <ublox_snout/serialization/ublox_snout_msgs.h>

template <typename T>
std::vector<std::pair<uint8_t,uint8_t> > ublox_snout::Message<T>::keys_;

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::ATT, 
                      ublox_snout_msgs, NavATT);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::CLOCK, 
                      ublox_snout_msgs, NavCLOCK);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::DGPS, 
                      ublox_snout_msgs, NavDGPS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::DOP, 
                      ublox_snout_msgs, NavDOP);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::POSECEF, 
                      ublox_snout_msgs, NavPOSECEF);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::POSLLH, 
                      ublox_snout_msgs, NavPOSLLH);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, 
                      ublox_snout_msgs::Message::NAV::RELPOSNED, 
                      ublox_snout_msgs, 
                      NavRELPOSNED);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::SBAS, 
                      ublox_snout_msgs, NavSBAS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::SOL, 
                      ublox_snout_msgs, NavSOL);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::PVT, 
                      ublox_snout_msgs, NavPVT);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::PVT, 
                      ublox_snout_msgs, NavPVT7);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::SAT, 
                      ublox_snout_msgs, NavSAT);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::STATUS, 
                      ublox_snout_msgs, NavSTATUS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::SVIN, 
                      ublox_snout_msgs, NavSVIN);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::SVINFO, 
                      ublox_snout_msgs, NavSVINFO);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::TIMEGPS, 
                      ublox_snout_msgs, NavTIMEGPS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::TIMEUTC, 
                      ublox_snout_msgs, NavTIMEUTC);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::VELECEF, 
                      ublox_snout_msgs, NavVELECEF);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::NAV, ublox_snout_msgs::Message::NAV::VELNED, 
                      ublox_snout_msgs, NavVELNED);

// ACK messages are declared differently because they both have the same 
// protocol, so only 1 ROS message is used
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::ACK, ublox_snout_msgs::Message::ACK::NACK, 
                      ublox_snout_msgs, Ack);
DECLARE_UBLOX_MESSAGE_ID(ublox_snout_msgs::Class::ACK, ublox_snout_msgs::Message::ACK::ACK, 
                      ublox_snout_msgs, Ack, ACK);

// INF messages are declared differently because they all have the same 
// protocol, so only 1 ROS message is used. DECLARE_UBLOX_MESSAGE can only
// be called once, and DECLARE_UBLOX_MESSAGE_ID is called for the following
// messages
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::INF, ublox_snout_msgs::Message::INF::ERROR, 
                      ublox_snout_msgs, Inf);
DECLARE_UBLOX_MESSAGE_ID(ublox_snout_msgs::Class::INF, 
                         ublox_snout_msgs::Message::INF::WARNING, 
                         ublox_snout_msgs, Inf, WARNING);
DECLARE_UBLOX_MESSAGE_ID(ublox_snout_msgs::Class::INF, 
                         ublox_snout_msgs::Message::INF::NOTICE, 
                         ublox_snout_msgs, Inf, NOTICE);
DECLARE_UBLOX_MESSAGE_ID(ublox_snout_msgs::Class::INF, 
                         ublox_snout_msgs::Message::INF::TEST, 
                         ublox_snout_msgs, Inf, TEST);
DECLARE_UBLOX_MESSAGE_ID(ublox_snout_msgs::Class::INF, 
                         ublox_snout_msgs::Message::INF::DEBUG, 
                         ublox_snout_msgs, Inf, DEBUG);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::ALM, 
                      ublox_snout_msgs, RxmALM);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::EPH, 
                      ublox_snout_msgs, RxmEPH);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::RAW, 
                      ublox_snout_msgs, RxmRAW);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::RAWX, 
                      ublox_snout_msgs, RxmRAWX);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::RTCM, 
                      ublox_snout_msgs, RxmRTCM);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::SFRB, 
                      ublox_snout_msgs, RxmSFRB);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::SFRBX, 
                      ublox_snout_msgs, RxmSFRBX);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::RXM, ublox_snout_msgs::Message::RXM::SVSI, 
                      ublox_snout_msgs, RxmSVSI);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::ANT, 
                      ublox_snout_msgs, CfgANT);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::CFG, 
                      ublox_snout_msgs, CfgCFG);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::DAT, 
                      ublox_snout_msgs, CfgDAT);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::DGNSS, 
                      ublox_snout_msgs, CfgDGNSS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::GNSS, 
                      ublox_snout_msgs, CfgGNSS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::HNR,
                      ublox_snout_msgs, CfgHNR);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::INF,
                      ublox_snout_msgs, CfgINF);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::MSG, 
                      ublox_snout_msgs, CfgMSG);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::NAV5, 
                      ublox_snout_msgs, CfgNAV5);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::NAVX5, 
                      ublox_snout_msgs, CfgNAVX5);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::NMEA, 
                      ublox_snout_msgs, CfgNMEA);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::NMEA, 
                      ublox_snout_msgs, CfgNMEA6);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::NMEA, 
                      ublox_snout_msgs, CfgNMEA7);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::PRT, 
                      ublox_snout_msgs, CfgPRT);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::RATE, 
                      ublox_snout_msgs, CfgRATE);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::RST, 
                      ublox_snout_msgs, CfgRST);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::TMODE3, 
                      ublox_snout_msgs, CfgTMODE3);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::CFG, ublox_snout_msgs::Message::CFG::USB, 
                      ublox_snout_msgs, CfgUSB);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::UPD, ublox_snout_msgs::Message::UPD::SOS, 
                      ublox_snout_msgs, UpdSOS);
// SOS and SOS_Ack have the same message ID, but different lengths
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::UPD, ublox_snout_msgs::Message::UPD::SOS, 
                      ublox_snout_msgs, UpdSOS_Ack);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::MON, ublox_snout_msgs::Message::MON::GNSS, 
                      ublox_snout_msgs, MonGNSS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::MON, ublox_snout_msgs::Message::MON::HW, 
                      ublox_snout_msgs, MonHW);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::MON, ublox_snout_msgs::Message::MON::HW, 
                      ublox_snout_msgs, MonHW6);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::MON, ublox_snout_msgs::Message::MON::VER, 
                      ublox_snout_msgs, MonVER);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::AID, ublox_snout_msgs::Message::AID::ALM, 
                      ublox_snout_msgs, AidALM);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::AID, ublox_snout_msgs::Message::AID::EPH, 
                      ublox_snout_msgs, AidEPH);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::AID, ublox_snout_msgs::Message::AID::HUI, 
                      ublox_snout_msgs, AidHUI);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::ESF, ublox_snout_msgs::Message::ESF::INS,
                      ublox_snout_msgs, EsfINS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::ESF, ublox_snout_msgs::Message::ESF::MEAS, 
                      ublox_snout_msgs, EsfMEAS);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::ESF, ublox_snout_msgs::Message::ESF::RAW, 
                      ublox_snout_msgs, EsfRAW);
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::ESF, ublox_snout_msgs::Message::ESF::STATUS, 
                      ublox_snout_msgs, EsfSTATUS);


DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::MGA, ublox_snout_msgs::Message::MGA::GAL, 
                      ublox_snout_msgs, MgaGAL);

DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::HNR, ublox_snout_msgs::Message::HNR::PVT, 
                      ublox_snout_msgs, HnrPVT);

// TIM messages
DECLARE_UBLOX_MESSAGE(ublox_snout_msgs::Class::TIM, ublox_snout_msgs::Message::TIM::TM2,
		      ublox_snout_msgs, TimTM2);

