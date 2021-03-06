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

#include <ublox_dog/serialization/ublox_dog_msgs.h>

template <typename T>
std::vector<std::pair<uint8_t,uint8_t> > ublox_dog::Message<T>::keys_;

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::ATT, 
                      ublox_dog_msgs, NavATT);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::CLOCK, 
                      ublox_dog_msgs, NavCLOCK);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::DGPS, 
                      ublox_dog_msgs, NavDGPS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::DOP, 
                      ublox_dog_msgs, NavDOP);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::POSECEF, 
                      ublox_dog_msgs, NavPOSECEF);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::POSLLH, 
                      ublox_dog_msgs, NavPOSLLH);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, 
                      ublox_dog_msgs::Message::NAV::RELPOSNED, 
                      ublox_dog_msgs, 
                      NavRELPOSNED);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::SBAS, 
                      ublox_dog_msgs, NavSBAS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::SOL, 
                      ublox_dog_msgs, NavSOL);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::PVT, 
                      ublox_dog_msgs, NavPVT);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::PVT, 
                      ublox_dog_msgs, NavPVT7);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::SAT, 
                      ublox_dog_msgs, NavSAT);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::STATUS, 
                      ublox_dog_msgs, NavSTATUS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::SVIN, 
                      ublox_dog_msgs, NavSVIN);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::SVINFO, 
                      ublox_dog_msgs, NavSVINFO);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::TIMEGPS, 
                      ublox_dog_msgs, NavTIMEGPS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::TIMEUTC, 
                      ublox_dog_msgs, NavTIMEUTC);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::VELECEF, 
                      ublox_dog_msgs, NavVELECEF);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::NAV, ublox_dog_msgs::Message::NAV::VELNED, 
                      ublox_dog_msgs, NavVELNED);

// ACK messages are declared differently because they both have the same 
// protocol, so only 1 ROS message is used
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::ACK, ublox_dog_msgs::Message::ACK::NACK, 
                      ublox_dog_msgs, Ack);
DECLARE_UBLOX_MESSAGE_ID(ublox_dog_msgs::Class::ACK, ublox_dog_msgs::Message::ACK::ACK, 
                      ublox_dog_msgs, Ack, ACK);

// INF messages are declared differently because they all have the same 
// protocol, so only 1 ROS message is used. DECLARE_UBLOX_MESSAGE can only
// be called once, and DECLARE_UBLOX_MESSAGE_ID is called for the following
// messages
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::INF, ublox_dog_msgs::Message::INF::ERROR, 
                      ublox_dog_msgs, Inf);
DECLARE_UBLOX_MESSAGE_ID(ublox_dog_msgs::Class::INF, 
                         ublox_dog_msgs::Message::INF::WARNING, 
                         ublox_dog_msgs, Inf, WARNING);
DECLARE_UBLOX_MESSAGE_ID(ublox_dog_msgs::Class::INF, 
                         ublox_dog_msgs::Message::INF::NOTICE, 
                         ublox_dog_msgs, Inf, NOTICE);
DECLARE_UBLOX_MESSAGE_ID(ublox_dog_msgs::Class::INF, 
                         ublox_dog_msgs::Message::INF::TEST, 
                         ublox_dog_msgs, Inf, TEST);
DECLARE_UBLOX_MESSAGE_ID(ublox_dog_msgs::Class::INF, 
                         ublox_dog_msgs::Message::INF::DEBUG, 
                         ublox_dog_msgs, Inf, DEBUG);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::ALM, 
                      ublox_dog_msgs, RxmALM);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::EPH, 
                      ublox_dog_msgs, RxmEPH);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::RAW, 
                      ublox_dog_msgs, RxmRAW);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::RAWX, 
                      ublox_dog_msgs, RxmRAWX);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::RTCM, 
                      ublox_dog_msgs, RxmRTCM);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::SFRB, 
                      ublox_dog_msgs, RxmSFRB);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::SFRBX, 
                      ublox_dog_msgs, RxmSFRBX);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::RXM, ublox_dog_msgs::Message::RXM::SVSI, 
                      ublox_dog_msgs, RxmSVSI);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::ANT, 
                      ublox_dog_msgs, CfgANT);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::CFG, 
                      ublox_dog_msgs, CfgCFG);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::DAT, 
                      ublox_dog_msgs, CfgDAT);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::DGNSS, 
                      ublox_dog_msgs, CfgDGNSS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::GNSS, 
                      ublox_dog_msgs, CfgGNSS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::HNR,
                      ublox_dog_msgs, CfgHNR);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::INF,
                      ublox_dog_msgs, CfgINF);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::MSG, 
                      ublox_dog_msgs, CfgMSG);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::NAV5, 
                      ublox_dog_msgs, CfgNAV5);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::NAVX5, 
                      ublox_dog_msgs, CfgNAVX5);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::NMEA, 
                      ublox_dog_msgs, CfgNMEA);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::NMEA, 
                      ublox_dog_msgs, CfgNMEA6);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::NMEA, 
                      ublox_dog_msgs, CfgNMEA7);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::PRT, 
                      ublox_dog_msgs, CfgPRT);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::RATE, 
                      ublox_dog_msgs, CfgRATE);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::RST, 
                      ublox_dog_msgs, CfgRST);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::TMODE3, 
                      ublox_dog_msgs, CfgTMODE3);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::CFG, ublox_dog_msgs::Message::CFG::USB, 
                      ublox_dog_msgs, CfgUSB);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::UPD, ublox_dog_msgs::Message::UPD::SOS, 
                      ublox_dog_msgs, UpdSOS);
// SOS and SOS_Ack have the same message ID, but different lengths
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::UPD, ublox_dog_msgs::Message::UPD::SOS, 
                      ublox_dog_msgs, UpdSOS_Ack);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::MON, ublox_dog_msgs::Message::MON::GNSS, 
                      ublox_dog_msgs, MonGNSS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::MON, ublox_dog_msgs::Message::MON::HW, 
                      ublox_dog_msgs, MonHW);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::MON, ublox_dog_msgs::Message::MON::HW, 
                      ublox_dog_msgs, MonHW6);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::MON, ublox_dog_msgs::Message::MON::VER, 
                      ublox_dog_msgs, MonVER);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::AID, ublox_dog_msgs::Message::AID::ALM, 
                      ublox_dog_msgs, AidALM);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::AID, ublox_dog_msgs::Message::AID::EPH, 
                      ublox_dog_msgs, AidEPH);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::AID, ublox_dog_msgs::Message::AID::HUI, 
                      ublox_dog_msgs, AidHUI);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::ESF, ublox_dog_msgs::Message::ESF::INS,
                      ublox_dog_msgs, EsfINS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::ESF, ublox_dog_msgs::Message::ESF::MEAS, 
                      ublox_dog_msgs, EsfMEAS);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::ESF, ublox_dog_msgs::Message::ESF::RAW, 
                      ublox_dog_msgs, EsfRAW);
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::ESF, ublox_dog_msgs::Message::ESF::STATUS, 
                      ublox_dog_msgs, EsfSTATUS);


DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::MGA, ublox_dog_msgs::Message::MGA::GAL, 
                      ublox_dog_msgs, MgaGAL);

DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::HNR, ublox_dog_msgs::Message::HNR::PVT, 
                      ublox_dog_msgs, HnrPVT);

// TIM messages
DECLARE_UBLOX_MESSAGE(ublox_dog_msgs::Class::TIM, ublox_dog_msgs::Message::TIM::TM2,
		      ublox_dog_msgs, TimTM2);

