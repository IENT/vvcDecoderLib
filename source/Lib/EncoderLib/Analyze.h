/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Analyze.h
    \brief    encoder analyzer class (header)
*/

#ifndef __ANALYZE__
#define __ANALYZE__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include "CommonLib/CommonDef.h"
#include "CommonLib/ChromaFormat.h"
#include "math.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if ENABLE_QPA
 #define FRAME_WEIGHTING 0 // WPSNR temporal weighting according to hierarchical coding structure; only for GOP size 16
#endif

/// encoder analyzer class
class Analyze
{
private:
  Double    m_dPSNRSum[MAX_NUM_COMPONENT];
  Double    m_dAddBits;
  UInt      m_uiNumPic;
  Double    m_dFrmRate; //--CFG_KDY
  Double    m_MSEyuvframe[MAX_NUM_COMPONENT]; // sum of MSEs
#if ENABLE_QPA && FRAME_WEIGHTING
  double    m_sumWSSD[MAX_NUM_COMPONENT];   // weighted SSDs
  double    m_sumW;
#endif

public:
  virtual ~Analyze()  {}
  Analyze() { clear(); }

  Void  addResult( Double psnr[MAX_NUM_COMPONENT], Double bits, const Double MSEyuvframe[MAX_NUM_COMPONENT])
  {
    m_dAddBits  += bits;
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      m_dPSNRSum[i] += psnr[i];
      m_MSEyuvframe[i] += MSEyuvframe[i];
    }

    m_uiNumPic++;
  }
#if ENABLE_QPA
 #if FRAME_WEIGHTING
  Void    addWeightedSSD(const double dWeightedSSD, const ComponentID compID) { m_sumWSSD[compID] += dWeightedSSD; }
  Void    addWeight     (const double dWeight) { m_sumW += dWeight; }
  Double  getWPSNR      (const ComponentID compID) const { return (m_sumWSSD[compID] > 0.0 ? 10.0 * log10(m_sumW / m_sumWSSD[compID]) : 999.99); }
 #else
  Double  getWPSNR      (const ComponentID compID) const { return m_dPSNRSum[compID] / (double)m_uiNumPic; }
 #endif
#endif
  Double  getPsnr(ComponentID compID) const { return  m_dPSNRSum[compID];  }
  Double  getBits()                   const { return  m_dAddBits;   }
  Void    setBits(Double numBits)     { m_dAddBits = numBits; }
  UInt    getNumPic()                 const { return  m_uiNumPic;   }

  Void    setFrmRate  (Double dFrameRate) { m_dFrmRate = dFrameRate; } //--CFG_KDY
  Void    clear()
  {
    m_dAddBits = 0;
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      m_dPSNRSum[i] = 0;
      m_MSEyuvframe[i] = 0;
#if ENABLE_QPA && FRAME_WEIGHTING
      m_sumWSSD[i] = 0;
#endif
    }
#if ENABLE_QPA && FRAME_WEIGHTING
    m_sumW = 0;
#endif
    m_uiNumPic = 0;
  }


  Void calculateCombinedValues(const ChromaFormat chFmt, Double &PSNRyuv, Double &MSEyuv, const BitDepths &bitDepths)
  {
    MSEyuv    = 0;
    Int scale = 0;

    Int maximumBitDepth = bitDepths.recon[CHANNEL_TYPE_LUMA];
    for (UInt channelTypeIndex = 1; channelTypeIndex < MAX_NUM_CHANNEL_TYPE; channelTypeIndex++)
    {
      if (bitDepths.recon[channelTypeIndex] > maximumBitDepth)
      {
        maximumBitDepth = bitDepths.recon[channelTypeIndex];
      }
    }

#if ENABLE_QPA
    const UInt maxval                = /*useWPSNR ? (1 << maximumBitDepth) - 1 :*/ 255 << (maximumBitDepth - 8); // fix with WPSNR: 1023 (4095) instead of 1020 (4080) for bit depth 10 (12)
#else
    const UInt maxval                = 255 << (maximumBitDepth - 8);
#endif
    const UInt numberValidComponents = getNumberValidComponents(chFmt);

    for (UInt comp=0; comp<numberValidComponents; comp++)
    {
      const ComponentID compID        = ComponentID(comp);
      const UInt        csx           = getComponentScaleX(compID, chFmt);
      const UInt        csy           = getComponentScaleY(compID, chFmt);
      const Int         scaleChan     = (4>>(csx+csy));
      const UInt        bitDepthShift = 2 * (maximumBitDepth - bitDepths.recon[toChannelType(compID)]); //*2 because this is a squared number

      const Double      channelMSE    = (m_MSEyuvframe[compID] * Double(1 << bitDepthShift)) / Double(getNumPic());

      scale  += scaleChan;
      MSEyuv += scaleChan * channelMSE;
    }

    MSEyuv /= Double(scale);  // i.e. divide by 6 for 4:2:0, 8 for 4:2:2 etc.
    PSNRyuv = (MSEyuv == 0) ? 999.99 : 10.0 * log10((maxval * maxval) / MSEyuv);
  }


#if ENABLE_QPA || WCG_WPSNR
  Void    printOut ( TChar cDelim, const ChromaFormat chFmt, const Bool printMSEBasedSNR, const Bool printSequenceMSE, const BitDepths &bitDepths, const bool useWPSNR = false )
#else
  Void    printOut ( TChar cDelim, const ChromaFormat chFmt, const Bool printMSEBasedSNR, const Bool printSequenceMSE, const BitDepths &bitDepths )
#endif
  {
#if !WCG_WPSNR
    MsgLevel e_msg_level = cDelim == 'a' ? INFO: DETAILS;
#else
    MsgLevel e_msg_level = (cDelim == 'a') || (cDelim == 'w') ? INFO : DETAILS;
#endif
    Double dFps     =   m_dFrmRate; //--CFG_KDY
    Double dScale   = dFps / 1000 / (Double)m_uiNumPic;

    Double MSEBasedSNR[MAX_NUM_COMPONENT];
    if (printMSEBasedSNR)
    {
      for (UInt componentIndex = 0; componentIndex < MAX_NUM_COMPONENT; componentIndex++)
      {
        const ComponentID compID = ComponentID(componentIndex);

        if (getNumPic() == 0)
        {
          MSEBasedSNR[compID] = 0 * dScale; // this is the same calculation that will be evaluated for any other statistic when there are no frames (it should result in NaN). We use it here so all the output is consistent.
        }
        else
        {
#if ENABLE_QPA
          const UInt maxval = /*useWPSNR ? (1 << bitDepths.recon[toChannelType(compID)]) - 1 :*/ 255 << (bitDepths.recon[toChannelType(compID)] - 8); // fix with WPSNR: 1023 (4095) instead of 1020 (4080) for bit depth 10 (12)
#else
          //NOTE: this is not the true maximum value for any bitDepth other than 8. It comes from the original HM PSNR calculation
          const UInt maxval = 255 << (bitDepths.recon[toChannelType(compID)] - 8);
#endif
          const Double MSE  = m_MSEyuvframe[compID];

          MSEBasedSNR[compID] = (MSE == 0) ? 999.99 : 10.0 * log10((maxval * maxval) / (MSE / (Double)getNumPic()));
        }
      }
    }

    switch (chFmt)
    {
      case CHROMA_400:
        if (printMSEBasedSNR)
        {
#if ENABLE_QPA || WCG_WPSNR
          if (useWPSNR) {
            msg( e_msg_level, "         \tTotal Frames |   "   "Bitrate     "  "Y-WPSNR" );
          } else
#endif
          msg( e_msg_level, "         \tTotal Frames |   "   "Bitrate     "  "Y-PSNR" );

          if (printSequenceMSE)
          {
            msg( e_msg_level, "    Y-MSE\n" );
          }
          else
          {
            msg( e_msg_level, "\n");
          }

          //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
          msg( e_msg_level, "Average: \t %8d    %c "          "%12.4lf  "    "%8.4lf",
                 getNumPic(), cDelim,
                 getBits() * dScale,
#if ENABLE_QPA
                 useWPSNR ? getWPSNR(COMPONENT_Y) :
#endif
                 getPsnr(COMPONENT_Y) / (Double)getNumPic() );

          if (printSequenceMSE)
          {
            msg( e_msg_level, "  %8.4lf\n", m_MSEyuvframe[COMPONENT_Y] / (Double)getNumPic() );
          }
          else
          {
            msg( e_msg_level, "\n");
          }

          msg( e_msg_level, "From MSE:\t %8d    %c "          "%12.4lf  "    "%8.4lf\n",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 MSEBasedSNR[COMPONENT_Y] );
        }
        else
        {
#if ENABLE_QPA || WCG_WPSNR
          if (useWPSNR) {
            msg( e_msg_level, "\tTotal Frames |   "   "Bitrate     "  "Y-WPSNR" );
          } else
#endif
          msg( e_msg_level, "\tTotal Frames |   "   "Bitrate     "  "Y-PSNR" );

          if (printSequenceMSE)
          {
            msg( e_msg_level, "    Y-MSE\n" );
          }
          else
          {
            msg( e_msg_level, "\n");
          }

          //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
          msg( e_msg_level, "\t %8d    %c "          "%12.4lf  "    "%8.4lf",
                 getNumPic(), cDelim,
                 getBits() * dScale,
#if ENABLE_QPA
                 useWPSNR ? getWPSNR(COMPONENT_Y) :
#endif
                 getPsnr(COMPONENT_Y) / (Double)getNumPic() );

          if (printSequenceMSE)
          {
            msg( e_msg_level, "  %8.4lf\n", m_MSEyuvframe[COMPONENT_Y] / (Double)getNumPic() );
          }
          else
          {
            msg( e_msg_level, "\n");
          }
        }
        break;
      case CHROMA_420:
      case CHROMA_422:
      case CHROMA_444:
        {
          Double PSNRyuv = MAX_DOUBLE;
          Double MSEyuv  = MAX_DOUBLE;

          calculateCombinedValues(chFmt, PSNRyuv, MSEyuv, bitDepths);

          if (printMSEBasedSNR)
          {
#if ENABLE_QPA || WCG_WPSNR
            if (useWPSNR) {
              msg( e_msg_level, "         \tTotal Frames |   "   "Bitrate     "  "Y-WPSNR   "  "U-WPSNR   "  "V-WPSNR   "  "YUV-WPSNR" );
            } else
#endif
            msg( e_msg_level, "         \tTotal Frames |   "   "Bitrate     "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR " );

            if (printSequenceMSE)
            {
              msg( e_msg_level, " Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" );
            }
            else
            {
              msg( e_msg_level, "\n");
            }

            //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
            msg( e_msg_level, "Average: \t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
                   getNumPic(), cDelim,
                   getBits() * dScale,
#if ENABLE_QPA
                   useWPSNR ? getWPSNR(COMPONENT_Y ) :
#endif
                   getPsnr(COMPONENT_Y ) / (Double)getNumPic(),
#if ENABLE_QPA
                   useWPSNR ? getWPSNR(COMPONENT_Cb) :
#endif
                   getPsnr(COMPONENT_Cb) / (Double)getNumPic(),
#if ENABLE_QPA
                   useWPSNR ? getWPSNR(COMPONENT_Cr) :
#endif
                   getPsnr(COMPONENT_Cr) / (Double)getNumPic(),
                   PSNRyuv );

            if (printSequenceMSE)
            {
              msg( e_msg_level, "  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                     m_MSEyuvframe[COMPONENT_Y ] / (Double)getNumPic(),
                     m_MSEyuvframe[COMPONENT_Cb] / (Double)getNumPic(),
                     m_MSEyuvframe[COMPONENT_Cr] / (Double)getNumPic(),
                     MSEyuv );
            }
            else
            {
              msg( e_msg_level, "\n");
            }

            msg( e_msg_level, "From MSE:\t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   MSEBasedSNR[COMPONENT_Y ],
                   MSEBasedSNR[COMPONENT_Cb],
                   MSEBasedSNR[COMPONENT_Cr],
                   PSNRyuv );
          }
          else
          {
#if ENABLE_QPA || WCG_WPSNR
            if (useWPSNR) {
              msg( e_msg_level, "\tTotal Frames |   "   "Bitrate     "  "Y-WPSNR   "  "U-WPSNR   "  "V-WPSNR   "  "YUV-WPSNR" );
            } else
#endif
            msg( e_msg_level, "\tTotal Frames |   "   "Bitrate     "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR " );

            if (printSequenceMSE)
            {
              msg( e_msg_level, " Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" );
            }
            else
            {
              msg( e_msg_level, "\n");
            }

            //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
            msg( e_msg_level, "\t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
                   getNumPic(), cDelim,
                   getBits() * dScale,
#if ENABLE_QPA
                   useWPSNR ? getWPSNR(COMPONENT_Y ) :
#endif
                   getPsnr(COMPONENT_Y ) / (Double)getNumPic(),
#if ENABLE_QPA
                   useWPSNR ? getWPSNR(COMPONENT_Cb) :
#endif
                   getPsnr(COMPONENT_Cb) / (Double)getNumPic(),
#if ENABLE_QPA
                   useWPSNR ? getWPSNR(COMPONENT_Cr) :
#endif
                   getPsnr(COMPONENT_Cr) / (Double)getNumPic(),
                   PSNRyuv );

            if (printSequenceMSE)
            {
              msg( e_msg_level, "  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                     m_MSEyuvframe[COMPONENT_Y ] / (Double)getNumPic(),
                     m_MSEyuvframe[COMPONENT_Cb] / (Double)getNumPic(),
                     m_MSEyuvframe[COMPONENT_Cr] / (Double)getNumPic(),
                     MSEyuv );
            }
            else
            {
              msg( e_msg_level, "\n");
            }
          }
        }
        break;
      default:
        msg( ERROR, "Unknown format during print out\n");
        exit(1);
        break;
    }
  }


  Void    printSummary(const ChromaFormat chFmt, const Bool printSequenceMSE, const BitDepths &bitDepths, const std::string &sFilename)
  {
    FILE* pFile = fopen (sFilename.c_str(), "at");

    Double dFps     =   m_dFrmRate; //--CFG_KDY
    Double dScale   = dFps / 1000 / (Double)m_uiNumPic;
    switch (chFmt)
    {
      case CHROMA_400:
        fprintf(pFile, "%f\t %f\n",
            getBits() * dScale,
            getPsnr(COMPONENT_Y) / (Double)getNumPic() );
        break;
      case CHROMA_420:
      case CHROMA_422:
      case CHROMA_444:
        {
          Double PSNRyuv = MAX_DOUBLE;
          Double MSEyuv  = MAX_DOUBLE;

          calculateCombinedValues(chFmt, PSNRyuv, MSEyuv, bitDepths);

          fprintf(pFile, "%f\t %f\t %f\t %f\t %f",
              getBits() * dScale,
              getPsnr(COMPONENT_Y ) / (Double)getNumPic(),
              getPsnr(COMPONENT_Cb) / (Double)getNumPic(),
              getPsnr(COMPONENT_Cr) / (Double)getNumPic(),
              PSNRyuv );

          if (printSequenceMSE)
          {
            fprintf(pFile, "\t %f\t %f\t %f\t %f\n",
                m_MSEyuvframe[COMPONENT_Y ] / (Double)getNumPic(),
                m_MSEyuvframe[COMPONENT_Cb] / (Double)getNumPic(),
                m_MSEyuvframe[COMPONENT_Cr] / (Double)getNumPic(),
                MSEyuv );
          }
          else
          {
            fprintf(pFile, "\n");
          }

          break;
        }

      default:
          msg( ERROR, "Unknown format during print out\n");
          exit(1);
          break;
    }

    fclose(pFile);
  }
};

extern Analyze             m_gcAnalyzeAll;
extern Analyze             m_gcAnalyzeI;
extern Analyze             m_gcAnalyzeP;
extern Analyze             m_gcAnalyzeB;
#if WCG_WPSNR
extern Analyze             m_gcAnalyzeWPSNR;
#endif
extern Analyze             m_gcAnalyzeAll_in;

//! \}

#endif // !defined(AFX_TENCANALYZE_H__C79BCAA2_6AC8_4175_A0FE_CF02F5829233__INCLUDED_)
