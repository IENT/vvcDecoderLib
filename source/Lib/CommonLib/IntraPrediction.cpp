/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar IntraPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    20, //   1xn
    20, //   2xn
    20, //   4xn
    14, //   8xn
    2,  //  16xn
    0,  //  32xn
#if HM_MDIS_AS_IN_JEM
    20, //  64xn
#else
    0,  //  64xn
#endif
    0,  // 128xn
  },
  { // Chroma
    40, //   1xn
    40, //   2xn
    40, //   4xn
    28, //   8xn
    4,  //  16xn
    0,  //  32xn
#if HM_MDIS_AS_IN_JEM
    40, //  64xn
#else
    0,  //  64xn
#endif
    0,  // 128xn
  }
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
{
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      m_piYuvExt[ch][buf] = nullptr;
    }
  }

  m_piTemp = nullptr;
#if JEM_TOOLS&&!JVET_K0190

  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
    m_pLumaRecBufferMul[i] = nullptr;
  }
#endif
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

Void IntraPrediction::destroy()
{
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      delete[] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
#if JEM_TOOLS&&!JVET_K0190

  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
    if (m_pLumaRecBufferMul[i])
    {
      delete[] m_pLumaRecBufferMul[i];
      m_pLumaRecBufferMul[i] = nullptr;
    }
  }
#endif
}

Void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1) * (MAX_CU_SIZE * 2 + 1);

    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[m_iYuvExtSize];
      }
    }
  }

#if JEM_TOOLS||JVET_K0190
  Int shift = bitDepthY + 4;
  for (Int i = 32; i < 64; i++)
  {
    m_auShiftLM[i - 32] = ((1 << shift) + i / 2) / i;
  }
#endif
  if (m_piTemp == nullptr)
  {
#if JVET_K0190
    m_piTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
#else
#if JEM_TOOLS
    const int MMLM_Lines = std::max<int>( 1, MMLM_SAMPLE_NEIGHBOR_LINES );
    m_piTemp = new Pel[( MAX_CU_SIZE + MMLM_Lines ) * ( MAX_CU_SIZE + MMLM_Lines )];
#else
#endif
    m_piTemp = new Pel[ MAX_CU_SIZE * MAX_CU_SIZE ];
#endif
  }
#if JEM_TOOLS&&!JVET_K0190
  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
    if (!m_pLumaRecBufferMul[i])
    {
      m_pLumaRecBufferMul[i] = new Pel[ (MAX_CU_SIZE + MMLM_SAMPLE_NEIGHBOR_LINES) * (MAX_CU_SIZE + MMLM_SAMPLE_NEIGHBOR_LINES) ];
    }
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const Size &dstSize )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  int idx, sum = 0;
  Pel dcVal;
  const int width  = dstSize.width;
  const int height = dstSize.height;
#if JVET_K0122
  const auto denom     = (width == height) ? (width << 1) : std::max(width,height);
  const auto divShift  = g_aucLog2[denom];
  const auto divOffset = (denom >> 1);

  if ( width >= height )
  {
#endif
  for( idx = 0; idx < width; idx++ )
  {
    sum += pSrc.at( 1 + idx, 0 );
  }
#if JVET_K0122
  }
  if ( width <= height )
  {   
#endif
  for( idx = 0; idx < height; idx++ )
  {
    sum += pSrc.at( 0, 1 + idx );
  }  
#if JVET_K0122
  }

  dcVal = (sum + divOffset) >> divShift;
#else
  dcVal = ( sum + ( ( width + height ) >> 1 ) ) / ( width + height );
#endif
  return dcVal;
}

#if JVET_K0500_WAIP
  int IntraPrediction::getWideAngle( int width, int height, int predMode )
  {
    if ( predMode > DC_IDX && predMode <= VDIA_IDX )
    {
      int modeShift = (std::min(2, abs(g_aucLog2[width] - g_aucLog2[height])) << 2) + 2;
      if ( width > height && predMode < 2 + modeShift )
      {
        predMode += (VDIA_IDX - 1);
      }
      else if ( height > width && predMode > VDIA_IDX - modeShift )
      {
        predMode -= (VDIA_IDX - 1);
      }
    }
    return predMode;
  }

  void IntraPrediction::setReferenceArrayLengths( const CompArea &area )
  {
    // set Top and Left reference samples length
    const int  width    = area.width;
    const int  height   = area.height;
    int blockShapeRatio = std::min(2, abs(g_aucLog2[width] - g_aucLog2[height]));

    m_leftRefLength     = (height << 1);
    m_topRefLength      = (width << 1);
    if( width > height )
    {
      m_leftRefLength  += (width >> blockShapeRatio) - height + ((width + 31) >> 5);
    }
    else if( height > width )
    {
      m_topRefLength   += (height >> blockShapeRatio) - width + ((height + 31) >> 5);
    }

  }
#endif

void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples )
{
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const Int            iWidth       = piPred.width;
  const Int            iHeight      = piPred.height;
  const UInt           uiDirMode    = PU::getFinalIntraMode( pu, channelType );


  CHECK( g_aucLog2[iWidth] < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed" );
  CHECK( g_aucLog2[iWidth] > 7, "Size not allowed" );
  CHECK( iWidth != iHeight && !pu.cs->pcv->rectCUs, "Rectangular block are only allowed with QTBT" );

#if JVET_K0500_WAIP
  const int  srcStride  = m_topRefLength  + 1;
  const int  srcHStride = m_leftRefLength + 1;
#else
  const Int  srcStride = ( iWidth + iHeight + 1 );
#endif

#if JVET_K0063_PDPC_SIMP
  Pel *ptrSrc = getPredictorPtr(compID, useFilteredPredSamples);
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compID));

  switch (uiDirMode)
  {
#if JVET_K0500_WAIP
    case(PLANAR_IDX): xPredIntraPlanar(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, *pu.cs->sps); break;
    case(DC_IDX):     xPredIntraDc(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, false); break;
    default:          xPredIntraAng(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, false); break;
#else
    case(PLANAR_IDX): xPredIntraPlanar(CPelBuf(ptrSrc, srcStride, srcStride), piPred, *pu.cs->sps); break;
    case(DC_IDX):     xPredIntraDc(CPelBuf(ptrSrc, srcStride, srcStride), piPred, channelType, false); break;
    default:          xPredIntraAng(CPelBuf(ptrSrc, srcStride, srcStride), piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, false); break;
#endif
  }

  Bool pdpcCondition = (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX || uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
  if (pdpcCondition)
  {
    const CPelBuf srcBuf = CPelBuf(ptrSrc, srcStride, srcStride);
    PelBuf dstBuf = piPred;
    const int scale = ((g_aucLog2[iWidth] - 2 + g_aucLog2[iHeight] - 2 + 2) >> 2);
    CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

    if (uiDirMode == PLANAR_IDX)
    {
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top + (64 - wL - wT) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
    else if (uiDirMode == DC_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = (wL >> 4) + (wT >> 4);
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top - wTL * topLeft + (64 - wL - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
    else if (uiDirMode == HOR_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wTL = wT;
          dstBuf.at(x, y) = ClipPel((wT * top - wTL * topLeft + (64 - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
    else if (uiDirMode == VER_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = wL;
          dstBuf.at(x, y) = ClipPel((wL * left - wTL * topLeft + (64 - wL + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
  }
#else
#if HEVC_USE_HOR_VER_PREDFILTERING
  const Bool enableEdgeFilters = !(CU::isRDPCMEnabled( *pu.cu ) && pu.cu->transQuantBypass);
#endif
  Pel *ptrSrc = getPredictorPtr( compID, useFilteredPredSamples );

#if JEM_TOOLS
#if HM_PDPC_AS_IN_JEM //PM: reproducing JEM behavior (however, is boundary condition for QTBT off case really useful?)
  bool pdpcCondition = (pu.cs->sps->getSpsNext().isIntraPDPC() && pu.cu->pdpc && (pu.cs->pcv->rectCUs || (pu.cu->lumaPos().x && pu.cu->lumaPos().y))) || (pu.cs->sps->getSpsNext().isPlanarPDPC() && (uiDirMode == PLANAR_IDX));
#else
  bool pdpcCondition = ((pu.cs->sps->getSpsNext().isIntraPDPC()) && pu.cu->pdpc) || (pu.cs->sps->getSpsNext().isPlanarPDPC() && (uiDirMode == PLANAR_IDX));
#endif

  if( pdpcCondition )
  {
    int idxW = std::min( 4, (int)g_aucLog2[iWidth]  - 1 );
    int idxH = std::min( 4, (int)g_aucLog2[iHeight] - 1 );
    const int *pPdpcParWidth;
    const int *pPdpcParHeight;
    if( pu.cs->sps->getSpsNext().isPlanarPDPC() )
    {
      pPdpcParWidth = g_pdpcParam[idxW];
      pPdpcParHeight = g_pdpcParam[idxH];
    }
    else
    {
#if INTRA67_3MPM
      pPdpcParWidth  = g_pdpc_pred_param[idxW][uiDirMode];
      pPdpcParHeight = g_pdpc_pred_param[idxH][uiDirMode];
#else
      pPdpcParWidth = g_pdpc_pred_param[idxW][g_intraMode65to33AngMapping[uiDirMode]];
      pPdpcParHeight = g_pdpc_pred_param[idxH][g_intraMode65to33AngMapping[uiDirMode]];
#endif
    }
    const int *pPdpcParMain   = (iWidth < iHeight) ? pPdpcParHeight : pPdpcParWidth;

#if JVET_K0500_WAIP
    const int doubleSize = m_topRefLength;
    const int doubleHSize = m_leftRefLength;

    Pel* piRefVector = m_piTempRef + doubleHSize;
    Pel* piLowpRefer = m_piFiltRef + doubleHSize;
#else
    const int srcStride  = iWidth + iHeight + 1;
    const int doubleSize = iWidth + iHeight;

    Pel* piRefVector = m_piTempRef + doubleSize;
    Pel* piLowpRefer = m_piFiltRef + doubleSize;
#endif

    for( int j = 0; j <= doubleSize; j++ ) { piRefVector[ j] = ptrSrc[j]; }
#if JVET_K0500_WAIP
    for( int i = 1; i <= doubleHSize; i++ ) { piRefVector[-i] = ptrSrc[i*srcStride]; }
#else
    for( int i = 1; i <= doubleSize; i++ ) { piRefVector[-i] = ptrSrc[i*srcStride]; }
#endif

    if( pPdpcParMain[5] != 0 )
    {
      xReferenceFilter( 
#if JVET_K0500_WAIP
        doubleHSize,
#endif
        doubleSize, pPdpcParMain[4], pPdpcParMain[5], piRefVector, piLowpRefer );

      // copy filtered ref. samples back to ref. buffer
      for( int j = 0; j <= doubleSize; j++ ) { ptrSrc[j]           = piLowpRefer[ j]; }
#if JVET_K0500_WAIP
      for( int i = 1; i <= doubleHSize; i++ ) { ptrSrc[i*srcStride] = piLowpRefer[-i]; }
#else
      for( int i = 1; i <= doubleSize; i++ ) { ptrSrc[i*srcStride] = piLowpRefer[-i]; }
#endif
    }

    const ClpRng& clpRng( pu.cu->cs->slice->clpRng(compID) );

    switch( uiDirMode )
    {
#if JVET_K0500_WAIP
    case(PLANAR_IDX):   xPredIntraPlanar(CPelBuf( ptrSrc, srcStride, doubleHSize + 1 ), piPred, *pu.cs->sps);         break;
    case(DC_IDX):       xPredIntraDc    (CPelBuf( ptrSrc, srcStride, doubleHSize + 1 ), piPred, channelType, false);  break; // including DCPredFiltering
#if HEVC_USE_HOR_VER_PREDFILTERING
    default:            xPredIntraAng   (CPelBuf( ptrSrc, srcStride, doubleHSize + 1 ), piPred, channelType,
                                         uiDirMode, clpRng, enableEdgeFilters, *pu.cs->sps, false);                 break;
#else
    default:            xPredIntraAng   (CPelBuf( ptrSrc, srcStride, doubleHSize + 1 ), piPred, channelType, uiDirMode,
                                         pu.cs->slice->clpRng(compID), *pu.cs->sps, false);                         break;
#endif
#else
    case( PLANAR_IDX ): xPredIntraPlanar( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, *pu.cs->sps );         break;
    case( DC_IDX ):     xPredIntraDc    ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType, false );  break; // including DCPredFiltering
#if HEVC_USE_HOR_VER_PREDFILTERING
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType,
                                         uiDirMode, clpRng, enableEdgeFilters, *pu.cs->sps, false );             break;
#else
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType,
                                          uiDirMode, clpRng, *pu.cs->sps, false );             break;
#endif
#endif
    }

    if( pPdpcParMain[5] != 0 )
    {
      // copy unfiltered ref. samples back to ref. buffer for weighted prediction
      for( int j = 0; j <= doubleSize; j++ ) { ptrSrc[j]           = piRefVector[ j]; }
#if JVET_K0500_WAIP
      for( int i = 1; i <= doubleHSize; i++ ) { ptrSrc[i*srcStride] = piRefVector[-i]; }
#else
      for( int i = 1; i <= doubleSize; i++ ) { ptrSrc[i*srcStride] = piRefVector[-i]; }
#endif
    }

    int scale     = (g_aucLog2[iWidth] + g_aucLog2[iHeight] < 10) ? 0 : 1;
    int parShift  = 6; //normalization factor
    int parScale  = 1 << parShift;
    int parOffset = 1 << (parShift - 1);

    for( int y = 0; y < iHeight; y++ )
    {
      int shiftRow     = y >> scale;
      int coeff_Top    = pPdpcParHeight[2] >> shiftRow;
      int coeff_offset = pPdpcParHeight[3] >> shiftRow;

      for( int x = 0; x < iWidth; x++ )
      {
        int shiftCol      = x >> scale;
        int coeff_Left    =  pPdpcParWidth[0] >> shiftCol;
        int coeff_TopLeft = (pPdpcParWidth[1] >> shiftCol) + coeff_offset;
        int coeff_Cur     = parScale - coeff_Left - coeff_Top + coeff_TopLeft;

        int sampleVal = (coeff_Left* piRefVector[-y - 1] + coeff_Top * piRefVector[x + 1] - coeff_TopLeft * piRefVector[0] + coeff_Cur * piPred.at( x, y ) + parOffset) >> parShift;
        piPred.at( x, y ) = ClipPel( sampleVal, clpRng );
      }
    }
  }
  else
#endif
  {
    switch( uiDirMode )
    {
#if JVET_K0500_WAIP
    case(DC_IDX):       xPredIntraDc    ( CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType );            break; // including DCPredFiltering
    case(PLANAR_IDX):   xPredIntraPlanar( CPelBuf(ptrSrc, srcStride, srcHStride), piPred, *pu.cs->sps );            break;
#if HEVC_USE_HOR_VER_PREDFILTERING
    default:            xPredIntraAng   ( CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, uiDirMode,
                                          pu.cs->slice->clpRng(compID), enableEdgeFilters, *pu.cs->sps );           break;
#else
    default:            xPredIntraAng   ( CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, uiDirMode,
                                          pu.cs->slice->clpRng(compID), *pu.cs->sps );            break;
#endif
#else
    case( DC_IDX ):     xPredIntraDc    ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType );            break; // including DCPredFiltering
    case( PLANAR_IDX ): xPredIntraPlanar( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, *pu.cs->sps );            break;
#if HEVC_USE_HOR_VER_PREDFILTERING
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType, uiDirMode,
                                          pu.cs->slice->clpRng( compID ), enableEdgeFilters, *pu.cs->sps );          break;
#else
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, channelType, uiDirMode,
                                          pu.cs->slice->clpRng( compID ), *pu.cs->sps );          break;
#endif
#endif
    }
  }
#endif
}
#if JVET_K0190
Void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, Int intraDir)
{
  Int  iLumaStride = 0;
  PelBuf Temp;
  iLumaStride = MAX_CU_SIZE + 1;
  Temp = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  Int a, b, iShift;
  xGetLMParameters(pu, compID, chromaArea, a, b, iShift);

  ////// final prediction
  piPred.copyFrom(Temp);
  piPred.linearTransform(a, iShift, b, true, pu.cs->slice->clpRng(compID));
}
#else
#if JEM_TOOLS
Void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, Int intraDir)
{
  bool DO_ELM = false;
  if (pu.cs->sps->getSpsNext().isELMModeMMLM())
  {
    if (intraDir == MMLM_CHROMA_IDX)
    {
      DO_ELM = true;
    }
    if (pu.cs->sps->getSpsNext().isELMModeMFLM() && (intraDir >= LM_CHROMA_F1_IDX && intraDir < (LM_CHROMA_F1_IDX + LM_FILTER_NUM)))
    {
      DO_ELM = true;
    }
  }
  int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
  if(DO_ELM)
  {
    Pel *pLumaSaved=0;
    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      pLumaSaved = m_piTemp;

      if (intraDir >= LM_CHROMA_F1_IDX && intraDir < (LM_CHROMA_F1_IDX + LM_FILTER_NUM))
      {
        Int iLumaIdx = intraDir - LM_CHROMA_F1_IDX;
        m_piTemp = m_pLumaRecBufferMul[iLumaIdx];
      }
    }
    // LLS parameters estimation -->
    MMLM_parameter parameters[2];
    Int iGroupNum = 2;
    xGetMMLMParameters(pu, compID, chromaArea, iGroupNum, parameters);

    Int  iLumaStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
    PelBuf Temp = PelBuf(m_piTemp + (iLumaStride + 1) *MMLM_Lines, iLumaStride, Size(chromaArea));//MMLM_SAMPLE_NEIGHBOR_LINES;
    Pel  *pLuma = Temp.bufAt(0, 0);

    Pel*  pPred = piPred.bufAt(0, 0);
    Int  uiPredStride = piPred.stride;

    UInt uiCWidth = chromaArea.width;
    UInt uiCHeight = chromaArea.height;

    for (Int i = 0; i < uiCHeight; i++)
    {
      for (Int j = 0; j < uiCWidth; j++)
      {
        Int a, b, iShift;
        if (pLuma[j] <= parameters[0].Sup)
        {
          a = parameters[0].a;
          b = parameters[0].b;
          iShift = parameters[0].shift;
        }
        else
        {
          a = parameters[1].a;
          b = parameters[1].b;
          iShift = parameters[1].shift;
        }

        pPred[j] = ( Pel ) ClipPel( ( ( a * pLuma[j] ) >> iShift ) + b, pu.cs->slice->clpRng( compID ) );
      }

      pPred += uiPredStride;
      pLuma += iLumaStride;
    }

    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      m_piTemp = pLumaSaved;
    }
  }
  else
  {
    Int  iLumaStride = 0;
    PelBuf Temp;
    if (pu.cs->sps->getSpsNext().isELMModeMMLM())
    {
      iLumaStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
      Temp = PelBuf(m_piTemp + (iLumaStride + 1) *MMLM_Lines, iLumaStride, Size(chromaArea)); //MMLM_SAMPLE_NEIGHBOR_LINES;
    }
    else
    {
      iLumaStride = MAX_CU_SIZE + 1;
      Temp = PelBuf(m_piTemp + (iLumaStride + 1) * 1, iLumaStride, Size(chromaArea));
    }

    Int a, b, iShift;
    Int iPredType = 0;
    xGetLMParameters(pu, compID, chromaArea, iPredType, a, b, iShift);

    ////// final prediction
    piPred.copyFrom(Temp);
    piPred.linearTransform(a, iShift, b, true, pu.cs->slice->clpRng(compID));
  }
}

Void IntraPrediction::addCrossColorResi(const ComponentID compID, PelBuf &piPred, const TransformUnit &tu, const CPelBuf &pResiCb)
{
  const CompArea& chromaArea = tu.block(compID);

  Int a, b, iShift;

  const PredictionUnit& pu = *(tu.cs->getPU(chromaArea.pos(), toChannelType(compID)));
  xGetLMParameters(pu, compID, chromaArea, Int(1), a, b, iShift);

  Int offset = 1 << (iShift - 1);

  if (a >= 0)
  {
    return;
  }

  const ClpRng& clpRng( tu.cu->cs->slice->clpRng(compID) );

  Pel*  pPred = piPred.buf;
  const Pel*  pResi = pResiCb.bufAt(0, 0);
  UInt uiPredStride = piPred.stride;
  UInt uiResiStride = pResiCb.stride;


  for (UInt uiY = 0; uiY < chromaArea.height; uiY++)
  {
    for (UInt uiX = 0; uiX < chromaArea.width; uiX++)
    {
      pPred[uiX] = ClipPel( pPred[uiX] + ((pResi[uiX] * a + offset) >> iShift), clpRng);
    }
    pPred += uiPredStride;
    pResi += uiResiStride;
  }
}
#endif
#endif

Void IntraPrediction::xFilterGroup(Pel* pMulDst[], Int i, Pel const * const piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable)
{
  pMulDst[0][i] = (piSrc[1] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[1][i] = (piSrc[iRecStride] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[3][i] = (piSrc[0] + piSrc[1] + 1) >> 1;

  pMulDst[2][i] = (piSrc[0] + piSrc[1] + piSrc[iRecStride] + piSrc[iRecStride + 1] + 2) >> 2;

}



/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
Void IntraPrediction::xPredIntraPlanar( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps )
{
  const UInt width  = pDst.width;
  const UInt height = pDst.height;
  const UInt log2W  = g_aucLog2[ width ];
  const UInt log2H  = g_aucLog2[ height ];

  Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const UInt offset = width * height;

  // Get left and above reference column and row
  for( Int k = 0; k < width + 1; k++ )
  {
    topRow[k] = pSrc.at( k + 1, 0 );
  }

  for( Int k = 0; k < height + 1; k++ )
  {
    leftColumn[k] = pSrc.at( 0, k + 1 );
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight = topRow[width];

  for( Int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for( Int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const UInt finalShift = 1 + log2W + log2H;
  const UInt stride     = pDst.stride;
  Pel*       pred       = pDst.buf;
  for( Int y = 0; y < height; y++, pred += stride )
  {
    Int horPred = leftColumn[y];

    for( Int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = topRow[x];
      pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
    }
  }
}




Void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter )
{
  const Pel dcval = xGetPredValDc( pSrc, pDst );
  pDst.fill( dcval );

#if HEVC_USE_DC_PREDFILTERING
  if( enableBoundaryFilter )
  {
    xDCPredFiltering( pSrc, pDst, channelType );
  }
#endif
}

#if HEVC_USE_DC_PREDFILTERING
/** Function for filtering intra DC predictor. This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void IntraPrediction::xDCPredFiltering(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType)
{
  UInt iWidth = pDst.width;
  UInt iHeight = pDst.height;
  Int x, y;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst.at(0, 0) = (Pel)((pSrc.at(1, 0) + pSrc.at(0, 1) + 2 * pDst.at(0, 0) + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst.at(x, 0) = (Pel)((pSrc.at(x + 1, 0)  +  3 * pDst.at(x, 0) + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1; y < iHeight; y++ )
    {
      pDst.at(0, y) = (Pel)((pSrc.at(0, y + 1) + 3 * pDst.at(0, y) + 2) >> 2);
    }
  }

  return;
}
#endif

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
*
* This function derives the prediction samples for the angular mode based on the prediction direction indicated by
* the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
* the reference row above the block in the case of vertical prediction or displacement of the rightmost column
* of the block and reference column left from the block in the case of the horizontal prediction. The displacement
* is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
* the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
* from the extended main reference.
*/
//NOTE: Bit-Limit - 25-bit source
#if HEVC_USE_HOR_VER_PREDFILTERING
Void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const UInt dirMode, const ClpRng& clpRng, const Bool bEnableEdgeFilters, const SPS& sps, const bool enableBoundaryFilter )
#else
Void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const UInt dirMode, const ClpRng& clpRng, const SPS& sps, const bool enableBoundaryFilter )
#endif
{
  Int width =Int(pDst.width);
  Int height=Int(pDst.height);

  CHECK( !( dirMode > DC_IDX && dirMode < NUM_LUMA_MODE ), "Invalid intra dir" );

#if JVET_K0500_WAIP
  int              predMode           = getWideAngle(width, height, dirMode);
  const bool       bIsModeVer         = predMode >= DIA_IDX;
  const int        intraPredAngleMode = (bIsModeVer) ? predMode - VER_IDX : -(predMode - HOR_IDX);
#else
  const Bool       bIsModeVer         = (dirMode >= DIA_IDX);
  const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
#endif
  const Int        absAngMode         = abs(intraPredAngleMode);
  const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
#if HEVC_USE_HOR_VER_PREDFILTERING
  const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);
#endif

  // Set bitshifts and scale the angle parameter to block size

#if JVET_K0500_WAIP
  static const int angTable[27]    = { 0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32,   35,  39,  45,  49,  54,  60,  68,  79,  93, 114 };
  static const int invAngTable[27] = { 0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256,  234, 210, 182, 167, 152, 137, 120, 104,  88,  72 }; // (256 * 32) / Angle
#else
  static const Int angTable[17]    = { 0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32 };
  static const Int invAngTable[17] = { 0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256 }; // (256 * 32) / Angle
#endif

  Int invAngle                    = invAngTable[absAngMode];
  Int absAng                      = angTable   [absAngMode];
  Int intraPredAngle              = signAng * absAng;

  Pel* refMain;
  Pel* refSide;

  Pel  refAbove[2 * MAX_CU_SIZE + 1];
  Pel  refLeft [2 * MAX_CU_SIZE + 1];


  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for( Int x = 0; x < width + 1; x++ )
    {
      refAbove[x + height - 1] = pSrc.at( x, 0 );
    }
    for( Int y = 0; y < height + 1; y++ )
    {
      refLeft[y + width - 1] = pSrc.at( 0, y );
    }
    refMain = (bIsModeVer ? refAbove + height : refLeft  + width ) - 1;
    refSide = (bIsModeVer ? refLeft  + width  : refAbove + height) - 1;

    // Extend the Main reference to the left.
    Int invAngleSum    = 128;       // rounding for (shift by 8)
    const Int refMainOffsetPreScale = bIsModeVer ? height : width;
    for( Int k = -1; k > (refMainOffsetPreScale * intraPredAngle) >> 5; k-- )
    {
      invAngleSum += invAngle;
      refMain[k] = refSide[invAngleSum>>8];
    }
  }
  else
  {
#if JVET_K0500_WAIP
    for( int x = 0; x < m_topRefLength + 1; x++ )
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for( int y = 0; y < m_leftRefLength + 1; y++ )
    {
      refLeft[y]  = pSrc.at(0, y);
    }
#else
    for( Int x = 0; x < width + height + 1; x++ )
    {
      refAbove[x] = pSrc.at(x, 0);
      refLeft[x]  = pSrc.at(0, x);
    }
#endif
    refMain = bIsModeVer ? refAbove : refLeft ;
    refSide = bIsModeVer ? refLeft  : refAbove;
  }

  // swap width/height if we are doing a horizontal mode:
  Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
  const Int dstStride = bIsModeVer ? pDst.stride : MAX_CU_SIZE;
  Pel *pDstBuf = bIsModeVer ? pDst.buf : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }


  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    for( Int y = 0; y < height; y++ )
    {
      for( Int x = 0; x < width; x++ )
      {
        pDstBuf[y*dstStride + x] = refMain[x + 1];
      }
    }
#if HEVC_USE_HOR_VER_PREDFILTERING
    if (edgeFilter)
    {
      for( Int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ( ( refSide[y + 1] - refSide[0] ) >> 1 ), clpRng );
      }
    }
#endif
  }
  else
  {
    Pel *pDsty=pDstBuf;

    for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
    {
      const Int deltaInt   = deltaPos >> 5;
      const Int deltaFract = deltaPos & (32 - 1);

#if HM_4TAPIF_AS_IN_JEM
      if( deltaFract )
#else
      if( absAng < 32 )
#endif
      {
#if JEM_TOOLS
        if( sps.getSpsNext().getUseIntra4Tap() )
        {
          Int         p[4];
          const Bool  useCubicFilter = (width <= 8);
          const Int  *f              = (useCubicFilter) ? g_intraCubicFilter[deltaFract] : g_intraGaussFilter[deltaFract];
          Int         refMainIndex   = deltaInt + 1;

          for( Int x = 0; x < width; x++, refMainIndex++ )
          {
            p[1] = refMain[refMainIndex];
            p[2] = refMain[refMainIndex + 1];

            p[0] = x == 0 ? p[1] : refMain[refMainIndex - 1];
            p[3] = x == (width - 1) ? p[2] : refMain[refMainIndex + 2];

            pDstBuf[y*dstStride + x] =  (Pel)((f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 128) >> 8);

            if( useCubicFilter ) // only cubic filter has negative coefficients and requires clipping
            {
              pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng );
            }
          }
        }
        else
#endif
        {
          // Do linear filtering
          const Pel *pRM = refMain + deltaInt + 1;
          Int lastRefMainPel = *pRM++;
          for( Int x = 0; x < width; pRM++, x++ )
          {
            Int thisRefMainPel = *pRM;
            pDsty[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
            lastRefMainPel = thisRefMainPel;
          }
        }
      }
      else
      {
        // Just copy the integer samples
        for( Int x = 0; x < width; x++ )
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
#if JVET_K0063_PDPC_SIMP
      const int numModes = 8;
      const int scale = ((g_aucLog2[width] - 2 + g_aucLog2[height] - 2 + 2) >> 2);
      CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

#if JVET_K0500_WAIP
      if (predMode == 2 || predMode == VDIA_IDX)
#else
      if (dirMode == 2 || dirMode == VDIA_IDX)
#endif
      {
        int wT = 16 >> std::min(31, ((y << 1) >> scale));

        for (int x = 0; x < width; x++)
        {
          int wL = 16 >> std::min(31, ((x << 1) >> scale));
          if (wT + wL == 0) break;

          int c = x + y + 1;
          const Pel left = refSide[c + 1];
          const Pel top = refMain[c + 1];

          pDsty[x] = ClipPel((wL * left + wT * top + (64 - wL - wT) * pDsty[x] + 32) >> 6, clpRng);
        }
      }
#if JVET_K0500_WAIP
      else if ((predMode >= VDIA_IDX - numModes && predMode != VDIA_IDX) || (predMode != 2 && predMode <= (2 + numModes)))
#else
      else if ((dirMode >= VDIA_IDX - numModes && dirMode < VDIA_IDX) || (dirMode > 2 && dirMode <= (2 + numModes)))
#endif
      {
        int invAngleSum0 = 2;
        for (int x = 0; x < width; x++)
        {
          invAngleSum0 += invAngle;
          int deltaPos0 = invAngleSum0 >> 2;
          int deltaFrac0 = deltaPos0 & 63;
          int deltaInt0 = deltaPos0 >> 6;

          int deltay = y + deltaInt0 + 1;
#if JVET_K0500_WAIP
          if (deltay >(bIsModeVer ? m_leftRefLength : m_topRefLength) - 1) break;
#else
          if (deltay > height + width - 1) break;
#endif

          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          if (wL == 0) break;
          Pel *p = refSide + deltay;

          Pel left = (((64 - deltaFrac0) * p[0] + deltaFrac0 * p[1] + 32) >> 6);
          pDsty[x] = ClipPel((wL * left + (64 - wL) * pDsty[x] + 32) >> 6, clpRng);
        }
      }
#endif
    }
#if HEVC_USE_HOR_VER_PREDFILTERING
    if( edgeFilter && absAng <= 1 )
    {
      for( Int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ((refSide[y + 1] - refSide[0]) >> 2), clpRng );
      }
    }
#endif
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    for( Int y = 0; y < height; y++ )
    {
      for( Int x = 0; x < width; x++ )
      {
        pDst.at( y, x ) = pDstBuf[x];
      }
      pDstBuf += dstStride;
    }
  }
#if JEM_TOOLS && JEM_USE_INTRA_BOUNDARY

  if( sps.getSpsNext().getUseIntraBoundaryFilter() && enableBoundaryFilter && isLuma( channelType ) && width > 2 && height > 2 )
  {
    if( dirMode == VDIA_IDX )
    {
      xIntraPredFilteringMode34( pSrc, pDst );
    }
    else  if( dirMode == 2 )
    {
      xIntraPredFilteringMode02( pSrc, pDst );
    }
    else if( ( dirMode <= 10 && dirMode > 2 ) || ( dirMode >= ( VDIA_IDX - 8 ) && dirMode < VDIA_IDX ) )
    {
      xIntraPredFilteringModeDGL( pSrc, pDst, dirMode );
    }
  }
#endif
}
#if JEM_TOOLS && JEM_USE_INTRA_BOUNDARY

Void IntraPrediction::xIntraPredFilteringMode34(const CPelBuf &pSrc, PelBuf &pDst)
{
  UInt iWidth  = pDst.width;
  UInt iHeight = pDst.height;

  Int y;

  for( y = 0; y < iHeight; y++ )
  {
    pDst.at( 0, y ) = (  8 * pDst.at( 0, y ) + 8 * pSrc.at( 0, y + 2 ) + 8 ) >> 4;
    pDst.at( 1, y ) = ( 12 * pDst.at( 1, y ) + 4 * pSrc.at( 0, y + 3 ) + 8 ) >> 4;

    if( iWidth > 2 )
    {
      pDst.at( 2, y ) = ( 14 * pDst.at( 2, y ) + 2 * pSrc.at( 0, y + 4 ) + 8 ) >> 4;
      pDst.at( 3, y ) = ( 15 * pDst.at( 3, y ) +     pSrc.at( 0, y + 5 ) + 8 ) >> 4;

    }
  }
}

Void IntraPrediction::xIntraPredFilteringMode02(const CPelBuf &pSrc, PelBuf &pDst)
{
  UInt iWidth  = pDst.width;
  UInt iHeight = pDst.height;

  Int x;

  for( x = 0; x < iWidth; x++ )
  {
    pDst.at( x, 0 ) = (  8 * pDst.at( x, 0 ) + 8 * pSrc.at( x + 2, 0 ) + 8 ) >> 4;
    pDst.at( x, 1 ) = ( 12 * pDst.at( x, 1 ) + 4 * pSrc.at( x + 3, 0 ) + 8 ) >> 4;

    if( iHeight > 2 )
    {
      pDst.at( x, 2 ) = ( 14 * pDst.at( x, 2 ) + 2 * pSrc.at( x + 4, 0 ) + 8 ) >> 4;
      pDst.at( x, 3 ) = ( 15 * pDst.at( x, 3 ) +     pSrc.at( x + 5, 0 ) + 8 ) >> 4;
    }
  }
}

Void IntraPrediction::xIntraPredFilteringModeDGL(const CPelBuf &pSrc, PelBuf &pDst, UInt uiMode)
{
  UInt iWidth = pDst.width;
  UInt iHeight = pDst.height;
  Int x, y;

  const Int aucAngPredFilterCoef[8][3] = {
    { 12, 3, 1 }, { 12, 3, 1 },
    { 12, 1, 3 }, { 12, 2, 2 },
    { 12, 2, 2 }, { 12, 3, 1 },
    {  8, 6, 2 }, {  8, 7, 1 },
  };
  const Int aucAngPredPosiOffset[8][2] = {
    { 2, 3 }, { 2, 3 },
    { 1, 2 }, { 1, 2 },
    { 1, 2 }, { 1, 2 },
    { 1, 2 }, { 1, 2 },
  };

  CHECK( !( ( uiMode >= ( VDIA_IDX - 8 ) && uiMode < VDIA_IDX ) || ( uiMode > 2 && uiMode <= ( 2 + 8 ) ) ), "Incorrect mode" );

  Bool bHorz    = (uiMode < DIA_IDX);
  UInt deltaAng = bHorz ? ((2 + 8) - uiMode) : (uiMode - (VDIA_IDX - 8));

  const Int *offset = aucAngPredPosiOffset[deltaAng];
  const Int *filter = aucAngPredFilterCoef[deltaAng];

  if (bHorz)
  {
    for (x = 0; x < iWidth; x++)
    {
      pDst.at( x, 0 ) = ( filter[0] * pDst.at( x                , 0 )
                        + filter[1] * pSrc.at( x + 1 + offset[0], 0 )
                        + filter[2] * pSrc.at( x + 1 + offset[1], 0 ) + 8 ) >> 4;
    }
  }
  else
  {
    for (y = 0; y < iHeight; y++)
    {
      pDst.at( 0, y ) = ( filter[0] * pDst.at( 0,   y )
                        + filter[1] * pSrc.at( 0, ( y + offset[0] + 1 ) )
                        + filter[2] * pSrc.at( 0, ( y + offset[1] + 1 ) ) + 8 ) >> 4;
    }
  }

  return;
}
#endif

void IntraPrediction::xReferenceFilter(
#if JVET_K0500_WAIP
  const int doubleHSize,
#endif  
  const int doubleSize, const int origWeight, const int filterOrder, Pel *piRefVector, Pel *piLowPassRef )
{
  const int imCoeff[3][4] =
  {
    { 20, 15, 6, 1 },
    { 16, 14, 7, 3 },
    { 14, 12, 9, 4 }
  };

  const int * piFc;

  int binBuff[4 * MAX_CU_SIZE + 9];
  int * piTmp = &binBuff[2 * MAX_CU_SIZE + 4];   // to  use negative indexes
  Pel * piDat = piRefVector;
  Pel * piRes = piLowPassRef;

#if JVET_K0500_WAIP
  for( int k = -doubleHSize; k <= doubleSize; k++ )
#else
  for( int k = -doubleSize; k <= doubleSize; k++ )
#endif
    piTmp[k] = piDat[k];

  for( int n = 1; n <= 3; n++ )
  {
#if JVET_K0500_WAIP
    piTmp[-doubleHSize - n] = piTmp[-doubleHSize - 1 + n];
#else
    piTmp[-doubleSize - n] = piTmp[-doubleSize - 1 + n];
#endif
    piTmp[ doubleSize + n] = piTmp[ doubleSize + 1 - n];
  }

  switch( filterOrder )
  {
  case 0:
    break;
  case 1:
#if JVET_K0500_WAIP
    for( int k = -doubleHSize; k <= doubleSize; k++ )
#else
    for( int k = -doubleSize; k <= doubleSize; k++ )
#endif
      piRes[k] = (Pel)(((piTmp[k] << 1) + piTmp[k - 1] + piTmp[k + 1] + 2) >> 2);
    break;
  case 2:
#if JVET_K0500_WAIP
    for( int k = -doubleHSize; k <= doubleSize; k++ )
#else
    for( int k = -doubleSize; k <= doubleSize; k++ )
#endif
      piRes[k] = (Pel)(((piTmp[k] << 1) + ((piTmp[k] + piTmp[k - 1] + piTmp[k + 1]) << 2) + piTmp[k - 2] + piTmp[k + 2] + 8) >> 4);
    break;
  case 3:
  case 5:
  case 7:
    piFc = imCoeff[(filterOrder - 3) >> 1];
#if JVET_K0500_WAIP
    for( int k = -doubleHSize; k <= doubleSize; k++ )
#else
    for( int k = -doubleSize; k <= doubleSize; k++ )
#endif
    {
      int s = 32 + piFc[0] * piTmp[k];
      for( int n = 1; n < 4; n++ )
        s += piFc[n] * (piTmp[k - n] + piTmp[k + n]);

      piRes[k] = (Pel)(s >> 6);
    }
    break;
  default:
    EXIT( "Invalid intra prediction reference filter order" );
  }

  int ParShift = 6; //normalization factor
  int ParScale = 1 << ParShift;
  int ParOffset = 1 << (ParShift - 1);

  if( origWeight != 0 )
  {
    int iCmptWeight = ParScale - origWeight;
#if JVET_K0500_WAIP
    for (int k = -doubleHSize; k <= doubleSize; k++)
#else
    for( int k = -doubleSize; k <= doubleSize; k++ )
#endif
      piLowPassRef[k] = (origWeight * piRefVector[k] + iCmptWeight * piLowPassRef[k] + ParOffset) >> ParShift;
  }
}

Bool IntraPrediction::useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const UInt &uiDirMode)
{
  return CU::isRDPCMEnabled(*pu.cu) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}

inline Bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline Int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *validFlags );
inline Int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *validFlags );
inline Int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *validFlags );
inline Int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *validFlags );

Void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const Bool bFilterRefSamples)
{
  const CodingStructure& cs   = *cu.cs;

  Pel *refBufUnfiltered   = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[area.compID][PRED_BUF_FILTERED];

#if JVET_K0500_WAIP
  setReferenceArrayLengths(area);
#endif

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  // ----- Step 2: filtered reference samples -----
  if( bFilterRefSamples )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps );
  }
}

void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu )
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
#if JVET_K0500_WAIP
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
#else
  const int  predSize           = tuWidth + tuHeight;
#endif
  const int  predStride         = predSize + 1;

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth          = pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX( area.compID, sps.getChromaFormatIdc() ));
  const int  unitHeight         = pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY( area.compID, sps.getChromaFormatIdc() ));

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
#if JVET_K0500_WAIP
  const int  totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
#else
  const int  totalLeftUnits     = (predSize + (unitHeight - 1)) / unitHeight;
#endif
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT );
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT, numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT, numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT, numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB, numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
#if JVET_K0500_WAIP
  CHECK((predHSize + 1) * predStride > m_iYuvExtSize, "Reference sample area not supported");
#else
  CHECK( predStride * predStride > m_iYuvExtSize, "Reference sample area not supported" );
#endif

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);


  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j]            = valueDC; }
#if JVET_K0500_WAIP
    for( int i = 1; i <= predHSize; i++ ) { ptrDst[i*predStride] = valueDC; }
#else
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = valueDC; }
#endif
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - srcStride - 1;
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j] = ptrSrc[j]; }
    // Fill left and below left border with rec. samples
    ptrSrc = srcBuf - 1;
#if JVET_K0500_WAIP
    for( int i = 1; i <= predHSize; i++ ) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
#else
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
#endif
  }
  else // reference samples are partially available
  {
    // BB: old implementation using tmpLineBuf
    // ---------------------------------------
    Pel  tmpLineBuf[5 * MAX_CU_SIZE];
    Pel* ptrTmp;
    int  unitIdx;

    // Initialize
    const int totalSamples = (totalLeftUnits * unitHeight) + ((totalAboveUnits + 1) * unitWidth); // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    for( int k = 0; k < totalSamples; k++ ) { tmpLineBuf[k] = valueDC; }

    // Fill top-left sample
    ptrSrc = srcBuf - srcStride - 1;
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight);
    unitIdx = totalLeftUnits;
    if( neighborFlags[unitIdx] )
    {
      Pel topLeftVal = ptrSrc[0];
      for( int j = 0; j < unitWidth; j++ ) { ptrTmp[j] = topLeftVal; }
    }

    // Fill left & below-left samples (downwards)
    ptrSrc += srcStride;
    ptrTmp--;
    unitIdx--;

    for( int k = 0; k < totalLeftUnits; k++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int i = 0; i < unitHeight; i++ ) { ptrTmp[-i] = ptrSrc[i*srcStride]; }
      }
      ptrSrc += unitHeight*srcStride;
      ptrTmp -= unitHeight;
      unitIdx--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
    ptrSrc = srcBuf - srcStride;
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight) + unitWidth; // offset line buffer by totalLeftUnits*unitHeight (for left/below-left) + unitWidth (for above-left)
    unitIdx = totalLeftUnits + 1;
    for( int k = 0; k < totalAboveUnits; k++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int j = 0; j < unitWidth; j++ ) { ptrTmp[j] = ptrSrc[j]; }
      }
      ptrSrc += unitWidth;
      ptrTmp += unitWidth;
      unitIdx++;
    }

    // Pad reference samples when necessary
    int  currUnit       = 0;
    Pel* ptrTmpCurrUnit = tmpLineBuf;

    if( !neighborFlags[0] )
    {
      int nextUnit = 1;
      while( nextUnit < totalUnits && !neighborFlags[nextUnit] )
      {
        nextUnit++;
      }
      Pel* ptrTmpRef = tmpLineBuf + ((nextUnit < totalLeftUnits) ? (nextUnit * unitHeight) : ((totalLeftUnits * (unitHeight - unitWidth)) + (nextUnit * unitWidth)));
      const Pel refSample = *ptrTmpRef;
      // Pad unavailable samples with new value
      // fill left column
      while( currUnit < std::min<int>( nextUnit, totalLeftUnits ) )
      {
        for( int i = 0; i < unitHeight; i++ ) { ptrTmpCurrUnit[i] = refSample; }
        ptrTmpCurrUnit += unitHeight;
        currUnit++;
      }
      // fill top row
      while( currUnit < nextUnit )
      {
        for( int j = 0; j < unitWidth; j++ ) { ptrTmpCurrUnit[j] = refSample; }
        ptrTmpCurrUnit += unitWidth;
        currUnit++;
      }
    }

    // pad all other reference samples.
    while( currUnit < totalUnits )
    {
      const int numSamplesInCurrUnit = (currUnit >= totalLeftUnits) ? unitWidth : unitHeight;
      if( !neighborFlags[currUnit] ) // samples not available
      {
        const Pel refSample = *(ptrTmpCurrUnit - 1);
        for( int k = 0; k < numSamplesInCurrUnit; k++ ) { ptrTmpCurrUnit[k] = refSample; }

      }
      ptrTmpCurrUnit += numSamplesInCurrUnit;
      currUnit++;
    }

    // Copy processed samples
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight) + (unitWidth - 1);
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j] = ptrTmp[j]; } // top left, top and top right samples

    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight);
#if JVET_K0500_WAIP
    for( int i = 1; i <= predHSize; i++ ) { ptrDst[i*predStride] = ptrTmp[-i]; }
#else
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = ptrTmp[-i]; }
#endif
  }
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps )
{
#if JVET_K0500_WAIP
  const int  predSize   = m_topRefLength;
  const int  predHSize  = m_leftRefLength;
#else
  const int  tuWidth    = area.width;
  const int  tuHeight   = area.height;
  const int  predSize   = tuWidth + tuHeight;
#endif
  const int  predStride = predSize + 1;


#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  // Strong intra smoothing
  ChannelType chType = toChannelType( area.compID );
  if( sps.getUseStrongIntraSmoothing() && isLuma( chType ) )
  {
#if JVET_K0500_WAIP
    const Pel bottomLeft = refBufUnfiltered[predStride * predHSize];
#else
    const Pel bottomLeft = refBufUnfiltered[predStride * predSize];
#endif
    const Pel topLeft    = refBufUnfiltered[0];
    const Pel topRight   = refBufUnfiltered[predSize];

    const int  threshold     = 1 << (sps.getBitDepth( chType ) - 5);
    const bool bilinearLeft  = abs( (bottomLeft + topLeft)  - (2 * refBufUnfiltered[predStride * tuHeight]) ) < threshold; //difference between the
    const bool bilinearAbove = abs( (topLeft    + topRight) - (2 * refBufUnfiltered[             tuWidth ]) ) < threshold; //ends and the middle

    if( tuWidth >= 32 && tuHeight >= 32 && bilinearLeft && bilinearAbove )
#if !HEVC_USE_INTRA_SMOOTHING_T32
    if( tuWidth > 32 && tuHeight > 32 )
#endif
#if !HEVC_USE_INTRA_SMOOTHING_T64
    if( tuWidth < 64 && tuHeight < 64 )
#endif
    {
#if JVET_K0500_WAIP
      Pel *piDestPtr = refBufFiltered + (predStride * predHSize); // bottom left
#else
      Pel *piDestPtr = refBufFiltered + (predStride * predSize); // bottom left
#endif

      // apply strong intra smoothing
#if JVET_K0500_WAIP
      for (int i = 0; i < predHSize; i++, piDestPtr -= predStride) //left column (bottom to top)
      {
        *piDestPtr = (((predHSize - i) * bottomLeft) + (i * topLeft) + predHSize / 2) / predHSize;
      }
#else
      for( UInt i = 0; i < predSize; i++, piDestPtr -= predStride ) //left column (bottom to top)
      {
        *piDestPtr = (((predSize - i) * bottomLeft) + (i * topLeft) + predSize / 2) / predSize;
      }
#endif
      for( UInt i = 0; i <= predSize; i++, piDestPtr++ )            //full top row (left-to-right)
      {
        *piDestPtr = (((predSize - i) * topLeft) + (i * topRight) + predSize / 2) / predSize;
      }

      return;
    }
  }
#endif

  // Regular reference sample filter
#if JVET_K0500_WAIP
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predHSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predHSize); // bottom left
#else
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predSize); // bottom left
#endif

  // bottom left (not filtered)
  *piDestPtr = *piSrcPtr;
  piDestPtr -= predStride;
  piSrcPtr  -= predStride;
  //left column (bottom to top)
#if JVET_K0500_WAIP
  for( int i = 1; i < predHSize; i++, piDestPtr -= predStride, piSrcPtr -= predStride)
#else
  for( UInt i=1; i < predSize; i++, piDestPtr-=predStride, piSrcPtr-=predStride )
#endif
  {
    *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[-predStride] + 2) >> 2;
  }
  //top-left
  *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[1] + 2) >> 2;
  piDestPtr++;
  piSrcPtr++;
  //top row (left-to-right)
  for( UInt i=1; i < predSize; i++, piDestPtr++, piSrcPtr++ )
  {
    *piDestPtr = (piSrcPtr[1] + 2 * piSrcPtr[0] + piSrcPtr[-1] + 2) >> 2;
  }
  // top right (not filtered)
  *piDestPtr=*piSrcPtr;
}

bool IntraPrediction::useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea )
{
  const SPS         &sps    = *pu.cs->sps;
  const ChannelType  chType = toChannelType( compID );

  // high level conditions
  if( sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() )                                       { return false; }
  if( !isLuma( chType ) && pu.chromaFormat != CHROMA_444 )                                               { return false; }

#if JEM_TOOLS
  // PDPC related conditions
  if( sps.getSpsNext().isIntraPDPC() )                                                                   { return false; }

  // NSST related conditions
#if !JVET_K0063_PDPC_SIMP // fix for BMS1.0 RA config (IntraPDPC=2) decoder mismatch
  if( sps.getSpsNext().isPlanarPDPC() && (sps.getSpsNext().getUseNSST() && pu.cu->nsstIdx == 0) )        { return false; }
#endif
#endif

  if( !modeSpecific )                                                                                    { return true; }

  // pred. mode related conditions
  const int dirMode = PU::getFinalIntraMode( pu, chType );
#if JVET_K0500_WAIP
  int predMode = getWideAngle(tuArea.blocks[compID].width, tuArea.blocks[compID].height, dirMode);
  if (predMode != dirMode && (predMode < 2 || predMode > VDIA_IDX))                                      { return true; }
#endif
#if JVET_K0063_PDPC_SIMP
  if (dirMode == DC_IDX)                                                                                 { return false; }
  if (dirMode == PLANAR_IDX)
  {
    return tuArea.blocks[compID].width * tuArea.blocks[compID].height > 32 ? true : false;
  }
#else
  if( dirMode == DC_IDX || (sps.getSpsNext().isPlanarPDPC() && dirMode == PLANAR_IDX) )                  { return false; }
#endif

  int diff = std::min<int>( abs( dirMode - HOR_IDX ), abs( dirMode - VER_IDX ) );
  int log2Size = ((g_aucLog2[tuArea.blocks[compID].width] + g_aucLog2[tuArea.blocks[compID].height]) >> 1);
  CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
  return (diff > m_aucIntraFilter[chType][log2Size]);
}


Bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs = *cu.cs;
  const Position refPos = posLT.offset(-1, -1);
  const CodingUnit* pcCUAboveLeft = cs.isDecomp( refPos, chType ) ? cs.getCURestricted( refPos, cu, chType ) : nullptr;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool bAboveLeftFlag;

  if (isConstrained)
  {
    bAboveLeftFlag = pcCUAboveLeft && CU::isIntra(*pcCUAboveLeft);
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }

  return bAboveLeftFlag;
}

Int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDx = uiNumUnitsInPU * unitWidth;

  for (UInt dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }
  return iNumIntra;
}

Int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDy = uiNumUnitsInPU * unitHeight;

  for (UInt dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

Int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  UInt maxDx = uiNumUnitsInPU * unitWidth;

  for (UInt dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }

  return iNumIntra;
}

Int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDy = uiNumUnitsInPU * unitHeight;

  for (UInt dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if ( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}
#if JEM_TOOLS||JVET_K0190
// LumaRecPixels
Void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea)
{
  Int iDstStride = 0;
  Pel* pDst0 = 0;
#if JVET_K0190
  iDstStride = MAX_CU_SIZE + 1;
  pDst0 = m_piTemp + iDstStride + 1; //MMLM_SAMPLE_NEIGHBOR_LINES;
#else
  int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
  iDstStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
  pDst0 = m_piTemp + (iDstStride + 1) * MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
#endif
  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(), recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)

#if !JVET_K0190
  Pel *pMulDst0[LM_FILTER_NUM];
  Int  iBufStride = pu.cs->sps->getSpsNext().isELMModeMMLM() ? MAX_CU_SIZE + MMLM_Lines : MAX_CU_SIZE; //MMLM_SAMPLE_NEIGHBOR_LINES
  Pel* pMulDst[LM_FILTER_NUM];
  if (pu.cs->sps->getSpsNext().isELMModeMFLM())
  {
    for (Int i = 0; i < LM_FILTER_NUM; i++)
    {
      pMulDst0[i] = m_pLumaRecBufferMul[i] + (iBufStride + 1) * MMLM_Lines;
    }
  }
#endif

  CHECK( lumaArea.width  == chromaArea.width, "" );
  CHECK( lumaArea.height == chromaArea.height, "" );

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  Int iRecStride        = Src.stride;
  Int iRecStride2       = iRecStride << 1;

  CodingStructure&      cs = *pu.cs;
  const CodingUnit& lumaCU = isChroma( pu.chType ) ? *pu.cs->picture->cs->getCU( lumaArea.pos(), CH_L ) : *pu.cu;
  const CodingUnit&     cu = *pu.cu;

  const CompArea& area = isChroma( pu.chType ) ? chromaArea : lumaArea;

  const SPS &sps = *cs.sps;

  const UInt uiTuWidth  = area.width;
  const UInt uiTuHeight = area.height;

  Int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  if( !cs.pcv->rectCUs )
  {
    iBaseUnitSize = sps.getMaxCUWidth() >> sps.getMaxCodingDepth();
  }

  const Int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const Int  iUnitHeight      = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits;
  const Int  iLeftUnits       = iTUHeightInUnits;

  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset( bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits );
  Bool bAboveAvaillable, bLeftAvaillable;

  Int availlableUnit = isLeftAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iLeftUnits, iUnitHeight, ( bNeighborFlags + iLeftUnits - 1 ) );

  if( lumaCU.cs->pcv->rectCUs )
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }
  else
  {
    bLeftAvaillable = availlableUnit == iTUWidthInUnits;
  }

  availlableUnit = isAboveAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iAboveUnits, iUnitWidth, ( bNeighborFlags + iLeftUnits + 1 ) );

  if( lumaCU.cs->pcv->rectCUs )
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }
  else
  {
    bAboveAvaillable = availlableUnit == iTUHeightInUnits;
  }


  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  if( bAboveAvaillable )
  {
    pDst  = pDst0    - iDstStride;
    piSrc = pRecSrc0 - iRecStride2;

    for( Int i = 0; i < uiCWidth; i++ )
    {
      if( i == 0 && !bLeftAvaillable )
      {
        pDst[i] = ( piSrc[2 * i] + piSrc[2 * i + iRecStride] + 1 ) >> 1;
      }
      else
      {
        pDst[i] = ( ( ( piSrc[2 * i             ] * 2 ) + piSrc[2 * i - 1             ] + piSrc[2 * i + 1             ] )
                  + ( ( piSrc[2 * i + iRecStride] * 2 ) + piSrc[2 * i - 1 + iRecStride] + piSrc[2 * i + 1 + iRecStride] )
                  + 4 ) >> 3;
      }
    }
#if !JVET_K0190
    if (pu.cs->sps->getSpsNext().isELMModeMMLM())
    {
      for (Int line = 2; line <= MMLM_Lines; line++)
      {
        pDst  = pDst0    - iDstStride  * line;
        piSrc = pRecSrc0 - iRecStride2 * line;

        for (Int i = 0; i < uiCWidth; i++)
        {
          if (i == 0 && !bLeftAvaillable)
          {
            pDst[i] = (piSrc[2 * i] + piSrc[2 * i + iRecStride] + 1) >> 1;
          }
          else
          {
            pDst[i] = ( ( (piSrc[2 * i             ] * 2 ) + piSrc[2 * i - 1             ] + piSrc[2 * i + 1             ] )
                      + ( (piSrc[2 * i + iRecStride] * 2 ) + piSrc[2 * i - 1 + iRecStride] + piSrc[2 * i + 1 + iRecStride] )
                      + 4 ) >> 3;
          }
        }
      }
    }

    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
        pMulDst[i] = pMulDst0[i] - iDstStride;
      }

      piSrc = pRecSrc0 - iRecStride2;

      for (Int i = 0; i < uiCWidth; i++)
      {

        xFilterGroup(pMulDst, i, &piSrc[2 * i], iRecStride, bAboveAvaillable, i != 0 || bLeftAvaillable);
      }

      if (pu.cs->sps->getSpsNext().isELMModeMMLM())
      {
        for (int line = 2; line <= MMLM_Lines; line++)

        {
          for (Int i = 0; i < LM_FILTER_NUM; i++)
          {
            pMulDst[i] = pMulDst0[i] - iDstStride * line;
          }

          piSrc = pRecSrc0 - iRecStride2 * line;

          for (Int i = 0; i < uiCWidth; i++)
          {
            xFilterGroup(pMulDst, i, &piSrc[2 * i], iRecStride, bAboveAvaillable, i != 0 || bLeftAvaillable);
          }
        }
      }
    }
#endif
  }

  if( bLeftAvaillable )
  {
    pDst  = pDst0    - 1;
    piSrc = pRecSrc0 - 3;

    for( Int j = 0; j < uiCHeight; j++ )
    {
      pDst[0] = ( ( piSrc[1             ] * 2 + piSrc[0         ] + piSrc[2             ] )
                + ( piSrc[1 + iRecStride] * 2 + piSrc[iRecStride] + piSrc[2 + iRecStride] )
                + 4 ) >> 3;

      piSrc += iRecStride2;
      pDst  += iDstStride;
    }
#if !JVET_K0190
    if (pu.cs->sps->getSpsNext().isELMModeMMLM())
    {
      for (Int line = 2; line <= MMLM_Lines; line++)
      {
        pDst  = pDst0    - line;
        piSrc = pRecSrc0 - 2 * line - 1;

        for (Int j = 0; j < uiCHeight; j++)
        {
          pDst[0] = ( ( piSrc[1             ] * 3 + piSrc[2             ] )
                    + ( piSrc[1 + iRecStride] * 3 + piSrc[2 + iRecStride] )
                    + 4 ) >> 3;
          piSrc += iRecStride2;
          pDst  += iDstStride;
        }
      }
    }

    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
        pMulDst[i] = pMulDst0[i] - 1;
      }

      piSrc = pRecSrc0 - 2;
      for (Int j = 0; j < uiCHeight; j++)
      {
        //Filter group 1

        xFilterGroup(pMulDst, 0, piSrc, iRecStride, j != 0 || bAboveAvaillable, bLeftAvaillable);

        piSrc += iRecStride2;

        for (Int i = 0; i < LM_FILTER_NUM; i++)
        {
          pMulDst[i] += iDstStride;
        }
      }

      if (pu.cs->sps->getSpsNext().isELMModeMMLM())
      {
        for (int line = 2; line <= MMLM_Lines; line++)
        {
          for (Int i = 0; i < LM_FILTER_NUM; i++)
          {
            pMulDst[i] = pMulDst0[i] - line;
          }
          piSrc = pRecSrc0 - 2 * line;

          for (Int j = 0; j < uiCHeight; j++)
          {

            xFilterGroup(pMulDst, 0, piSrc, iRecStride, j != 0 || bAboveAvaillable, bLeftAvaillable);

            piSrc += iRecStride2;

            for (Int i = 0; i < LM_FILTER_NUM; i++)
            {
              pMulDst[i] += iDstStride;
            }
          }
        }
      }
    }
#endif
  }


  // inner part from reconstructed picture buffer
  for( Int j = 0; j < uiCHeight; j++ )
  {
    for( Int i = 0; i < uiCWidth; i++ )
    {
      if( i == 0 && !bLeftAvaillable )
      {
        pDst0[i] = ( pRecSrc0[2 * i] + pRecSrc0[2 * i + iRecStride] + 1 ) >> 1;
      }
      else
      {
        pDst0[i] = ( pRecSrc0[2 * i             ] * 2 + pRecSrc0[2 * i + 1             ] + pRecSrc0[2 * i - 1             ]
                   + pRecSrc0[2 * i + iRecStride] * 2 + pRecSrc0[2 * i + 1 + iRecStride] + pRecSrc0[2 * i - 1 + iRecStride]
                   + 4 ) >> 3;
      }
    }

    pDst0    += iDstStride;
    pRecSrc0 += iRecStride2;
  }
#if !JVET_K0190
  if (pu.cs->sps->getSpsNext().isELMModeMFLM())
  {
    pRecSrc0 = Src.bufAt(0, 0);

    for (Int j = 0; j < uiCHeight; j++)
    {
      for (Int i = 0; i < uiCWidth; i++)
      {
        xFilterGroup(pMulDst0, i, &pRecSrc0[2 * i], iRecStride, j != 0 || bAboveAvaillable, i != 0 || bLeftAvaillable);
      }
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
        pMulDst0[i] += iDstStride;
      }

      pRecSrc0 += iRecStride2;
    }
  }
#endif
}

static int GetFloorLog2( unsigned x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits++;
    x >>= 1;
  }
  return bits;
}
#endif

#if JEM_TOOLS&&!JVET_K0190
Int IntraPrediction::xCalcLMParametersGeneralized(Int x, Int y, Int xx, Int xy, Int count, Int bitDepth, Int &a, Int &b, Int &iShift)
{

  UInt uiInternalBitDepth = bitDepth;
  if (count == 0)
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
    return -1;
  }
  CHECK(count > 512, "");

  Int avgX = (x * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  Int avgY = (y * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  avgX = (x * g_aiLMDivTableHigh[count - 1] + avgX) >> 16;
  avgY = (y * g_aiLMDivTableHigh[count - 1] + avgY) >> 16;


  Int RErrX = x - avgX * count;
  Int RErrY = y - avgY * count;

  Int iB = 7;
  iShift = 13 - iB;

  {
    Int a1 = xy - (avgX*avgY * count) - avgX*RErrY - avgY*RErrX;
    Int a2 = xx - (avgX*avgX * count) - 2 * avgX*RErrX;

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2(abs(a1)) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2(abs(a2)) - iShiftA2;

    if (iScaleShiftA1 < 0)
    {
      iScaleShiftA1 = 0;
    }

    if (iScaleShiftA2 < 0)
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      UInt a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if (iScaleShiftA < 0)
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
    a = a << iB;

    Short n = 0;
    if (a != 0)
    {
      n = GetFloorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
    }

    iShift = (iShift + iB) - n;
    a = a >> n;

    b = avgY - ((a * avgX) >> iShift);

    return 0;
  }
}


Int IntraPrediction::xLMSampleClassifiedTraining(Int count, Int LumaSamples[], Int ChrmSamples[], Int GroupNum, Int bitDepth, MMLM_parameter parameters[])
{

  //Initialize

  for (Int i = 0; i < GroupNum; i++)
  {
    parameters[i].Inf = 0;
    parameters[i].Sup = (1 << bitDepth) - 1;
    parameters[i].a = 0;
    parameters[i].b = 1 << (bitDepth - 1);
    parameters[i].shift = 0;
  }

  if (count < 4)//
  {
    return -1;
  }

  Int GroupTag[1024];

  Int GroupCount[3] = { 0, 0, 0 };

  Int mean = 0;
  Int meanC = 0;

  Int iMaxLuma = -1;
  Int iMinLuma = 0xffffff;
  for (int i = 0; i < count; i++)
  {
    mean += LumaSamples[i];
    meanC += ChrmSamples[i];
    if (LumaSamples[i] < iMinLuma)
    {
      iMinLuma = LumaSamples[i];
    }
    if (LumaSamples[i] > iMaxLuma)
    {
      iMaxLuma = LumaSamples[i];
    }
  }

  CHECK(count > 512, "");

  Int meand = (mean  * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  Int meanCd = (meanC * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  mean = (mean  * g_aiLMDivTableHigh[count - 1] + meand + 32768) >> 16;
  meanC = (meanC * g_aiLMDivTableHigh[count - 1] + meanCd + 32768) >> 16;


  Int meanDiff = meanC - mean;

  mean = std::max(1, mean);

  Int iTh[2] = { 0, 0 };

  if (GroupNum == 2)
  {
    iTh[0] = mean;

    parameters[0].Inf = 0;
    parameters[0].Sup = mean - 1;

    parameters[1].Inf = mean;
    parameters[1].Sup = (1 << bitDepth) - 1;

  }
  else if (GroupNum == 3)
  {
    iTh[0] = std::max(iMinLuma + 1, (iMinLuma + mean + 1) >> 1);
    iTh[1] = std::min(iMaxLuma - 1, (iMaxLuma + mean + 1) >> 1);

    parameters[0].Inf = 0;
    parameters[0].Sup = iTh[0] - 1;

    parameters[1].Inf = iTh[0];
    parameters[1].Sup = iTh[1] - 1;

    parameters[2].Inf = iTh[1];
    parameters[2].Sup = (1 << bitDepth) - 1;
  }
  else
  {
    CHECK(1, "");
  }
  for (Int i = 0; i < count; i++)
  {
    if (LumaSamples[i] < iTh[0])
    {
      GroupTag[i] = 0;
      GroupCount[0]++;
    }
    else if (LumaSamples[i] < iTh[1] || GroupNum == 2)
    {
      GroupTag[i] = 1;
      GroupCount[1]++;
    }
    else
    {
      GroupTag[i] = 2;
      GroupCount[2]++;
    }
  }
  Int iBiggestGroup = 0;
  for (Int i = 1; i < GroupNum; i++)
  {
    if (GroupCount[i] > iBiggestGroup)
    {
      iBiggestGroup = i;
    }
  }

  for (int group = 0; group < GroupNum; group++)
  {
    // If there is only 1 sample in a group, add the nearest value of the two neighboring pixels to the group.
    if (GroupCount[group] < 2)
    {
      for (int i = 0; i < count; i++)
      {
        if (GroupTag[i] == group)
        {
          for (Int k = 1; (i + k < count) || (i - k >= 0); k++)
          {
            if (i + k < count && GroupTag[i + k] == iBiggestGroup)
            {
              GroupTag[i + k] = group;
              GroupCount[group]++;
              GroupCount[iBiggestGroup]--;
              break;
            }
            if (i - k >= 0 && GroupTag[i - k] == iBiggestGroup)
            {
              GroupTag[i - k] = group;
              GroupCount[group]++;
              GroupCount[iBiggestGroup]--;
              break;
            }
          }
          break;
        }
      }
    }
  }


  Int x[3], y[3], xy[3], xx[3];
  for (Int group = 0; group < GroupNum; group++)
  {
    x[group] = y[group] = xy[group] = xx[group] = 0;
  }
  for (Int i = 0; i < count; i++)
  {
    Int group = GroupTag[i];
    x[group] += LumaSamples[i];
    y[group] += ChrmSamples[i];
    xx[group] += LumaSamples[i] * LumaSamples[i];
    xy[group] += LumaSamples[i] * ChrmSamples[i];
  }

  for (Int group = 0; group < GroupNum; group++)
  {
    Int a, b, iShift;
    if (GroupCount[group] > 1)
    {
      xCalcLMParametersGeneralized(x[group], y[group], xx[group], xy[group], GroupCount[group], bitDepth, a, b, iShift);

      parameters[group].a = a;
      parameters[group].b = b;
      parameters[group].shift = iShift;
    }
    else
    {
      parameters[group].a = 0;
      parameters[group].b = meanDiff;
      parameters[group].shift = 0;
    }
  }
  return 0;
}

Int IntraPrediction::xGetMMLMParameters(const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, Int &numClass, MMLM_parameter parameters[])
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure&  cs = *pu.cs;
  const CodingUnit& cu = *pu.cu;
  Bool bQTBT = cu.slice->getSPS()->getSpsNext().getUseQTBT();

  const SPS &sps = *cs.sps;
  const UInt uiTuWidth = chromaArea.width;
  const UInt uiTuHeight = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const Int iBaseUnitSize = 1 << MIN_CU_LOG2;
  const Int  iUnitWidth = iBaseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const Int  iUnitHeight = iBaseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);


  const Int  iTUWidthInUnits = uiTuWidth / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits = iTUWidthInUnits;
  const Int  iLeftUnits = iTUHeightInUnits;

  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset(bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits);

  Bool bAboveAvaillable, bLeftAvaillable;

  Int availlableUnit = isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, iAboveUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + 1));

  if (bQTBT)
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }
  else
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }

  availlableUnit = isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, iLeftUnits, iUnitHeight, (bNeighborFlags + iLeftUnits - 1));

  if (bQTBT)
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }
  else
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }


  Pel *pSrcColor0, *pCurChroma0;
  Int  iSrcStride, iCurStride;

  UInt uiInternalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
  iSrcStride = (MAX_CU_SIZE)+MMLM_Lines;
  PelBuf Temp = PelBuf(m_piTemp + (iSrcStride + 1) * MMLM_Lines, iSrcStride, Size(chromaArea));

  iSrcStride = Temp.stride;
  pSrcColor0 = Temp.bufAt(0, 0);

  PelBuf recoC = pu.cs->picture->getRecoBuf(chromaArea);
  iCurStride = recoC.stride;
  pCurChroma0 = recoC.bufAt(0, 0);


  Int count = 0;
  Int LumaSamples[512];
  Int ChrmSamples[512];


  Int i, j;

  Pel *pSrc = pSrcColor0 - iSrcStride;
  Pel *pCur = pCurChroma0 - iCurStride;

  Bool bAdditionalLine = true;

  if (bAboveAvaillable)
  {
    for (j = 0; j < uiCWidth; j++)
    {
      LumaSamples[count] = pSrc[j];
      ChrmSamples[count] = pCur[j];
      count++;
    }
    if (bAdditionalLine)
    {
      for (int line = 2; line <= MMLM_Lines; line++)
      {
        pSrc = pSrcColor0 - line * iSrcStride;
        pCur = pCurChroma0 - line * iCurStride;

        for (j = 0; j < uiCWidth; j++)
        {
          LumaSamples[count] = pSrc[j];
          ChrmSamples[count] = pCur[j];
          count++;
        }
      }
    }
  }

  if (bLeftAvaillable)
  {
    pSrc = pSrcColor0 - 1;
    pCur = pCurChroma0 - 1;

    for (i = 0; i < uiCHeight; i++)
    {
      LumaSamples[count] = pSrc[0];
      ChrmSamples[count] = pCur[0];
      count++;

      pSrc += iSrcStride;
      pCur += iCurStride;
    }

    if (bAdditionalLine)
    {
      for (int line = 2; line <= MMLM_Lines; line++)
      {
        pSrc = pSrcColor0 - line;
        pCur = pCurChroma0 - line;

        for (i = 0; i < uiCHeight; i++)
        {
          LumaSamples[count] = pSrc[0];
          ChrmSamples[count] = pCur[0];
          count++;

          pSrc += iSrcStride;
          pCur += iCurStride;
        }
      }
    }
  }

  xLMSampleClassifiedTraining(count, LumaSamples, ChrmSamples, numClass, uiInternalBitDepth, parameters);
  return 2;
}
#endif

#if JEM_TOOLS||JVET_K0190
Void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea,
#if !JVET_K0190
  Int iPredType,
#endif
  Int& a, Int&  b, Int& iShift)
{
  CHECK( compID == COMPONENT_Y, "" );

  const SizeType uiCWidth  = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure&  cs = *(pu.cs);
  const CodingUnit& cu = *(pu.cu);

  const SPS &sps        = *cs.sps;
  const UInt uiTuWidth  = chromaArea.width;
  const UInt uiTuHeight = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const Int iBaseUnitSize = 1 << MIN_CU_LOG2;
  const Int  iUnitWidth   = iBaseUnitSize >> getComponentScaleX( chromaArea.compID, nChromaFormat );
  const Int  iUnitHeight  = iBaseUnitSize >> getComponentScaleX( chromaArea.compID, nChromaFormat );


  const Int  iTUWidthInUnits  = uiTuWidth / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits;
  const Int  iLeftUnits       = iTUHeightInUnits;

  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset( bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits );

  Bool bAboveAvaillable, bLeftAvaillable;

  Int availlableUnit = isAboveAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, iAboveUnits, iUnitWidth, ( bNeighborFlags + iLeftUnits + 1 ) );
  bAboveAvaillable = availlableUnit == iTUWidthInUnits;

  availlableUnit = isLeftAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, iLeftUnits, iUnitHeight, ( bNeighborFlags + iLeftUnits - 1 ) );
  bLeftAvaillable = availlableUnit == iTUHeightInUnits;

  Pel *pSrcColor0, *pCurChroma0;
  Int  iSrcStride,  iCurStride;

#if JVET_K0190
  PelBuf Temp;  
  iSrcStride = MAX_CU_SIZE + 1;
  Temp = PelBuf(m_piTemp + iSrcStride + 1, iSrcStride, Size(chromaArea));
  pSrcColor0 = Temp.bufAt(0, 0);
  pCurChroma0 = getPredictorPtr(compID);
#if JVET_K0500_WAIP
  iCurStride = m_topRefLength + 1;
#else
  iCurStride = uiCWidth + uiCHeight + 1;
#endif
  pCurChroma0 += iCurStride + 1;
#else
  if( iPredType == 0 ) //chroma from luma
  {
    PelBuf Temp;
    int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
    iSrcStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
    Temp = PelBuf(m_piTemp + (iSrcStride + 1) * MMLM_Lines, iSrcStride, Size(chromaArea)); //MMLM_SAMPLE_NEIGHBOR_LINES
    pSrcColor0 = Temp.bufAt(0, 0);
    pCurChroma0   = getPredictorPtr( compID );
#if JVET_K0500_WAIP
    iCurStride    = m_topRefLength + 1;
#else
    iCurStride    = uiCWidth + uiCHeight + 1;
#endif
    pCurChroma0  += iCurStride + 1;
  }
  else
  {
    CHECK( !( compID == COMPONENT_Cr ), "called for incorrect color channel" );

    pSrcColor0   = getPredictorPtr( COMPONENT_Cb );
    pCurChroma0  = getPredictorPtr( COMPONENT_Cr );

#if JVET_K0500_WAIP
    iSrcStride   = m_topRefLength + 1;
#else
    iSrcStride   = ( uiCWidth + uiCHeight + 1 );
#endif
    iCurStride   = iSrcStride;

    pSrcColor0  += iSrcStride + 1;
    pCurChroma0 += iCurStride + 1;
  }
#endif
  int x = 0, y = 0, xx = 0, xy = 0;
  int iCountShift = 0;
  unsigned uiInternalBitDepth = sps.getBitDepth( CHANNEL_TYPE_CHROMA );

  Pel *pSrc = pSrcColor0  - iSrcStride;
  Pel *pCur = pCurChroma0 - iCurStride;

  int       minDim        = bLeftAvaillable && bAboveAvaillable ? 1 << g_aucPrevLog2[std::min( uiCHeight, uiCWidth )] : 1 << g_aucPrevLog2[bLeftAvaillable ? uiCHeight : uiCWidth];
  int       minStep       = 1;
  int       numSteps      = cs.pcv->rectCUs ? minDim / minStep : minDim;

  if( bAboveAvaillable )
  {
    for( int j = 0; j < numSteps; j++ )
    {
      int idx = ( j * minStep * uiCWidth ) / minDim;

      x  += pSrc[idx];
      y  += pCur[idx];
      xx += pSrc[idx] * pSrc[idx];
      xy += pSrc[idx] * pCur[idx];
    }

    iCountShift = g_aucLog2[minDim / minStep];
  }

  if( bLeftAvaillable )
  {
    pSrc = pSrcColor0  - 1;
    pCur = pCurChroma0 - 1;

    for( int i = 0; i < numSteps; i++ )
    {
      int idx = ( i * uiCHeight * minStep ) / minDim;
      x  += pSrc[iSrcStride * idx];
      y  += pCur[iCurStride * idx];
      xx += pSrc[iSrcStride * idx] * pSrc[iSrcStride * idx];
      xy += pSrc[iSrcStride * idx] * pCur[iCurStride * idx];
    }

    iCountShift += bAboveAvaillable ? 1 : g_aucLog2[minDim / minStep];
  }

  if( !bLeftAvaillable && !bAboveAvaillable )
  {
    a = 0;
#if !JVET_K0190

    if( iPredType == 0 )
    {
#endif
      b = 1 << ( uiInternalBitDepth - 1 );
#if !JVET_K0190
    }
    else
    {
      b = 0;
    }
#endif
    iShift = 0;
    return;
  }

  Int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if( iTempShift > 0 )
  {
    x  = ( x  + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y  + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }


  /////// xCalcLMParameters

  Int avgX = x >> iCountShift;
  Int avgY = y >> iCountShift;

  Int RErrX = x & ( ( 1 << iCountShift ) - 1 );
  Int RErrY = y & ( ( 1 << iCountShift ) - 1 );

  Int iB = 7;
  iShift = 13 - iB;


  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << ( uiInternalBitDepth - 1 );
    iShift = 0;
  }
  else
  {
    Int a1 = xy - ( avgX * avgY << iCountShift ) -     avgX * RErrY - avgY * RErrX;
    Int a2 = xx - ( avgX * avgX << iCountShift ) - 2 * avgX * RErrX;
#if !JVET_K0190
    if( iPredType == 1 ) // Cr residual predicted from Cb residual, Cr from Cb
    {
      a1 += -1 * ( xx >> ( CR_FROM_CB_REG_COST_SHIFT + 1 ) );
      a2 +=        xx >>   CR_FROM_CB_REG_COST_SHIFT;
    }
#endif
    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }

    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if( a2s >= 32 )
    {
      UInt a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3( -( 1 << ( 15 - iB ) ), ( 1 << ( 15 - iB ) ) - 1, a );
    a = a << iB;

    Short n = 0;
    if( a != 0 )
    {
      n = GetFloorLog2( abs( a ) + ( ( a < 0 ? -1 : 1 ) - 1 ) / 2 ) - 5;
    }

    iShift = ( iShift + iB ) - n;
    a = a >> n;

    b = avgY - ( ( a * avgX ) >> iShift );
  }
}

#endif

//! \}
