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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"

#include <memory.h>
#include <algorithm>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
, m_maxCompIDToPred ( MAX_NUM_COMPONENT )
, m_pcRdCost        ( nullptr )
#if JEM_TOOLS
, m_pGradX0         ( nullptr )
, m_pGradY0         ( nullptr )
, m_pGradX1         ( nullptr )
, m_pGradY1         ( nullptr )
#endif
{
  for( UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  for( UInt c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

#if JEM_TOOLS
  m_LICMultApprox[0] = 0;
  for( int k = 1; k < 64; k++ )
  {
    m_LICMultApprox[k] = ( ( 1 << 15 ) + ( k >> 1 ) ) / k;
  }

  for( UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( UInt tmplt = 0; tmplt < 2; tmplt++ )
    {
      m_acYuvPredFrucTemplate[tmplt][ch] = nullptr;
    }
    m_cYuvPredTempDMVR[ch] = nullptr;
  }
#endif
}

InterPrediction::~InterPrediction()
{
  destroy();
}

Void InterPrediction::destroy()
{
  for( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( UInt c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  for( UInt c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

#if JEM_TOOLS
  xFree( m_pGradX0 );   m_pGradX0 = nullptr;
  xFree( m_pGradY0 );   m_pGradY0 = nullptr;
  xFree( m_pGradX1 );   m_pGradX1 = nullptr;
  xFree( m_pGradY1 );   m_pGradY1 = nullptr;
  m_tmpObmcBuf.destroy();

  for( UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( UInt tmplt = 0; tmplt < 2; tmplt++ )
    {
      xFree( m_acYuvPredFrucTemplate[tmplt][ch] );
      m_acYuvPredFrucTemplate[tmplt][ch] = nullptr;
    }
    xFree( m_cYuvPredTempDMVR[ch] );
    m_cYuvPredTempDMVR[ch] = nullptr;
  }
#endif
}

Void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC )
{
  m_pcRdCost = pcRdCost;


  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    for( UInt c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
#if JEM_TOOLS
      Int extWidth  = MAX_CU_SIZE + DMVR_INTME_RANGE*2 + 16;
      Int extHeight = MAX_CU_SIZE + DMVR_INTME_RANGE*2 + 1;
#else
      Int extWidth  = MAX_CU_SIZE + 16;
      Int extHeight = MAX_CU_SIZE + 1;
#endif
      for( UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
      {
        m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

        for( UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
        {
          m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        }
      }

      // new structure
      for( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        m_acYuvPred[i][c] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
    }

    m_iRefListIdx = -1;
    
#if JEM_TOOLS
    m_pGradX0     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
    m_pGradY0     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
    m_pGradX1     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
    m_pGradY1     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );

    m_tmpObmcBuf.create( UnitArea( chromaFormatIDC, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );

    for( UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
    {
      for( UInt tmplt = 0; tmplt < 2; tmplt++ )
      {
        m_acYuvPredFrucTemplate[tmplt][ch] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
      m_cYuvPredTempDMVR[ch] = ( Pel* ) xMalloc( Pel, ( MAX_CU_SIZE + DMVR_INTME_RANGE*2 ) * ( MAX_CU_SIZE + DMVR_INTME_RANGE*2 ) );
    }
#endif
  }

#if JEM_TOOLS
  m_uiaBIOShift[0] = 0;
  for (Int i = 1; i < 64; i++)
  {
    m_uiaBIOShift[i] = ((1 << 15) + i / 2) / i;
  }
#endif
#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter( true );
#endif
}

#if JEM_TOOLS
Bool checkIdenticalMotion( const PredictionUnit &pu, bool checkAffine )
#else
Bool checkIdenticalMotion( const PredictionUnit &pu )
#endif
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      Int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      Int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
#if JEM_TOOLS
        if( !pu.cu->affine )
#endif
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
#if JEM_TOOLS
        else
        {
          CHECK( !checkAffine, "In this case, checkAffine should be on." );
          const CMotionBuf &mb = pu.getMotionBuf();
#if JVET_K_AFFINE_BUG_FIXES
#if JVET_K0337_AFFINE_6PARA
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#else
          if ( (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
#endif
#else
          if( ( mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1] ) && ( mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1] ) && ( mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1] ) )
#endif
          {
            return true;
          }
        }
#endif
      }
    }
  }

  return false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      Int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      Int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
#if JEM_TOOLS
        if( !pu.cu->affine )
#endif
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
#if JEM_TOOLS
        else
        {
          const CMotionBuf &mb = pu.getMotionBuf();
#if JVET_K_AFFINE_BUG_FIXES
#if JVET_K0337_AFFINE_6PARA
          if ( ( pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
            || ( pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1]) ) )
#else
          if ( (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
#endif
#else
          if( ( mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1] ) && ( mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1] ) && ( mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1] ) )
#endif
          {
            return true;
          }
        }
#endif
      }
    }
  }

  return false;
}

#if JEM_TOOLS
Void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{
#if !JVET_K0346
  const SPSNext& spsNext  = pu.cs->sps->getSpsNext();
#endif

  // compute the location of the current PU
  Position puPos    = pu.lumaPos();
  Size puSize       = pu.lumaSize();

#if JVET_K0346
  int numPartLine, numPartCol, puHeight, puWidth;
#else
  int iNumPartLine, iNumPartCol, iPUHeight, iPUWidth;
#endif
  if( pu.mergeType == MRG_TYPE_FRUC )
  {
    UInt nRefineBlockSize = xFrucGetSubBlkSize( pu, puSize.width, puSize.height );

#if JVET_K0346
    puHeight     = std::min(puSize.width, nRefineBlockSize);
    puWidth      = std::min(puSize.height, nRefineBlockSize);
#else
    iPUHeight     = std::min( puSize.width,  nRefineBlockSize );
    iPUWidth      = std::min( puSize.height, nRefineBlockSize );
#endif
  }
  else
  {
#if JVET_K0346
    const Slice& slice = *pu.cs->slice;
    numPartLine = std::max(puSize.width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    numPartCol  = std::max(puSize.height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    puHeight    = numPartCol == 1 ? puSize.height : 1 << slice.getSubPuMvpSubblkLog2Size();
    puWidth     = numPartLine == 1 ? puSize.width : 1 << slice.getSubPuMvpSubblkLog2Size();
#else
    iNumPartLine  = std::max( puSize.width  >> spsNext.getSubPuMvpLog2Size(), 1u );
    iNumPartCol   = std::max( puSize.height >> spsNext.getSubPuMvpLog2Size(), 1u );
    iPUHeight     = iNumPartCol  == 1 ? puSize.height : 1 << spsNext.getSubPuMvpLog2Size();
    iPUWidth      = iNumPartLine == 1 ? puSize.width  : 1 << spsNext.getSubPuMvpLog2Size();
#endif
  }

  PredictionUnit subPu;

  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  // join sub-pus containing the same motion
#if JVET_K0346
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      subPu.mvRefine = false;
      motionCompensation(subPu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
#else
  for( int y = puPos.y; y < puPos.y + puSize.height; y += iPUHeight )
  {
    for( int x = puPos.x; x < puPos.x + puSize.width; x += iPUWidth )
    {
      const MotionInfo &curMi = pu.getMotionInfo( Position{ x, y } );

      int dx = iPUWidth;
      int dy = iPUHeight;

      subPu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( x, y, dx, dy ) ) );
      subPu                   = curMi;
      PelUnitBuf subPredBuf   = predBuf.subBuf( UnitAreaRelative( pu, subPu ) );

      subPu.mvRefine = false;
      motionCompensation( subPu, subPredBuf, eRefPicList );
    }
  }
#endif
}
#endif

#if JEM_TOOLS
Void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const Bool& bi, const Bool& bBIOApplied /*= false*/, const Bool& bDMVRApplied /*= false*/ )
#else
Void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const Bool& bi )
#endif
{
  const SPS &sps = *pu.cs->sps;

  Int iRefIdx = pu.refIdx[eRefPicList];
  Mv mv[3];

#if JEM_TOOLS
  if( pu.cu->affine )
  {
    CHECK( iRefIdx < 0, "iRefIdx incorrect." );

    const CMotionBuf &mb = pu.getMotionBuf();
    mv[0] = mb.at( 0,            0             ).mv[eRefPicList];
    mv[1] = mb.at( mb.width - 1, 0             ).mv[eRefPicList];
    mv[2] = mb.at( 0,            mb.height - 1 ).mv[eRefPicList];
#if !JVET_K_AFFINE_BUG_FIXES
    clipMv(mv[1], pu.cu->lumaPos(), sps);
    clipMv(mv[2], pu.cu->lumaPos(), sps);
#endif
  }
  else
#endif
  {
    mv[0] = pu.mv[eRefPicList];
  }
#if JVET_K_AFFINE_BUG_FIXES
  if ( !pu.cu->affine )
#endif
  clipMv(mv[0], pu.cu->lumaPos(), sps);


  for( UInt comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );

#if JEM_TOOLS
    if( pu.cu->affine )
    {
      xPredAffineBlk( compID, pu, pu.cu->slice->getRefPic(eRefPicList, iRefIdx), mv, pcYuvPred, bi, pu.cu->slice->clpRng( compID ), bBIOApplied );
    }
    else
#endif
    {
      xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng( compID )
#if JEM_TOOLS
                     , bBIOApplied, bDMVRApplied, FRUC_MERGE_OFF, true
#endif
                    );
    }
  }
}

#if JEM_TOOLS
Void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred, Bool obmc)
#else
Void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred)
#endif
{
  const PPS   &pps   = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;

#if JEM_TOOLS
  bool bBIOApplied = false;
  if ( pu.cs->sps->getSpsNext().getUseBIO() )
  {
    if( pu.cu->LICFlag || pu.cu->affine || obmc )
    {
      bBIOApplied = false;
    }
    else if( PU::isBIOLDB( pu ) )
    {
      bBIOApplied = true;
    }
    else
    {
      const bool bBIOcheck0 = !( pps.getWPBiPred() && slice.getSliceType() == B_SLICE );
      const bool bBIOcheck1 = !( pps.getUseWP()    && slice.getSliceType() == P_SLICE );
      if( bBIOcheck0
          && bBIOcheck1
          && PU::isBiPredFromDifferentDir( pu )
        )
      {
        bBIOApplied = true;
      }
    }
  }

  bool bDMVRApplied = false;
  if ( pu.cs->sps->getSpsNext().getUseDMVR() )
  {
    if ( pu.mvRefine
        && pu.mergeFlag
        && pu.mergeType == MRG_TYPE_DEFAULT_N
        && ! pu.frucMrgMode
        && ! pu.cu->LICFlag
        && ! pu.cu->affine
        && PU::isBiPredFromDifferentDir( pu )
       )
    {
      bDMVRApplied = true;
    }
  }
#endif

  for (UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK( pu.refIdx[refList] >= slice.getNumRefIdx( eRefPicList ), "Invalid reference index" );
    m_iRefListIdx = refList;

    PelUnitBuf pcMbBuf = ( pu.chromaFormat == CHROMA_400 ?
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y())) :
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())) );

    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      xPredInterUni ( pu, eRefPicList, pcMbBuf, true
#if JEM_TOOLS
                      , bBIOApplied, bDMVRApplied
#endif
                     );
    }
    else
    {
#if JEM_TOOLS
      if( !pu.cu->LICFlag && ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true, bBIOApplied, false );
      }
      else
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, false, bBIOApplied, false );
      }
#else
      if( ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true );
      }
      else
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, false );
      }
#endif
    }
  }

#if JEM_TOOLS
  if ( bDMVRApplied )
  {
    xProcessDMVR( pu, pcYuvPred, slice.clpRngs(), bBIOApplied );
  }
#endif

  CPelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())) );
  CPelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())) );
#if JEM_TOOLS
  if( !pu.cu->LICFlag && pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( !pu.cu->LICFlag && pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), bBIOApplied );
  }
#else
  if( pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs() );
  }
#endif
}


Void InterPrediction::xPredInterBlk ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const Bool& bi, const ClpRng& clpRng
#if JEM_TOOLS
                                      , const Bool& bBIOApplied /*=false*/, const Bool& bDMVRApplied /*= false*/, const Int& nFRUCMode /*= FRUC_MERGE_OFF*/, const Bool& doLic /*= true*/
#endif
                                    )
{
  JVET_J0090_SET_REF_PICTURE( refPic, compID );
#if JEM_TOOLS
  const Int       nFilterIdx = nFRUCMode ? pu.cs->slice->getSPS()->getSpsNext().getFRUCRefineFilter() : 0;
  const ChromaFormat  chFmt  = pu.chromaFormat;
  const bool          rndRes = ( !bi || pu.cu->LICFlag );

  int iAddPrecShift = 0;

  if( _mv.highPrec )
  {
    CHECKD( !pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!" );

    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY( compID, chFmt );

  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );

  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;

  CHECKD( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( xFrac & 3 ) != 0 ), "Invalid fraction" );
  CHECKD( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( yFrac & 3 ) != 0 ), "Invalid fraction" );
#else
  const ChromaFormat  chFmt  = pu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = 2 + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + ::getComponentScaleY( compID, chFmt );
  
  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );
#endif

  PelBuf &dstBuf  = dstPic.bufs[compID];
  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  CPelBuf refBuf;
  {
    Position offset = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
    refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  }

#if JEM_TOOLS
  if( bBIOApplied && compID == COMPONENT_Y && !bDMVRApplied )
  {
    Pel*  pGradY    = m_pGradY0;
    Pel*  pGradX    = m_pGradX0;

    Int   iWidthG   = width;
    Int   iHeightG  = height;

    if( m_iRefListIdx == 0 )
    {
      pGradY  = m_pGradY0;
      pGradX  = m_pGradX0;
    }
    else
    {
      pGradY  = m_pGradY1;
      pGradX  = m_pGradX1;
    }

    const Int         refStride   = refBuf.stride;
    const Pel* const  ref         = refBuf.buf;

    xGradFilterY  ( ref, refStride, pGradY, iWidthG, iWidthG, iHeightG, yFrac, xFrac, clpRng.bd );
    xGradFilterX  ( ref, refStride, pGradX, iWidthG, iWidthG, iHeightG, yFrac, xFrac, clpRng.bd );
  }
#endif

  if( yFrac == 0 )
  {
#if JEM_TOOLS
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes, chFmt, clpRng, nFilterIdx);
#else
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes, chFmt, clpRng);
#endif
  }
  else if( xFrac == 0 )
  {
#if JEM_TOOLS
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes, chFmt, clpRng, nFilterIdx);
#else
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes, chFmt, clpRng);
#endif
  }
  else
  {
    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);

    Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
#if JEM_TOOLS
    if( isLuma(compID) && nFilterIdx == 1 )
    {
      vFilterSize = NTAPS_LUMA_FRUC;
    }
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng, nFilterIdx);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, rndRes, chFmt, clpRng, nFilterIdx);
#else
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, rndRes, chFmt, clpRng);
#endif
    JVET_J0090_SET_CACHE_ENABLE( true );
  }

#if JEM_TOOLS
  if( pu.cu->LICFlag && doLic )
  {
    xLocalIlluComp( pu, compID, *refPic, _mv, bi, dstBuf );
  }
#endif
}

#if JEM_TOOLS
Void InterPrediction::xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const Bool& bi, const ClpRng& clpRng, const Bool& bBIOApplied /*= false*/ )
{
#if JVET_K0337_AFFINE_6PARA
  if ( (pu.cu->affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2])
    || (pu.cu->affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1])
    )
#else
  if( _mv[0] == _mv[1] )
#endif
  {
#if JVET_K_AFFINE_BUG_FIXES
    Mv mvTemp = _mv[0];
    clipMv( mvTemp, pu.cu->lumaPos(), *pu.cs->sps );
    xPredInterBlk( compID, pu, refPic, mvTemp, dstPic, bi, clpRng, bBIOApplied, false, FRUC_MERGE_OFF, true );
#else
    xPredInterBlk( compID, pu, refPic, _mv[0], dstPic, bi, clpRng, bBIOApplied, false, FRUC_MERGE_OFF, true );
#endif
    return;
  }

  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat chFmt = pu.chromaFormat;
  Int iScaleX = ::getComponentScaleX( compID, chFmt );
  Int iScaleY = ::getComponentScaleY( compID, chFmt );

  Mv mvLT =_mv[0];
  Mv mvRT =_mv[1];
  Mv mvLB =_mv[2];

  mvLT.setHighPrec();
  mvRT.setHighPrec();
  mvLB.setHighPrec();

  // get affine sub-block width and height
  const Int width  = pu.Y().width;
  const Int height = pu.Y().height;
#if JVET_K0184_AFFINE_4X4
  Int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  Int blockHeight = AFFINE_MIN_BLOCK_SIZE;
#else
  Int blockWidth   = width;
  Int blockHeight  = height;
  Int mvWx = std::max<int>( abs((mvRT - mvLT).getHor()), abs((mvRT - mvLT).getVer()) );
  Int mvWy = std::max<int>( abs((mvLB - mvLT).getHor()), abs((mvLB - mvLT).getVer()) );

  Int iMvPrecision = 4;
  iMvPrecision -= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;

  if (mvWx)
  {
    blockWidth = std::max<int>( (Int)( ( width >> iMvPrecision ) / mvWx ), 1 );
    while (width % blockWidth)
    {
      blockWidth--;
    }
    blockWidth = std::max<int>( AFFINE_MIN_BLOCK_SIZE, blockWidth );
  }
  if (mvWy)
  {
    blockHeight = std::max<int>( (Int)( ( height >> iMvPrecision ) / mvWy ), 1 );
    while (height % blockHeight)
    {
      blockHeight--;
    }
    blockHeight = std::max<int>( AFFINE_MIN_BLOCK_SIZE, blockHeight );
  }
#endif

  blockWidth  >>= iScaleX;
  blockHeight >>= iScaleY;
  const Int cxWidth  = width  >> iScaleX;
  const Int cxHeight = height >> iScaleY;
  const Int iHalfBW  = blockWidth  >> 1;
  const Int iHalfBH  = blockHeight >> 1;

  
#if JVET_K_AFFINE_REFACTOR
  const Int iBit = MAX_CU_DEPTH;
  Int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - g_aucLog2[cxWidth]);
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - g_aucLog2[cxWidth]);
#if JVET_K0337_AFFINE_6PARA
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - g_aucLog2[cxHeight]);
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - g_aucLog2[cxHeight]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }
#else
  iDMvVerX = -iDMvHorY;
  iDMvVerY = iDMvHorX;
#endif
#else
  // convert to 2^(storeBit + iBit) precision
  const Int iBit = 8;
  const Int iDMvHorX = ( (mvRT - mvLT).getHor() << iBit ) / cxWidth;    // deltaMvHor
  const Int iDMvHorY = ( (mvRT - mvLT).getVer() << iBit ) / cxWidth;
  const Int iDMvVerX = -iDMvHorY;                                       // deltaMvVer
  const Int iDMvVerY = iDMvHorX;
#endif

  Int iMvScaleHor = mvLT.getHor() << iBit;
  Int iMvScaleVer = mvLT.getVer() << iBit;
#if !JVET_K_AFFINE_REFACTOR
  Int iMvYHor = iMvScaleHor;
  Int iMvYVer = iMvScaleVer;
#endif
  const SPS &sps    = *pu.cs->sps;
  const Int iMvShift = 4;
  const Int iOffset  = 8;
  const Int iHorMax = ( sps.getPicWidthInLumaSamples()     + iOffset -      pu.Y().x - 1 ) << iMvShift;
  const Int iHorMin = (      -(Int)pu.cs->pcv->maxCUWidth  - iOffset - (Int)pu.Y().x + 1 ) << iMvShift;
  const Int iVerMax = ( sps.getPicHeightInLumaSamples()    + iOffset -      pu.Y().y - 1 ) << iMvShift;
  const Int iVerMin = (      -(Int)pu.cs->pcv->maxCUHeight - iOffset - (Int)pu.Y().y + 1 ) << iMvShift;

  PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
  const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

  const Int shift = iBit - 4 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE + 2;

  // get prediction block by block
  for ( Int h = 0; h < cxHeight; h += blockHeight )
  {
    for ( Int w = 0; w < cxWidth; w += blockWidth )
    {
#if JVET_K_AFFINE_REFACTOR
      Int iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
      Int iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
#if JVET_K_AFFINE_BUG_FIXES
      roundAffineMv( iMvScaleTmpHor, iMvScaleTmpVer, shift );
#else
      iMvScaleTmpHor >>= shift;
      iMvScaleTmpVer >>= shift;
#endif
#else
      Int iMvScaleTmpHor = ( iMvScaleHor + iDMvHorX * iHalfBW + iDMvVerX * iHalfBH ) >> shift;
      Int iMvScaleTmpVer = ( iMvScaleVer + iDMvHorY * iHalfBW + iDMvVerY * iHalfBH ) >> shift;
#endif

      // clip and scale
      iMvScaleTmpHor = std::min<int>( iHorMax, std::max<int>( iHorMin, iMvScaleTmpHor ) );
      iMvScaleTmpVer = std::min<int>( iVerMax, std::max<int>( iVerMin, iMvScaleTmpVer ) );

      // get the MV in high precision
      Int xFrac, yFrac, xInt, yInt;

      if (!iScaleX)
      {
        xInt  = iMvScaleTmpHor >> 4;
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt  = iMvScaleTmpHor >> 5;
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt  = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt  = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }

      const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID] ) );
      PelBuf &dstBuf = dstPic.bufs[compID];

      if ( yFrac == 0 )
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, xFrac, !bi, chFmt, clpRng );
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVer( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, clpRng );
      }
      else
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf - ((vFilterSize>>1) -1)*refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( false );
        m_if.filterVer( compID, tmpBuf.buf + ((vFilterSize>>1) -1)*tmpBuf.stride, tmpBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, false, !bi, chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( true );
      }
#if !JVET_K_AFFINE_REFACTOR
      // switch from x to x+AffineBlockSize, add deltaMvHor
      iMvScaleHor += (iDMvHorX*blockWidth);
      iMvScaleVer += (iDMvHorY*blockWidth);
#endif
    }

#if !JVET_K_AFFINE_REFACTOR
    // switch from y to y+AffineBlockSize add deltaMvVer
    iMvYHor += (iDMvVerX*blockHeight);
    iMvYVer += (iDMvVerY*blockHeight);

    iMvScaleHor = iMvYHor;
    iMvScaleVer = iMvYVer;
#endif
  }
}
#endif

int getMSB( unsigned x )
{
  int msb = 0, bits = ( sizeof(int) << 3 ), y = 1;
  while( x > 1u )
  {
    bits >>= 1;
    y      = x >> bits;
    if( y )
    {
      x    = y;
      msb += bits;
    }
  }
  msb += y;
  return msb;
}

#if JEM_TOOLS
void InterPrediction::xGetLICParams( const CodingUnit& cu,
                                     const ComponentID compID,
                                     const Picture&    refPic,
                                     const Mv&         mv,
                                           int&        shift,
                                           int&        scale,
                                           int&        offset )
{
  const int       lumaShift     = ( mv.highPrec ? 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 2 );
  const int       horShift      = ( lumaShift + ::getComponentScaleX(compID, cu.chromaFormat) );
  const int       verShift      = ( lumaShift + ::getComponentScaleY(compID, cu.chromaFormat) );
  const int       horIntMv      = ( mv.getHor() + ( ( 1 << horShift ) >> 1 ) ) >> horShift;
  const int       verIntMv      = ( mv.getVer() + ( ( 1 << verShift ) >> 1 ) ) >> verShift;
  const int       cuWidth       = cu.blocks[compID].width;
  const int       cuHeight      = cu.blocks[compID].height;
  const int       bitDepth      = cu.cs->sps->getBitDepth( toChannelType( compID ) );
  const int       precShift     = std::max( 0, bitDepth - 12 );
  const int       maxNumMinus1  = 30 - 2 * std::min( bitDepth, 12 ) - 1;
  const int       minDimBit     = g_aucPrevLog2[std::min( cuHeight, cuWidth )];
  const int       minDim        = 1 << minDimBit;
        int       minStepBit    = ( !cu.cs->pcv->rectCUs || minDim > 8 ? 1 : 0 );
        while   ( minDimBit > minStepBit + maxNumMinus1 )  { minStepBit++; } //make sure log2(2*minDim/tmpStep) + 2*min(bitDepth,12) <= 30
  const int       numSteps      = minDim >> minStepBit;
  const Picture&  currPic       = *cu.cs->picture;
  const int       dimShift      = minDimBit - minStepBit;

  //----- get correlation data -----
  int x = 0, y = 0, xx = 0, xy = 0, cntShift = 0;
  const CodingUnit* const cuAbove = cu.cs->getCU( cu.blocks[compID].pos().offset(  0, -1 ), toChannelType( compID ) );
  const CodingUnit* const cuLeft  = cu.cs->getCU( cu.blocks[compID].pos().offset( -1,  0 ), toChannelType( compID ) );
  const CPelBuf recBuf            = cuAbove || cuLeft ? currPic.getRecoBuf( cu.cs->picture->blocks[compID] ) : CPelBuf();
  const CPelBuf refBuf            = cuAbove || cuLeft ? refPic .getRecoBuf( refPic.blocks[compID]          ) : CPelBuf();

  // above
  if( cuAbove )
  {
    Mv            subPelMv  ( horIntMv << horShift, verIntMv << verShift, mv.highPrec );
                  clipMv    ( subPelMv, cuAbove->lumaPos(), *cu.cs->sps );
    const int     hOff    = ( subPelMv.getHor() >> horShift );
    const int     vOff    = ( subPelMv.getVer() >> verShift ) - 1;
    const Pel*    ref     = refBuf.bufAt( cu.blocks[compID].pos().offset( hOff, vOff ) );
    const Pel*    rec     = recBuf.bufAt( cu.blocks[compID].pos().offset(    0,   -1 ) );

    for( int k = 0; k < numSteps; k++ )
    {
      int refVal  = ref[( ( k * cuWidth ) >> dimShift )] >> precShift;
      int recVal  = rec[( ( k * cuWidth ) >> dimShift )] >> precShift;

      JVET_J0090_CACHE_ACCESS( &ref[( ( k * cuWidth ) >> dimShift )], __FILE__, __LINE__ );
      x          += refVal;
      y          += recVal;
      xx         += refVal * refVal;
      xy         += refVal * recVal;
    }

    cntShift = dimShift;
  }
  // left
  if( cuLeft )
  {
    Mv            subPelMv  ( horIntMv << horShift, verIntMv << verShift, mv.highPrec );
                  clipMv    ( subPelMv, cuLeft->lumaPos(), *cu.cs->sps );
    const int     hOff    = ( subPelMv.getHor() >> horShift ) - 1;
    const int     vOff    = ( subPelMv.getVer() >> verShift );
    const Pel*    ref     = refBuf.bufAt( cu.blocks[compID].pos().offset( hOff, vOff ) );
    const Pel*    rec     = recBuf.bufAt( cu.blocks[compID].pos().offset(   -1,    0 ) );

    for( int k = 0; k < numSteps; k++ )
    {
      int refVal     = ref[refBuf.stride * ( ( k * cuHeight ) >> dimShift )] >> precShift;
      int recVal     = rec[recBuf.stride * ( ( k * cuHeight ) >> dimShift )] >> precShift;
      x             += refVal;
      y             += recVal;
      xx            += refVal * refVal;
      xy            += refVal * recVal;
    }

    cntShift += ( cntShift ? 1 : dimShift );
  }

  //----- determine scale and offset -----
  shift = m_LICShift;
  if( cntShift == 0 )
  {
    scale   = ( 1 << shift );
    offset  = 0;
    return;
  }

  const int cropShift     = std::max( 0, bitDepth - precShift + cntShift - 15 );
  const int xzOffset      = ( xx >> m_LICRegShift );
  const int sumX          = x << precShift;
  const int sumY          = y << precShift;
  const int sumXX         = ( ( xx + xzOffset ) >> ( cropShift << 1 ) ) << cntShift;
  const int sumXY         = ( ( xy + xzOffset ) >> ( cropShift << 1 ) ) << cntShift;
  const int sumXsumX      = ( x >> cropShift ) * ( x >> cropShift );
  const int sumXsumY      = ( x >> cropShift ) * ( y >> cropShift );
        int a1            = sumXY - sumXsumY;
        int a2            = sumXX - sumXsumX;
        int scaleShiftA2  = getMSB( abs( a2 ) ) - 6;
        int scaleShiftA1  = scaleShiftA2 - m_LICShiftDiff;
            scaleShiftA2  = std::max( 0, scaleShiftA2 );
            scaleShiftA1  = std::max( 0, scaleShiftA1 );
  const int scaleShiftA   = scaleShiftA2 + 15 - shift - scaleShiftA1;
            a1            =               a1 >> scaleShiftA1;
            a2            = Clip3( 0, 63, a2 >> scaleShiftA2 );
            scale         = int( ( int64_t( a1 ) * int64_t( m_LICMultApprox[a2] ) ) >> scaleShiftA );
            scale         = Clip3( 0, 1 << ( shift + 2 ), scale );
  const int maxOffset     = ( 1 << ( bitDepth - 1 ) ) - 1;
  const int minOffset     = -1 - maxOffset;
            offset        = ( sumY - ( ( scale * sumX ) >> shift ) + ( ( 1 << ( cntShift ) ) >> 1 ) ) >> cntShift;
            offset        = Clip3( minOffset, maxOffset, offset );
}

void InterPrediction::xLocalIlluComp( const PredictionUnit& pu,
                                      const ComponentID     compID,
                                      const Picture&        refPic,
                                      const Mv&             mv,
                                      const bool            biPred,
                                            PelBuf&         dstBuf )
{
  int shift = 0, scale = 0, offset = 0;

  xGetLICParams( *pu.cu, compID, refPic, mv, shift, scale, offset );

  const int bitDepth   = pu.cs->sps->getBitDepth( toChannelType( compID ) );
  const ClpRng& clpRng = pu.cu->cs->slice->clpRng(compID);

  dstBuf.linearTransform( scale, shift, offset, true, clpRng );

  if( biPred )
  {
    const int biShift   =  IF_INTERNAL_PREC - bitDepth;
    const Pel biOffset  = -IF_INTERNAL_OFFS;
    ClpRng clpRngDummy;
    dstBuf.linearTransform( 1, -biShift, biOffset, false, clpRngDummy );
  }
}



void InterPrediction::applyBiOptFlow( const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, const Int &iRefIdx0, const Int &iRefIdx1, PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths )
{
  const int     iHeight     = pcYuvDst.Y().height;
  const int     iWidth      = pcYuvDst.Y().width;
  int           iHeightG    = iHeight;
  int           iWidthG     = iWidth;
//  int           iStrideTemp = 2 + 2 * iWidthG;
  Pel*          pGradX0     = m_pGradX0;
  Pel*          pGradX1     = m_pGradX1;
  Pel*          pGradY0     = m_pGradY0;
  Pel*          pGradY1     = m_pGradY1;
  const Pel*    pSrcY0      = pcYuvSrc0.Y().buf;
  const Pel*    pSrcY1      = pcYuvSrc1.Y().buf;
  const int     iSrc0Stride = pcYuvSrc0.Y().stride;
  const int     iSrc1Stride = pcYuvSrc1.Y().stride;
  Pel*          pDstY       = pcYuvDst.Y().buf;
  const int     iDstStride  = pcYuvDst.Y().stride;
  const Pel*    pSrcY0Temp  = pSrcY0;
  const Pel*    pSrcY1Temp  = pSrcY1;

  int dT0 = pu.cs->slice->getRefPOC( REF_PIC_LIST_0 , iRefIdx0 ) - pu.cs->slice->getPOC();
  int dT1 = pu.cs->slice->getPOC() - pu.cs->slice->getRefPOC( REF_PIC_LIST_1 , iRefIdx1 );
  if( dT0 * dT1 < 0 )
  {
    Pel* tmpGradX0 = m_pGradX0;
    Pel* tmpGradX1 = m_pGradX1;
    Pel* tmpGradY0 = m_pGradY0;
    Pel* tmpGradY1 = m_pGradY1;
    for( int y = 0; y < iHeightG; y++ )
    {
      for( int x = 0; x < iWidthG ; x++ )
      {
        tmpGradX0[x] *= dT0;
        tmpGradX1[x] *= dT1;
        tmpGradY0[x] *= dT0;
        tmpGradY1[x] *= dT1;
      }
      tmpGradX0 += iWidthG;
      tmpGradX1 += iWidthG;
      tmpGradY0 += iWidthG;
      tmpGradY1 += iWidthG;
    }
  }
  
  const ClpRng& clpRng        = pu.cu->cs->slice->clpRng(COMPONENT_Y);
  const int   bitDepth        = clipBitDepths.recon[ toChannelType(COMPONENT_Y) ];
  const int   shiftNum        = IF_INTERNAL_PREC + 1 - bitDepth;
  const int   offset          = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
  const bool  bShortRefMV     = ( pu.cs->slice->getCheckLDC() && PU::isBIOLDB(pu) );
  const Int64 limit           = ( 12 << (IF_INTERNAL_PREC - bShortRefMV - bitDepth ) );
  const Int64 regularizator_1 = 500 * (1<<(bitDepth-8)) * (1<<(bitDepth-8));
  const Int64 regularizator_2 = regularizator_1<<1;
  const Int64 denom_min_1     = 700 * (1<<(bitDepth-8)) * (1<<(bitDepth-8));
  const Int64 denom_min_2     = denom_min_1<<1;

  Int64* m_piDotProductTemp1 = m_piDotProduct1;
  Int64* m_piDotProductTemp2 = m_piDotProduct2;
  Int64* m_piDotProductTemp3 = m_piDotProduct3;
  Int64* m_piDotProductTemp5 = m_piDotProduct5;
  Int64* m_piDotProductTemp6 = m_piDotProduct6;

  Int64 temp=0, tempX=0, tempY=0;
  for( int y = 0; y < iHeightG; y++ )
  {
    for( int x = 0; x < iWidthG; x++ )
    {
      temp  = (Int64)( pSrcY0Temp[x] - pSrcY1Temp[x] );
      tempX = (Int64)( pGradX0   [x] + pGradX1   [x] );
      tempY = (Int64)( pGradY0   [x] + pGradY1   [x] );
      m_piDotProductTemp1[x] =  tempX * tempX;
      m_piDotProductTemp2[x] =  tempX * tempY;
      m_piDotProductTemp3[x] = -tempX * temp<<5;
      m_piDotProductTemp5[x] =  tempY * tempY<<1;
      m_piDotProductTemp6[x] = -tempY * temp<<6;
    }
    pSrcY0Temp          += iSrc0Stride;
    pSrcY1Temp          += iSrc1Stride;
    pGradX0             += iWidthG;
    pGradX1             += iWidthG;
    pGradY0             += iWidthG;
    pGradY1             += iWidthG;
    m_piDotProductTemp1 += iWidthG;
    m_piDotProductTemp2 += iWidthG;
    m_piDotProductTemp3 += iWidthG;
    m_piDotProductTemp5 += iWidthG;
    m_piDotProductTemp6 += iWidthG;
  }

  Int xUnit = (iWidth >> 2);
  Int yUnit = (iHeight >> 2);

  Pel *pDstY0 = pDstY;
  pGradX0 = m_pGradX0; pGradX1 = m_pGradX1;
  pGradY0 = m_pGradY0; pGradY1 = m_pGradY1;

  for (Int yu = 0; yu < yUnit; yu++)
  {
    for (Int xu = 0; xu < xUnit; xu++)
    {
      Int64 sGxdI = 0, sGydI = 0, sGxGy = 0, sGx2 = 0, sGy2 = 0;
      Int64 tmpx = 0, tmpy = 0;

      m_piDotProductTemp1 = m_piDotProduct1 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp2 = m_piDotProduct2 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp3 = m_piDotProduct3 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp5 = m_piDotProduct5 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp6 = m_piDotProduct6 + ((yu*iWidthG + xu) << 2);

      calcBlkGradient(xu << 2, yu << 2, m_piDotProductTemp1, m_piDotProductTemp2, m_piDotProductTemp3, m_piDotProductTemp5, m_piDotProductTemp6,
                      sGx2, sGy2, sGxGy, sGxdI, sGydI, iWidthG, iHeightG);

      sGxdI >>= 4;
      sGydI >>= 4;
      sGxGy >>= 4;
      sGx2 >>= 4;
      sGy2 >>= 4;

      sGx2 += regularizator_1;
      sGy2 += regularizator_2;

      if (sGx2 > denom_min_1)
      {
        tmpx = divide64(sGxdI, sGx2);
        tmpx = Clip3(-limit, limit, tmpx);
      }
      if (sGy2 > denom_min_2)
      {
        tmpy = divide64((sGydI - tmpx * sGxGy), sGy2);
        tmpy = Clip3(-limit, limit, tmpy);
      }

      pSrcY0Temp = pSrcY0 + ((yu*iSrc0Stride + xu) << 2);
      pSrcY1Temp = pSrcY1 + ((yu*iSrc0Stride + xu) << 2);
      pGradX0 = m_pGradX0 + ((yu*iWidthG + xu) << 2);
      pGradX1 = m_pGradX1 + ((yu*iWidthG + xu) << 2);
      pGradY0 = m_pGradY0 + ((yu*iWidthG + xu) << 2);
      pGradY1 = m_pGradY1 + ((yu*iWidthG + xu) << 2);

      pDstY0 = pDstY + ((yu*iDstStride + xu) << 2);

      // apply BIO offset for the sub-block
      for (int y = 0; y < 4; y++)
      {
        for (int x = 0; x < 4; x++)
        {
          Int b = (Int)tmpx * (pGradX0[x] - pGradX1[x]) + (Int)tmpy * (pGradY0[x] - pGradY1[x]);
          b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));

          pDstY0[x] = ( ClipPel( ( Short ) ( ( pSrcY0Temp[x] + pSrcY1Temp[x] + b + offset ) >> shiftNum ), clpRng ) );
        }
        pDstY0 += iDstStride; pSrcY0Temp += iSrc0Stride; pSrcY1Temp += iSrc1Stride;
        pGradX0 += iWidthG; pGradX1 += iWidthG; pGradY0 += iWidthG; pGradY1 += iWidthG;
      }

    }  // xu
  }  // yu
}
#endif

#if JEM_TOOLS
Void InterPrediction::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const Bool& bBIOApplied )
#else
Void InterPrediction::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs )
#endif
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
#if JEM_TOOLS
    if( bBIOApplied )
    {
      applyBiOptFlow( pu, pcYuvSrc0, pcYuvSrc1, iRefIdx0, iRefIdx1, pcYuvDst, clipBitDepths );
    }
    pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, bBIOApplied );
#else
    pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs );
#endif
  }
  else if( iRefIdx0 >= 0 && iRefIdx1 < 0 )
  {
    pcYuvDst.copyClip( pcYuvSrc0, clpRngs );
  }
  else if( iRefIdx0 < 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst.copyClip( pcYuvSrc1, clpRngs );
  }
}

Void InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList )
{
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  if( eRefPicList != REF_PIC_LIST_X )
  {
#if JEM_TOOLS
    if( !pu.cu->LICFlag && ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
#else
    if( ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
#endif
    {
      xPredInterUni         ( pu,          eRefPicList, predBuf, true );
      xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred );
    }
    else
    {
      xPredInterUni( pu, eRefPicList, predBuf, false );
    }
  }
  else
  {
#if JEM_TOOLS
    if( pu.mergeType != MRG_TYPE_DEFAULT_N )
    {
      xSubPuMC( pu, predBuf, eRefPicList );
    }
    else if( xCheckIdenticalMotion( pu ) )
#else
    if( xCheckIdenticalMotion( pu ) )
#endif
    {
      xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false );
    }
    else
    {
      xPredInterBi( pu, predBuf );
    }
  }
  return;
}

Void InterPrediction::motionCompensation( CodingUnit &cu, const RefPicList &eRefPicList )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
#if JEM_TOOLS
    pu.mvRefine = true;
    motionCompensation( pu, predBuf, eRefPicList );
    pu.mvRefine = false;
#else
    motionCompensation( pu, predBuf, eRefPicList );
#endif
  }
}

Void InterPrediction::motionCompensation( PredictionUnit &pu, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  motionCompensation( pu, predBuf, eRefPicList );
}

#if JEM_TOOLS
/** Function for sub-block based Overlapped Block Motion Compensation (OBMC).
*
* This function can:
* 1. Perform sub-block OBMC for a CU.
* 2. Before motion estimation, subtract (scaled) predictors generated by applying neighboring motions to current CU/PU from the original signal of current CU/PU,
*    to make the motion estimation biased to OBMC.
*/
Void InterPrediction::subBlockOBMC( CodingUnit  &cu )
{
  if( !cu.cs->sps->getSpsNext().getUseOBMC() || !cu.obmcFlag )
  {
    return;
  }

  for( auto &pu : CU::traversePUs( cu ) )
  {
    subBlockOBMC( pu, nullptr, false );
  }
}

Void InterPrediction::subBlockOBMC( PredictionUnit  &pu, PelUnitBuf* pDst, Bool bOBMC4ME )
{
  if( !pu.cs->sps->getSpsNext().getUseOBMC() || !pu.cu->obmcFlag )
  {
    return;
  }

  CodingStructure &cs        = *pu.cs;

  PelUnitBuf pcYuvPred       = pDst == nullptr ? pu.cs->getPredBuf( pu ) : *pDst;
  PelUnitBuf pcYuvTmpPred1   = m_tmpObmcBuf.subBuf( UnitAreaRelative( *pu.cu, pu ) );

  const PartSize   ePartSize = pu.cu->partSize;
  const UnitArea   orgPuArea = pu;

  const UInt uiWidth         = pu.lwidth();
  const UInt uiHeight        = pu.lheight();

  const UInt uiMinCUW        = pu.cs->pcv->minCUWidth;

  const UInt uiOBMCBlkSize   = pu.cs->sps->getSpsNext().getOBMCBlkSize();

  const UInt uiHeightInBlock = uiHeight         / uiMinCUW;
  const UInt uiWidthInBlock  = uiWidth          / uiMinCUW;
  const UInt uiHeightInCU    = pu.cu->lheight() / uiMinCUW;
  const UInt uiWidhtInCU     = pu.cu->lwidth()  / uiMinCUW;
  const UInt uiStep          = uiOBMCBlkSize    / uiMinCUW;

  const Bool bOBMCSimp       = cs.pcv->only2Nx2N ? uiWidth * uiHeight < 64 : ( pu.cu->lwidth() == 8 && ePartSize != SIZE_2Nx2N );

  const Bool bATMVP          = ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.mergeType == MRG_TYPE_SUBPU_ATMVP_EXT );
  const Bool bFruc           = ( pu.frucMrgMode == FRUC_MERGE_TEMPLATE || pu.frucMrgMode == FRUC_MERGE_BILATERALMV );
  const Bool bAffine         = pu.cu->affine;

        Int  i1stPUWidth     = -1, i1stPUHeight = -1;
        Bool b2ndPU          = false;
  const Bool bVerticalPU     = false;
  const Bool bHorizontalPU   = false;

  const Bool bTwoPUs         = ( bVerticalPU || bHorizontalPU );

  if( bTwoPUs )
  {
    PredictionUnit* fPU = pu.cu->firstPU;
    i1stPUWidth   = fPU->lwidth();
    i1stPUHeight  = fPU->lheight();

    if( pu.cu->firstPU->next && pu.lumaPos() == fPU->next->lumaPos() )
    {
      b2ndPU = true;
    }

    i1stPUWidth  /= uiMinCUW;
    i1stPUHeight /= uiMinCUW;
  }

  const Int avgLength      = ( cs.pcv->rectCUs ) ? 1 << ( ( ( ( int ) log2( pu.cu->lumaSize().width ) + ( int ) log2( pu.cu->lumaSize().height ) - 3 ) >> 1 ) + MIN_CU_LOG2 ) : pu.cu->lumaSize().width;
  const Int nRefineBlkSize = std::max( avgLength >> pu.cs->slice->getSPS()->getSpsNext().getFRUCSmallBlkRefineDepth(), FRUC_MERGE_REFINE_MINBLKSIZE );

  const Bool bNormal2Nx2N  = ePartSize == SIZE_2Nx2N && !bATMVP && !bFruc && !bAffine;
  const Bool bSubMotion    = ePartSize == SIZE_2Nx2N && ( bATMVP || bFruc || bAffine );

  MotionInfo currMi  = pu.getMotionInfo();
  MotionInfo NeighMi = MotionInfo();

  Int maxDir =  bNormal2Nx2N ? 2 : 4;

  for (Int iSubX = 0; iSubX < uiWidthInBlock; iSubX += uiStep)
  {
    for (Int iSubY = 0; iSubY < uiHeightInBlock; iSubY += uiStep)
    {
      if (bNormal2Nx2N && iSubX && iSubY)
      {
        continue;
      }

      Bool bCURBoundary = bVerticalPU   ? ( iSubX == uiWidhtInCU  - uiStep ) : b2ndPU ? iSubX + i1stPUWidth   == uiWidhtInCU  - uiStep : ( iSubX == uiWidhtInCU  - uiStep ) ;
      Bool bCUBBoundary = bHorizontalPU ? ( iSubY == uiHeightInCU - uiStep ) : b2ndPU ? iSubY + i1stPUHeight  == uiHeightInCU - uiStep : ( iSubY == uiHeightInCU - uiStep ) ;

      for (Int iDir = 0; iDir < maxDir; iDir++) //iDir: 0 - above, 1 - left, 2 - below, 3 - right
      {
        if ((iDir == 3 && bCURBoundary) || (iDir == 2 && bCUBBoundary))
        {
          continue;
        }

        Bool bVerPUBound = false;
        Bool bHorPUBound = false;

        if( bNormal2Nx2N ) //skip unnecessary check for CU boundary
        {
          if( ( iDir == 1 && !iSubY && iSubX ) || ( iDir == 0 && !iSubX && iSubY ) )
          {
            continue;
          }
        }
        else
        {
          Bool bCheckNeig = bSubMotion || ( iSubX == 0 && iDir == 1 ) || ( iSubY == 0 && iDir == 0 ); //CU boundary or NxN or 2nx2n_ATMVP
          if( !bCheckNeig && bTwoPUs )
          {
            bCheckNeig |= bFruc;
            bCheckNeig |= bATMVP;

            //PU boundary
            if( !b2ndPU )
            {
              bVerPUBound = bVerticalPU   && ( ( iDir == 2 && iSubY == i1stPUHeight - uiStep ) );
              bHorPUBound = bHorizontalPU && ( ( iDir == 3 && iSubX == i1stPUWidth  - uiStep ) );
            }

            bCheckNeig |= ( bVerPUBound || bHorPUBound );
          }
          if( !bCheckNeig )
          {
            continue;
          }
        }

#if JVET_K0346
        bool bSubBlockOBMCSimp = (bOBMCSimp || ((pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.mergeType == MRG_TYPE_SUBPU_ATMVP_EXT) && (1 << pu.cs->slice->getSubPuMvpSubblkLog2Size()) == 4));
#else
        Bool bSubBlockOBMCSimp = ( bOBMCSimp || ( (pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.mergeType == MRG_TYPE_SUBPU_ATMVP_EXT ) && ( 1 << pu.cs->sps->getSpsNext().getSubPuMvpLog2Size() ) == 4 ) );
#endif
        bSubBlockOBMCSimp |= ( bFruc && nRefineBlkSize == 4 );
        bSubBlockOBMCSimp |= bAffine;

        if( PU::getNeighborMotion( pu, NeighMi, Position( iSubX * uiMinCUW, iSubY * uiMinCUW ), iDir, ( bATMVP || bFruc || bAffine ) ) )
        {
          //store temporary motion information
          pu              = NeighMi;
          pu.cu->partSize = SIZE_2Nx2N;
          pu.cu->affine   = false;
          pu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( orgPuArea.lumaPos().offset( iSubX * uiMinCUW, iSubY * uiMinCUW ), Size{ uiOBMCBlkSize, uiOBMCBlkSize } ) ) );

          const UnitArea predArea = UnitAreaRelative( orgPuArea, pu );

          PelUnitBuf cPred = pcYuvPred    .subBuf( predArea );
          PelUnitBuf cTmp1 = pcYuvTmpPred1.subBuf( predArea );

          xSubBlockMotionCompensation( pu, cTmp1 );

          if( bOBMC4ME )
          {
            xSubtractOBMC( pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
          }
          else
          {
            xSubblockOBMC( COMPONENT_Y,  pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cb, pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cr, pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
          }

          //restore motion information
          pu.cu->partSize  = ePartSize;
          pu               = currMi;
          pu.cu->affine    = bAffine;
          pu.UnitArea::operator=( orgPuArea );
        }
      }
    }
  }
}


// Function for (weighted) averaging predictors of current block and predictors generated by applying neighboring motions to current block.
Void InterPrediction::xSubblockOBMC(const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, Int iDir, Bool bOBMCSimp)
{
  int iWidth  = pu.blocks[eComp].width;
  int iHeight = pu.blocks[eComp].height;

  if( iWidth == 0 || iHeight == 0 )
  {
    return;
  }

  PelBuf *pDst = &pcYuvPredDst.bufs[eComp];//.at(0, 0);
  PelBuf *pSrc = &pcYuvPredSrc.bufs[eComp];//.at(0, 0);

  if( iDir == 0 ) //above
  {
    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, 0 ) = ( 3 * pDst->at( i, 0 ) + pSrc->at( i, 0 ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( i, 1 ) = ( 7 * pDst->at( i, 1 ) + pSrc->at( i, 1 ) + 4 ) >> 3;
      }

      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( i, 2 ) = ( 15 * pDst->at( i, 2 ) + pSrc->at( i, 2 ) +  8 ) >> 4;
        pDst->at( i, 3 ) = ( 31 * pDst->at( i, 3 ) + pSrc->at( i, 3 ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( 0, i ) = ( 3 * pDst->at( 0, i ) + pSrc->at( 0, i ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( 1, i ) = ( 7 * pDst->at( 1, i ) + pSrc->at( 1, i ) + 4 ) >> 3;
      }

      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( 2, i ) = ( 15 * pDst->at( 2, i ) + pSrc->at( 2, i ) +  8 ) >> 4;
        pDst->at( 3, i ) = ( 31 * pDst->at( 3, i ) + pSrc->at( 3, i ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, iHeight - 1 ) = ( 3 * pDst->at( i, iHeight - 1 ) + pSrc->at( i, iHeight - 1 ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( i, iHeight - 2 ) = ( 7 * pDst->at( i, iHeight - 2 ) + pSrc->at( i, iHeight - 2 ) + 4 ) >> 3;
      }
      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( i, iHeight - 3 ) = ( 15 * pDst->at( i, iHeight - 3 ) + pSrc->at( i, iHeight - 3 ) +  8 ) >> 4;
        pDst->at( i, iHeight - 4 ) = ( 31 * pDst->at( i, iHeight - 4 ) + pSrc->at( i, iHeight - 4 ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 3 ) //right
  {
    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( iWidth - 1, i ) = ( 3 * pDst->at( iWidth - 1, i ) + pSrc->at( iWidth - 1, i ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( iWidth - 2, i ) = ( 7 * pDst->at( iWidth - 2, i ) + pSrc->at( iWidth - 2, i ) + 4 ) >> 3;
      }

      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( iWidth - 3, i ) = ( 15 * pDst->at( iWidth - 3, i ) + pSrc->at( iWidth - 3, i ) +  8 ) >> 4;
        pDst->at( iWidth - 4, i ) = ( 31 * pDst->at( iWidth - 4, i ) + pSrc->at( iWidth - 4, i ) + 16 ) >> 5;
      }
    }
  }
}

// Function for subtracting (scaled) predictors generated by applying neighboring motions to current block from the original signal of current block.
Void InterPrediction::xSubtractOBMC( PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, Int iDir, Bool bOBMCSimp )
{
  int iWidth  = pu.lwidth();
  int iHeight = pu.lheight();

  PelBuf *pDst = &pcYuvPredDst.bufs[COMPONENT_Y];
  PelBuf *pSrc = &pcYuvPredSrc.bufs[COMPONENT_Y];

  if( iDir == 0 ) //above
  {
    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, 0 ) += ( pDst->at( i, 0 ) - pSrc->at( i, 0 ) + 2 ) >> 2;
      pDst->at( i, 1 ) += ( pDst->at( i, 1 ) - pSrc->at( i, 1 ) + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      for( int i = 0; i < iWidth; i++ )
      {
        pDst->at( i, 2 ) += ( pDst->at( i, 2 ) - pSrc->at( i, 2 ) +  8 ) >> 4;
        pDst->at( i, 3 ) += ( pDst->at( i, 3 ) - pSrc->at( i, 3 ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( 0, i ) += ( pDst->at( 0, i ) - pSrc->at( 0, i ) + 2 ) >> 2;
      pDst->at( 1, i ) += ( pDst->at( 1, i ) - pSrc->at( 1, i ) + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      for( int i = 0; i < iHeight; i++ )
      {
        pDst->at( 2, i ) += ( pDst->at( 2, i ) - pSrc->at( 2, i ) +  8 ) >> 4;
        pDst->at( 3, i ) += ( pDst->at( 3, i ) - pSrc->at( 3, i ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    if( !bOBMCSimp )
    {
      for( int i = 0; i < iWidth; i++ )
      {
        pDst->at( i, iHeight - 4 ) += ( pDst->at( i, iHeight - 4 ) - pSrc->at( i, iHeight - 4 ) + 16 ) >> 5;
        pDst->at( i, iHeight - 3 ) += ( pDst->at( i, iHeight - 3 ) - pSrc->at( i, iHeight - 3 ) +  8 ) >> 4;
      }
    }

    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, iHeight - 2 ) += ( pDst->at( i, iHeight - 2 ) - pSrc->at( i, iHeight - 2 ) + 4 ) >> 3;
      pDst->at( i, iHeight - 1 ) += ( pDst->at( i, iHeight - 1 ) - pSrc->at( i, iHeight - 1 ) + 2 ) >> 2;
    }
  }

  if( iDir == 3 ) //right
  {
    if( !bOBMCSimp )
    {
      for( int i = 0; i < iHeight; i++ )
      {
        pDst->at( iWidth - 4, i ) += ( pDst->at( iWidth - 4, i ) - pSrc->at( iWidth - 4, i ) + 16 ) >> 5;
        pDst->at( iWidth - 3, i ) += ( pDst->at( iWidth - 3, i ) - pSrc->at( iWidth - 3, i ) +  8 ) >> 4;
      }
    }

    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( iWidth - 2, i ) += ( pDst->at( iWidth - 2, i ) - pSrc->at( iWidth - 2, i ) + 4 ) >> 3;
      pDst->at( iWidth - 1, i ) += ( pDst->at( iWidth - 1, i ) - pSrc->at( iWidth - 1, i ) + 2 ) >> 2;
    }
  }
}
#endif

#if JEM_TOOLS
Void InterPrediction::xSubBlockMotionCompensation( PredictionUnit &pu, PelUnitBuf &pcYuvPred )
{
  if( xCheckIdenticalMotion( pu ) )
  {
    xPredInterUni( pu, REF_PIC_LIST_0, pcYuvPred, false );
  }
  else
  {
    xPredInterBi( pu, pcYuvPred, true );
  }
}
#endif

#if JEM_TOOLS
Void InterPrediction::xGradFilterX( const Pel* piRefY, Int iRefStride, Pel* piDstY, Int iDstStride, Int iWidth, Int iHeight, Int iMVyFrac, Int iMVxFrac, const Int bitDepth )
{
  static const int iBIOGradShift = 4;
  if( iMVyFrac == 0 )
  {
    gradFilter1DHor( piRefY, iRefStride,  iWidth, iHeight, iDstStride,  piDstY, iMVxFrac, iBIOGradShift );
    return;
  }

  int   tmpStride = iWidth + BIO_FILTER_LENGTH_MINUS_1;
  Pel*  tmp       = m_filteredBlockTmp[0][0];
  int   shift0    = bitDepth-8;
  int   shift1    = 6 + iBIOGradShift - shift0;
  fracFilter2DVer ( piRefY - BIO_FILTER_HALF_LENGTH_MINUS_1, iRefStride, iWidth+BIO_FILTER_LENGTH_MINUS_1, iHeight, tmpStride,  tmp,    iMVyFrac, shift0 );
  JVET_J0090_SET_CACHE_ENABLE( false );
  gradFilter2DHor ( tmp    + BIO_FILTER_HALF_LENGTH_MINUS_1, tmpStride,  iWidth,                           iHeight, iDstStride, piDstY, iMVxFrac, shift1 );
  JVET_J0090_SET_CACHE_ENABLE( true );
}

Void InterPrediction::xGradFilterY( const Pel* piRefY, Int iRefStride, Pel* piDstY, Int iDstStride, Int iWidth, Int iHeight, Int iMVyFrac, Int iMVxFrac, const Int bitDepth )
{
  static const Int iBIOGradShift = 4;
  if( iMVxFrac == 0 )
  {
    gradFilter1DVer( piRefY, iRefStride, iWidth, iHeight, iDstStride, piDstY, iMVyFrac, iBIOGradShift );
    return;
  }

  Int   tmpStride = iWidth + BIO_FILTER_LENGTH_MINUS_1;
  Pel*  tmp       = m_filteredBlockTmp[0][0];
  Int   shift0    = bitDepth-8;
  Int   shift1    = 6 + iBIOGradShift - shift0;
  gradFilter2DVer ( piRefY - BIO_FILTER_HALF_LENGTH_MINUS_1, iRefStride, iWidth+BIO_FILTER_LENGTH_MINUS_1, iHeight, tmpStride,  tmp,    iMVyFrac, shift0 );
  JVET_J0090_SET_CACHE_ENABLE( false );
  fracFilter2DHor ( tmp    + BIO_FILTER_HALF_LENGTH_MINUS_1, tmpStride,  iWidth,                           iHeight, iDstStride, piDstY, iMVxFrac, shift1 );
  JVET_J0090_SET_CACHE_ENABLE( true );
}

Pel InterPrediction::optical_flow_averaging( Int64 s1, Int64 s2, Int64 s3, Int64 s5, Int64 s6, Pel pGradX0, Pel pGradX1, Pel pGradY0, Pel pGradY1, Pel pSrcY0Temp, Pel pSrcY1Temp,
                                             const int shiftNum, const int offset, const Int64 limit, const Int64 denom_min_1, const Int64 denom_min_2, const ClpRng& clpRng )
{
  Int64 vx = 0;
  Int64 vy = 0;
  Int64 b=0;

  if( s1 > denom_min_1 )
  {
    vx = s3  / s1;
    vx = ( vx > limit ? limit : vx < -limit ? -limit : vx );
  }
  if( s5 > denom_min_2 )
  {
    vy = ( s6 - vx*s2 ) / s5;
    vy = ( vy > limit ? limit : vy < -limit ? -limit : vy );
  }

  b = vx * ( pGradX0 - pGradX1 ) + vy * ( pGradY0 - pGradY1 );
  b = ( b > 0 ? (b+32)>>6 : -((-b+32)>>6) );
  return ClipPel( (Short)((pSrcY0Temp + pSrcY1Temp + b + offset) >> shiftNum), clpRng );
}

const Short m_lumaGradientFilter[4<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][BIO_FILTER_LENGTH] =
{
  {       8,     -39,      -3,      46,     -17,       5 },   //0
  {       8,     -32,     -13,      50,     -18,       5 },   //1 -->-->
  {       7,     -27,     -20,      54,     -19,       5 },   //2 -->
  {       6,     -21,     -29,      57,     -18,       5 },   //3 -->-->
  {       4,     -17,     -36,      60,     -15,       4 },   //4
  {       3,      -9,     -44,      61,     -15,       4 },   //5 -->-->
  {       1,      -4,     -48,      61,     -13,       3 },   //6 -->
  {       0,       1,     -54,      60,      -9,       2 },   //7 -->-->
  {      -1,       4,     -57,      57,      -4,       1 },   //8
  {      -2,       9,     -60,      54,      -1,       0 },   //9 -->-->
  {      -3,      13,     -61,      48,       4,      -1 },   //10-->
  {      -4,      15,     -61,      44,       9,      -3 },   //11-->-->
  {      -4,      15,     -60,      36,      17,      -4 },   //12
  {      -5,      18,     -57,      29,      21,      -6 },   //13-->-->
  {      -5,      19,     -54,      20,      27,      -7 },   //14-->
  {      -5,      18,     -50,      13,      32,      -8 }    //15-->-->
};

const Short m_lumaInterpolationFilter[4<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][BIO_FILTER_LENGTH] =
{
  {       0,       0,      64,       0,       0,       0 },   //0
  {       1,      -3,      64,       4,      -2,       0 },   //1 -->-->
  {       1,      -6,      62,       9,      -3,       1 },   //2 -->
  {       2,      -8,      60,      14,      -5,       1 },   //3 -->-->
  {       2,      -9,      57,      19,      -7,       2 },   //4
  {       3,     -10,      53,      24,      -8,       2 },   //5 -->-->
  {       3,     -11,      50,      29,      -9,       2 },   //6 -->
  {       3,     -11,      44,      35,     -10,       3 },   //7 -->-->
  {       1,      -7,      38,      38,      -7,       1 },   //8
  {       3,     -10,      35,      44,     -11,       3 },   //9 -->-->
  {       2,      -9,      29,      50,     -11,       3 },   //10-->
  {       2,      -8,      24,      53,     -10,       3 },   //11-->-->
  {       2,      -7,      19,      57,      -9,       2 },   //12
  {       1,      -5,      14,      60,      -8,       2 },   //13-->-->
  {       1,      -3,       9,      62,      -6,       1 },   //14-->
  {       0,      -2,       4,      64,      -3,       1 }    //15-->-->
};

inline Void InterPrediction::gradFilter2DVer( const Pel* piSrc, Int iSrcStride,  Int width, Int height, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1 * iSrcStride;
  const Pel*  piSrcTmp1     = piSrcTmp  + iSrcStride;
  const Pel*  piSrcTmp2     = piSrcTmp1 + iSrcStride;
  const Pel*  piSrcTmp3     = piSrcTmp2 + iSrcStride;
  const Pel*  piSrcTmp4     = piSrcTmp3 + iSrcStride;
  const Pel*  piSrcTmp5     = piSrcTmp4 + iSrcStride;
  const Short *const coeffs = m_lumaGradientFilter[iMV];

  const int   iOffSet       = ( iShift > 0 ? ( 1 << ( iShift - 1 ) ) : 0 );

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      JVET_J0090_CACHE_ACCESS( piSrcTmp,  __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp1, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp2, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp3, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp4, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp5, __FILE__, __LINE__ );
      iSum     =  (  coeffs[0] * *piSrcTmp++
                   + coeffs[1] * *piSrcTmp1++
                   + coeffs[2] * *piSrcTmp2++
                   + coeffs[3] * *piSrcTmp3++
                   + coeffs[4] * *piSrcTmp4++
                   + coeffs[5] * *piSrcTmp5++ );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
    }

    piSrcTmp  += iSrcStride;
    piSrcTmp1 += iSrcStride;
    piSrcTmp2 += iSrcStride;
    piSrcTmp3 += iSrcStride;
    piSrcTmp4 += iSrcStride;
    piSrcTmp5 += iSrcStride;
    piDst     += iDstStride;
  }
}

inline Void InterPrediction::gradFilter1DVer( const Pel* piSrc, Int iSrcStride, Int width, Int height, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1 * iSrcStride;
  const Pel*  piSrcTmp1     = piSrcTmp  + iSrcStride;
  const Pel*  piSrcTmp2     = piSrcTmp1 + iSrcStride;
  const Pel*  piSrcTmp3     = piSrcTmp2 + iSrcStride;
  const Pel*  piSrcTmp4     = piSrcTmp3 + iSrcStride;
  const Pel*  piSrcTmp5     = piSrcTmp4 + iSrcStride;
  const Short *const coeffs = m_lumaGradientFilter[iMV];

  const int   iOffSet       = 1 << ( iShift - 1 );

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      JVET_J0090_CACHE_ACCESS( piSrcTmp,  __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp1, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp2, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp3, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp4, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp5, __FILE__, __LINE__ );
      iSum     =   (  coeffs[0] * *piSrcTmp++
                    + coeffs[1] * *piSrcTmp1++
                    + coeffs[2] * *piSrcTmp2++
                    + coeffs[3] * *piSrcTmp3++
                    + coeffs[4] * *piSrcTmp4++
                    + coeffs[5] * *piSrcTmp5++ );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
    }

    piSrcTmp  += iSrcStride;
    piSrcTmp1 += iSrcStride;
    piSrcTmp2 += iSrcStride;
    piSrcTmp3 += iSrcStride;
    piSrcTmp4 += iSrcStride;
    piSrcTmp5 += iSrcStride;
    piDst     += iDstStride;
  }
}

inline Void InterPrediction::gradFilter1DHor( const Pel* piSrc, Int iSrcStride, Int width, Int height, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1;
  const int   iOffSet       = 1 << ( iShift - 1 );
  const Short *const coeffs = m_lumaGradientFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      for ( size_t i = 0 ; i < 6 ; i++ )
      {
        JVET_J0090_CACHE_ACCESS( piSrcTmp + i, __FILE__, __LINE__ );
      }
      iSum     =  (  coeffs[0] * piSrcTmp[0]
                   + coeffs[1] * piSrcTmp[1]
                   + coeffs[2] * piSrcTmp[2]
                   + coeffs[3] * piSrcTmp[3]
                   + coeffs[4] * piSrcTmp[4]
                   + coeffs[5] * piSrcTmp[5] );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;

      piSrcTmp++;
    }

    piSrcTmp += iSrcStride;
    piDst    += iDstStride;
  }
}

inline Void InterPrediction::gradFilter2DHor( const Pel* piSrc, Int iSrcStride, Int width, Int height, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel*        piDst         = rpiDst;
  Int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1;
  const Int   iOffSet       = ( iShift > 0 ? 1 << ( iShift - 1 ) : 0 );
  const Short *const coeffs = m_lumaGradientFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      for ( size_t i = 0 ; i < 6 ; i++ )
      {
        JVET_J0090_CACHE_ACCESS( piSrcTmp + i, __FILE__, __LINE__ );
      }
      iSum     =   (  coeffs[0] * piSrcTmp[0]
                    + coeffs[1] * piSrcTmp[1]
                    + coeffs[2] * piSrcTmp[2]
                    + coeffs[3] * piSrcTmp[3]
                    + coeffs[4] * piSrcTmp[4]
                    + coeffs[5] * piSrcTmp[5] );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;

      piSrcTmp++;
    }

    piSrcTmp += iSrcStride;
    piDst    += iDstStride;
  }
}

inline Void InterPrediction::fracFilter2DVer( const Pel* piSrc, Int iSrcStride, Int width, Int height, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1 * iSrcStride;
  const Pel*  piSrcTmp1     = piSrcTmp  + iSrcStride;
  const Pel*  piSrcTmp2     = piSrcTmp1 + iSrcStride;
  const Pel*  piSrcTmp3     = piSrcTmp2 + iSrcStride;
  const Pel*  piSrcTmp4     = piSrcTmp3 + iSrcStride;
  const Pel*  piSrcTmp5     = piSrcTmp4 + iSrcStride;
  const int   iOffSet       = ( iShift > 0 ? ( 1 << ( iShift - 1 ) ) - ( 8192 << iShift ) : -8192 );
  const Short *const coeffs = m_lumaInterpolationFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      JVET_J0090_CACHE_ACCESS( piSrcTmp,  __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp1, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp2, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp3, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp4, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp5, __FILE__, __LINE__ );
      iSum     =  (  coeffs[0] * *piSrcTmp ++
                   + coeffs[1] * *piSrcTmp1++
                   + coeffs[2] * *piSrcTmp2++
                   + coeffs[3] * *piSrcTmp3++
                   + coeffs[4] * *piSrcTmp4++
                   + coeffs[5] * *piSrcTmp5++ );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
    }

    piSrcTmp  += iSrcStride;
    piSrcTmp1 += iSrcStride;
    piSrcTmp2 += iSrcStride;
    piSrcTmp3 += iSrcStride;
    piSrcTmp4 += iSrcStride;
    piSrcTmp5 += iSrcStride;
    piDst     += iDstStride;
  }
}

inline Void InterPrediction::fracFilter2DHor( const Pel* piSrc, Int iSrcStride, Int width, Int height, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1;
  const int   iOffSet       = ( iShift > 0 ? 1 << ( iShift - 1 ) : 0 );
  const Short *const coeffs = m_lumaInterpolationFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      for ( size_t i = 0 ; i < 6 ; i++ )
      {
        JVET_J0090_CACHE_ACCESS( piSrcTmp + i, __FILE__, __LINE__ );
      }
      iSum     = (  coeffs[0] * piSrcTmp[0]
                  + coeffs[1] * piSrcTmp[1]
                  + coeffs[2] * piSrcTmp[2]
                  + coeffs[3] * piSrcTmp[3]
                  + coeffs[4] * piSrcTmp[4]
                  + coeffs[5] * piSrcTmp[5] );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
      piSrcTmp++;
    }

    piSrcTmp += iSrcStride;
    piDst    += iDstStride;
  }
}

inline Int GetMSB64( UInt64 x )
{
  Int iMSB = 0, bits = (sizeof(Int64) << 3);
  UInt64 y = 1;

  while (x > 1)
  {
    bits >>= 1;
    y = x >> bits;

    if (y)
    {
      x = y;
      iMSB += bits;
    }
  }

  iMSB += (Int)y;

  return iMSB;
}

inline Int64 InterPrediction::divide64( Int64 numer, Int64 denom )
{
  Int64 d;
  const Int64 iShiftA2 = 6;
  const Int64 iAccuracyShift = 15;
  const Int64 iMaxVal = 63;
  Int64 iScaleShiftA2 = 0;
  Int64 iScaleShiftA1 = 0;

  UChar signA1 = numer < 0;
  UChar signA2 = denom < 0;

  numer = (signA1) ? -numer : numer;
  denom = (signA2) ? -denom : denom;

  iScaleShiftA2 = GetMSB64(denom) - iShiftA2;
  iScaleShiftA1 = iScaleShiftA2 - 12;

  if (iScaleShiftA1 < 0)
  {
    iScaleShiftA1 = 0;
  }

  if (iScaleShiftA2 < 0)
  {
    iScaleShiftA2 = 0;
  }

  Int64 iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iScaleShiftA1;

  Int64 a2s = (denom >> iScaleShiftA2) > iMaxVal ? iMaxVal : (denom >> iScaleShiftA2);
  Int64 a1s = (numer >> iScaleShiftA1);

  Int64 aI64 = (a1s * (Int64)m_uiaBIOShift[a2s]) >> iScaleShiftA;

  d = (signA1 + signA2 == 1) ? -aI64 : aI64;

  return d;
}

Void InterPrediction::calcBlkGradient( Int sx, Int sy, Int64 *arraysGx2, Int64 *arraysGxGy, Int64 *arraysGxdI, Int64 *arraysGy2, Int64 *arraysGydI, Int64 &sGx2, Int64 &sGy2, Int64 &sGxGy, Int64 &sGxdI, Int64 &sGydI, Int iWidth, Int iHeight )
{
  static const UInt weightTbl[8][8] = { {1, 2, 3, 4, 4, 3, 2, 1},
      {2, 4, 6, 8, 8, 6, 4, 2},
      {3, 6, 9, 12, 12, 9, 6, 3},
      {4, 8, 12, 16, 16, 12, 8, 4},
      {4, 8, 12, 16, 16, 12, 8, 4},
      {3, 6, 9, 12, 12, 9, 6, 3},
      {2, 4, 6, 8, 8, 6, 4, 2},
      {1, 2, 3, 4, 4, 3, 2, 1 } };

  Int64 *pGx2 = arraysGx2;
  Int64 *pGy2 = arraysGy2;
  Int64 *pGxGy = arraysGxGy;
  Int64 *pGxdI = arraysGxdI;
  Int64 *pGydI = arraysGydI;

  Int x0;

  if (sy > 0 && iHeight > 4)
  {
    pGx2 -= (iWidth << 1);
    pGy2 -= (iWidth << 1);
    pGxGy -= (iWidth << 1);
    pGxdI -= (iWidth << 1);
    pGydI -= (iWidth << 1);
  }

  for (Int y = -2; y < 6; y++)
  {
    for (Int x = -2; x < 6; x++)
    {
      UInt weight = weightTbl[y + 2][x + 2];
      x0 = x;
      if (sx + x < 0)       x0 = 0;
      if (sx + x >= iWidth) x0 = 3;

      sGx2 += weight*pGx2[x0];
      sGy2 += weight*pGy2[x0];
      sGxGy += weight*pGxGy[x0];
      sGxdI += weight*pGxdI[x0];
      sGydI += weight*pGydI[x0];
    }

    if (sy + y < 0 || sy + y >= iHeight - 1)
    {
      continue;
    }

    pGx2 += iWidth;
    pGy2 += iWidth;
    pGxGy += iWidth;
    pGxdI += iWidth;
    pGydI += iWidth;
  }
}

static const Int FRUC_MERGE_MV_SEARCHPATTERN_CROSS    = 0;
static const Int FRUC_MERGE_MV_SEARCHPATTERN_SQUARE   = 1;
static const Int FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND  = 2;
static const Int FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON  = 3;

UInt InterPrediction::xFrucGetTempMatchCost( PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, UInt uiMVCost )
{
  const Int nMVUnit = 2;

  UInt uiCost = uiMVCost;

  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR = pu.cu->LICFlag;

  const SPS &sps = *pu.cs->sps;

  if( m_bFrucTemplateAvailabe[0] )
  {
    Mv mvTop( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvTop.setHighPrec();
    }
    mvTop += rCurMvField.mv;

    clipMv(mvTop, pu.cu->lumaPos(), sps);

    PelUnitBuf pcBufPredRefTop = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );
    PelUnitBuf pcBufPredCurTop = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );

    CHECK( rCurMvField.refIdx < 0, "invalid ref idx" );

    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, rCurMvField.refIdx), mvTop, pcBufPredRefTop, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);

    m_pcRdCost->setDistParam( cDistParam, pcBufPredCurTop.Y(), pcBufPredRefTop.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false );

    uiCost += cDistParam.distFunc( cDistParam ); //TODO: check distFunc
  }

  if( m_bFrucTemplateAvailabe[1] )
  {
    Mv mvLeft( - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvLeft.setHighPrec();
    }
    mvLeft += rCurMvField.mv;

    clipMv(mvLeft, pu.cu->lumaPos(), sps);

    PelUnitBuf pcBufPredRefLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );
    PelUnitBuf pcBufPredCurLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );

    CHECK( rCurMvField.refIdx < 0, "invalid ref idx" );

    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, rCurMvField.refIdx), mvLeft, pcBufPredRefLeft, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);

    m_pcRdCost->setDistParam( cDistParam, pcBufPredCurLeft.Y(), pcBufPredRefLeft.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false );

    uiCost += cDistParam.distFunc( cDistParam ); //TODO: check distFunc
  }

  return uiCost;
}

UInt InterPrediction::xFrucGetBilaMatchCost( PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, MvField& rPairMVField, UInt uiMVCost )
{
  UInt uiCost = MAX_UINT;

  if( PU::getMvPair( pu, eCurRefPicList , rCurMvField , rPairMVField ) )
  {
    RefPicList eTarRefPicList = ( RefPicList )( 1 - ( Int )eCurRefPicList );
    PelUnitBuf pcBufPredA = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], nWidth, nHeight) ) );
    PelUnitBuf pcBufPredB = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], nWidth, nHeight) ) );

    CHECK( rCurMvField.refIdx < 0 || rPairMVField.refIdx < 0, "invalid ref idx" );

    const SPS &sps = *pu.cs->sps;

    Mv mvAp = rCurMvField.mv;
    clipMv(mvAp, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, rCurMvField.refIdx), mvAp, pcBufPredA, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_BILATERALMV, false);

    Mv mvBp = rPairMVField.mv;
    clipMv(mvBp, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eTarRefPicList, rPairMVField.refIdx), mvBp, pcBufPredB, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_BILATERALMV, false);

    DistParam cDistParam;
    cDistParam.applyWeight = false;
    cDistParam.useMR = pu.cu->LICFlag;
    m_pcRdCost->setDistParam( cDistParam, pcBufPredA.Y(), pcBufPredB.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false );
    uiCost = cDistParam.distFunc( cDistParam )  + uiMVCost; //TODO: check distFunc
  }

  return uiCost;
}

Bool InterPrediction::xFrucIsInList( const MvField & rMvField , std::list<MvField> & rList )
{
  std::list<MvField>::iterator pos = rList.begin();
  while( pos != rList.end() )
  {
    if( rMvField == *pos )
      return( true );
    pos++;
  }
  return( false );
}

Void InterPrediction::xFrucInsertMv2StartList( const MvField & rMvField , std::list<MvField> & rList, Bool setHighPrec )
{
  CHECK( rMvField.refIdx < 0, "invalid ref idx" );

  MvField mf = rMvField;
  if( setHighPrec )
  {
    mf.mv.setHighPrec();
  }

  // do not use zoom in FRUC for now
  if( xFrucIsInList( mf , rList ) == false )
    rList.push_back( mf );
}

Void InterPrediction::xFrucCollectBlkStartMv( PredictionUnit& pu, const MergeCtx& mergeCtx, RefPicList eTargetRefList, Int nTargetRefIdx, AMVPInfo* pInfo )
{
  // add merge candidates to the list
  m_listMVFieldCand[0].clear();
  m_listMVFieldCand[1].clear();

  if ((nTargetRefIdx >= 0) && pInfo)   //Here we are in AMVP mode
  {
    // add AMVP candidates to the list
    for (Int nAMVPIndex = 0; nAMVPIndex < pInfo->numCand; nAMVPIndex++)
    {
      MvField mvCnd;
      mvCnd.setMvField( pInfo->mvCand[nAMVPIndex], nTargetRefIdx );
      xFrucInsertMv2StartList( mvCnd, m_listMVFieldCand[eTargetRefList], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
    }
  }

  for( Int nMergeIndex = 0; nMergeIndex < mergeCtx.numValidMergeCand << 1; nMergeIndex++ )
  {
    Bool mrgTpDflt = ( pu.cs->sps->getSpsNext().getUseSubPuMvp() ) ? mergeCtx.mrgTypeNeighbours[nMergeIndex>>1] == MRG_TYPE_DEFAULT_N : true;
    if( mergeCtx.mvFieldNeighbours[nMergeIndex].refIdx >= 0 && mrgTpDflt )
    {
      if( nTargetRefIdx >= 0 && ( mergeCtx.mvFieldNeighbours[nMergeIndex].refIdx != nTargetRefIdx || ( nMergeIndex & 0x01 ) != ( Int )eTargetRefList ) )
        continue;
      xFrucInsertMv2StartList( mergeCtx.mvFieldNeighbours[nMergeIndex] , m_listMVFieldCand[nMergeIndex&0x01], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
    }
  }

  // add uni-lateral candidates to the list
  if( pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
  {
    Position puPos    = pu.lumaPos();
    Size puSize       = pu.lumaSize();

    int halfPuHeight = ( ( puSize.height + 4 ) / 2 ) & ( ~3 );
    int halfPuWidth  = ( ( puSize.width  + 4 ) / 2 ) & ( ~3 );

    MvField mvCand;

    for( int y = puPos.y; y < puPos.y + puSize.height; y += halfPuHeight )
    {
      for( int x = puPos.x; x < puPos.x + puSize.width; x += halfPuWidth )
      {
        const MotionInfo &frucMi = pu.getMotionInfoFRUC( Position{ x, y } );

        for( Int nList = 0 ; nList < 2 ; nList++ )
        {
          RefPicList eCurList = ( RefPicList )nList;
          if( frucMi.interDir & ( 1 << eCurList ) )
          {
            if( nTargetRefIdx >= 0 && ( frucMi.refIdx[eCurList] != nTargetRefIdx || eCurList != eTargetRefList ) )
              continue;
            mvCand.setMvField( frucMi.mv[eCurList], frucMi.refIdx[eCurList] );
            xFrucInsertMv2StartList( mvCand , m_listMVFieldCand[nList&0x01], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
          }
        }
      }
    }
  }

  // add some neighbors if not already present
  Int nbSpatialCandTested = NB_FRUC_CAND_ADDED;

  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  for (Int neighbor = 0; neighbor < nbSpatialCandTested; neighbor++)
  {
    if (neighbor == 0)       // top neighbor
    {
      neibPos = pu.lumaPos().offset( 0, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 1)  // left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, 0 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 2)  // top-left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 3)  // top-right neighbor
    {
      neibPos = pu.Y().topRight().offset( 1, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 4)  // below-left neighbor
    {
      neibPos = pu.Y().bottomLeft().offset( -1, 1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else
    {
      THROW("Invalid neighbor id!");
    }

    if (neibPU)
    {
      for (Int nList = 0; nList < 2; nList++)
      {
        RefPicList eCurList = (RefPicList)nList;
        if( neibPU->getMotionInfo( neibPos ).interDir & ( 1 << eCurList ) )
        {
          if ( (nTargetRefIdx >= 0) && ((neibPU->getMotionInfo( neibPos ).refIdx[eCurList] != nTargetRefIdx) || (eCurList != eTargetRefList)) )
            continue;
          MvField mvCand;
          mvCand.setMvField( neibPU->getMotionInfo( neibPos ).mv[eCurList], neibPU->getMotionInfo( neibPos ).refIdx[eCurList] );
          xFrucInsertMv2StartList( mvCand , m_listMVFieldCand[nList&0x01], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
      }
    }
  }
}

Void InterPrediction::xFrucCollectSubBlkStartMv( PredictionUnit& pu, const MergeCtx& mergeCtx, RefPicList eRefPicList , const MvField& rMvStart , Int nSubBlkWidth , Int nSubBlkHeight, Position basePuPos )
{
  std::list<MvField> & rStartMvList = m_listMVFieldCand[eRefPicList];
  rStartMvList.clear();

  // start Mv
  xFrucInsertMv2StartList( rMvStart , rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );

  // add some neighbors if not already present

  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  for (Int neighbor = 0; neighbor < std::max(2,NB_FRUC_CAND_ADDED_SUB); neighbor++)
  {
    if (neighbor == 0)       // top neighbor
    {
      neibPos = pu.lumaPos().offset( 0, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 1)  // left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, 0 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 2)  // top-left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 3)  // top-right neighbor
    {
      neibPos = pu.Y().topRight().offset( 1, -1 );
      neibPU = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 4)  // below-left neighbor
    {
      neibPos = pu.Y().bottomLeft().offset( -1, 1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else
    {
      THROW("Invalid neighbor id!");
    }

    if( neibPU && neibPU->getMotionInfo( neibPos ).interDir & ( 1 << eRefPicList ) && neibPU->getMotionInfo( neibPos ).refIdx[eRefPicList] == rMvStart.refIdx )
    {
      MvField mvCand;
      mvCand.setMvField( neibPU->getMotionInfo( neibPos ).mv[eRefPicList], neibPU->getMotionInfo( neibPos ).refIdx[eRefPicList] );
      xFrucInsertMv2StartList( mvCand , rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
    }
  }

  Int nCurPOC    = pu.cs->slice->getPOC();
  Int nCurRefPOC = pu.cs->slice->getRefPOC( eRefPicList, rMvStart.refIdx );

  // scaled TMVP, collocated positions and bottom right positions
  Int nMaxPositions = 1;

  for( Int n = 0; n < nMaxPositions; n++ )
  {
    for( Int nRefIdx = pu.cs->slice->getNumRefIdx( eRefPicList ) - 1; nRefIdx >= 0; nRefIdx-- )
    {
      MvField mvCand;
      const Picture* pColPic  = pu.cs->slice->getRefPic( eRefPicList, nRefIdx );

      const unsigned scale = ( pu.cs->pcv->noMotComp ? 1 : 4 * std::max<Int>(1, 4 * AMVP_DECIMATION_FACTOR / 4) );

      const unsigned mask  = ~( scale - 1 );

      Int x_off = n == 0 ? 0 : nSubBlkWidth;
      Int y_off = n == 0 ? 0 : nSubBlkHeight;

      Position _pos = Position{ pu.lumaPos().x + x_off, pu.lumaPos().y + y_off };

      const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

      const MotionInfo &colMi = pColPic->cs->getMotionInfo( pos );

      for( Int nRefListColPic = 0; nRefListColPic < 2; nRefListColPic++ )
      {
        if( colMi.interDir & ( 1 << nRefListColPic ) ) // TODO: check if refIdx is always NOT_VALID, not 0 as set
        {
          CHECK( colMi.isInter == false, "invalid motion info" );
          Mv rColMv = colMi.mv[nRefListColPic];

          if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
          {
            rColMv.setHighPrec();
          }

          mvCand.refIdx = rMvStart.refIdx;
          mvCand.mv     = PU::scaleMv( rColMv , nCurPOC , nCurRefPOC , pColPic->getPOC(), pColPic->cs->slice->getRefPOC( ( RefPicList )nRefListColPic , colMi.refIdx[nRefListColPic] ), pu.cs->slice );
          if( mvCand.refIdx < 0 )
          {
            printf( "base" );
          }
          xFrucInsertMv2StartList( mvCand , rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
      }
    }
  }

  if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
  {
    //add supPu merge candidates
    Position subPos   = pu.lumaPos() - basePuPos;
#if JVET_K0346
    const Slice& slice = *pu.cs->slice;
    int numPartLine   = std::max(pu.lumaSize().width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    int numPartCol    = std::max(pu.lumaSize().height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    int puHeight      = numPartCol == 1 ? pu.lumaSize().height : 1 << slice.getSubPuMvpSubblkLog2Size();
    int puWidth       = numPartLine == 1 ? pu.lumaSize().width : 1 << slice.getSubPuMvpSubblkLog2Size();
#else
    int iNumPartLine  = std::max( pu.lumaSize().width  >> pu.cs->sps->getSpsNext().getSubPuMvpLog2Size(), 1u );
    int iNumPartCol   = std::max( pu.lumaSize().height >> pu.cs->sps->getSpsNext().getSubPuMvpLog2Size(), 1u );
    int iPUHeight     = iNumPartCol  == 1 ? pu.lumaSize().height : 1 << pu.cs->sps->getSpsNext().getSubPuMvpLog2Size();
    int iPUWidth      = iNumPartLine == 1 ? pu.lumaSize().width  : 1 << pu.cs->sps->getSpsNext().getSubPuMvpLog2Size();
#endif

#if JVET_K0346
    for (int y = subPos.y; y < subPos.y + pu.lumaSize().height; y += puHeight)
    {
      for (int x = subPos.x; x < subPos.x + pu.lumaSize().width; x += puWidth)
#else
    for( int y = subPos.y; y < subPos.y + pu.lumaSize().height; y += iPUHeight )
    {
      for( int x = subPos.x; x < subPos.x + pu.lumaSize().width; x += iPUWidth )
#endif
      {
        const MotionInfo subPuMi = mergeCtx.subPuMvpMiBuf.at( g_miScaling.scale( Position( x, y ) ) );
        if( rMvStart.refIdx == subPuMi.refIdx[eRefPicList] && subPuMi.interDir & ( 1 << eRefPicList ) )
        {
          MvField mvCand = MvField( subPuMi.mv[eRefPicList], subPuMi.refIdx[eRefPicList] );
          xFrucInsertMv2StartList( mvCand, rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
        const MotionInfo subPuExtMi = mergeCtx.subPuMvpExtMiBuf.at( g_miScaling.scale( Position( x, y ) ) );
        if( rMvStart.refIdx == subPuExtMi.refIdx[eRefPicList] && subPuExtMi.interDir & ( 1 << eRefPicList ) )
        {
          MvField mvCand = MvField( subPuExtMi.mv[eRefPicList], subPuExtMi.refIdx[eRefPicList] );
          xFrucInsertMv2StartList( mvCand, rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
      }
    }
  }
}

UInt InterPrediction::xFrucFindBestMvFromList( MvField* pBestMvField, RefPicList& rBestRefPicList, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, Bool bTM, Bool bMvCost )
{
  UInt uiMinCost = MAX_UINT;

  Int nRefPicListStart = 0;
  Int nRefPicListEnd = 1;
  if( bTM || bMvCost )  // Limit search to bestList in Template mode and for all sub-blocks (Template and Bilateral modes)
  {
    nRefPicListStart = nRefPicListEnd = rBestRefPicList;
  }
  for( Int nRefPicList = nRefPicListStart ; nRefPicList <= nRefPicListEnd ; nRefPicList++ )
  {
    RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
    for( std::list<MvField>::iterator pos = m_listMVFieldCand[eCurRefPicList].begin() ; pos != m_listMVFieldCand[eCurRefPicList].end() ; pos++ )
    {
      MvField mvPair;

      if( !bTM && eCurRefPicList == REF_PIC_LIST_1 && !pu.cs->slice->getCheckLDC() )
      {
        // for normal B picture
        if( !PU::getMvPair( pu, REF_PIC_LIST_1 , *pos , mvPair ) || xFrucIsInList( mvPair , m_listMVFieldCand[0] ) )
          // no paired MV or the pair has been checked in list0
          continue;
      }

      UInt uiCost = 0;
      if( bMvCost )
      {
        uiCost = xFrucGetMvCost( rMvStart.mv , pos->mv, MAX_INT, FRUC_MERGE_REFINE_MVWEIGHT, pu.cs->sps->getSpsNext().getUseHighPrecMv() ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0 );
        if( uiCost > uiMinCost )
          continue;
      }

      if( bTM )
      {
        uiCost = xFrucGetTempMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, *pos, uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, *pos, mvPair, uiCost );
      }

      if( uiCost < uiMinCost )
      {
        uiMinCost = uiCost;
        pBestMvField[eCurRefPicList] = *pos;
        if( !bTM )
        {
          rBestRefPicList = eCurRefPicList;
          pBestMvField[!eCurRefPicList] = mvPair;
        }
      }
    }
  }

  return uiMinCost;
}

Bool InterPrediction::deriveFRUCMV( PredictionUnit &pu )
{
  CHECK( !pu.mergeFlag, "merge mode must be used here" );

  Bool bAvailable = false;

  MergeCtx mrgCtx;
  if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( pu.lumaSize() );
    mrgCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
    mrgCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
  }

  PU::getInterMergeCandidates( pu, mrgCtx );

  bAvailable = xFrucFindBlkMv( pu, mrgCtx );
  if( bAvailable )
  {
    xFrucRefineSubBlkMv( pu, mrgCtx, pu.frucMrgMode == FRUC_MERGE_TEMPLATE );
  }

  return bAvailable;
}

UInt InterPrediction::xFrucGetMvCost( const Mv& rMvStart, const Mv& rMvCur, Int nSearchRange, Int nWeighting, UInt precShift )
{
  Mv mvDist = rMvStart - rMvCur;
  UInt uiCost = MAX_UINT;
  if( mvDist.getAbsHor() <= nSearchRange && mvDist.getAbsVer() <= nSearchRange )
  {
    uiCost = ( mvDist.getAbsHor() + mvDist.getAbsVer() ) * nWeighting;
    uiCost >>= precShift;
  }

  return uiCost;
}

UInt InterPrediction::xFrucRefineMv( MvField* pBestMvField, RefPicList eCurRefPicList, UInt uiMinCost, Int nSearchMethod, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, Bool bTM, Bool bMvCostZero )
{
  Int nSearchStepShift = 0;
  if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
  {
    nSearchStepShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  switch( nSearchMethod )
  {
    case 0:
      // cross
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1 );
      }
      break;
    case 1:
      // square
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_SQUARE>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
      }
      break;
    case 2:
      // diamond
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, 1, bMvCostZero );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
      }
      break;
    case 3:
      // no refinement
      break;
    case 4:
      // hexagon
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, 1, bMvCostZero );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
      }
      break;
    case 5:
      // adaptive cross
      if( rMvStart.refIdx != pBestMvField[eCurRefPicList].refIdx || rMvStart.mv != pBestMvField[eCurRefPicList].mv )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
        if( nSearchStepShift > 0 )
        {
          uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
        }
      }
      break;
    default:
      THROW( "Invalid search method" );
  }

  return uiMinCost;
}

template<Int SearchPattern>
UInt InterPrediction::xFrucRefineMvSearch ( MvField* pBestMvField, RefPicList eCurRefPicList, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, UInt uiMinDist, Bool bTM, Int nSearchStepShift, UInt uiMaxSearchRounds, Bool bMvCostZero )
{
  const Mv mvSearchOffsetCross  [4] = { Mv(  0 , 1 ) , Mv( 1 , 0 ) , Mv(  0 , -1 ) , Mv( -1 ,  0 ) };
  const Mv mvSearchOffsetSquare [8] = { Mv( -1 , 1 ) , Mv( 0 , 1 ) , Mv(  1 ,  1 ) , Mv(  1 ,  0 ) , Mv(  1 , -1 ) , Mv(  0 , -1 ) , Mv( -1 , -1 ) , Mv( -1 , 0 )  };
  const Mv mvSearchOffsetDiamond[8] = { Mv(  0 , 2 ) , Mv( 1 , 1 ) , Mv(  2 ,  0 ) , Mv(  1 , -1 ) , Mv(  0 , -2 ) , Mv( -1 , -1 ) , Mv( -2 ,  0 ) , Mv( -1 , 1 ) };
  const Mv mvSearchOffsetHexagon[6] = { Mv(  2 , 0 ) , Mv( 1 , 2 ) , Mv( -1 ,  2 ) , Mv( -2 ,  0 ) , Mv( -1 , -2 ) , Mv(  1 , -2 ) };

  Int nDirectStart = 0 , nDirectEnd = 0 , nDirectRounding = 0 , nDirectMask = 0;
  const Mv * pSearchOffset;
  if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_CROSS )
  {
    nDirectEnd = 3;
    nDirectRounding = 4;
    nDirectMask = 0x03;
    pSearchOffset = mvSearchOffsetCross;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_SQUARE )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetSquare;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetDiamond;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
  {
    nDirectEnd = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    THROW( "Invalid search pattern" );
  }

  Int nBestDirect;
  Int rSearchRange = pu.cs->sps->getSpsNext().getFRUCRefineRange();
  if( !pu.cs->sps->getSpsNext().getUseHighPrecMv() )
  {
    rSearchRange >>= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }
  for( UInt uiRound = 0 ; uiRound < uiMaxSearchRounds ; uiRound++ )
  {
    nBestDirect = -1;
    MvField mvCurCenter = pBestMvField[eCurRefPicList];
    for( Int nIdx = nDirectStart ; nIdx <= nDirectEnd ; nIdx++ )
    {
      Int nDirect;
      if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        nDirect = ( nIdx + nDirectRounding ) & nDirectMask;
      }

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      MvField mvCand = mvCurCenter;
      if( mvCand.mv.highPrec && !mvOffset.highPrec )
      {
        mvOffset.highPrec = true;
      }
      mvCand.mv += mvOffset;
      UInt uiCost = xFrucGetMvCost( rMvStart.mv, mvCand.mv, rSearchRange, FRUC_MERGE_REFINE_MVWEIGHT, pu.cs->sps->getSpsNext().getUseHighPrecMv() ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0 );
      if (bMvCostZero && uiCost != MAX_UINT )
      {
        uiCost = 0;
      }
      if( uiCost > uiMinDist )
        continue;
      MvField mvPair;

      if( bTM )
      {
        if( PU::isSameMVField( pu, eCurRefPicList, mvCand, ( RefPicList )( !eCurRefPicList ), pBestMvField[!eCurRefPicList] ) )
          continue;
        uiCost = xFrucGetTempMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, mvCand, uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, mvCand, mvPair, uiCost );
      }
      if( uiCost < uiMinDist )
      {
        uiMinDist = uiCost;
        pBestMvField[eCurRefPicList] = mvCand;
        if( !bTM )
          pBestMvField[!eCurRefPicList] = mvPair;
        nBestDirect = nDirect;
      }
    }

    if( nBestDirect == -1 )
      break;
    Int nStep = 1;
    if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_SQUARE || SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
    {
      nStep = 2 - ( nBestDirect & 0x01 );
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  return uiMinDist;
}

Bool InterPrediction::frucFindBlkMv4Pred( PredictionUnit& pu, RefPicList eTargetRefPicList, const Int nTargetRefIdx, AMVPInfo* pInfo )
{
  MergeCtx mrgCtx;
  if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( pu.lumaSize() );
    mrgCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
    mrgCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
  }

  PU::getInterMergeCandidates( pu, mrgCtx );

  Bool bAvailable = false;
  if( pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
  {
    Int nWidth  = pu.lumaSize().width;
    Int nHeight = pu.lumaSize().height;
    if( xFrucGetCurBlkTemplate( pu, nWidth, nHeight ) )
    {
      // find best start
      xFrucCollectBlkStartMv( pu, mrgCtx, eTargetRefPicList, nTargetRefIdx, pInfo );
      MvField mvStart[2] , mvFinal[2];
      UInt uiMinCost = xFrucFindBestMvFromList( mvStart, eTargetRefPicList, pu, mvStart[eTargetRefPicList], nWidth, nHeight, true, false );
      if( mvStart[eTargetRefPicList].refIdx >= 0 )
      {
        // refine Mv
        mvFinal[eTargetRefPicList] = mvStart[eTargetRefPicList];
        xFrucRefineMv( mvFinal, eTargetRefPicList, uiMinCost, 2, pu, mvStart[eTargetRefPicList], nWidth, nHeight, true );
        bAvailable = true;
        // save Mv
        pu.mv[eTargetRefPicList] = mvFinal[eTargetRefPicList].mv;
      }
    }
  }

  return bAvailable;
}

Bool InterPrediction::xFrucFindBlkMv( PredictionUnit& pu, const MergeCtx& mergeCtx )
{
  Bool bAvailable = false;
  Int nWidth  = pu.lumaSize().width;
  Int nHeight = pu.lumaSize().height;
  MvField mvStart[2] , mvFinal[2];

  const Int nSearchMethod = 2;
  if( pu.frucMrgMode == FRUC_MERGE_BILATERALMV )
  {
    if( !pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() || pu.cs->slice->isInterP() )
      return( false );

    xFrucCollectBlkStartMv( pu, mergeCtx );

    RefPicList eBestRefPicList = REF_PIC_LIST_0;
    UInt uiMinCost = xFrucFindBestMvFromList( mvStart, eBestRefPicList, pu, mvStart[eBestRefPicList], nWidth, nHeight, false, false );

    if( mvStart[eBestRefPicList].refIdx >= 0 )
    {
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];
      uiMinCost = xFrucRefineMv( mvFinal, eBestRefPicList, uiMinCost, nSearchMethod, pu, mvStart[eBestRefPicList], nWidth, nHeight, false );
      //Save best list for sub-blocks
      m_bilatBestRefPicList = eBestRefPicList;
      bAvailable = true;
    }
  }
  else if( pu.frucMrgMode == FRUC_MERGE_TEMPLATE )
  {
    if( !pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
      return( false );
    if( !xFrucGetCurBlkTemplate( pu, nWidth, nHeight ) )
      return( false );

    xFrucCollectBlkStartMv( pu, mergeCtx );

    UInt uiMinCost[2];
    // find the best Mvs from the two lists first and then refine Mvs: try to avoid duplicated Mvs
    for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
      uiMinCost[nRefPicList] = xFrucFindBestMvFromList( mvStart, eCurRefPicList, pu, mvStart[nRefPicList], nWidth, nHeight, true, false );
    }
    mvFinal[0] = mvStart[0];
    mvFinal[1] = mvStart[1];
    for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      if( mvStart[nRefPicList].refIdx >= 0 )
      {
        uiMinCost[nRefPicList] = xFrucRefineMv( mvFinal, ( RefPicList )nRefPicList, uiMinCost[nRefPicList], nSearchMethod, pu, mvStart[nRefPicList], nWidth, nHeight, true, true );
        bAvailable = true;
      }
    }

    if (mvFinal[0].refIdx >= 0 && mvFinal[1].refIdx >= 0)
    {
      //calculate cost for bi-refinement
      xFrucUpdateTemplate( pu, nWidth, nHeight, REF_PIC_LIST_0, mvFinal[REF_PIC_LIST_0] );
      UInt uiCostBi = xFrucGetTempMatchCost( pu, nWidth, nHeight, REF_PIC_LIST_1, mvFinal[REF_PIC_LIST_1], 0 );

      if (2 * uiCostBi <= 5 * std::min(uiMinCost[0], uiMinCost[1]) )  // if (uiMinCostBi <= 5/4*2*min(uiMinCost[0], uiMinCost[1]) )
      {
        //do nothing
      }
      else if (uiMinCost[0] <= uiMinCost[1])
      {
        mvFinal[1].refIdx = -1;
        mvFinal[1].mv.set( 0, 0 );
      }
      else
      {
        mvFinal[0].refIdx = -1;
        mvFinal[0].mv.set( 0, 0 );
      }
    }
  }
  else
  {
    THROW( "Invalid mode" );
  }

  if( bAvailable )
  {
    // save Mv
    pu.mv    [REF_PIC_LIST_0] = mvFinal[0].mv;
    pu.refIdx[REF_PIC_LIST_0] = mvFinal[0].refIdx;
    pu.mv    [REF_PIC_LIST_1] = mvFinal[1].mv;
    pu.refIdx[REF_PIC_LIST_1] = mvFinal[1].refIdx;
    pu.interDir = ( mvFinal[0].refIdx >= 0 ) + ( ( mvFinal[1].refIdx >=0 ) << 1 );

    CHECK( pu.interDir < 1 || pu.interDir > 3, "invalid inter dir " );

    PU::spanMotionInfo( pu );
  }

  return bAvailable;
}

Bool InterPrediction::xFrucRefineSubBlkMv( PredictionUnit& pu, const MergeCtx &mergeCtx, Bool bTM )
{
  Int nRefineBlockSize = xFrucGetSubBlkSize( pu, pu.lumaSize().width, pu.lumaSize().height );

  const Int nSearchMethod = 5;

  Position puPos  = pu.lumaPos();
  PredictionUnit subPu;
  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeFlag = pu.mergeFlag;
  subPu.mergeType = pu.mergeType;
  subPu.idx       = pu.idx;

  for( Int y = puPos.y; y < puPos.y + pu.lumaSize().height; y += nRefineBlockSize )
  {
    for( Int x = puPos.x; x < puPos.x + pu.lumaSize().width; x += nRefineBlockSize )
    {
      MotionInfo mi = pu.getMotionInfo( Position{ x, y } );

      // start from the best Mv of the full block
      MvField mvStart[2] , mvFinal[2];
      mvStart[0].setMvField( mi.mv[REF_PIC_LIST_0] , mi.refIdx[REF_PIC_LIST_0] );
      mvStart[1].setMvField( mi.mv[REF_PIC_LIST_1] , mi.refIdx[REF_PIC_LIST_1] );
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];

      Int dx = nRefineBlockSize;
      Int dy = nRefineBlockSize;

      subPu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( x, y, dx, dy ) ) );
      subPu = mi;

      // refinement
      if( bTM )
      {
        if( !xFrucGetCurBlkTemplate( subPu, nRefineBlockSize, nRefineBlockSize ) ) // TODO: sub pu position!!!
          continue;
        for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
        {
          if( mvStart[nRefPicList].refIdx >= 0 )
          {
            RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
            xFrucCollectSubBlkStartMv( subPu, mergeCtx, eCurRefPicList, mvStart[eCurRefPicList], nRefineBlockSize, nRefineBlockSize, puPos );
            UInt uiMinCost = xFrucFindBestMvFromList( mvFinal, eCurRefPicList, subPu, mvStart[eCurRefPicList], nRefineBlockSize, nRefineBlockSize, bTM, true );
            uiMinCost = xFrucRefineMv( mvFinal, eCurRefPicList, uiMinCost, nSearchMethod, subPu, mvStart[eCurRefPicList], nRefineBlockSize, nRefineBlockSize, bTM );
          }
        }
      }
      else
      {
        //Use same reference frame as for entire block, i.e. same refidx and same list
        RefPicList eBestRefPicList = m_bilatBestRefPicList;
        xFrucCollectSubBlkStartMv( subPu, mergeCtx, eBestRefPicList, mvStart[eBestRefPicList], nRefineBlockSize, nRefineBlockSize, puPos );

        UInt uiMinCost = xFrucFindBestMvFromList( mvFinal, eBestRefPicList, subPu, mvStart[eBestRefPicList], nRefineBlockSize, nRefineBlockSize, bTM, true );
        uiMinCost = xFrucRefineMv( mvFinal, eBestRefPicList, uiMinCost, nSearchMethod, subPu, mvStart[eBestRefPicList], nRefineBlockSize, nRefineBlockSize, bTM );
      }

      // save Mv
      if( !( mvFinal[0] == mvStart[0] && mvFinal[1] == mvStart[1] ) )
      {
        subPu.mv    [REF_PIC_LIST_0] = mvFinal[0].mv;
        subPu.refIdx[REF_PIC_LIST_0] = mvFinal[0].refIdx;
        subPu.mv    [REF_PIC_LIST_1] = mvFinal[1].mv;
        subPu.refIdx[REF_PIC_LIST_1] = mvFinal[1].refIdx;
        subPu.interDir = ( mvFinal[0].refIdx >= 0 ) + ( ( mvFinal[1].refIdx >=0 ) << 1 );
        subPu.mergeType = MRG_TYPE_DEFAULT_N; // only a hack, TODO: do it better
        CHECK( subPu.interDir < 1 || subPu.interDir > 3, "invalid inter dir" );
        PU::spanMotionInfo( subPu );
      }
    }
  }

  return true;
}

Int InterPrediction::xFrucGetSubBlkSize( PredictionUnit& pu, Int nBlkWidth, Int nBlkHeight )
{
  Int avgLength = pu.cs->pcv->rectCUs ? 1 << ( ( ( ( int ) log2( pu.cu->lumaSize().width ) + ( int ) log2( pu.cu->lumaSize().height ) - 3 ) >> 1 ) + MIN_CU_LOG2 ) : pu.cu->lumaSize().width;
  Int nRefineBlkSize = std::max( avgLength >> pu.cs->slice->getSPS()->getSpsNext().getFRUCSmallBlkRefineDepth(), FRUC_MERGE_REFINE_MINBLKSIZE );

  while( true )
  {
    Int nMask = nRefineBlkSize - 1;
    if( nRefineBlkSize > std::min( nBlkWidth , nBlkHeight ) || ( nBlkWidth & nMask ) || ( nBlkHeight & nMask ) )
      nRefineBlkSize >>= 1;
    else
      break;
  }
  CHECK( nRefineBlkSize < FRUC_MERGE_REFINE_MINBLKSIZE, "invalid refine block size" );
  return( nRefineBlkSize );
}

Bool InterPrediction::xFrucGetCurBlkTemplate( PredictionUnit& pu, Int nCurBlkWidth, Int nCurBlkHeight )
{
  m_bFrucTemplateAvailabe[0] = xFrucIsTopTempAvailable( pu );
  m_bFrucTemplateAvailabe[1] = xFrucIsLeftTempAvailable( pu );

  if( !m_bFrucTemplateAvailabe[0] && !m_bFrucTemplateAvailabe[1] )
    return false;

  const Int nMVUnit = 2;

  if( m_bFrucTemplateAvailabe[0] )
  {
    Mv mvTop( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    PelUnitBuf pcMbBuf = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[0][0], nCurBlkWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvTop.setHighPrec();
    }
    xPredInterBlk(COMPONENT_Y, pu, pu.cs->slice->getPic(), mvTop, pcMbBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
  }

  if( m_bFrucTemplateAvailabe[1] )
  {
    Mv mvLeft( - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    PelUnitBuf pcMbBuf = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[1][0], FRUC_MERGE_TEMPLATE_SIZE, nCurBlkHeight) ) );

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvLeft.setHighPrec();
    }
    xPredInterBlk(COMPONENT_Y, pu, pu.cs->slice->getPic(), mvLeft, pcMbBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
  }

  return true;
}

Bool InterPrediction::xFrucIsTopTempAvailable( PredictionUnit& pu )
{
  const CodingStructure &cs     = *pu.cs;
        Position posRT          = pu.Y().topRight();
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );

  return ( puAbove && pu.cu != puAbove->cu );
}

Bool InterPrediction::xFrucIsLeftTempAvailable( PredictionUnit& pu )
{
  const CodingStructure &cs    = *pu.cs;
        Position posLB         = pu.Y().bottomLeft();
  const PredictionUnit *puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );

  return ( puLeft && pu.cu !=puLeft->cu );
}

Void InterPrediction::xFrucUpdateTemplate( PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField )
{
  const Int nMVUnit = 2;

  const MvField *pMvFieldOther = &rCurMvField;
  PelUnitBuf pcBufPredRefTop  = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );
  PelUnitBuf pcBufPredRefLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );

  PelUnitBuf pcBufPredCurTop  = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );
  PelUnitBuf pcBufPredCurLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );

  const SPS &sps = *pu.cs->sps;

  if( m_bFrucTemplateAvailabe[0] )
  {
    Mv mvOther( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    mvOther += pMvFieldOther->mv;

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvOther.setHighPrec();
    }

    clipMv(mvOther, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, pMvFieldOther->refIdx), mvOther, pcBufPredRefTop, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
    pcBufPredCurTop.removeHighFreq( pcBufPredRefTop, false, pu.cs->slice->clpRngs() );
  }

  if (m_bFrucTemplateAvailabe[1])
  {
    Mv mvOther( -( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ), 0 );
    mvOther += pMvFieldOther->mv;

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvOther.setHighPrec();
    }

    clipMv(mvOther, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, pMvFieldOther->refIdx), mvOther, pcBufPredRefLeft, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
    pcBufPredCurLeft.removeHighFreq( pcBufPredRefLeft, false, pu.cs->slice->clpRngs() );
  }
}

Void InterPrediction::xPredInterLines( const PredictionUnit& pu, const Picture* refPic, Mv &_mv, PelUnitBuf &dstPic, const Bool &bi, const ClpRng& clpRng )
{
  clipMv( _mv, pu.lumaPos(), *pu.cs->sps );

  const ChromaFormat chFmt  = pu.chromaFormat;
  const ComponentID compID  = COMPONENT_Y;

  int iAddPrecShift = 0;

  if( _mv.highPrec )
  {
    CHECK( !pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!" );

    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY( compID, chFmt );

  Position offset      = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
  const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  PelBuf &dstBuf = dstPic.bufs[compID];

  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );

  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;

  CHECK( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( xFrac & 3 ) != 0 ), "Invalid fraction" );
  CHECK( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( yFrac & 3 ) != 0 ), "Invalid fraction" );

  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  if( yFrac == 0 )
  {
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, !bi, chFmt, clpRng, 0);
  }
  else if( xFrac == 0 )
  {
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, !bi, chFmt, clpRng, 0);
  }
  else
  {
    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], Size(width,height));

    Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,      chFmt, clpRng, 0);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, !bi, chFmt, clpRng, 0);
    JVET_J0090_SET_CACHE_ENABLE( true );
  }
}

Void InterPrediction::xFillPredBlckAndBorder( const PredictionUnit& pu, RefPicList eRefPicList, Int iWidth, Int iHeight, PelBuf &cTmpY )
{
  Int iRefIdx = pu.refIdx[eRefPicList];
  Mv  mvOrg   = pu.mv[eRefPicList];
  Mv  mv;

  const Picture* refPic = pu.cu->slice->getRefPic(eRefPicList, iRefIdx);

  const Int nMVUnit = 2;

  Int dstStride = MAX_CU_SIZE + DMVR_INTME_RANGE*2;

  JVET_J0090_SET_REF_PICTURE( refPic, COMPONENT_Y );
  PelUnitBuf cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0], dstStride, iWidth + 2 * DMVR_INTME_RANGE, iHeight + 2 * DMVR_INTME_RANGE ) ) );
  cPred.Y().subBuf( Position{ DMVR_INTME_RANGE, DMVR_INTME_RANGE }, pu.lumaSize() ).copyFrom( cTmpY.subBuf( Position{ 0, 0 }, pu.lumaSize() ) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0], dstStride, iWidth + 2 * DMVR_INTME_RANGE, DMVR_INTME_RANGE ) ) );
  //top row
  mv = mvOrg + Mv( -(DMVR_INTME_RANGE << nMVUnit), -(DMVR_INTME_RANGE << nMVUnit) );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0] + dstStride*DMVR_INTME_RANGE, MAX_CU_SIZE + DMVR_INTME_RANGE*2, DMVR_INTME_RANGE, iHeight ) ) );
  //left column
  mv = mvOrg + Mv( -(DMVR_INTME_RANGE << nMVUnit), 0 );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0] + dstStride*DMVR_INTME_RANGE + iWidth + DMVR_INTME_RANGE, MAX_CU_SIZE + DMVR_INTME_RANGE*2, DMVR_INTME_RANGE, iHeight ) ) );
  //right column
  mv = mvOrg + Mv( (iWidth << nMVUnit), 0 );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0] + dstStride*(iHeight + DMVR_INTME_RANGE), MAX_CU_SIZE + DMVR_INTME_RANGE*2, iWidth + 2 * DMVR_INTME_RANGE, DMVR_INTME_RANGE ) ) );
  //bottom row
  mv = mvOrg + Mv( -(DMVR_INTME_RANGE << nMVUnit), (iHeight << nMVUnit) );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );
}

UInt InterPrediction::xDirectMCCost( Int iBitDepth, Pel* pRef, UInt uiRefStride, const Pel* pOrg, UInt uiOrgStride, Int iWidth, Int iHeight )
{
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR = false;

  m_pcRdCost->setDistParam( cDistParam, pOrg, pRef, uiOrgStride, uiRefStride, iBitDepth, COMPONENT_Y, iWidth, iHeight );

  UInt uiCost = cDistParam.distFunc( cDistParam );

  return uiCost;
}

Void InterPrediction::xBIPMVRefine( PredictionUnit& pu, RefPicList eRefPicList, Int iWidth, Int iHeight, const CPelUnitBuf &pcYuvOrg, UInt uiMaxSearchRounds, UInt nSearchStepShift, UInt& uiMinCost, Bool fullPel /*= true*/ )
{
  const Mv mvSearchOffsetSquare[8] = { Mv(-1 , 1) , Mv(0 , 1) , Mv(1 , 1) , Mv(1 , 0) , Mv(1 , -1) , Mv(0 , -1) , Mv(-1 , -1) , Mv(-1 , 0) };

  Int nDirectStart = 0, nDirectEnd = 0, nDirectRounding = 0, nDirectMask = 0;
  const Mv * pSearchOffset;

  nDirectEnd = 7;
  nDirectRounding = 8;
  nDirectMask = 0x07;
  pSearchOffset = mvSearchOffsetSquare;

  Mv cMvOrg = pu.mv[eRefPicList];
  Mv cBestMv = cMvOrg;

  Int nBestDirect;

  for (UInt uiRound = 0; uiRound < uiMaxSearchRounds; uiRound++)
  {
    nBestDirect = -1;
    Mv cMvCtr = cBestMv;

    for (Int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      Int nDirect = (nIdx + nDirectRounding) & nDirectMask;

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;

      if( pu.cu->slice->getSPS()->getSpsNext().getUseHighPrecMv() )
      {
        CHECK( !cMvOrg.highPrec, "wrong" );
        mvOffset.highPrec = true;
      }

      Mv cMvTemp = cMvCtr;
      cMvTemp += mvOffset;

      UInt uiCost;

      if ( fullPel )
      {
        Mv cMvD = cMvTemp;
        cMvD -= cMvOrg;
        cMvD >>= nSearchStepShift;

        CHECK( cMvD.getAbsHor() > DMVR_INTME_RANGE || cMvD.getAbsVer() > DMVR_INTME_RANGE, "wrong");

        Int iRefStride = MAX_CU_SIZE + DMVR_INTME_RANGE*2;

        Pel* pRef = m_cYuvPredTempDMVR[0] + (DMVR_INTME_RANGE + cMvD.getVer()) * iRefStride + DMVR_INTME_RANGE + cMvD.getHor();
        uiCost = xDirectMCCost( pu.cs->sps->getBitDepth(toChannelType(COMPONENT_Y)), pRef, iRefStride, (Pel*) pcYuvOrg.Y().buf, pcYuvOrg.bufs[0].stride, iWidth, iHeight );
      }
      else
      {
        Int iRefStride = MAX_CU_SIZE + DMVR_INTME_RANGE*2 + 16;
        Pel* pRef = m_filteredBlock[pSearchOffset[nDirect].getAbsHor()][pSearchOffset[nDirect].getAbsVer()][0]; //TODO: to be checked if correct

        if (pSearchOffset[nDirect].getHor() == 1)
        {
          pRef++;
        }
        if (pSearchOffset[nDirect].getVer() == 1)
        {
          pRef += iRefStride;
        }
        uiCost = xDirectMCCost( pu.cs->sps->getBitDepth(toChannelType(COMPONENT_Y)), pRef, iRefStride, (Pel*) pcYuvOrg.Y().buf, pcYuvOrg.bufs[0].stride, iWidth, iHeight );
      }

      if (uiCost < uiMinCost)
      {
        uiMinCost = uiCost;
        nBestDirect = nDirect;
        cBestMv = cMvTemp;
      }
    }

    if (nBestDirect == -1)
      break;
    Int nStep = 2 - (nBestDirect & 0x01);

    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  pu.mv[eRefPicList] = cBestMv;
}

Void InterPrediction::xProcessDMVR( PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bBIOApplied )
{
  const int iRefIdx0  = pu.refIdx[0];
  const int iRefIdx1  = pu.refIdx[1];

  PelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y())) :
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[0][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvDst.Cr())) );
  PelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y())) :
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[1][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvDst.Cr())) );


  //mv refinement
  UInt searchStepShift = 2;
  if( pu.cu->slice->getSPS()->getSpsNext().getUseHighPrecMv() )
  {
    searchStepShift += 2;
    pu.mv[0].setHighPrec();
    pu.mv[1].setHighPrec();
  }

  pcYuvDst.addAvg( srcPred0, srcPred1, clpRngs, false, true );

  //list 0
  //get init cost
  srcPred0.Y().toLast( clpRngs.comp[COMPONENT_Y] );
  UInt uiMinCost = xDirectMCCost( clpRngs.comp[COMPONENT_Y].bd, pcYuvDst.Y().buf, pcYuvDst.Y().stride, srcPred0.Y().buf, srcPred0.Y().stride, pu.lumaSize().width, pu.lumaSize().height );

  xFillPredBlckAndBorder( pu, REF_PIC_LIST_0, pu.lumaSize().width, pu.lumaSize().height, srcPred0.Y() );

  xBIPMVRefine( pu, REF_PIC_LIST_0, pu.lumaSize().width, pu.lumaSize().height, pcYuvDst, DMVR_INTME_RANGE, searchStepShift, uiMinCost );

  Mv mv = pu.mv[0];
  m_iRefListIdx = 0;

  clipMv( mv, pu.lumaPos(), *pu.cs->sps );


  for( UInt comp = COMPONENT_Y; comp < srcPred0.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );

    xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_0, iRefIdx0 ), mv, srcPred0, true, clpRngs.comp[compID], bBIOApplied, false, FRUC_MERGE_OFF, true );
  }

  //list 1
  //get init cost
  srcPred1.Y().toLast( clpRngs.comp[COMPONENT_Y] );
  uiMinCost = xDirectMCCost( clpRngs.comp[COMPONENT_Y].bd, pcYuvDst.Y().buf, pcYuvDst.Y().stride, srcPred1.Y().buf, srcPred1.Y().stride, pu.lumaSize().width, pu.lumaSize().height );

  xFillPredBlckAndBorder( pu, REF_PIC_LIST_1, pu.lumaSize().width, pu.lumaSize().height, srcPred1.Y() );

  xBIPMVRefine( pu, REF_PIC_LIST_1, pu.lumaSize().width, pu.lumaSize().height, pcYuvDst, DMVR_INTME_RANGE, searchStepShift, uiMinCost );

  mv = pu.mv[1];
  m_iRefListIdx = 1;

  clipMv( mv, pu.lumaPos(), *pu.cs->sps );


  for( UInt comp = COMPONENT_Y; comp < srcPred1.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );

    xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_1, iRefIdx1 ), mv, srcPred1, true, clpRngs.comp[compID], bBIOApplied, false, FRUC_MERGE_OFF, true );
  }

  PU::spanMotionInfo( pu );
}
#endif

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign( CacheModel *cache )
{
  m_cacheModel = cache;
  m_if.cacheAssign( cache );
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif

//! \}
