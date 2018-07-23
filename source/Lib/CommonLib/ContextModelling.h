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

/** \file     ContextModelling.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTMODELLING__
#define __CONTEXTMODELLING__


#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"

#include <bitset>


struct CoeffCodingContext
{
public:
#if HEVC_USE_SIGN_HIDING
  CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide);
#else
  CoeffCodingContext( const TransformUnit& tu, ComponentID component );
#endif
public:
  void initSubblock     ( int SubsetId, bool sigGroupFlag = false );
public:
  void  resetSigGroup   ()                      { m_sigCoeffGroupFlag.reset( m_subSetPos ); }
  void  setSigGroup     ()                      { m_sigCoeffGroupFlag.set( m_subSetPos ); }
  void  setScanPosLast  ( int       posLast )   { m_scanPosLast = posLast; }
  void  setGt2Flag      ( bool      gt2Flag )   { m_prevGt2 = gt2Flag; }
  void  setGoRiceStats  ( unsigned  GRStats )   { m_currentGolombRiceStatistic = GRStats; }
  void  incGoRiceStats  ()                      { m_currentGolombRiceStatistic++; }
  void  decGoRiceStats  ()                      { m_currentGolombRiceStatistic--; }
public:
  ComponentID     compID          ()                        const { return m_compID; }
  int             subSetId        ()                        const { return m_subSetId; }
  int             subSetPos       ()                        const { return m_subSetPos; }
  int             cgPosY          ()                        const { return m_subSetPosY; }
  int             cgPosX          ()                        const { return m_subSetPosX; }
  unsigned        width           ()                        const { return m_width; }
  unsigned        height          ()                        const { return m_height; }
  unsigned        log2CGSize      ()                        const { return m_log2CGSize; }
  unsigned        log2BlockWidth  ()                        const { return m_log2BlockWidth; }
  unsigned        log2BlockHeight ()                        const { return m_log2BlockHeight; }
  unsigned        log2BlockSize   ()                        const { return m_log2BlockSize; }
  bool            updGoRiceStats  ()                        const { return m_useGoRiceParAdapt; }
  bool            extPrec         ()                        const { return m_extendedPrecision; }
  int             maxLog2TrDRange ()                        const { return m_maxLog2TrDynamicRange; }
  unsigned        maxNumCoeff     ()                        const { return m_maxNumCoeff; }
  unsigned        currGoRiceStats ()                        const { return m_currentGolombRiceStatistic; }
  bool            alignFlag       ()                        const { return m_AlignFlag; }
  int             scanPosLast     ()                        const { return m_scanPosLast; }
  int             minSubPos       ()                        const { return m_minSubPos; }
  int             maxSubPos       ()                        const { return m_maxSubPos; }
  bool            isLast          ()                        const { return ( ( m_scanPosLast >> m_log2CGSize ) == m_subSetId ); }
  bool            isNotFirst      ()                        const { return ( m_subSetId != 0 ); }
  bool            isSigGroup      ( int       scanPosCG )   const { return m_sigCoeffGroupFlag[ m_scanCG[ scanPosCG ] ]; }
  bool            isSigGroup      ()                        const { return m_sigCoeffGroupFlag[ m_subSetPos ]; }
#if HEVC_USE_SIGN_HIDING
  bool            signHiding      ()                        const { return m_signHiding; }
  bool            hideSign        ( int       posFirst,
                                    int       posLast   )   const { return ( m_signHiding && ( posLast - posFirst >= SBH_THRESHOLD ) ); }
#endif
  CoeffScanType   scanType        ()                        const { return m_scanType; }
  unsigned        blockPos        ( int       scanPos   )   const { return m_scan[ scanPos ]; }
  unsigned        posX            ( int       scanPos   )   const { return m_scanPosX[ scanPos ]; }
  unsigned        posY            ( int       scanPos   )   const { return m_scanPosY[ scanPos ]; }
  unsigned        maxLastPosX     ()                        const { return m_maxLastPosX; }
  unsigned        maxLastPosY     ()                        const { return m_maxLastPosY; }
  unsigned        lastXCtxId      ( unsigned  posLastX  )   const { return m_CtxSetLastX( m_lastOffsetX + ( posLastX >> m_lastShiftX ) ); }
  unsigned        lastYCtxId      ( unsigned  posLastY  )   const { return m_CtxSetLastY( m_lastOffsetY + ( posLastY >> m_lastShiftY ) ); }
  unsigned        sigGroupCtxId   ()                        const { return m_sigGroupCtxId; }
#if HM_QTBT_AS_IN_JEM_CONTEXT // ctx modeling for subblocks != 4x4
  unsigned        sigCtxId        ( int       scanPos   )   const;
#else
  unsigned        sigCtxId        ( int       scanPos   )   const { return m_sigCtxSet( m_sigScanCtxId[ scanPos ] ); }
#endif
  unsigned        greater1CtxId   ( int       gt1Ctx    )   const { return m_gt1FlagCtxSet( gt1Ctx ); }
  unsigned        greater2CtxId   ()                        const { return m_gt2FlagCtxId; }

#if JEM_TOOLS
  unsigned        altResiCompId   ()                        const { return m_altResiCompId; }
#endif

  unsigned        sigGroupCtxIdOfs() const
  {
    return Ctx::SigCoeffGroup[ m_chType + 2 ]( 0 );
  }
  unsigned        sigCtxId        ( int       scanPos,
                              const TCoeff*   coeff,
                                    int       strd = 0  )
  {
    const UInt posY = m_scanPosY[scanPos];
    const UInt posX = m_scanPosX[scanPos];

    strd = strd == 0 ? m_width : strd;
    const TCoeff *pData = coeff + posX + posY * strd;
    const Int   widthM1 = m_width - 1;
    const Int  heightM1 = m_height - 1;
    const Int      diag = posX + posY;

    Int numPos = 0;

    if( posX < widthM1 )
    {
      numPos += pData[ 1 ] != 0;
      if( posX < widthM1 - 1 )
      {
        numPos += pData[ 2 ] != 0;
      }
      if( posY < heightM1 )
      {
        numPos += pData[ m_width + 1 ] != 0;
      }
    }
    if( posY < heightM1 )
    {
      numPos += pData[ m_width ] != 0;
      if( posY < heightM1 - 1 )
      {
        numPos += pData[ 2 * m_width ] != 0;
      }
    }

    const Int ctxIdx = std::min( numPos, 5 );
          Int ctxOfs = diag < 2 ? 6 : 0;

    if( m_chType == CHANNEL_TYPE_LUMA )
    {
      ctxOfs += diag < 5 ? 6 : 0;
    }

    if( m_log2BlockSize > 2 && m_chType == CHANNEL_TYPE_LUMA )
    {
      ctxOfs += 18 << std::min( 1, ( (int)m_log2BlockSize - 3 ) );
    }

    return m_sigCtxSet( ctxOfs + ctxIdx );
  }

  unsigned        sigCtxIdOfs() const
  {
    return m_sigCtxSet( 0 );
  }

  unsigned        greater1CtxId(    int       scanPos,
                              const TCoeff*   coeff,
                                    int       strd = 0  )
  {
    const UInt posY = m_scanPosY[scanPos];
    const UInt posX = m_scanPosX[scanPos];

    strd = strd == 0 ? m_width : strd;
    const TCoeff *pData = coeff + posX + posY * strd;
    const Int   widthM1 = m_width - 1;
    const Int  heightM1 = m_height - 1;
    const Int      diag = posX + posY;

    Int numPos = 0;

    if( posX < widthM1 )
    {
      numPos += abs( pData[ 1 ] ) > 1;
      if( posX < widthM1 - 1 )
      {
        numPos += abs( pData[ 2 ] ) > 1;
      }
      if( posY < heightM1 )
      {
        numPos += abs( pData[ m_width + 1 ] ) > 1;
      }
    }
    if( posY < heightM1 )
    {
      numPos += abs( pData[ m_width ] ) > 1;
      if( posY < heightM1 - 1 )
      {
        numPos += abs( pData[ 2 * m_width ] ) > 1;
      }
    }

    const Int ctxIdx = std::min( numPos, 4 ) + 1;
          Int ctxOfs = 0;

    if( m_chType == CHANNEL_TYPE_LUMA )
    {
      ctxOfs += diag < 3 ? 10 : ( diag < 10 ? 5 : 0 );
    }

    return m_gt1FlagCtxSet( ctxOfs + ctxIdx );
  }

  unsigned        greater1CtxIdOfs() const
  {
    return m_gt1FlagCtxSet( 0 );
  }

  unsigned        greater2CtxId(    int       scanPos,
                              const TCoeff*   coeff,
                                    int       strd = 0  )
  {
    const UInt posY = m_scanPosY[ scanPos ];
    const UInt posX = m_scanPosX[ scanPos ];

    strd = strd == 0 ? m_width : strd;
    const TCoeff *pData = coeff + posX + posY * strd;
    const Int   widthM1 = m_width - 1;
    const Int  heightM1 = m_height - 1;
    const Int      diag = posX + posY;

    Int numPos = 0;

    if( posX < widthM1 )
    {
      numPos += abs( pData[ 1 ] ) > 2;
      if( posX < widthM1 - 1 )
      {
        numPos += abs( pData[ 2 ] ) > 2;
      }
      if( posY < heightM1 )
      {
        numPos += abs( pData[ m_width + 1 ] ) > 2;
      }
    }
    if( posY < heightM1 )
    {
      numPos += abs( pData[ m_width ] ) > 2;
      if( posY < heightM1 - 1 )
      {
        numPos += abs( pData[ 2 * m_width ] ) > 2;
      }
    }

    const Int ctxIdx = std::min( numPos, 4 ) + 1;
          Int ctxOfs = 0;

    if( m_chType == CHANNEL_TYPE_LUMA )
    {
      ctxOfs += diag < 3 ? 10 : ( diag < 10 ? 5 : 0 );
    }

    return m_gt1FlagCtxSet( ctxOfs + ctxIdx );
  }

  unsigned        GoRicePar       ( int       scanPos,
                              const TCoeff*   coeff,
                                    int       strd = 0  )
  {
    const UInt posY = m_scanPosY[ scanPos ];
    const UInt posX = m_scanPosX[ scanPos ];

    strd = strd == 0 ? m_width : strd;
    const TCoeff *pData = coeff + posX + posY * strd;
    const Int   widthM1 = m_width - 1;
    const Int  heightM1 = m_height - 1;
//    const Int      diag = posX + posY;

    Int numPos = 0;
    Int sumAbs = 0;

    if( posX < widthM1 )
    {
      sumAbs += abs( pData[ 1 ] );
      numPos += pData[ 1 ] != 0;
      if( posX < widthM1 - 1 )
      {
        sumAbs += abs( pData[ 2 ] );
        numPos += pData[ 2 ] != 0;
      }
      if( posY < heightM1 )
      {
        sumAbs += abs( pData[ m_width + 1 ] );
        numPos += pData[ m_width + 1 ] != 0;
      }
    }
    if( posY < heightM1 )
    {
      sumAbs += abs( pData[ m_width ] );
      numPos += pData[ m_width ] != 0;
      if( posY < heightM1 - 1 )
      {
        sumAbs += abs( pData[ 2 * m_width ] );
        numPos += pData[ 2 * m_width ] != 0;
      }
    }

    unsigned val   = sumAbs - numPos;
    unsigned order = 0;
    for( order = 0; order < MAX_GR_ORDER_RESIDUAL; order++ )
    {
      if( ( 1 << ( order + 3 ) ) > ( val + 4 ) )
      {
        break;
      }
    }
    return ( order == MAX_GR_ORDER_RESIDUAL ? ( MAX_GR_ORDER_RESIDUAL - 1 ) : order );
  }

#if JEM_TOOLS
  unsigned        emtNumSigCoeff()                          const { return m_emtNumSigCoeff; }
  void            setEmtNumSigCoeff( unsigned val )               { m_emtNumSigCoeff = val; }

#endif
private:
  // constant
  const ComponentID         m_compID;
  const ChannelType         m_chType;
  const unsigned            m_width;
  const unsigned            m_height;
  const unsigned            m_log2CGWidth;
  const unsigned            m_log2CGHeight;
  const unsigned            m_log2CGSize;
  const unsigned            m_widthInGroups;
  const unsigned            m_heightInGroups;
  const unsigned            m_log2BlockWidth;
  const unsigned            m_log2BlockHeight;
#if JEM_TOOLS && HEVC_USE_MDCS
  const unsigned            m_log2WidthInGroups;
  const unsigned            m_log2HeightInGroups;
#endif
  const unsigned            m_log2BlockSize;
  const unsigned            m_maxNumCoeff;
  const bool                m_AlignFlag;
#if HEVC_USE_SIGN_HIDING
  const bool                m_signHiding;
#endif
  const bool                m_useGoRiceParAdapt;
  const bool                m_extendedPrecision;
  const int                 m_maxLog2TrDynamicRange;
  CoeffScanType             m_scanType;
  const unsigned*           m_scan;
  const unsigned*           m_scanPosX;
  const unsigned*           m_scanPosY;
  const unsigned*           m_scanCG;
  const CtxSet              m_CtxSetLastX;
  const CtxSet              m_CtxSetLastY;
  const unsigned            m_maxLastPosX;
  const unsigned            m_maxLastPosY;
  const int                 m_lastOffsetX;
  const int                 m_lastOffsetY;
  const int                 m_lastShiftX;
  const int                 m_lastShiftY;
  const bool                m_TrafoBypass;
  const int                 m_SigBlockType;
#if !HM_QTBT_AS_IN_JEM_CONTEXT
  const uint8_t**           m_SigScanPatternBase;
#endif
  CtxSet                    m_sigCtxSet;
  // modified
  int                       m_scanPosLast;
  int                       m_subSetId;
  int                       m_subSetPos;
  int                       m_subSetPosX;
  int                       m_subSetPosY;
  int                       m_minSubPos;
  int                       m_maxSubPos;
  unsigned                  m_sigGroupCtxId;
#if HM_QTBT_AS_IN_JEM_CONTEXT
  int                       m_sigCGPattern;
#else
  const uint8_t*            m_sigScanCtxId;
#endif
  CtxSet                    m_gt1FlagCtxSet;
  unsigned                  m_gt2FlagCtxId;
  unsigned                  m_currentGolombRiceStatistic;
  bool                      m_prevGt2;
  std::bitset<MLS_GRP_NUM>  m_sigCoeffGroupFlag;
#if JEM_TOOLS
  unsigned                  m_altResiCompId;
#endif
#if JEM_TOOLS
  unsigned                  m_emtNumSigCoeff;
#endif
};


class CUCtx
{
public:
  CUCtx()              : isDQPCoded(false), isChromaQpAdjCoded(false),
                         numNonZeroCoeffNonTs(0) {}
  CUCtx(int _qp)       : isDQPCoded(false), isChromaQpAdjCoded(false),
                         numNonZeroCoeffNonTs(0), qp(_qp) {}
  ~CUCtx() {}
public:
  bool      isDQPCoded;
  bool      isChromaQpAdjCoded;
  UInt      numNonZeroCoeffNonTs;
  SChar     qp;                   // used as a previous(last) QP and for QP prediction
};

class MergeCtx
{
public:
  MergeCtx() : numValidMergeCand( 0 ), hasMergedCandList( false ) { for( unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++ ) mrgTypeNeighbours[i] = MRG_TYPE_DEFAULT_N; }
  ~MergeCtx() {}
public:
  MvField       mvFieldNeighbours [ MRG_MAX_NUM_CANDS << 1 ]; // double length for mv of both lists
#if JEM_TOOLS
  bool          LICFlags          [ MRG_MAX_NUM_CANDS      ];
#endif
  unsigned char interDirNeighbours[ MRG_MAX_NUM_CANDS      ];
  MergeType     mrgTypeNeighbours [ MRG_MAX_NUM_CANDS      ];
  int           numValidMergeCand;
  bool          hasMergedCandList;

#if JEM_TOOLS
  MotionBuf     subPuMvpMiBuf;
  MotionBuf     subPuMvpExtMiBuf;
  MotionBuf     subPuFrucMiBuf;
#endif
  Void setMergeInfo( PredictionUnit& pu, int candIdx );
};


namespace DeriveCtx
{
unsigned CtxCUsplit   ( const CodingStructure& cs, Partitioner& partitioner );
unsigned CtxBTsplit   ( const CodingStructure& cs, Partitioner& partitioner );
#if ENABLE_BMS
unsigned CtxQtCbf     ( const ComponentID compID, const unsigned trDepth );
#else
unsigned CtxQtCbf     ( const ComponentID compID );
#endif
unsigned CtxInterDir  ( const PredictionUnit& pu );
unsigned CtxSkipFlag  ( const CodingUnit& cu );
#if JEM_TOOLS
unsigned CtxIMVFlag   ( const CodingUnit& cu );
unsigned CtxAffineFlag( const CodingUnit& cu );
unsigned CtxFrucFlag  ( const PredictionUnit& pu );
unsigned CtxFrucMode  ( const PredictionUnit& pu );
#endif
}

#endif // __CONTEXTMODELLING__
