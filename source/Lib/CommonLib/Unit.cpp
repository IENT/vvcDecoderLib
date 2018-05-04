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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

#include "ChromaFormat.h"

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

Void CompArea::xRecalcLumaToChroma()
{
  const UInt csx = getComponentScaleX(compID, chromaFormat);
  const UInt csy = getComponentScaleY(compID, chromaFormat);

  x      >>= csx;
  y      >>= csy;
  width  >>= csx;
  height >>= csy;
}

Position CompArea::chromaPos() const
{
  if (isLuma(compID))
  {
    UInt scaleX = getComponentScaleX(compID, chromaFormat);
    UInt scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize() const
{
  if( isChroma( compID ) )
  {
    UInt scaleX = getComponentScaleX( compID, chromaFormat );
    UInt scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize() const
{
  if( isLuma( compID ) )
  {
    UInt scaleX = getComponentScaleX( compID, chromaFormat );
    UInt scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos() const
{
  if( isChroma( compID ) )
  {
    UInt scaleX = getComponentScaleX( compID, chromaFormat );
    UInt scaleY = getComponentScaleY( compID, chromaFormat );

    return Position( x << scaleX, y << scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos( const ComponentID compID ) const
{
  return isLuma( compID ) ? lumaPos() : chromaPos();
}

Position CompArea::chanPos( const ChannelType chType ) const
{
  return isLuma( chType ) ? lumaPos() : chromaPos();
}

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) { }

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area &_area) : chromaFormat(_chromaFormat), blocks(getNumberValidComponents(_chromaFormat))
{
  const UInt numCh = getNumberValidComponents(chromaFormat);

  for (UInt i = 0; i < numCh; i++)
  {
    blocks[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
  }
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY) : chromaFormat(_chromaFormat), blocks { blkY } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY) } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr)  : chromaFormat(_chromaFormat), blocks { blkY, blkCb, blkCr } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,      CompArea &&blkCb,      CompArea &&blkCr) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) } {}

Bool UnitArea::contains(const UnitArea& other) const
{
  Bool ret = true;
  Bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

Bool UnitArea::contains( const UnitArea& other, const ChannelType chType ) const
{
  Bool ret = true;
  Bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( toChannelType( blk.compID ) == chType && blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

Void UnitArea::repositionTo(const UnitArea& unitArea)
{
  for(UInt i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

const UnitArea UnitArea::singleComp(const ComponentID compID) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (blk.compID == compID)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

const UnitArea UnitArea::singleChan(const ChannelType chType) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (toChannelType(blk.compID) == chType)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

CodingUnit::CodingUnit(const UnitArea &unit)                                : UnitArea(unit),                 cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }

CodingUnit& CodingUnit::operator=( const CodingUnit& other )
{
  slice             = other.slice;
  predMode          = other.predMode;
  partSize          = other.partSize;
  qtDepth           = other.qtDepth;
  depth             = other.depth;
  btDepth           = other.btDepth;
  mtDepth           = other.mtDepth;
  splitSeries       = other.splitSeries;
  skip              = other.skip;
#if JEM_TOOLS
  affine            = other.affine;
#endif
  transQuantBypass  = other.transQuantBypass;
#if JEM_TOOLS
  pdpc              = other.pdpc;
#endif
  ipcm              = other.ipcm;
  qp                = other.qp;
  chromaQpAdj       = other.chromaQpAdj;
  rootCbf           = other.rootCbf;
#if JEM_TOOLS
  nsstIdx           = other.nsstIdx;
  emtFlag           = other.emtFlag;
#endif
#if JEM_TOOLS
  LICFlag           = other.LICFlag;
#endif
#if HEVC_TILES_WPP
  tileIdx           = other.tileIdx;
#endif
#if JEM_TOOLS
  imv               = other.imv;
  imvNumCand        = other.imvNumCand;
  obmcFlag          = other.obmcFlag;
#endif

  return *this;
}

Void CodingUnit::initData()
{
  predMode          = NUMBER_OF_PREDICTION_MODES;
  partSize          = NUMBER_OF_PART_SIZES;
  qtDepth           = 0;
  depth             = 0;
  btDepth           = 0;
  mtDepth           = 0;
  splitSeries       = 0;
  skip              = false;
#if JEM_TOOLS
  affine            = false;
#endif
  transQuantBypass  = false;
#if JEM_TOOLS
  pdpc              = false;
#endif
  ipcm              = false;
  qp                = 0;
  chromaQpAdj       = 0;
  rootCbf           = true;
#if JEM_TOOLS
  nsstIdx           = 0;
  emtFlag           = 0;
#endif
#if JEM_TOOLS
  LICFlag           = false;
#endif
#if HEVC_TILES_WPP
  tileIdx           = 0;
#endif
#if JEM_TOOLS
  imv               = 0;
  imvNumCand        = 0;
  obmcFlag          = false;
#endif
}


// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

PredictionUnit::PredictionUnit(const UnitArea &unit)                                : UnitArea(unit)                , cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }
PredictionUnit::PredictionUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }

Void PredictionUnit::initData()
{
  // intra data - need this default initialization for PCM
  intraDir[0] = DC_IDX;
  intraDir[1] = PLANAR_IDX;

  // inter data
  mergeFlag   = false;
  mergeIdx    = MAX_UCHAR;
  interDir    = MAX_UCHAR;
  mergeType   = MRG_TYPE_DEFAULT_N;
#if JEM_TOOLS
  frucMrgMode = 0;
  mvRefine    = false;
#endif
  for (UInt i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    mv[i]     .setZero();
    mvd[i]    .setZero();
  }
}

PredictionUnit& PredictionUnit::operator=(const IntraPredictionData& predData)
{
  for (UInt i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    intraDir[i] = predData.intraDir[i];
  }

  return *this;
}

PredictionUnit& PredictionUnit::operator=(const InterPredictionData& predData)
{
  mergeFlag   = predData.mergeFlag;
  mergeIdx    = predData.mergeIdx;
  interDir    = predData.interDir;
  mergeType   = predData.mergeType;
#if JEM_TOOLS
  frucMrgMode = predData.frucMrgMode;
  mvRefine    = predData.mvRefine;
#endif
  for (UInt i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i]   = predData.mvpIdx[i];
    mvpNum[i]   = predData.mvpNum[i];
    mv[i]       = predData.mv[i];
    mvd[i]      = predData.mvd[i];
    refIdx[i]   = predData.refIdx[i];
  }

  return *this;
}

PredictionUnit& PredictionUnit::operator=( const PredictionUnit& other )
{
  for( UInt i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    intraDir[ i ] = other.intraDir[ i ];
  }

  mergeFlag   = other.mergeFlag;
  mergeIdx    = other.mergeIdx;
  interDir    = other.interDir;
  mergeType   = other.mergeType;
#if JEM_TOOLS
  frucMrgMode = other.frucMrgMode;
  mvRefine    = other.mvRefine;
#endif
  for (UInt i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i]   = other.mvpIdx[i];
    mvpNum[i]   = other.mvpNum[i];
    mv[i]       = other.mv[i];
    mvd[i]      = other.mvd[i];
    refIdx[i]   = other.refIdx[i];
  }

  return *this;
}

PredictionUnit& PredictionUnit::operator=( const MotionInfo& mi )
{
  interDir = mi.interDir;

  for( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.refIdx[i];
    mv    [i] = mi.mv[i];
  }

  return *this;
}

const MotionInfo& PredictionUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& PredictionUnit::getMotionInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getMotionInfo( pos );
}

MotionBuf PredictionUnit::getMotionBuf()
{
  return cs->getMotionBuf( *this );
}

CMotionBuf PredictionUnit::getMotionBuf() const
{
  return cs->getMotionBuf( *this );
}

#if JEM_TOOLS
const MotionInfo& PredictionUnit::getMotionInfoFRUC() const
{
  return cs->getMotionInfoFRUC( lumaPos() );
}

const MotionInfo& PredictionUnit::getMotionInfoFRUC( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getMotionInfoFRUC( pos );
}

MotionBuf PredictionUnit::getMotionBufFRUC()
{
  return cs->getMotionBufFRUC( *this );
}
#endif

// ---------------------------------------------------------------------------
// transform unit method definitions
// ---------------------------------------------------------------------------

TransformUnit::TransformUnit(const UnitArea& unit) : UnitArea(unit), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
    m_pcmbuf[i] = nullptr;
  }

  initData();
}

TransformUnit::TransformUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
    m_pcmbuf[i] = nullptr;
  }

  initData();
}

Void TransformUnit::initData()
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    cbf[i]           = 0;
    rdpcm[i]         = NUMBER_OF_RDPCM_MODES;
    transformSkip[i] = false;
    compAlpha[i]     = 0;
  }
#if HEVC_USE_RQT || ENABLE_BMS
  depth              = 0;
#endif
#if JEM_TOOLS
  emtIdx             = 0;
#endif

}

Void TransformUnit::init(TCoeff **coeffs, Pel **pcmbuf)
{
  UInt numBlocks = getNumberValidTBlocks(*cs->pcv);

  for (UInt i = 0; i < numBlocks; i++)
  {
    m_coeffs[i] = coeffs[i];
    m_pcmbuf[i] = pcmbuf[i];
  }
}

TransformUnit& TransformUnit::operator=(const TransformUnit& other)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  unsigned numBlocks = ::getNumberValidTBlocks(*cs->pcv);
  for( unsigned i = 0; i < numBlocks; i++ )
  {
    CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

    UInt area = blocks[i].area();

    if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
    if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);

    cbf[i]           = other.cbf[i];
    rdpcm[i]         = other.rdpcm[i];
    transformSkip[i] = other.transformSkip[i];
    compAlpha[i]     = other.compAlpha[i];
  }
#if HEVC_USE_RQT || ENABLE_BMS
  depth              = other.depth;
#endif
#if JEM_TOOLS
  emtIdx             = other.emtIdx;
#endif
  return *this;
}

Void TransformUnit::copyComponentFrom(const TransformUnit& other, const ComponentID i)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

  UInt area = blocks[i].area();

  if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
  if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);

  cbf[i]           = other.cbf[i];
  rdpcm[i]         = other.rdpcm[i];
  transformSkip[i] = other.transformSkip[i];
  compAlpha[i]     = other.compAlpha[i];

#if HEVC_USE_RQT || ENABLE_BMS
  depth            = other.depth;

#endif
  if( isLuma( i ) )
  {
#if JEM_TOOLS
    emtIdx         = other.emtIdx;
#endif
  }
}

       CoeffBuf TransformUnit::getCoeffs(const ComponentID id)       { return  CoeffBuf(m_coeffs[id], blocks[id]); }
const CCoeffBuf TransformUnit::getCoeffs(const ComponentID id) const { return CCoeffBuf(m_coeffs[id], blocks[id]); }

       PelBuf   TransformUnit::getPcmbuf(const ComponentID id)       { return  PelBuf  (m_pcmbuf[id], blocks[id]); }
const CPelBuf   TransformUnit::getPcmbuf(const ComponentID id) const { return CPelBuf  (m_pcmbuf[id], blocks[id]); }
