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

/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"


#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>
#if ENABLE_WPP_PARALLELISM
#include <mutex>
extern std::recursive_mutex g_cache_mutex;
#endif



//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

void EncCu::create( EncCfg* encCfg )
{
  unsigned      uiMaxWidth    = encCfg->getMaxCUWidth();
  unsigned      uiMaxHeight   = encCfg->getMaxCUHeight();
  ChromaFormat  chromaFormat  = encCfg->getChromaFormatIdc();
  bool          BTnoRQT       = encCfg->getQTBT();

  unsigned      numWidths     = gp_sizeIdxInfo->numWidths();
  unsigned      numHeights    = gp_sizeIdxInfo->numHeights();
#if JVET_K0357_AMVR
  unsigned      maxMEPart     = BTnoRQT ? 1 : NUMBER_OF_PART_SIZES;
#endif
  m_pTempCS = new CodingStructure**  [numWidths];
  m_pBestCS = new CodingStructure**  [numWidths];

  for( unsigned w = 0; w < numWidths; w++ )
  {
    m_pTempCS[w] = new CodingStructure*  [numHeights];
    m_pBestCS[w] = new CodingStructure*  [numHeights];

    for( unsigned h = 0; h < numHeights; h++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( h );

      if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
      {
        m_pTempCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pTempCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
        m_pBestCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
      }
      else
      {
        m_pTempCS[w][h] = nullptr;
        m_pBestCS[w][h] = nullptr;
      }
    }
  }

#if JVET_K0357_AMVR
  // WIA: only the weight==height case is relevant without QTBT
  m_pImvTempCS = nullptr;

  if( IMV_OFF != encCfg->getIMV() && !BTnoRQT )
  {
    m_pImvTempCS = new CodingStructure**[numWidths];

    for( unsigned w = 0; w < numWidths; w++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( w );

      m_pImvTempCS[w] = new CodingStructure*[maxMEPart];

      for( unsigned p = 0; p < maxMEPart; p++ )
      {
        if( gp_sizeIdxInfo->isCuSize( width ) )
        {
          m_pImvTempCS[w][p] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
          m_pImvTempCS[w][p]->create( chromaFormat, Area( 0, 0, width, height ), false );
        }
        else
        {
          m_pImvTempCS[w][p] = nullptr;
        }
      }
    }
  }
#endif

#if JEM_TOOLS
  m_pTempCUWoOBMC  = nullptr;
  m_pPredBufWoOBMC = nullptr;

  if( encCfg->getUseOBMC() )
  {
    m_pTempCUWoOBMC  = new CodingStructure**[numWidths];
    m_pPredBufWoOBMC = new PelStorage*[numWidths];

    for( unsigned w = 0; w < numWidths; w++ )
    {
      m_pTempCUWoOBMC [w] = new CodingStructure*[numHeights];
      m_pPredBufWoOBMC[w] = new PelStorage[numHeights];

      for( unsigned h = 0; h < numHeights; h++ )
      {
        uint32_t width  = gp_sizeIdxInfo->sizeFrom( w );
        uint32_t height = gp_sizeIdxInfo->sizeFrom( h );

        if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
        {

          m_pTempCUWoOBMC[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
          m_pTempCUWoOBMC[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
          m_pPredBufWoOBMC[w][h].create( UnitArea( chromaFormat, Area( 0, 0, width, height ) ) );
        }
      }
    }
  }
#endif

  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  if( encCfg->getQTBT() )
  {
    m_modeCtrl = new EncModeCtrlMTnoRQT();
  }
  else
  {
    THROW( "Unknown partitioner!" );
  }

#if REUSE_CU_RESULTS
  m_modeCtrl->create( *encCfg );

#endif
  for( unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++ )
  {
    m_acMergeBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }

  m_CtxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
}


void EncCu::destroy()
{
  bool          BTnoRQT   = m_pcEncCfg->getQTBT();
#if JVET_K0357_AMVR
  unsigned      maxMEPart = BTnoRQT ? 1 : NUMBER_OF_PART_SIZES;
#endif

  unsigned numWidths  = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

  for( unsigned w = 0; w < numWidths; w++ )
  {
    for( unsigned h = 0; h < numHeights; h++ )
    {
      if( BTnoRQT || w == h )
      {
        if( m_pBestCS[w][h] ) m_pBestCS[w][h]->destroy();
        if( m_pTempCS[w][h] ) m_pTempCS[w][h]->destroy();

        delete m_pBestCS[w][h];
        delete m_pTempCS[w][h];
      }
    }

    delete[] m_pTempCS[w];
    delete[] m_pBestCS[w];
  }

  delete[] m_pBestCS; m_pBestCS = nullptr;
  delete[] m_pTempCS; m_pTempCS = nullptr;

#if REUSE_CU_RESULTS
  m_modeCtrl->destroy();

#endif
  delete m_modeCtrl;
  m_modeCtrl = nullptr;

#if JVET_K0357_AMVR
  // WIA: only the weight==height case is relevant without QTBT
  if( m_pImvTempCS )
  {
    for( unsigned w = 0; w < numWidths; w++ )
    {
      for( unsigned p = 0; p < maxMEPart; p++ )
      {
        if( m_pImvTempCS[w][p] ) m_pImvTempCS[w][p]->destroy();
        delete m_pImvTempCS[w][p];
      }
      delete[] m_pImvTempCS[w];
    }

    delete[] m_pImvTempCS;
    m_pImvTempCS = nullptr;
  }

#endif
#if JEM_TOOLS
  if( m_pTempCUWoOBMC )
  {
    for( unsigned w = 0; w < numWidths; w++ )
    {
      for( unsigned h = 0; h < numHeights; h++ )
      {
        if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( w ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( h ) ) )
        {
          m_pTempCUWoOBMC[w][h]->destroy();
          delete m_pTempCUWoOBMC[w][h];

          m_pPredBufWoOBMC[w][h].destroy();
        }
      }
      delete[] m_pTempCUWoOBMC[w];
      delete[] m_pPredBufWoOBMC[w];
    }

    delete[] m_pTempCUWoOBMC;
    m_pTempCUWoOBMC = nullptr;

    delete[] m_pPredBufWoOBMC;
    m_pPredBufWoOBMC = nullptr;
  }

#endif
  for( unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++ )
  {
    m_acMergeBuffer[ui].destroy();
  }
}



EncCu::~EncCu()
{
}



/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int tId ) )
{
  m_pcEncCfg           = pcEncLib;
  m_pcIntraSearch      = pcEncLib->getIntraSearch( PARL_PARAM0( tId ) );
  m_pcInterSearch      = pcEncLib->getInterSearch( PARL_PARAM0( tId ) );
  m_pcTrQuant          = pcEncLib->getTrQuant( PARL_PARAM0( tId ) );
  m_pcRdCost           = pcEncLib->getRdCost ( PARL_PARAM0( tId ) );
  m_CABACEstimator     = pcEncLib->getCABACEncoder( PARL_PARAM0( tId ) )->getCABACEstimator( &sps );
#if JVET_K0346
  m_CABACEstimator->setEncCu(this);
#endif
  m_CtxCache           = pcEncLib->getCtxCache( PARL_PARAM0( tId ) );
  m_pcRateCtrl         = pcEncLib->getRateCtrl();
  m_pcSliceEncoder     = pcEncLib->getSliceEncoder();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_pcEncLib           = pcEncLib;
  m_dataId             = tId;
#endif

#if REUSE_CU_RESULTS
  DecCu::init( m_pcTrQuant, m_pcIntraSearch, m_pcInterSearch );

#endif
  m_modeCtrl->init( m_pcEncCfg, m_pcRateCtrl, m_pcRdCost );

  m_pcInterSearch->setModeCtrl( m_modeCtrl );
#if !JVET_K0220_ENC_CTRL
  m_pcIntraSearch->setModeCtrl( m_modeCtrl );
#endif
#if JVET_K0346
  ::memset(m_subMergeBlkSize, 0, sizeof(m_subMergeBlkSize));
  ::memset(m_subMergeBlkNum, 0, sizeof(m_subMergeBlkNum));
  m_prevPOC = MAX_UINT;
  m_clearSubMergeStatic = false;
#endif
#if JVET_K0076_CPR
  if (m_pcEncCfg->getIBCHashSearch())
  {
    m_ibcHashMap.init(m_pcEncCfg->getSourceWidth(), m_pcEncCfg->getSourceHeight());
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] )
{
#if JVET_K0076_CPR
  if (m_pcEncCfg->getIBCHashSearch() && ctuRsAddr == 0)
  {
    m_ibcHashMap.rebuildPicHashMap(cs.picture->getOrigBuf());
  }
#endif
  m_modeCtrl->initCTUEncoding( *cs.slice );

#if ENABLE_SPLIT_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 )
  {
    for( int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++ )
    {
      EncCu*            jobEncCu  = m_pcEncLib->getCuEncoder( cs.picture->scheduler.getSplitDataId( jId ) );
      CacheBlkInfoCtrl* cacheCtrl = dynamic_cast< CacheBlkInfoCtrl* >( jobEncCu->m_modeCtrl );
      if( cacheCtrl )
      {
        cacheCtrl->init( *cs.slice );
      }
    }
  }

  if( auto* cacheCtrl = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) ) { cacheCtrl->tick(); }
#endif
  // init the partitioning manager
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );
  partitioner->initCtu( area, CH_L, *cs.slice );
#if JVET_K0076_CPR
  m_pcInterSearch->resetCtuRecord();
  m_ctuIbcSearchRangeX = m_pcEncCfg->getIBCLocalSearchRangeX();
  m_ctuIbcSearchRangeY = m_pcEncCfg->getIBCLocalSearchRangeY();
  if (m_pcEncCfg->getIBCMode() && m_pcEncCfg->getIBCHashSearch() && (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
  {
    const int hashHitRatio = m_ibcHashMap.getHashHitRatio(area.Y()); // in percent
    if (hashHitRatio < 5)
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
    if (cs.slice->getNumRefIdx(REF_PIC_LIST_0) > 1)
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
  }
#endif //CPR
  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];

  cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false );
  cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false );
  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
  tempCS->baseQP       = bestCS->baseQP       = currQP[CH_L];
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  xCompressCU( tempCS, bestCS, *partitioner );


  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
  cs.useSubStructure( *bestCS, partitioner->chType, CS::getArea( *bestCS, area, partitioner->chType ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );

  if( !cs.pcv->ISingleTree && cs.slice->isIRAP() && cs.pcv->chrFormat != CHROMA_400 )
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner->initCtu( area, CH_C, *cs.slice );

    cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false );
    cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false );
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
    tempCS->baseQP       = bestCS->baseQP       = currQP[CH_C];
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];

    xCompressCU( tempCS, bestCS, *partitioner );

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
    cs.useSubStructure( *bestCS, partitioner->chType, CS::getArea( *bestCS, area, partitioner->chType ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );
  }

#if JVET_K0390_RATECTRL
  if (m_pcEncCfg->getUseRateCtrl())
  {
    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_actualMSE = (double)bestCS->dist / (double)m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr).m_numberOfPixel;
  }
#endif
  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;
  delete partitioner;

#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 && m_pcEncCfg->getNumWppThreads() > 1 )
  {
    cs.picture->finishCtuPart( area );
  }
#endif

  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->partSize == NUMBER_OF_PART_SIZES      , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const int iStrideOrg)
{
  int k, i, j, jj;
  int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j = 0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj    ] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj    ] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return(iSumHad);
}

int  EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  int  xBl, yBl;
  const int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  int  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for( yBl = 0; ( yBl + iBlkSize ) <= buf.height; yBl += iBlkSize )
  {
    for( xBl = 0; ( xBl + iBlkSize ) <= buf.width; xBl += iBlkSize )
    {
      const Pel* pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice( pOrg, iStrideOrig );
    }
  }
  return( iSumHad );
}

void EncCu::xCheckBestMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( !tempCS->cus.empty() )
  {
    if( tempCS->cus.size() == 1 )
    {
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK( cu.skip && !cu.firstPU->mergeFlag, "Skip flag without a merge flag is not allowed!" );
    }

#if WCG_EXT
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda() );
#endif

    if( m_modeCtrl->useModeResult( encTestMode, tempCS, partitioner ) )
    {
      if( tempCS->cus.size() == 1 )
      {
        // if tempCS is not a split-mode
        CodingUnit &cu = *tempCS->cus.front();

        if( CU::isLosslessCoded( cu ) && !cu.ipcm )
        {
          xFillPCMBuffer( cu );
        }
      }

      std::swap( tempCS, bestCS );
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
}

void EncCu::xCompressCU( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
#if ENABLE_SPLIT_PARALLELISM
  CHECK( m_dataId != tempCS->picture->scheduler.getDataId(), "Working in the wrong dataId!" );

  if( m_pcEncCfg->getNumSplitThreads() != 1 && tempCS->picture->scheduler.getSplitJobId() == 0 )
  {
    if( m_modeCtrl->isParallelSplit( *tempCS, partitioner ) )
    {
      m_modeCtrl->setParallelSplit( true );
      xCompressCUParallel( tempCS, bestCS, partitioner );
      return;
    }
  }

#endif

  Slice&   slice      = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const uint32_t uiLPelX  = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY  = tempCS->area.Y().lumaPos().y;

#if JEM_TOOLS || JVET_K0357_AMVR
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth()  );
#endif
#if JEM_TOOLS
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );
#endif

  const UnitArea currCsArea = clipArea( CS::getArea( *bestCS, bestCS->area, partitioner.chType ), *tempCS->picture );
#if JVET_K0357_AMVR
  if( m_pImvTempCS && !slice.isIntra() )
  {
    const unsigned maxMEPart = tempCS->pcv->only2Nx2N ? 1 : NUMBER_OF_PART_SIZES;
    for( unsigned p = 0; p < maxMEPart; p++ )
    {
      tempCS->initSubStructure( *m_pImvTempCS[wIdx][p], partitioner.chType, partitioner.currArea(), false );
    }
  }
#endif

#if JEM_TOOLS
  if( m_pTempCUWoOBMC && !slice.isIntra() )
  {
    tempCS->initSubStructure( *m_pTempCUWoOBMC[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
  }
#endif
#if JVET_K0076_CPR
  tempCS->chType = partitioner.chType;
  bestCS->chType = partitioner.chType;
#endif
  m_modeCtrl->initCULevel( partitioner, *tempCS );

  m_CurrCtx->start = m_CABACEstimator->getCtx();

  m_cuChromaQpOffsetIdxPlus1 = 0;

  if( slice.getUseChromaQpAdj() )
  {
    int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
      std::max<int>( 0, sps.getLog2DiffMaxMinCodingBlockSize() - int( pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() ) );
    m_cuChromaQpOffsetIdxPlus1 = ( ( uiLPelX >> lgMinCuSize ) + ( uiTPelY >> lgMinCuSize ) ) % ( pps.getPpsRangeExtension().getChromaQpOffsetListLen() + 1 );
  }

  if( !m_modeCtrl->anyMode() )
  {
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cux", uiLPelX ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuy", uiTPelY ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuw", tempCS->area.lwidth() ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuh", tempCS->area.lheight() ) );
  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );

  do
  {
    const EncTestMode currTestMode = m_modeCtrl->currTestMode();

#if SHARP_LUMA_DELTA_QP
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && partitioner.currDepth <= pps.getMaxCuDQPDepth() )
    {
#if ENABLE_SPLIT_PARALLELISM
      CHECK( tempCS->picture->scheduler.getSplitJobId() > 0, "Changing lambda is only allowed in the master thread!" );
#endif
      if (currTestMode.qp >= 0)
      {
        updateLambda(&slice, currTestMode.qp);
      }
    }
#endif

    if( currTestMode.type == ETM_INTER_ME )
    {
#if JEM_TOOLS
      bool tryObmc = true;
#endif
#if JVET_K0357_AMVR
      if( ( currTestMode.opts & ETO_IMV ) != 0 )
      {
#if JEM_TOOLS
        tryObmc = xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode);
#else
        xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode);
#endif
      }
      else
#endif
      {
        xCheckRDCostInter( tempCS, bestCS, partitioner, currTestMode );
      }

#if JEM_TOOLS
      if( tryObmc )
      {
        xCheckRDCostInterWoOBMC( tempCS, bestCS, partitioner, currTestMode );
      }
#endif
    }
#if JEM_TOOLS || JVET_K_AFFINE
    else if( currTestMode.type == ETM_AFFINE )
    {
      xCheckRDCostAffineMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
#endif
#if REUSE_CU_RESULTS
    else if( currTestMode.type == ETM_RECO_CACHED )
    {
      xReuseCachedResult( tempCS, bestCS, partitioner );
    }
#endif
    else if( currTestMode.type == ETM_MERGE_SKIP )
    {
      xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
#if JEM_TOOLS
    else if( currTestMode.type == ETM_MERGE_FRUC )
    {
      xCheckRDCostMerge2Nx2NFRUC( tempCS, bestCS, partitioner, currTestMode );
    }
#endif
    else if( currTestMode.type == ETM_INTRA )
    {
      xCheckRDCostIntra( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_IPCM )
    {
      xCheckIntraPCM( tempCS, bestCS, partitioner, currTestMode );
    }
#if JVET_K0076_CPR
    else if (currTestMode.type == ETM_IBC)
    {
      xCheckRDCostIntraBC(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (currTestMode.type == ETM_IBC_MERGE)
    {
      xCheckRDCostIntraBCMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
    }
#endif // CPR
    else if( isModeSplit( currTestMode ) )
    {

      xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode );
    }
    else
    {
      THROW( "Don't know how to handle mode: type = " << currTestMode.type << ", size = " << currTestMode.partSize << ", options = " << currTestMode.opts );
    }
  } while( m_modeCtrl->nextMode( *tempCS, partitioner ) );

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
#if ENABLE_SPLIT_PARALLELISM
  if( bestCS->cus.empty() )
  {
    CHECK( bestCS->cost != MAX_DOUBLE, "Cost should be maximal if no encoding found" );
    CHECK( bestCS->picture->scheduler.getSplitJobId() == 0, "Should always get a result in serial case" );

    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

#endif
  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;

  bestCS->picture->getRecoBuf( currCsArea ).copyFrom( bestCS->getRecoBuf( currCsArea ) );
  m_modeCtrl->finishCULevel( partitioner );

#if ENABLE_SPLIT_PARALLELISM
  if( tempCS->picture->scheduler.getSplitJobId() == 0 && m_pcEncCfg->getNumSplitThreads() != 1 )
  {
    tempCS->picture->finishParallelPart( currCsArea );
  }

#endif
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->partSize == NUMBER_OF_PART_SIZES      , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

#if SHARP_LUMA_DELTA_QP
void EncCu::updateLambda( Slice* slice, double dQP )
{
#if WCG_EXT
  int    NumberBFrames = ( m_pcEncCfg->getGOPSize() - 1 );
  int    SHIFT_QP = 12;
  double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)(slice->getPic()->fieldPic ? NumberBFrames/2 : NumberBFrames) );

#if DISTORTION_LAMBDA_BUGFIX
  int bitdepth_luma_qp_scale = 6
                               * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                                  - DISTORTION_PRECISION_ADJUSTMENT(slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
#else
#if FULL_NBIT
  int bitdepth_luma_qp_scale = 6 * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
  int    bitdepth_luma_qp_scale = 0;
#endif
#endif
  double qp_temp = (double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;

  double dQPFactor = m_pcEncCfg->getGOPEntry( m_pcSliceEncoder->getGopId() ).m_QPFactor;

  if( slice->getSliceType() == I_SLICE )
  {
    if( m_pcEncCfg->getIntraQpFactor() >= 0.0 /*&& m_pcEncCfg->getGOPEntry( m_pcSliceEncoder->getGopId() ).m_sliceType != I_SLICE*/ )
    {
      dQPFactor = m_pcEncCfg->getIntraQpFactor();
    }
    else
    {
      if( m_pcEncCfg->getLambdaFromQPEnable() )
      {
        dQPFactor = 0.57;
      }
      else
      {
        dQPFactor = 0.57*dLambda_scale;
      }
    }
  }
  else if( m_pcEncCfg->getLambdaFromQPEnable() )
  {
    dQPFactor = 0.57*dQPFactor;
  }

  double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
  int depth = slice->getDepth();

  if( !m_pcEncCfg->getLambdaFromQPEnable() && depth>0 )
  {
    int qp_temp_slice = slice->getSliceQp() + bitdepth_luma_qp_scale - SHIFT_QP; // avoid lambda  over adjustment,  use slice_qp here
    dLambda *= Clip3( 2.00, 4.00, (qp_temp_slice / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
  }
  if( !m_pcEncCfg->getUseHADME() && slice->getSliceType( ) != I_SLICE )
  {
    dLambda *= 0.95;
  }

  const int temporalId = m_pcEncCfg->getGOPEntry( m_pcSliceEncoder->getGopId() ).m_temporalId;
  const std::vector<double> &intraLambdaModifiers = m_pcEncCfg->getIntraLambdaModifier();
  double lambdaModifier;
  if( slice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcEncCfg->getLambdaModifier(temporalId);
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[(temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size() - 1)];
  }
  dLambda *= lambdaModifier;

  int qpBDoffset = slice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  int iQP = Clip3(-qpBDoffset, MAX_QP, (int)floor(dQP + 0.5));
  m_pcSliceEncoder->setUpLambda(slice, dLambda, iQP);

#else
  int iQP = (int)dQP;
  const double oldQP     = (double)slice->getSliceQpBase();
  const double oldLambda = m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), slice->getDepth(), oldQP, oldQP, iQP);
  const double newLambda = oldLambda * pow (2.0, (dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const double chromaLambda = newLambda / m_pcRdCost->getChromaWeight();
  const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
  m_pcTrQuant->setLambdas (lambdaArray);
#else
  m_pcTrQuant->setLambda (newLambda);
#endif
  m_pcRdCost->setLambda( newLambda, slice->getSPS()->getBitDepths() );
#endif
}
#endif

#if ENABLE_SPLIT_PARALLELISM
//#undef DEBUG_PARALLEL_TIMINGS
//#define DEBUG_PARALLEL_TIMINGS 1
void EncCu::xCompressCUParallel( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth() );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  Picture* picture = tempCS->picture;

  int numJobs = m_modeCtrl->getNumParallelJobs( *bestCS, partitioner );

  bool    jobUsed                            [NUM_RESERVERD_SPLIT_JOBS];
  std::fill( jobUsed, jobUsed + NUM_RESERVERD_SPLIT_JOBS, false );

  const UnitArea currArea = CS::getArea( *tempCS, partitioner.currArea(), partitioner.chType );
#if ENABLE_WPP_PARALLELISM
  const int      wppTId   = picture->scheduler.getWppThreadId();
#endif
  const bool doParallel   = !m_pcEncCfg->getForceSingleSplitThread();
#if _MSC_VER && ENABLE_WPP_PARALLELISM
#pragma omp parallel for schedule(dynamic,1) num_threads(NUM_SPLIT_THREADS_IF_MSVC) if(doParallel)
#else
  omp_set_num_threads( m_pcEncCfg->getNumSplitThreads() );

#pragma omp parallel for schedule(dynamic,1) if(doParallel)
#endif
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    // thread start
#if ENABLE_WPP_PARALLELISM
    picture->scheduler.setWppThreadId( wppTId );
#endif
    picture->scheduler.setSplitThreadId();
    picture->scheduler.setSplitJobId( jId );

    Partitioner* jobPartitioner = PartitionerFactory::get( *tempCS->slice );
    EncCu*       jobCuEnc       = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );
    auto*        jobBlkCache    = dynamic_cast<CacheBlkInfoCtrl*>( jobCuEnc->m_modeCtrl );

    jobPartitioner->copyState( partitioner );
    jobCuEnc      ->copyState( this, *jobPartitioner, currArea, true );

    if( jobBlkCache )
    {
      jobBlkCache->tick();
    }

    CodingStructure *&jobBest = jobCuEnc->m_pBestCS[wIdx][hIdx];
    CodingStructure *&jobTemp = jobCuEnc->m_pTempCS[wIdx][hIdx];

    jobUsed[jId] = true;

    jobCuEnc->xCompressCU( jobTemp, jobBest, *jobPartitioner );

    delete jobPartitioner;

    picture->scheduler.setSplitJobId( 0 );
    // thread stop
  }
  picture->scheduler.setSplitThreadId( 0 );

  int    bestJId  = 0;
  double bestCost = bestCS->cost;
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    EncCu* jobCuEnc = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );

    if( jobUsed[jId] && jobCuEnc->m_pBestCS[wIdx][hIdx]->cost < bestCost )
    {
      bestCost = jobCuEnc->m_pBestCS[wIdx][hIdx]->cost;
      bestJId  = jId;
    }
  }

  if( bestJId > 0 )
  {
    copyState( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( bestJId ) ), partitioner, currArea, false );
    m_CurrCtx->best = m_CABACEstimator->getCtx();

    tempCS = m_pTempCS[wIdx][hIdx];
    bestCS = m_pBestCS[wIdx][hIdx];
  }

  const int      bitDepthY = tempCS->sps->getBitDepth( CH_L );
  const UnitArea clipdArea = clipArea( currArea, *picture );

  CHECK( calcCheckSum( picture->getRecoBuf( clipdArea.Y() ), bitDepthY ) != calcCheckSum( bestCS->getRecoBuf( clipdArea.Y() ), bitDepthY ), "Data copied incorrectly!" );

  picture->finishParallelPart( currArea );

  if( auto *blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) )
  {
    for( int jId = 1; jId <= numJobs; jId++ )
    {
      if( !jobUsed[jId] || jId == bestJId ) continue;

      auto *jobBlkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) )->m_modeCtrl );
      CHECK( !jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!" );
      blkCache->CacheBlkInfoCtrl::copyState( *jobBlkCache, partitioner.currArea() );
    }

    blkCache->tick();
  }

}

void EncCu::copyState( EncCu* other, Partitioner& partitioner, const UnitArea& currArea, const bool isDist )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth () );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  if( isDist )
  {
    other->m_pBestCS[wIdx][hIdx]->initSubStructure( *m_pBestCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
    other->m_pTempCS[wIdx][hIdx]->initSubStructure( *m_pTempCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
  }
  else
  {
          CodingStructure* dst =        m_pBestCS[wIdx][hIdx];
    const CodingStructure *src = other->m_pBestCS[wIdx][hIdx];
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;

    dst->useSubStructure( *src, partitioner.chType, currArea, KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );
    dst->cost           =  src->cost;
    dst->dist           =  src->dist;
    dst->fracBits       =  src->fracBits;
    dst->features       =  src->features;
  }

  if( isDist )
  {
    m_CurrCtx = m_CtxBuffer.data();
  }

  m_pcInterSearch->copyState( *other->m_pcInterSearch );
  m_modeCtrl     ->copyState( *other->m_modeCtrl, partitioner.currArea() );
  m_pcRdCost     ->copyState( *other->m_pcRdCost );
  m_pcTrQuant    ->copyState( *other->m_pcTrQuant );

  m_CABACEstimator->getCtx() = other->m_CABACEstimator->getCtx();
}
#endif

void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  const int qp                = encTestMode.qp;
  const PPS &pps              = *tempCS->pps;
  const Slice &slice          = *tempCS->slice;
  const bool bIsLosslessMode  = false; // False at this level. Next level down may set it to true.
  const int oldPrevQp         = tempCS->prevQP[partitioner.chType];
  const uint32_t currDepth        = partitioner.currDepth;

  const PartSplit split = getPartSplit( encTestMode );

  CHECK( split == CU_DONT_SPLIT, "No proper split provided!" );

  tempCS->initStructData( qp, bIsLosslessMode );

#if !JVET_K0220_ENC_CTRL
  partitioner.splitCurrArea( split, *tempCS );

#endif
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
#if JVET_K0220_ENC_CTRL

  if( tempCS->sps->getSpsNext().getUseQTBT() )
  {
    const TempCtx ctxStartSP( m_CtxCache, SubCtx( Ctx::SplitFlag,   m_CABACEstimator->getCtx() ) );
    const TempCtx ctxStartBT( m_CtxCache, SubCtx( Ctx::BTSplitFlag, m_CABACEstimator->getCtx() ) );

    m_CABACEstimator->resetBits();

    if( partitioner.getImplicitSplit( *tempCS ) != CU_QUAD_SPLIT )
    {
      if( partitioner.canSplit( CU_QUAD_SPLIT, *tempCS ) )
      {
        m_CABACEstimator->split_cu_flag( split == CU_QUAD_SPLIT, *tempCS, partitioner );
      }
      if( split != CU_QUAD_SPLIT )
      {
        m_CABACEstimator->split_cu_mode_mt( split, *tempCS, partitioner );
      }
    }

    const double factor = ( tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075 );
    const double cost   = m_pcRdCost->calcRdCost( uint64_t( m_CABACEstimator->getEstFracBits() + ( ( bestCS->fracBits ) / factor ) ), Distortion( bestCS->dist / factor ) );

    m_CABACEstimator->getCtx() = SubCtx( Ctx::SplitFlag,   ctxStartSP );
    m_CABACEstimator->getCtx() = SubCtx( Ctx::BTSplitFlag, ctxStartBT );

    if( cost > bestCS->cost )
    {
      xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
      return;
    }
  }

  partitioner.splitCurrArea( split, *tempCS );

#endif
  m_CurrCtx++;

  tempCS->getRecoBuf().fill( 0 );

  do
  {
    const auto &subCUArea  = partitioner.currArea();

    if( tempCS->picture->Y().contains( subCUArea.lumaPos() ) )
    {
      const unsigned wIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lwidth () );
      const unsigned hIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lheight() );

      CodingStructure *tempSubCS = m_pTempCS[wIdx][hIdx];
      CodingStructure *bestSubCS = m_pBestCS[wIdx][hIdx];

      tempCS->initSubStructure( *tempSubCS, partitioner.chType, subCUArea, false );
      tempCS->initSubStructure( *bestSubCS, partitioner.chType, subCUArea, false );

      xCompressCU( tempSubCS, bestSubCS, partitioner );

      if( bestSubCS->cost == MAX_DOUBLE )
      {
        CHECK( split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split" );
        tempCS->cost = MAX_DOUBLE;
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
        return;
      }

      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure( *bestSubCS, partitioner.chType, CS::getArea( *tempCS, subCUArea, partitioner.chType ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );

      if(currDepth < pps.getMaxCuDQPDepth())
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }

      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
    }
  } while( partitioner.nextPart( *tempCS ) );

  partitioner.exitCurrSplit();

  m_CurrCtx--;

  // Finally, generate split-signaling bits for RD-cost check
  const PartSplit implicitSplit = partitioner.getImplicitSplit( *tempCS );

  {
    bool enforceQT = implicitSplit == CU_QUAD_SPLIT;
#if HM_QTBT_REPRODUCE_FAST_LCTU_BUG

    // LARGE CTU bug
    if( m_pcEncCfg->getQTBT() && m_pcEncCfg->getUseFastLCTU() )
    {
      unsigned minDepth = 0;
      unsigned maxDepth = g_aucLog2[tempCS->sps->getSpsNext().getCTUSize()] - g_aucLog2[tempCS->sps->getSpsNext().getMinQTSize( slice.getSliceType(), partitioner.chType )];

      if( auto ad = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner ) )
      {
        ad->setMaxMinDepth( minDepth, maxDepth, *tempCS );
      }

      if( minDepth > partitioner.currQtDepth )
      {
        // enforce QT
        enforceQT = true;
      }
    }
#endif

    if( !enforceQT )
    {
      m_CABACEstimator->resetBits();

      if( partitioner.canSplit( CU_QUAD_SPLIT, *tempCS ) )
      {
        m_CABACEstimator->split_cu_flag( split == CU_QUAD_SPLIT, *tempCS, partitioner );
      }
#if JVET_K0554
      if( split != CU_QUAD_SPLIT )
#else
      if( split != CU_QUAD_SPLIT && implicitSplit == CU_DONT_SPLIT )
#endif
      {
        m_CABACEstimator->split_cu_mode_mt( split, *tempCS, partitioner );
      }

      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

  tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

  // Check Delta QP bits for splitted structure
  xCheckDQP( *tempCS, partitioner, true );

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
#if HEVC_TILES_WPP
    const TileMap& tileMap = *tempCS->picture->tileMap;
#endif
#if HEVC_TILES_WPP || HEVC_DEPENDENT_SLICES
    const uint32_t CtuAddr             = CU::getCtuAddr( *bestCS->getCU( partitioner.chType ) );
#endif
    const bool isEndOfSlice        =    slice.getSliceMode() == FIXED_NUMBER_OF_BYTES
                                      && ((slice.getSliceBits() + CS::getEstBits(*bestCS)) > slice.getSliceArgument() << 3)
#if HEVC_TILES_WPP
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceCurStartCtuTsAddr())
#endif
#if HEVC_DEPENDENT_SLICES
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceSegmentCurStartCtuTsAddr());
#else
                                      ;
#endif

#if HEVC_DEPENDENT_SLICES
    const bool isEndOfSliceSegment =    slice.getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES
                                      && ((slice.getSliceSegmentBits() + CS::getEstBits(*bestCS)) > slice.getSliceSegmentArgument() << 3)
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceSegmentCurStartCtuTsAddr());
                                          // Do not need to check slice condition for slice-segment since a slice-segment is a subset of a slice.
    if (isEndOfSlice || isEndOfSliceSegment)
#else
    if(isEndOfSlice)
#endif
    {
      bestCS->cost = MAX_DOUBLE;
    }
  }


  // RD check for sub partitioned coding structure.
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

  tempCS->releaseIntermediateData();

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}


void EncCu::xCheckRDCostIntra( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  double bestInterCost             = m_modeCtrl->getBestInterCost();
  double costSize2Nx2NemtFirstPass = m_modeCtrl->getEmtSize2Nx2NFirstPassCost();
  double costSizeNxNemtFirstPass   = MAX_DOUBLE;
  bool skipSecondEmtPass           = m_modeCtrl->getSkipSecondEMTPass();
  const SPS &sps                   = *tempCS->sps;
#endif
#if JEM_TOOLS
#if !JVET_K0220_ENC_CTRL
  auto slsCtrl                = dynamic_cast<SaveLoadEncInfoCtrl*>( m_modeCtrl );
#endif
#endif
  const PPS &pps              = *tempCS->pps;
  const CodingUnit *bestCU    = bestCS->getCU( partitioner.chType );
#if JEM_TOOLS
  const int nsstIdx           = ( encTestMode.opts & ETO_NSST ) >> ETO_NSST_SHIFT;
#endif
#if JEM_TOOLS
  const bool usePDPC          = ( encTestMode.opts & ETO_PDPC ) != 0;
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  const int maxSizeEMT        = pps.pcv->noRQT ? EMT_INTRA_MAX_CU_WITH_QTBT : EMT_INTRA_MAX_CU;
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
#if HM_EMT_NSST_AS_IN_JEM
  uint8_t considerEmtSecondPass = ( sps.getSpsNext().getUseIntraEMT() && isLuma( partitioner.chType ) && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ) ? 1 : 0;
#else
  uint8_t considerEmtSecondPass = ( sps.getSpsNext().getUseIntraEMT() && isLuma( partitioner.chType ) && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT && nsstIdx == 0 ) ? 1 : 0;
#endif
#endif
#if JEM_TOOLS
  CHECK( usePDPC && sps.getSpsNext().isPlanarPDPC(), "PDPC cannot be on with Planar-PDPC" );
#endif

  Distortion interHad = m_modeCtrl->getInterHad();



#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  for( uint8_t emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
#else
  for( uint8_t numPasses = 0; numPasses < 1; numPasses++ )
#endif
  {
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    //Possible early EMT tests interruptions
#if !JVET_K0220_ENC_CTRL
    //1) saveLoadTag code for EMT
    if( sps.getSpsNext().getUseQTBT() && slsCtrl && m_pcEncCfg->getUseSaveLoadEncInfo() )
    {
      if( m_pcEncCfg->getIntraEMT() && LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( tempCS->area ) && ( emtCuFlag > 0 ) != slsCtrl->getSaveLoadEmtCuFlag( tempCS->area ) )
      {
        continue;
      }
    }
#endif
    //2) Second EMT pass. This "if clause" is necessary because of the NSST and PDPC "for loops".
    if( emtCuFlag && skipSecondEmtPass )
    {
      continue;
    }
#endif
    //3) if interHad is 0, only try further modes if some intra mode was already better than inter
    if( m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && bestCU && CU::isInter( *bestCS->getCU( partitioner.chType ) ) && interHad == 0 )
    {
      continue;
    }

    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

    CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.skip             = false;
    cu.partSize         = encTestMode.partSize;
    cu.predMode         = MODE_INTRA;
    cu.transQuantBypass = encTestMode.lossless;
#if JEM_TOOLS
    cu.pdpc             = usePDPC;
#endif
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;
  //cu.ipcm             = false;
#if JEM_TOOLS
    cu.nsstIdx          = nsstIdx;
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    cu.emtFlag          = emtCuFlag;
#endif

    CU::addPUs( cu );

    tempCS->interHad    = interHad;

    if( isLuma( partitioner.chType ) )
    {
      m_pcIntraSearch->estIntraPredLumaQT( cu, partitioner );

#if DISTORTION_TYPE_BUGFIX
      if (m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == std::numeric_limits<Distortion>::max()
          && tempCS->interHad == 0)
#else
      if (m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == MAX_UINT && tempCS->interHad == 0)
#endif
      {
        interHad = 0;
        // JEM assumes only perfect reconstructions can from now on beat the inter mode
        m_modeCtrl->enforceInterHad( 0 );
        continue;
      }

      if( !CS::isDualITree( *tempCS ) )
      {
        cu.cs->picture->getRecoBuf( cu.Y() ).copyFrom( cu.cs->getRecoBuf( COMPONENT_Y ) );
      }
    }

    if( tempCS->area.chromaFormat != CHROMA_400 && ( partitioner.chType == CHANNEL_TYPE_CHROMA || !CS::isDualITree( *tempCS ) ) )
    {
      m_pcIntraSearch->estIntraPredChromaQT( cu, partitioner );
    }

    cu.rootCbf = false;

    for( uint32_t t = 0; t < getNumberValidTBlocks( *cu.cs->pcv ); t++ )
    {
      cu.rootCbf |= cu.firstTU->cbf[t] != 0;
    }

    // Get total bits for current mode: encode CU
    m_CABACEstimator->resetBits();

    if( pps.getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    if( !cu.cs->slice->isIntra() 
#if JVET_K0076_CPR_DT
      && cu.Y().valid()
#endif
      )
    {
      m_CABACEstimator->cu_skip_flag ( cu );
    }
    m_CABACEstimator->pred_mode      ( cu );
#if JEM_TOOLS
    m_CABACEstimator->pdpc_flag      ( cu );
#endif
    m_CABACEstimator->cu_pred_data   ( cu );
    m_CABACEstimator->pcm_data       ( cu );


    // Encode Coefficients
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual( cu, partitioner, cuCtx );

    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );

#if JEM_TOOLS
    // Check if secondary transform (NSST) is too expensive
    const int nonZeroCoeffThr = CS::isDualITree( *tempCS ) ? ( isLuma( partitioner.chType ) ? NSST_SIG_NZ_LUMA : NSST_SIG_NZ_CHROMA ) : NSST_SIG_NZ_LUMA + NSST_SIG_NZ_CHROMA;
    if( nsstIdx && tempCS->pcv->noRQT && cuCtx.numNonZeroCoeffNonTs <= nonZeroCoeffThr )
    {
      bool isMDIS = false;
      if( sps.getSpsNext().isPlanarPDPC() )
      {
        CHECK( CU::getNumPUs( cu ) > 1, "PLanarPDPC: encoder MDIS condition not defined for multi PU" );
        const PredictionUnit* pu = cu.firstPU;
        isMDIS = IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, *pu, true, *pu );
#if HM_MDIS_AS_IN_JEM
        if( pu->intraDir[0] == PLANAR_IDX ) { isMDIS |= IntraPrediction::getPlanarMDISCondition( *pu ); }
#endif
      }

      if( cuCtx.numNonZeroCoeffNonTs > 0 || isMDIS )
      {
        tempCS->cost = MAX_DOUBLE;
      }
    }
    if( nsstIdx && !tempCS->pcv->noRQT && cu.rootCbf == 0 )
    {
      tempCS->cost = MAX_DOUBLE;
    }
#endif

#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    // we save the cost of the modes for the first EMT pass
    if( !emtCuFlag ) static_cast< double& >( cu.partSize == SIZE_2Nx2N ? costSize2Nx2NemtFirstPass : costSizeNxNemtFirstPass ) = tempCS->cost;
#endif

#if WCG_EXT
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );


#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
#if JVET_K0220_ENC_CTRL
    if( !emtCuFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA && cu.partSize == SIZE_2Nx2N && m_pcEncCfg->getFastInterEMT() )
#else
    if( !emtCuFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA && cu.partSize == SIZE_2Nx2N && m_pcEncCfg->getFastInterEMT() && ( m_pcEncCfg->getUseSaveLoadEncInfo() ? ( bestInterCost < MAX_DOUBLE ) : true ) )
#endif
    {
      const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
      if( costSize2Nx2NemtFirstPass > thEmtInterFastSkipIntra * bestInterCost )
      {
        skipSecondEmtPass = true;
        m_modeCtrl->setSkipSecondEMTPass( true );
        break;
      }
    }

#endif
  } //for emtCuFlag
}

void EncCu::xCheckIntraPCM(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  cu.skip             = false;
  cu.partSize         = SIZE_2Nx2N;
  cu.predMode         = MODE_INTRA;
  cu.transQuantBypass = encTestMode.lossless;
#if JEM_TOOLS
  cu.pdpc             = false;
#endif
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
  cu.ipcm             = true;

  tempCS->addPU(tempCS->area, partitioner.chType);

  tempCS->addTU( tempCS->area, partitioner.chType );

  m_pcIntraSearch->IPCMSearch(*tempCS, partitioner);

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  m_CABACEstimator->resetBits();

  if( tempCS->pps->getTransquantBypassEnabledFlag() )
  {
    m_CABACEstimator->cu_transquant_bypass_flag( cu );
  }

  if( !cu.cs->slice->isIntra() 
#if JVET_K0076_CPR_DT
    && cu.Y().valid()
#endif
    )
  {
    m_CABACEstimator->cu_skip_flag ( cu );
  }
  m_CABACEstimator->pred_mode      ( cu );
  m_CABACEstimator->pcm_data       ( cu );


  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit( *tempCS, partitioner );

  xCheckDQP( *tempCS, partitioner );

#if WCG_EXT
  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
}

void EncCu::xCheckDQP( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx )
{
  CHECK( bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit( cs ) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case" );
  CHECK( !bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case" );

  if( !cs.pps->getUseDQP() )
  {
    return;
  }

  if( bKeepCtx && partitioner.currDepth != cs.pps->getMaxCuDQPDepth() )
  {
    return;
  }

  if( !bKeepCtx && partitioner.currDepth > cs.pps->getMaxCuDQPDepth() )
  {
    return;
  }

  CodingUnit* cuFirst = cs.getCU( partitioner.chType );

  CHECK( !cuFirst, "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
    if( cu->rootCbf )
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cuFirst, cs.prevQP[partitioner.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_CtxCache );
    if( !bKeepCtx ) ctxTemp = SubCtx( Ctx::DeltaQP, m_CABACEstimator->getCtx() );

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta( *cuFirst, predQP, cuFirst->qp );

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);


    if( !bKeepCtx ) m_CABACEstimator->getCtx() = SubCtx( Ctx::DeltaQP, ctxTemp );

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for( const auto &cu : cs.cus )
    {
      if( cu->rootCbf )
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for( const auto &cu : cs.cus )
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xFillPCMBuffer( CodingUnit &cu )
{
  const ChromaFormat format        = cu.chromaFormat;
  const uint32_t numberValidComponents = getNumberValidComponents(format);

  for( auto &tu : CU::traverseTUs( cu ) )
  {
    for( uint32_t ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID compID = ComponentID( ch );

      const CompArea &compArea = tu.blocks[ compID ];

      const CPelBuf source      = tu.cs->getOrgBuf( compArea );
             PelBuf destination = tu.getPcmbuf( compID );

      destination.copyFrom( source );
    }
  }
}


void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

#if JEM_TOOLS || JVET_K0346
  if( sps.getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mergeCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
#if JEM_TOOLS
    mergeCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
#endif
  }
#endif

  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.partSize = SIZE_2Nx2N;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx  = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;

    PU::getInterMergeCandidates(pu, mergeCtx);
  }


  bool candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (uint32_t ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip       = false;
  unsigned                                    uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
  PelUnitBuf                                  acMergeBuffer    [ MRG_MAX_NUM_CANDS ];
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList;
  bool                                        mrgTempBufSet    = false;

#if DMVR_JVET_LOW_LATENCY_K0217
  Mv                                          refinedMvdL0[MRG_MAX_NUM_CANDS];
#endif

  for( unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++ )
  {
    RdModeList.push_back( i );
  }

  if( m_pcEncCfg->getUseFastMerge() )
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
    bestIsSkip       = false;

    if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
    {
#if JVET_K0076_CPR
      if (slice.getSPS()->getSpsNext().getIBCMode())
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
      }
      else
#endif
      bestIsSkip = blkCache->isSkip( tempCS->area );
    }

    static_vector<double, MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if( !bestIsSkip )
    {
      RdModeList.clear();
      mrgTempBufSet       = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( encTestMode.lossless );

      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
      cu.skip             = false;
      cu.partSize         = SIZE_2Nx2N;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below
#if JEM_TOOLS
      cu.obmcFlag         = sps.getSpsNext().getUseOBMC();
#endif

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      const bool bUseHadamard= !encTestMode.lossless;
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );
#if JVET_K0076_CPR
      uint32_t uiIBCCand = 0;
      uint32_t uiNumValidMv = mergeCtx.numValidMergeCand;
#endif
      for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
      {
#if JVET_K0076_CPR
        if ((mergeCtx.interDirNeighbours[uiMergeCand] == 1 || mergeCtx.interDirNeighbours[uiMergeCand] == 3) && tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[uiMergeCand << 1].refIdx)->getPOC() == tempCS->slice->getPOC())
        {
          uiIBCCand++;
          uiNumValidMv--;
          continue;
        }
#endif
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf( localUnitArea );

        mergeCtx.setMergeInfo( pu, uiMergeCand );

        PU::spanMotionInfo( pu, mergeCtx );

        distParam.cur = acMergeBuffer[uiMergeCand].Y();

#if JEM_TOOLS
        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu,  acMergeBuffer[uiMergeCand] );
        pu.mvRefine = false;

        m_pcInterSearch->subBlockOBMC      ( pu, &acMergeBuffer[uiMergeCand], false );
#else
        m_pcInterSearch->motionCompensation( pu,  acMergeBuffer[uiMergeCand] );
#endif
        
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
        {
          mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
#if DMVR_JVET_LOW_LATENCY_K0217
          refinedMvdL0[uiMergeCand] = pu.mvd[0];
#endif
        }

#if DISTORTION_TYPE_BUGFIX
        Distortion uiSad = distParam.distFunc(distParam);
#else
        uint32_t uiSad = distParam.distFunc(distParam);
#endif
        uint32_t uiBitsCand = uiMergeCand + 1;
        if( uiMergeCand == tempCS->slice->getMaxNumMergeCand() - 1 )
        {
          uiBitsCand--;
        }
        double cost     = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;

        updateCandList( uiMergeCand, cost, RdModeList, candCostList, uiNumMrgSATDCand );
#if JVET_K0076_CPR
        CHECK(std::min(uiMergeCand + 1 - uiIBCCand, uiNumMrgSATDCand) != RdModeList.size(), "");
#else
        CHECK( std::min( uiMergeCand + 1, uiNumMrgSATDCand ) != RdModeList.size(), "" );
#endif
      }
#if JVET_K0076_CPR
      if (uiNumValidMv < uiNumMrgSATDCand)
        uiNumMrgSATDCand = uiNumValidMv;
#endif
      // Try to limit number of candidates using SATD-costs
      for( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }
#if JVET_K0352_MERGE_ENCOPT
    else
    {
      uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
    }
#endif
  }

  const uint32_t iteration = encTestMode.lossless ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for( uint32_t uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
  {
    for( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx];
#if JVET_K0076_CPR
      if ((mergeCtx.interDirNeighbours[uiMergeCand] == 1 || mergeCtx.interDirNeighbours[uiMergeCand] == 3) && tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[uiMergeCand << 1].refIdx)->getPOC() == tempCS->slice->getPOC())
      {
        continue;
      }
#endif
      if( ( (uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand] )
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
      cu.skip             = false;
      cu.partSize         = SIZE_2Nx2N;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
#if JEM_TOOLS
      cu.obmcFlag         = sps.getSpsNext().getUseOBMC();
#endif
      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      mergeCtx.setMergeInfo( pu, uiMergeCand );
      PU::spanMotionInfo( pu, mergeCtx );

      if( mrgTempBufSet )
      {
#if DMVR_JVET_LOW_LATENCY_K0217
        pu.mvd[0] = refinedMvdL0[uiMergeCand];
#endif
        tempCS->getPredBuf().copyFrom( acMergeBuffer[ uiMergeCand ]);
      }
      else
      {
#if JEM_TOOLS
        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu );
        pu.mvRefine = false;

        m_pcInterSearch->subBlockOBMC      ( pu );
#else
        m_pcInterSearch->motionCompensation( pu );
#endif
        
      }

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass
#if JEM_TOOLS || JVET_K0357_AMVR
        , NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
        , 1
#endif
        , uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL );

      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
      {
        bestIsSkip = bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }// end loop uiMrgHADIdx

    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
}

#if JEM_TOOLS || JVET_K_AFFINE
void EncCu::xCheckRDCostAffineMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;
  }

#if JVET_K_AFFINE_BUG_FIXES
  if ( bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8 )
#else
  if( tempCS->pcv->rectCUs && bestCS->area.lumaSize().area() < 64 )
#endif
  {
    return;
  }

  MvField       affineMvField[2][3];
  unsigned char interDirNeighbours;
  int           numValidMergeCand;
  bool          hasNoResidual = false;
#if JVET_K0248_GBI
  uint8_t       gbiIdx = GBI_DEFAULT;
#endif

#if JEM_TOOLS
  const SPS &sps       = *tempCS->sps;
#endif

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  cu.skip             = false;
  cu.partSize         = encTestMode.partSize;
  cu.affine           = true;
  cu.predMode         = MODE_INTER;
#if JEM_TOOLS
  cu.LICFlag          = false;
#endif
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
#if JEM_TOOLS
  cu.obmcFlag         = sps.getSpsNext().getUseOBMC();
#endif

  CU::addPUs( cu );

  cu.firstPU->mergeFlag = true;
  cu.firstPU->mergeIdx  = 0;
#if JVET_K0248_GBI
  PU::getAffineMergeCand( *cu.firstPU, affineMvField, interDirNeighbours, gbiIdx, numValidMergeCand );
#else  
  PU::getAffineMergeCand( *cu.firstPU, affineMvField, interDirNeighbours, numValidMergeCand );
#endif
  if( numValidMergeCand == -1 )
  {
    return;
  }

  cu.firstPU->interDir = interDirNeighbours;
  PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_0], REF_PIC_LIST_0 );
  PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_1], REF_PIC_LIST_1 );
#if JVET_K0248_GBI
  cu.GBiIdx = gbiIdx;
#endif

  PU::spanMotionInfo( *cu.firstPU );

  m_pcInterSearch->motionCompensation( cu );
#if JEM_TOOLS
  m_pcInterSearch->subBlockOBMC      ( cu );
#endif

  xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
#if JEM_TOOLS || JVET_K0357_AMVR
    , NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    , 1
#endif
    , &hasNoResidual);

  if( ! (encTestMode.lossless || hasNoResidual) )
  {
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    tempCS->copyStructure( *bestCS, partitioner.chType );
    tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );

    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 1
#if JEM_TOOLS || JVET_K0357_AMVR
      , NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
      , 1
#endif
      , &hasNoResidual);
  }
}
#endif
#if JVET_K0076_CPR
//////////////////////////////////////////////////////////////////////////////////////////////
// ibc merge/skip mode check
void EncCu::xCheckRDCostIntraBCMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  assert(tempCS->chType != CHANNEL_TYPE_CHROMA); // chroma IBC is derived

  if (tempCS->area.lwidth() > IBC_MAX_CAND_SIZE || tempCS->area.lheight() > IBC_MAX_CAND_SIZE) // currently only check 32x32 and below block for ibc merge/skip
  {
    return;
  }
  const SPS &sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  MergeCtx mergeCtx;

#if JEM_TOOLS
  if (sps.getSpsNext().getUseSubPuMvp())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
    mergeCtx.subPuMvpExtMiBuf = MotionBuf(m_SubPuExtMiBuf, bufSize);
  }
#endif
  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.partSize = SIZE_2Nx2N;
    cu.predMode = MODE_INTER;
    cu.ibc = true;
    cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    PU::getInterMergeCandidates(pu, mergeCtx);
  }

  int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (unsigned int ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

  bool                                        bestIsSkip = false;
  unsigned                                    uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList(MRG_MAX_NUM_CANDS);
  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    RdModeList[i] = i;
  }

  {
    static_vector<double, MRG_MAX_NUM_CANDS>  candCostList(MRG_MAX_NUM_CANDS, MAX_DOUBLE);

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    {
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(encTestMode.lossless);

      CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
      cu.skip = false;
      cu.partSize = SIZE_2Nx2N;
      cu.predMode = MODE_INTER;
      cu.ibc = true;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
#if JEM_TOOLS
      cu.obmcFlag = 0;
#endif
      DistParam distParam;
      const bool bUseHadamard = !encTestMode.lossless;
      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType); //tempCS->addPU(cu);
      Picture* refPic = pu.cu->slice->getPic();
      const CPelBuf refBuf = refPic->getRecoBuf(pu.blocks[COMPONENT_Y]);
      const Pel*        piRefSrch = refBuf.buf;
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      int iRefStride = refBuf.stride;
      const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
      int iNumValidBv = mergeCtx.numValidMergeCand;
      for (unsigned int uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++)
      {
        if (mergeCtx.interDirNeighbours[uiMergeCand] != 1)
        {
          iNumValidBv--;
          continue;
        }
        if (tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[uiMergeCand << 1].refIdx)->getPOC() != tempCS->slice->getPOC())
        {
          iNumValidBv--;
          continue;
        }
        mergeCtx.setMergeInfo(pu, uiMergeCand); // set bv info in merge mode
        const int cuPelX = pu.Y().x;
        const int cuPelY = pu.Y().y;
        int iRoiWidth = pu.lwidth();
        int iRoiHeight = pu.lheight();
        const int iPicWidth = pu.cs->slice->getSPS()->getPicWidthInLumaSamples();
        const int iPicHeight = pu.cs->slice->getSPS()->getPicHeightInLumaSamples();
        const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
        int xPred = pu.bv.getHor();
        int yPred = pu.bv.getVer();

        if (!PU::isBlockVectorValid(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, 0, 0, xPred, yPred, lcuWidth)) // not valid bv derived
        {
          iNumValidBv--;
          continue;
        }
        PU::spanMotionInfo(pu, mergeCtx);

        distParam.cur.buf = piRefSrch + iRefStride * yPred + xPred;

        Distortion uiSad = distParam.distFunc(distParam);
        unsigned int uiBitsCand = uiMergeCand + 1;
        if (uiMergeCand == tempCS->slice->getMaxNumMergeCand() - 1)
        {
          uiBitsCand--;
        }
        double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;

        updateCandList(uiMergeCand, cost, RdModeList, candCostList, uiNumMrgSATDCand);
      }

      // Try to limit number of candidates using SATD-costs
      if (iNumValidBv)
      {
        uiNumMrgSATDCand = iNumValidBv;
        for (unsigned int i = 1; i < iNumValidBv; i++)
        {
          if (candCostList[i] > MRG_FAST_RATIO*candCostList[0])
          {
            uiNumMrgSATDCand = i;
            break;
          }
        }
      }
      else
      {
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        return;
      }

      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    }
  }


  const unsigned int iteration = encTestMode.lossless ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for (unsigned int uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++)
  {
    for (unsigned int uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++)
    {
      unsigned int uiMergeCand = RdModeList[uiMrgHADIdx];
      if (mergeCtx.interDirNeighbours[uiMergeCand] != 1)
      {
        continue;
      }
      if (tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[uiMergeCand << 1].refIdx)->getPOC() != tempCS->slice->getPOC())
      {
        continue;
      }
      if (!(uiNoResidualPass == 1 && candHasNoResidual[uiMergeCand] == 1))
      {
        if (!(bestIsSkip && (uiNoResidualPass == 0)))
        {
#if JEM_TOOLS
          unsigned char considerEmtSecondPass = 0;
          bool skipSecondEmtPass = true;
          bool hasResidual[2] = { false, false };
          double emtCost[2] = { MAX_DOUBLE, MAX_DOUBLE };

          // CU-level optimization
          for (unsigned char emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++)
          {
            if (m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass)
            {
              continue;
            }

#endif
            // first get merge candidates
            CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
            cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
            cu.skip = false;
            cu.partSize = SIZE_2Nx2N;
            cu.predMode = MODE_INTER;
            cu.ibc = true;
            cu.transQuantBypass = encTestMode.lossless;
            cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
            cu.qp = encTestMode.qp;
#if JEM_TOOLS
            cu.emtFlag = false;
            cu.obmcFlag = false;
#endif
            PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);// tempCS->addPU(cu);
            pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
            pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block

            mergeCtx.setMergeInfo(pu, uiMergeCand);
            PU::spanMotionInfo(pu, mergeCtx);

            assert(mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_IBC); //  should be IBC candidate at this round
#if JVET_K0076_CPR_DT
            const bool chroma = !(CS::isDualITree(*tempCS));
#endif
            //  MC
            m_pcInterSearch->motionCompensation(pu
#if JVET_K0076_CPR_DT
              , REF_PIC_LIST_0
              , true, chroma
#endif
            );
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (uiNoResidualPass != 0)
#if JVET_K0076_CPR_DT
              , true, chroma
#endif
            );
            xEncodeDontSplit(*tempCS, partitioner);

            if (tempCS->pps->getUseDQP() && (partitioner.currDepth) <= tempCS->pps->getMaxCuDQPDepth())
            {
              xCheckDQP(*tempCS, partitioner);
            }

#if JEM_TOOLS
            hasResidual[emtCuFlag] = cu.rootCbf;
            emtCost[emtCuFlag] = tempCS->cost;
#endif
            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

            tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
#if JEM_TOOLS
          }
          if (uiNoResidualPass == 0 && (emtCost[0] <= emtCost[1] ? !hasResidual[0] : !hasResidual[1]))
#else
            if (uiNoResidualPass == 0 && !cu.rootCbf)
#endif
            {
              // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
              candHasNoResidual[uiMergeCand] = 1;
            }

            if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
            {
              bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
            }
        }
      }
    }
  }

}

void EncCu::xCheckRDCostIntraBC(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
  cu.skip = false;
  cu.partSize = encTestMode.partSize;
  cu.predMode = MODE_INTER;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.ibc = true;
#if JEM_TOOLS
  cu.imv = 0;
#endif
  CU::addPUs(cu);

  PredictionUnit& pu = *cu.firstPU;
  pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
  pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block

  pu.interDir = 1; // use list 0 for IBC mode
  pu.refIdx[REF_PIC_LIST_0] = pu.cs->slice->getNumRefIdx(REF_PIC_LIST_0) - 1; // last idx in the list

#if JVET_K0076_CPR_DT
  if (partitioner.chType == CHANNEL_TYPE_LUMA)
#endif
  {
    bool bValid = m_pcInterSearch->predIntraBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap);

    if (bValid)
    {
      PU::spanMotionInfo(pu);
#if JVET_K0076_CPR_DT
      const bool chroma = !(CS::isDualITree(*tempCS));
#endif
      //  MC
      m_pcInterSearch->motionCompensation(pu
#if JVET_K0076_CPR_DT
        , REF_PIC_LIST_0
        , true, chroma
#endif
      );

#if JEM_TOOLS
      double    bestCost = bestCS->cost;
      unsigned char    considerEmtSecondPass = 0;
      bool      skipSecondEmtPass = true;
      double    emtFirstPassCost = MAX_DOUBLE;

      // CU-level optimization

      for (unsigned char emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++)
      {
        if (m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass)
        {
          continue;
        }

        tempCS->getCU(tempCS->chType)->emtFlag = emtCuFlag;
#endif
        m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false
#if JVET_K0076_CPR_DT
          , true, chroma
#endif
        );
#if JEM_TOOLS
        if (m_pcEncCfg->getFastInterEMT())
        {
          emtFirstPassCost = (!emtCuFlag) ? tempCS->cost : emtFirstPassCost;
        }
#endif
        xEncodeDontSplit(*tempCS, partitioner);

        if (tempCS->pps->getUseDQP() && (partitioner.currDepth) <= tempCS->pps->getMaxCuDQPDepth())
        {
          xCheckDQP(*tempCS, partitioner);
        }

        DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
        xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
#if JEM_TOOLS
        //now we check whether the second pass should be skipped or not
        if (!emtCuFlag && considerEmtSecondPass)
        {
          static const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
          if (m_pcEncCfg->getFastInterEMT() && (!cu.firstTU->cbf[COMPONENT_Y] || emtFirstPassCost > bestCost * thresholdToSkipEmtSecondPass))
          {
            skipSecondEmtPass = true;
          }
          else //EMT will be checked
          {
            if (bestCost == bestCS->cost) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
            {
              tempCS->clearTUs();
            }
            else
            {
              tempCS->initStructData(bestCS->currQP[bestCS->chType], bestCS->isLossless);

              tempCS->copyStructure(*bestCS, partitioner.chType);
              tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
            }

            //we need to restart the distortion for the new tempCS, the bit count and the cost
            tempCS->dist = 0;
            tempCS->fracBits = 0;
            tempCS->cost = MAX_DOUBLE;
          }
        }
      }
#endif
    } // bValid
    else
    {
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
    }
  }
#if JVET_K0076_CPR_DT // chroma CU ibc comp
  else
  {
    bool success = true;
    // chroma tree, reuse luma bv at minimal block level
    // enabled search only when each chroma sub-block has a BV from its luma sub-block
    assert(tempCS->getIbcLumaCoverage(pu.Cb()) == IBC_LUMA_COVERAGE_FULL);
    // check if each BV for the chroma sub-block is valid
    //static const UInt unitArea = MIN_PU_SIZE * MIN_PU_SIZE;
    const CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
    PredictionUnit subPu;
    subPu.cs = pu.cs;
    subPu.cu = pu.cu;
    const ComponentID compID = COMPONENT_Cb; // use Cb to represent both Cb and CR, as their structures are the same
    int shiftHor = ::getComponentScaleX(compID, pu.chromaFormat);
    int shiftVer = ::getComponentScaleY(compID, pu.chromaFormat);
    //const ChromaFormat  chFmt = pu.chromaFormat;

    for (int y = lumaArea.y; y < lumaArea.y + lumaArea.height; y += MIN_PU_SIZE)
    {
      for (int x = lumaArea.x; x < lumaArea.x + lumaArea.width; x += MIN_PU_SIZE)
      {
        const MotionInfo &curMi = pu.cs->picture->cs->getMotionInfo(Position{ x, y });
        int interpolationSamplesX = (pu.chromaFormat == CHROMA_422 || pu.chromaFormat == CHROMA_420) ? ((curMi.bv.getHor() & 0x1) << 1) : 0;
        int interpolationSamplesY = (pu.chromaFormat == CHROMA_420) ? ((curMi.bv.getVer() & 0x1) << 1) : 0;

        subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, MIN_PU_SIZE, MIN_PU_SIZE)));
        Position offset = subPu.blocks[compID].pos().offset(((curMi.bv.getHor() - interpolationSamplesX) >> shiftHor), ((curMi.bv.getVer() - interpolationSamplesY) >> shiftVer));
        Position refEndPos(offset.x + subPu.blocks[compID].size().width - 1 + (interpolationSamplesX > 0 ? 3 : 0), offset.y + subPu.blocks[compID].size().height - 1 + (interpolationSamplesY > 0 ? 3 : 0));

        if (!subPu.cs->isDecomp(refEndPos, toChannelType(compID)) || !subPu.cs->isDecomp(offset, toChannelType(compID))) // ref block is not yet available for this chroma sub-block
        {
          success = false;
          break;
        }
      }
      if (!success)
        break;
    }
    ////////////////////////////////////////////////////////////////////////////

    if (success)
    {
      //pu.mergeType = MRG_TYPE_IBC;
      m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, false, true); // luma=0, chroma=1
      m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, false, true);

      xEncodeDontSplit(*tempCS, partitioner);

      xCheckDQP(*tempCS, partitioner);

      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());

      xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    }
    else
    {
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
    }
  }
#endif
}
// check ibc mode in encoder RD
//////////////////////////////////////////////////////////////////////////////////////////////
#endif // CPR
void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

#if JVET_K0248_GBI
  m_pcInterSearch->resetBufferedUniMotions();
  int gbiLoopNum = ( tempCS->slice->isInterB() ? GBI_NUM : 1 );
  gbiLoopNum = ( tempCS->sps->getSpsNext().getUseGBi() ? gbiLoopNum : 1 );

  double curBestCost = bestCS->cost;
  double equGBiCost = MAX_DOUBLE;

  for( int gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
  {
    if( m_pcEncCfg->getUseGBiFast())
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestGBiIdx = blkCache->getGbiIdx(bestCS->area);

        if( isBestInter && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT && g_GbiSearchOrder[gbiLoopIdx] != bestGBiIdx )
        {
          continue;
        }
      }
    }

    if( !tempCS->slice->getCheckLDC() )
    {
      if(gbiLoopIdx != 0 && gbiLoopIdx != 3 && gbiLoopIdx != 4 )
      {
        continue;
      }
    }
#endif

#if JEM_TOOLS
  const SPS &sps      = *tempCS->sps;
#endif
  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  cu.skip             = false;
  cu.partSize         = encTestMode.partSize;
//cu.affine
  cu.predMode         = MODE_INTER;
#if JEM_TOOLS
  cu.LICFlag          = ( ( encTestMode.opts & ETO_LIC ) != 0 );
#endif
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
#if JEM_TOOLS
  cu.obmcFlag         = sps.getSpsNext().getUseOBMC();
#endif
  CU::addPUs( cu );

#if JVET_K0248_GBI
  cu.GBiIdx = g_GbiSearchOrder[gbiLoopIdx];
  uint8_t gbiIdx = cu.GBiIdx;
  bool  testGbi = ( gbiIdx != GBI_DEFAULT );
#endif

  m_pcInterSearch->predInterSearch( cu, partitioner );

#if JEM_TOOLS || JVET_K0357_AMVR
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
#endif

#if JVET_K0248_GBI
  gbiIdx = CU::getValidGbiIdx( cu );
  if(testGbi && gbiIdx == GBI_DEFAULT ) // Enabled GBi but the search results is uni.
  {
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    continue; 
  }
  CHECK( !(testGbi || (!testGbi && gbiIdx == GBI_DEFAULT) ), " !( bTestGbi || (!bTestGbi && gbiIdx == GBI_DEFAULT ) )" );

  bool isEqualUni = false;
  if( m_pcEncCfg->getUseGBiFast())
  {
    if( cu.firstPU->interDir != 3 && testGbi == 0 )
    {
      isEqualUni = true;
    }
  }
#endif

#if JEM_TOOLS
  if( m_pTempCUWoOBMC )
  {
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );
    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure( *tempCS, partitioner.chType );

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom( tempCS->getPredBuf( cu ) );

    m_pcInterSearch->subBlockOBMC( cu );

    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf( cu ).copyFrom( tempCS->getPredBuf( cu ) );
  }
#endif
  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
#if JEM_TOOLS || JVET_K0357_AMVR
    , m_pImvTempCS ? m_pImvTempCS[wIdx][encTestMode.partSize] : NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    , 1
#endif
    , 0
#if JVET_K0248_GBI
    , &equGBiCost
#endif
  );
#if JVET_K0248_GBI
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  double skipTH = MAX_DOUBLE;
  skipTH = (m_pcEncCfg->getUseGBiFast() ? 1.05 : MAX_DOUBLE );
  if( equGBiCost > curBestCost * skipTH )
  {
    break;
  }

  if( m_pcEncCfg->getUseGBiFast())
  {
    if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
    {
      break;
    }
  }
 }  // for( UChar gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
#endif

}


#if JEM_TOOLS
void EncCu::xCheckRDCostInterWoOBMC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( !tempCS->sps->getSpsNext().getUseOBMC() )
  {
    return;
  }

  if( m_modeCtrl->getFastDeltaQp() )
  {
    if( encTestMode.partSize != SIZE_2Nx2N || tempCS->area.lumaSize().width > tempCS->pcv->fastDeltaQPCuMaxSize )
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  const SPS &sps = *tempCS->sps;
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

  CodingStructure* CSWoOBMC = m_pTempCUWoOBMC[wIdx][hIdx];
  CodingUnit *cu = CSWoOBMC->getCU( partitioner.chType );

  if( !cu->obmcFlag || !CU::isObmcFlagCoded( *cu ) )
  {
    return;
  }

#if DISTORTION_TYPE_BUGFIX
  const Distortion uiSADOBMCOff =
    m_pcRdCost->getDistPart(tempCS->getOrgBuf(cu->Y()), m_pPredBufWoOBMC[wIdx][hIdx].Y(),
                            sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD_FULL_NBIT);
  const Distortion uiSADOBMCOn =
    m_pcRdCost->getDistPart(tempCS->getOrgBuf(cu->Y()), CSWoOBMC->getPredBuf(cu->Y()),
                            sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD_FULL_NBIT);
#else
  const uint32_t uiSADOBMCOff = m_pcRdCost->getDistPart(tempCS->getOrgBuf(cu->Y()), m_pPredBufWoOBMC[wIdx][hIdx].Y(),
                                                    sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD_FULL_NBIT);
  const uint32_t uiSADOBMCOn  = m_pcRdCost->getDistPart(tempCS->getOrgBuf(cu->Y()), CSWoOBMC->getPredBuf(cu->Y()),
                                                   sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD_FULL_NBIT);
#endif
  const double    dOBMCThOff = 1.0;
  const bool   bCheckOBMCOff = uiSADOBMCOff * dOBMCThOff < uiSADOBMCOn;

  if( !bCheckOBMCOff )
  {
    return;
  }

  tempCS->copyStructure( *CSWoOBMC,partitioner.chType );
  tempCS->getPredBuf( *cu ).copyFrom( m_pPredBufWoOBMC[wIdx][hIdx] );
  cu           = tempCS->getCU( partitioner.chType );
  cu->obmcFlag = false;
  CHECK( cu->firstPU->mergeFlag && cu->partSize == SIZE_2Nx2N, "Merge2Nx2Ns is on" );

  xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
#if JEM_TOOLS || JVET_K0357_AMVR
    , m_pImvTempCS ? m_pImvTempCS[wIdx][encTestMode.partSize] : NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    , !m_pcEncCfg->getFastInterEMT()
#endif
  );
}
#endif

#if JEM_TOOLS
void EncCu::xCheckRDCostMerge2Nx2NFRUC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;
#if !JVET_K0220_ENC_CTRL
  auto slsCtrl = dynamic_cast< SaveLoadEncInfoCtrl* >( m_modeCtrl );
#endif

  if( !slice.getSPS()->getSpsNext().getUseFRUCMrgMode() )
  {
    return;
  }

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;   // never check merge in fast delta-qp mode
  }

  bool transQuantBypass = tempCS->isLossless;

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;
  Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
  mergeCtx.subPuFrucMiBuf = MotionBuf( m_SubPuFrucBuf, bufSize );

  const uint8_t uhFRUCME[2] = { FRUC_MERGE_BILATERALMV, FRUC_MERGE_TEMPLATE };

  PelUnitBuf acMergeBuffer[2];

  for( int nME = 0; nME < 2; nME++ )
  {
#if !JVET_K0220_ENC_CTRL
    if( slsCtrl && m_pcEncCfg->getUseSaveLoadEncInfo() && LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( tempCS->area ) && uhFRUCME[nME] != slsCtrl->getSaveLoadFrucMode( tempCS->area ) )
    {
      continue;
    }

#endif
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

    bool bAvailable        = false;

    CodingUnit &cu         = tempCS->addCU( tempCS->area, partitioner.chType );

    partitioner.setCUData( cu );
    cu.slice               = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx             = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  //cu.skip                = false;
    cu.partSize            = SIZE_2Nx2N;
  //cu.affine
    cu.predMode            = MODE_INTER;
    cu.LICFlag             = ( ( encTestMode.opts & ETO_LIC ) != 0 );
    cu.obmcFlag            = sps.getSpsNext().getUseOBMC();

    PredictionUnit &puFruc = tempCS->addPU( cu, partitioner.chType );
    puFruc.mergeFlag       = true;
    puFruc.mergeIdx        = 0;
    puFruc.frucMrgMode     = uhFRUCME[nME];
    puFruc.mergeType       = MRG_TYPE_FRUC;

    bAvailable = m_pcInterSearch->deriveFRUCMV( puFruc );

    if( bAvailable )
    {
      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.lwidth(), tempCS->area.lheight() ) );
      acMergeBuffer[nME] = m_acMergeBuffer[nME].getBuf( localUnitArea );

      m_pcInterSearch->motionCompensation( puFruc,  acMergeBuffer[nME] );
      m_pcInterSearch->subBlockOBMC      ( puFruc, &acMergeBuffer[nME], false );

      CHECK( mergeCtx.subPuFrucMiBuf.area() == 0 || !mergeCtx.subPuFrucMiBuf.buf, "Buffer not initialized" );
      mergeCtx.subPuFrucMiBuf.fill( MotionInfo() );
      mergeCtx.subPuFrucMiBuf.copyFrom( puFruc.getMotionBuf() );

      uint32_t iteration = encTestMode.lossless ? 1 : 2;
      bool candHasNoResidual = false;
      // 2. Pass: check candidates using full RD test
      for( uint32_t uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
      {
        if( (uiNoResidualPass != 0) && candHasNoResidual )
        {
          continue;
        }

        // first get merge candidates
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

        partitioner.setCUData( cu );
        cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
        cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
        cu.skip             = false;
        cu.partSize         = SIZE_2Nx2N;
      //cu.affine
        cu.predMode         = MODE_INTER;
        cu.LICFlag          = ( ( encTestMode.opts & ETO_LIC ) != 0 );
        cu.transQuantBypass = transQuantBypass;
        cu.chromaQpAdj      = transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
        cu.qp               = encTestMode.qp;
        cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

        PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
        pu.mergeFlag        = true;
        pu.mergeIdx         = 0;
        pu.frucMrgMode      = uhFRUCME[nME];

        pu.mergeType        = MRG_TYPE_FRUC_SET;
        PU::spanMotionInfo( pu, mergeCtx );
        pu.mergeType        = MRG_TYPE_FRUC;

        tempCS->getPredBuf().copyFrom( acMergeBuffer[ nME ]);

        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass
#if JEM_TOOLS || JVET_K0357_AMVR
          , NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
          , 1
#endif
          , uiNoResidualPass == 0 ? &candHasNoResidual : NULL );
      } // end loop uiNoResidualPass
    }
  }
}
#endif


#if JVET_K0357_AMVR
bool EncCu::xCheckRDCostInterIMV( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  int iIMV = int( ( encTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT );
  // Only int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK( iIMV != 1 && iIMV != 2 && iIMV != 3, "Unsupported IMV Mode" );
  // Fast 4-Pel Mode

  EncTestMode encTestModeBase = encTestMode;                                        // copy for clearing non-IMV options
  encTestModeBase.opts        = EncTestModeOpts( encTestModeBase.opts & ETO_IMV );  // clear non-IMV options (is that intended?)

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingStructure* pcCUInfo2Reuse = nullptr;
#if JEM_TOOLS
  const SPS &sps                  = *tempCS->sps;
#endif

  if( m_pImvTempCS && encTestMode.partSize != SIZE_2Nx2N && ( ( encTestMode.opts & ETO_FORCE_MERGE ) == 0 ) )
  {
    pcCUInfo2Reuse = m_pImvTempCS[gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth() )][encTestMode.partSize];

    if( pcCUInfo2Reuse != nullptr )
    {
      CHECK( tempCS->area != pcCUInfo2Reuse->area, " mismatch" );
      tempCS->copyStructure( *pcCUInfo2Reuse, partitioner.chType );
      tempCS->fracBits = 0;
      tempCS->dist     = 0;
      tempCS->cost     = MAX_DOUBLE;
    }
  }

#if JVET_K0248_GBI
  m_pcInterSearch->resetBufferedUniMotions();
  int gbiLoopNum = ( tempCS->slice->isInterB() ? GBI_NUM : 1 );
  gbiLoopNum = ( pcCUInfo2Reuse != NULL ? 1 : gbiLoopNum);
  gbiLoopNum = ( tempCS->slice->getSPS()->getSpsNext().getUseGBi() ? gbiLoopNum : 1 );

  double curBestCost = bestCS->cost; 
  double equGBiCost = MAX_DOUBLE;

  for( int gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
  {
    if( m_pcEncCfg->getUseGBiFast())
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if (blkCache)
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestGBiIdx = blkCache->getGbiIdx(bestCS->area);

        if (isBestInter && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT && g_GbiSearchOrder[gbiLoopIdx] != bestGBiIdx)
        {
          continue;
        }
      }
    }

    if( !tempCS->slice->getCheckLDC() )
    {
      if (gbiLoopIdx != 0 && gbiLoopIdx != 3 && gbiLoopIdx != 4)
      {
        continue;
      }
    }
#endif

  CodingUnit &cu = ( pcCUInfo2Reuse != nullptr ) ? *tempCS->getCU( partitioner.chType ) : tempCS->addCU( tempCS->area, partitioner.chType );

  if( pcCUInfo2Reuse == nullptr )
  {
    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.skip             = false;
    cu.partSize         = encTestMode.partSize;
  //cu.affine
    cu.predMode         = MODE_INTER;
#if JEM_TOOLS
    cu.LICFlag          = ( ( encTestMode.opts & ETO_FORCE_MERGE ) ? false : ( ( encTestMode.opts & ETO_LIC ) != 0 ) );
#endif
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;

    CU::addPUs( cu );
  }
  else
  {
    CHECK( cu.partSize != encTestMode.partSize,    "Mismatch" );
    CHECK( cu.skip,                                "Mismatch" );
    CHECK( cu.qtDepth  != partitioner.currQtDepth, "Mismatch" );
    CHECK( cu.btDepth  != partitioner.currBtDepth, "Mismatch" );
    CHECK( cu.mtDepth  != partitioner.currMtDepth, "Mismatch" );
    CHECK( cu.depth    != partitioner.currDepth,   "Mismatch" );
  }

  cu.imv      = iIMV > 1 ? 2 : 1;
#if JVET_K1000_SIMPLIFIED_EMT
  cu.emtFlag  = false;
#endif
#if JEM_TOOLS
  cu.obmcFlag = sps.getSpsNext().getUseOBMC();
#endif

#if JVET_K0248_GBI
  bool testGbi;
  uint8_t gbiIdx;
#endif
  
  if( pcCUInfo2Reuse != nullptr )
  {
    // reuse the motion info from pcCUInfo2Reuse
    CU::resetMVDandMV2Int( cu, m_pcInterSearch );

#if JVET_K0248_GBI
    CHECK( cu.GBiIdx < 0 || cu.GBiIdx >= GBI_NUM, "cu.GBiIdx < 0 || cu.GBiIdx >= GBI_NUM" );
    gbiIdx = CU::getValidGbiIdx( cu );
    testGbi = (gbiIdx != GBI_DEFAULT );
#endif

    if( !CU::hasSubCUNonZeroMVd( cu ) )
    {
      m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
      return false;
    }
    else
    {
      m_pcInterSearch->motionCompensation( cu );
    }
  }
  else
  {
#if JVET_K0248_GBI 
    cu.GBiIdx = g_GbiSearchOrder[gbiLoopIdx];
    gbiIdx = cu.GBiIdx;
    testGbi = (gbiIdx != GBI_DEFAULT );
#endif

    m_pcInterSearch->predInterSearch( cu, partitioner );

#if JVET_K0248_GBI
    gbiIdx = CU::getValidGbiIdx( cu );
#endif
  }

#if JVET_K0248_GBI
  if(testGbi && gbiIdx == GBI_DEFAULT ) // Enabled GBi but the search results is uni.
  {
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    continue; 
  }
  CHECK( !(testGbi || ( !testGbi && gbiIdx == GBI_DEFAULT )), " !( bTestGbi || (!bTestGbi && gbiIdx == GBI_DEFAULT ) )" );

  bool isEqualUni = false;
  if( m_pcEncCfg->getUseGBiFast())
  {
    if(cu.firstPU->interDir != 3 && testGbi == 0)
    {
      isEqualUni = true;
    }
  }
#endif

  if( !CU::hasSubCUNonZeroMVd( cu ) )
  {
    m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
    return false;
  }
#if JEM_TOOLS
  if( m_pTempCUWoOBMC )
  {
    const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure( *tempCS, partitioner.chType );

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom( tempCS->getPredBuf( cu ) );

    m_pcInterSearch->subBlockOBMC( cu );

    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf( cu ).copyFrom( tempCS->getPredBuf( cu ) );
  }
#endif

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestModeBase, 0
#if JEM_TOOLS || JVET_K0357_AMVR
    , NULL
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    , true
#endif
    , 0
#if JVET_K0248_GBI
    , &equGBiCost
#endif
  );

#if JVET_K0248_GBI
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  double skipTH = MAX_DOUBLE;
  skipTH = (m_pcEncCfg->getUseGBiFast() ? 1.05 : MAX_DOUBLE );
  if( equGBiCost > curBestCost * skipTH ) 
  {
    break;
  }

  if( m_pcEncCfg->getUseGBiFast())
  {
    if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
    {
      break;
    }
  }
 } // for( UChar gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )

#endif
  return true;
}
#endif

void EncCu::xEncodeInterResidual( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, int residualPass
#if JEM_TOOLS || JVET_K0357_AMVR
  , CodingStructure* imvCS
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  , int emtMode
#endif
  , bool* bestHasNonResi
#if JVET_K0248_GBI
  , double* equGBiCost
#endif
  )
{
  if( residualPass == 1 && encTestMode.lossless )
  {
    return;
  }

  CodingUnit*            cu        = tempCS->getCU( partitioner.chType );
  double   bestCostInternal        = MAX_DOUBLE;
#if JEM_TOOLS
  const bool bMergeFruc            = (ETM_MERGE_SKIP == encTestMode.type ) || (ETM_MERGE_FRUC == encTestMode.type);
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  double           bestCost        = bestCS->cost;
  const SPS&            sps        = *tempCS->sps;
  const int      maxSizeEMT        = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
  bool              swapped        = false; // avoid unwanted data copy
  bool             reloadCU        = false;
  const bool considerEmtSecondPass = emtMode && sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT;

  int minEMTMode = 0;
  int maxEMTMode = (considerEmtSecondPass?1:0);

  if( emtMode == 2 )
  {
    minEMTMode = maxEMTMode = (cu->emtFlag?1:0);
  }

  for( int curEmtMode = minEMTMode; curEmtMode <= maxEMTMode; curEmtMode++ )
#endif
  {
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    if( reloadCU )
    {
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
    }

    reloadCU    = true; // enable cu reloading
#endif
    cu->skip    = false;
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    cu->emtFlag = curEmtMode;
#endif

    const bool skipResidual = residualPass == 1;
    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );

    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );

#if JVET_K0248_GBI
    if( ETM_INTER_ME == encTestMode.type )
    {
      if( equGBiCost != NULL )
      {
        if( tempCS->cost < ( *equGBiCost ) && cu->GBiIdx == GBI_DEFAULT )
        {
          ( *equGBiCost ) = tempCS->cost;
        }
      }
      else
      {
        CHECK( equGBiCost == NULL, "equGBiCost == NULL" );
      }
    }
#endif

#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    double emtFirstPassCost = tempCS->cost;
#endif
#if JEM_TOOLS
    double emtFirstPassCostFruc = tempCS->cost;
#endif
#if JVET_K0357_AMVR
    if( imvCS && (tempCS->cost < imvCS->cost) )
    {
      if( imvCS->cost != MAX_DOUBLE )
      {
        imvCS->initStructData( encTestMode.qp, encTestMode.lossless );
      }
      imvCS->copyStructure( *tempCS, partitioner.chType );
    }
#endif
    if( NULL != bestHasNonResi && (bestCostInternal > tempCS->cost) )
    {
      bestCostInternal = tempCS->cost;
      *bestHasNonResi  = !cu->rootCbf;
    }

#if WCG_EXT
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    //now we check whether the second pass should be skipped or not
    if( !curEmtMode && maxEMTMode )
    {
      const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
      const bool bCond1 = !cu->firstTU->cbf[COMPONENT_Y];

#if JEM_TOOLS
      const bool bCond2 = bMergeFruc && (emtFirstPassCostFruc > (bestCost * thresholdToSkipEmtSecondPass));
      const bool bCond3 = !bMergeFruc && (emtFirstPassCost > (bestCost * thresholdToSkipEmtSecondPass));

      if( m_pcEncCfg->getFastInterEMT() && (bCond1 || bCond2 || bCond3 ) )
#else
      const bool bCond3 = emtFirstPassCost > ( bestCost * thresholdToSkipEmtSecondPass );

      if( m_pcEncCfg->getFastInterEMT() && (bCond1 || bCond3 ) ) 
#endif
      {
        maxEMTMode = 0; // do not test EMT
      }
    }
#endif
  } //end emt loop
}


void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  {
    if( partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_flag( false, cs, partitioner );
    }
    if( partitioner.canSplit( CU_MT_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_mode_mt( CU_DONT_SPLIT, cs, partitioner );
    }
  }

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );

}

#if REUSE_CU_RESULTS
void EncCu::xReuseCachedResult( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const SPS &sps = *tempCS->sps;

  BestEncInfoCache* bestEncCache = dynamic_cast<BestEncInfoCache*>( m_modeCtrl );
  CHECK( !bestEncCache, "If this mode is chosen, mode controller has to implement the mode caching capabilities" );
  EncTestMode cachedMode;

  if( bestEncCache->setCsFrom( *tempCS, cachedMode, partitioner ) )
  {
    CodingUnit& cu = *tempCS->cus.front();
    partitioner.setCUData( cu );

    if( CU::isIntra( cu ) )
    {
      xReconIntraQT( cu );
    }
    else
    {
      xDeriveCUMV( cu );
      xReconInter( cu );
    }

    Distortion finalDistortion = 0;
    const int  numValidComponents = getNumberValidComponents( tempCS->area.chromaFormat );

    for( int comp = 0; comp < numValidComponents; comp++ )
    {
      const ComponentID compID = ComponentID( comp );

      if( CS::isDualITree( *tempCS ) && toChannelType( compID ) != partitioner.chType )
      {
        continue;
      }

      CPelBuf reco = tempCS->getRecoBuf( compID );
      CPelBuf org  = tempCS->getOrgBuf ( compID );

#if WCG_EXT
      if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
      {
#if WCG_EXT_BUGFIX
        const CPelBuf orgLuma = tempCS->getOrgBuf(tempCS->area.blocks[COMPONENT_Y]);
#else
        const CPelBuf orgLuma = tempCS->getOrgBuf(cs.area.blocks[COMPONENT_Y]);
#endif
        finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
      }
      else
#endif
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }

    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    m_CABACEstimator->resetBits();

    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->coding_unit( cu, partitioner, cuCtx );

    tempCS->dist     = finalDistortion;
    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

    xEncodeDontSplit( *tempCS,         partitioner );
    xCheckDQP       ( *tempCS,         partitioner );
    xCheckBestMode  (  tempCS, bestCS, partitioner, cachedMode );
  }
  else
  {
    THROW( "Should never happen!" );
  }
}

#endif

//! \}
