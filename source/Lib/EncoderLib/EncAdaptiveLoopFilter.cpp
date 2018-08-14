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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"

#if JVET_K0371_ALF
#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#define AlfCtx(c) SubCtx( Ctx::ctbAlfFlag, c )

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
  : m_CABACEstimator( nullptr )
{
  for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_alfCovariance[i] = nullptr;
  }
  for( int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_alfCovarianceFrame[i] = nullptr;
  }
  m_filterCoeffQuant = nullptr;
  m_filterCoeffSet = nullptr;
  m_diffFilterCoeff = nullptr;
}

void EncAdaptiveLoopFilter::create( const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  AdaptiveLoopFilter::create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth );

  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    ChannelType chType = (ChannelType)channelIdx;
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
    m_alfCovarianceFrame[chType] = new AlfCovariance*[m_filterShapes[chType].size()];
    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];
      for( int k = 0; k < numClasses; k++ )
      {
        m_alfCovarianceFrame[chType][i][k].create( m_filterShapes[chType][i].numCoeff );
      }
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
    ChannelType chType = toChannelType( ComponentID( compIdx ) );
    int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

    m_alfCovariance[compIdx] = new AlfCovariance**[m_filterShapes[chType].size()];

    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = new AlfCovariance*[m_numCTUsInPic];
      for( int j = 0; j < m_numCTUsInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance[numClasses];
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovariance[compIdx][i][j][k].create( m_filterShapes[chType][i].numCoeff );
        }
      }
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for( int j = 0; j <= MAX_NUM_ALF_CLASSES; j++ )
    {
      m_alfCovarianceMerged[i][j].create( m_filterShapes[COMPONENT_Y][i].numCoeff );
    }
  }

  m_filterCoeffQuant = new int[MAX_NUM_ALF_LUMA_COEFF];
  m_filterCoeffSet = new int*[MAX_NUM_ALF_CLASSES];
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }
}

void EncAdaptiveLoopFilter::destroy()
{
  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    if( m_alfCovarianceFrame[channelIdx] )
    {
      ChannelType chType = (ChannelType)channelIdx;
      int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovarianceFrame[channelIdx][i][k].destroy();
        }
        delete[] m_alfCovarianceFrame[channelIdx][i];
        m_alfCovarianceFrame[channelIdx][i] = nullptr;
      }
      delete[] m_alfCovarianceFrame[channelIdx];
      m_alfCovarianceFrame[channelIdx] = nullptr;
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    if( m_ctuEnableFlagTmp[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp[compIdx];
      m_ctuEnableFlagTmp[compIdx] = nullptr;
    }

    if( m_alfCovariance[compIdx] )
    {
      ChannelType chType = toChannelType( ComponentID( compIdx ) );
      int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int j = 0; j < m_numCTUsInPic; j++ )
        {
          for( int k = 0; k < numClasses; k++ )
          {
            m_alfCovariance[compIdx][i][j][k].destroy();
          }
          delete[] m_alfCovariance[compIdx][i][j];
          m_alfCovariance[compIdx][i][j] = nullptr;

        }
        delete[] m_alfCovariance[compIdx][i];
        m_alfCovariance[compIdx][i] = nullptr;

      }
      delete[] m_alfCovariance[compIdx];
      m_alfCovariance[compIdx] = nullptr;
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for( int j = 0; j <= MAX_NUM_ALF_CLASSES; j++ )
    {
      m_alfCovarianceMerged[i][j].destroy();
    }
  }

  if( m_filterCoeffSet )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if( m_diffFilterCoeff )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_diffFilterCoeff[i];
      m_diffFilterCoeff[i] = nullptr;
    }
    delete[] m_diffFilterCoeff;
    m_diffFilterCoeff = nullptr;
  }

  delete[] m_filterCoeffQuant;
  m_filterCoeffQuant = nullptr;

  AdaptiveLoopFilter::destroy();
}

#if JEM_TOOLS
void EncAdaptiveLoopFilter::initCABACEstimator( CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
#else
void EncAdaptiveLoopFilter::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
#endif
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_CtxCache = ctxCache;
#if JEM_TOOLS
  m_CABACEstimator->initCtxModels( *pcSlice, cabacDataStore );
#else
  m_CABACEstimator->initCtxModels( *pcSlice );
#endif
  m_CABACEstimator->resetBits();
}

void EncAdaptiveLoopFilter::ALFProcess( CodingStructure& cs, const double *lambdas, AlfSliceParam& alfSliceParam )
{
  // set available filter shapes
  alfSliceParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU ALF enable flags, it was already reset before ALF process
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
  }

  // reset ALF parameters
  alfSliceParam.reset();
#if DISTORTION_LAMBDA_BUGFIX
  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA]);
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA]);
#else
  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA] - 8);
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA] - 8);
#endif
  m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double(1 << shiftLuma);
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double(1 << shiftChroma);
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double(1 << shiftChroma);

  PelUnitBuf orgYuv = cs.getOrgBuf();

  m_tempBuf.copyFrom( cs.getRecoBuf() );
  PelUnitBuf recYuv = m_tempBuf.getBuf( cs.area );
  recYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );

  // derive classification
  const CPelBuf& recLuma = recYuv.get( COMPONENT_Y );
  Area blk( 0, 0, recLuma.width, recLuma.height );
  deriveClassification( m_classifier, recLuma, blk );

  // get CTB stats for filtering
  deriveStatsForFiltering( orgYuv, recYuv );

  // derive filter (luma)
  alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_LUMA );

  // derive filter (chroma)
  if( alfSliceParam.enabledFlag[COMPONENT_Y] )
  {
    alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_CHROMA );
  }
}

double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel, const int numClasses, const int numCoeff, double& distUnfilter )
{
  TempCtx        ctxTempStart( m_CtxCache );
  TempCtx        ctxTempBest( m_CtxCache );
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;

  double cost = 0;
  distUnfilter = 0;

  setEnableFlag(m_alfSliceParamTemp, channel, true);
  if( isChroma( channel ) )
  {
    m_alfSliceParamTemp.chromaCtbPresentFlag = false;
  }

  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    for( int compID = compIDFirst; compID <= compIDLast; compID++ )
    {
      double distUnfilterCtu = getUnfilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses );

      ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 1;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfSliceParamTemp );
      double costOn = distUnfilterCtu + getFilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfSliceParamTemp.numLumaFilters - 1, numCoeff );
      costOn += m_lambda[compID] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
      ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );

      m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 0;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfSliceParamTemp);
      double costOff = distUnfilterCtu + m_lambda[compID] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();

      if( costOn < costOff )
      {
        cost += costOn;
        m_CABACEstimator->getCtx() = AlfCtx( ctxTempBest );
        m_ctuEnableFlag[compID][ctuIdx] = 1;
      }
      else
      {
        cost += costOff;
        m_ctuEnableFlag[compID][ctuIdx] = 0;
        distUnfilter += distUnfilterCtu;
      }
    }
  }

  if( isChroma( channel ) )
  {
    setEnableFlag(m_alfSliceParamTemp, channel, m_ctuEnableFlag);
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[channel];
  }

  return cost;
}

void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfSliceParam& alfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel )
{
  const TempCtx  ctxStart( m_CtxCache, AlfCtx( m_CABACEstimator->getCtx() ) );
  TempCtx        ctxBest( m_CtxCache );

  double costMin = MAX_DOUBLE;

  std::vector<AlfFilterShape>& alfFilterShape = alfSliceParam.filterShapes[channel];
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
  const int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  for( int iShapeIdx = 0; iShapeIdx < alfFilterShape.size(); iShapeIdx++ )
  {
    m_alfSliceParamTemp = alfSliceParam;
    if( isLuma( channel ) )
    {
      m_alfSliceParamTemp.lumaFilterType = alfFilterShape[iShapeIdx].filterType;
    }

    //1. get unfiltered distortion
    double cost = getUnfilteredDistortion( m_alfCovarianceFrame[channel][iShapeIdx], channel );
    cost /= 1.001; // slight preference for unfiltered choice

    if( cost < costMin )
    {
      costMin = cost;
      setEnableFlag( alfSliceParam, channel, false );
      // no CABAC signalling
      ctxBest = AlfCtx( ctxStart );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 0 );
      if( isChroma( channel ) )
      {
        alfSliceParam.chromaCtbPresentFlag = false;
      }
    }

    //2. all CTUs are on
    if( isChroma( channel ) )
    {
      m_alfSliceParamTemp.chromaCtbPresentFlag = true;
    }
    setEnableFlag( m_alfSliceParamTemp, channel, true );
    m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
    setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );
    cost = getFilterCoeffAndCost( cs, 0, channel, false, iShapeIdx, uiCoeffBits );

    if( cost < costMin )
    {
      costMin = cost;
      copyAlfSliceParam( alfSliceParam, m_alfSliceParamTemp, channel );
      ctxBest = AlfCtx( m_CABACEstimator->getCtx() );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 1 );
    }

    //3. CTU decision
    double distUnfilter = 0;
    const int iterNum = 2 * 2 + 1;

    for( int iter = 0; iter < iterNum; iter++ )
    {
      if ((iter & 0x01) == 0)
      {
        m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
        cost = m_lambda[channel] * uiCoeffBits;
        cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel, numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter);
        if (cost < costMin)
        {
          costMin = cost;
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
          copyAlfSliceParam(alfSliceParam, m_alfSliceParamTemp, channel);
        }
      }
      else
      {
        // unfiltered distortion is added due to some CTBs may not use filter
        cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits);
      }
    }//for iter
  }//for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
  copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, channel );

  //filtering
  reconstructCoeff( alfSliceParam, channel, isLuma( channel ) );

  for( int compIdx = compIDFirst; compIdx <= compIDLast; compIdx++ )
  {
    ComponentID compID = (ComponentID)compIdx;
    if( alfSliceParam.enabledFlag[compID] )
    {
      const PreCalcValues& pcv = *cs.pcv;
      int ctuIdx = 0;
      const int chromaScaleX = getComponentScaleX( compID, recBuf.chromaFormat );
      const int chromaScaleY = getComponentScaleY( compID, recBuf.chromaFormat );
      AlfFilterType filterType = isLuma( compID ) ? alfSliceParam.lumaFilterType : ALF_FILTER_5;
      short* coeff = isLuma( compID ) ? m_coeffFinal : alfSliceParam.chromaCoeff;

      for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
      {
        for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
        {
          const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
          const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY );

          if( m_ctuEnableFlag[compID][ctuIdx] )
          {
            if( filterType == ALF_FILTER_5 )
            {
              m_filter5x5Blk( m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx] );
            }
            else if( filterType == ALF_FILTER_7 )
            {
              m_filter7x7Blk( m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx] );
            }
            else
            {
              CHECK( 0, "Wrong ALF filter type" );
            }
          }
          ctuIdx++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( &alfSliceParamDst, &alfSliceParamSrc, sizeof( AlfSliceParam ) );
  }
  else
  {
    alfSliceParamDst.enabledFlag[COMPONENT_Cb] = alfSliceParamSrc.enabledFlag[COMPONENT_Cb];
    alfSliceParamDst.enabledFlag[COMPONENT_Cr] = alfSliceParamSrc.enabledFlag[COMPONENT_Cr];
    alfSliceParamDst.chromaCtbPresentFlag      = alfSliceParamSrc.chromaCtbPresentFlag;
    memcpy( alfSliceParamDst.chromaCoeff, alfSliceParamSrc.chromaCoeff, sizeof( alfSliceParamDst.chromaCoeff ) );
  }
}
double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits )
{
  //collect stat based on CTU decision
  if( bReCollectStat )
  {
    getFrameStats( channel, iShapeIdx );
  }

  double dist = distUnfilter;
  uiCoeffBits = 0;
  int uiSliceFlag = 0;
  AlfFilterShape& alfFilterShape = m_alfSliceParamTemp.filterShapes[channel][iShapeIdx];
  //get filter coeff
  if( isLuma( channel ) )
  {
    //distortion
    dist += mergeFiltersAndCost( m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits );
  }
  else
  {
    //distortion
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterCoeffQuant, m_alfCovarianceFrame[channel][iShapeIdx][0].E, m_alfCovarianceFrame[channel][iShapeIdx][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true );
    memcpy( m_filterCoeffSet[0], m_filterCoeffQuant, sizeof( *m_filterCoeffQuant ) * alfFilterShape.numCoeff );
    //setEnableFlag( m_alfSliceParamTemp, channel, m_ctuEnableFlag );
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
    {
      m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
    }
    uiCoeffBits += getCoeffRate( m_alfSliceParamTemp, true );
    uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }

  double rate = uiCoeffBits + uiSliceFlag;
  if (isLuma(channel) || (!m_alfSliceParamTemp.chromaCtbPresentFlag))
  {
    if (isChroma(channel))
    {
      CHECK(m_alfSliceParamTemp.chromaCtbPresentFlag, "chromaCTB is on");
    }
    else
    {
      CHECK(!m_alfSliceParamTemp.enabledFlag[COMPONENT_Y], "Slice Y is off");
    }
    m_CABACEstimator->resetBits();
    m_CABACEstimator->codeAlfCtuEnableFlags( cs, channel, &m_alfSliceParamTemp);
    rate += FracBitsScale * (double)m_CABACEstimator->getEstFracBits();
  }
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma )
{
  int iBits = 0;
  if( !isChroma )
  {
    iBits++;                                               // alf_coefficients_delta_flag
    if( !alfSliceParam.coeffDeltaFlag )
    {
      if( alfSliceParam.numLumaFilters > 1 )
      {
        iBits++;                                           // coeff_delta_pred_mode_flag
      }
    }
  }

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  AlfFilterShape alfShape( isChroma ? 5 : ( alfSliceParam.lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  const short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;

  // vlc for all
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( isChroma || !alfSliceParam.coeffDeltaFlag || alfSliceParam.filterCoeffFlag[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        int coeffVal = abs( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] );

        for( int k = 1; k < 15; k++ )
        {
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
        }
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Golomb parameters
  iBits += lengthUvlc( kMin - 1 );  // "min_golomb_order"
  int golombOrderIncreaseFlag = 0;

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    golombOrderIncreaseFlag = ( m_kMinTab[idx] != kMin ) ? 1 : 0;
    CHECK( !( m_kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
    iBits += golombOrderIncreaseFlag;                           //golomb_order_increase_flag
    kMin = m_kMinTab[idx];
  }

  if( !isChroma )
  {
    if( alfSliceParam.coeffDeltaFlag )
    {
      iBits += numFilters;             //filter_coefficient_flag[i]
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.filterCoeffFlag[ind] && alfSliceParam.coeffDeltaFlag )
    {
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      iBits += lengthGolomb( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }
  return iBits;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel )
{
  double dist = 0;
  if( isLuma( channel ) )
  {
    dist = getUnfilteredDistortion( cov, MAX_NUM_ALF_CLASSES );
  }
  else
  {
    dist = getUnfilteredDistortion( cov, 1 ) + lengthTruncatedUnary( 0, 3 ) * m_lambda[COMPONENT_Cb];
  }
  return dist;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, const int numClasses )
{
  double dist = 0;
  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

double EncAdaptiveLoopFilter::getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff )
{
  double dist = 0;

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    int filterIdx = numClasses == 1 ? 0 : m_filterIndices[numFiltersMinus1][classIdx];
    dist += calcErrorForCoeffs( cov[classIdx].E, cov[classIdx].y, m_filterCoeffSet[filterIdx], numCoeff, m_NUM_BITS );
  }

  return dist;
}

double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits )
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;

  mergeClasses( covFrame, covMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );

  while( numFilters >= 1 )
  {
    dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab );
    // filter coeffs are stored in m_filterCoeffSet
    distForce0 = getDistForce0( alfShape, numFilters, errorForce0CoeffTab, codedVarBins );
    coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, predMode ); 
    coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFilters, codedVarBins );

    cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

    if( cost0 < cost )
    {
      cost = cost0;
    }

    if( cost <= costMin )
    {
      costMin = cost;
      numFiltersBest = numFilters;
      bestPredMode = predMode;
    }
    numFilters--;
  }

  dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab );
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, predMode );
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins );

  cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

  alfSliceParam.numLumaFilters = numFiltersBest;
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfSliceParam.coeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
    alfSliceParam.coeffDeltaPredModeFlag = bestPredMode;
  }
  else
  {
    distReturn = distForce0;
    alfSliceParam.coeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    memcpy( alfSliceParam.filterCoeffFlag, codedVarBins, sizeof( codedVarBins ) );
    alfSliceParam.coeffDeltaPredModeFlag = 0;

    for( int varInd = 0; varInd < numFiltersBest; varInd++ )
    {
      if( codedVarBins[varInd] == 0 )
      {
        memset( m_filterCoeffSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
      }
    }
  }

  for( int ind = 0; ind < alfSliceParam.numLumaFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff; i++ )
    {
      if( alfSliceParam.coeffDeltaPredModeFlag )
      {
        alfSliceParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_diffFilterCoeff[ind][i];
      }
      else
      {
        alfSliceParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      }
    }
  }

  memcpy( alfSliceParam.filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof( short ) * MAX_NUM_ALF_CLASSES );
  uiCoeffBits += getNonFilterCoeffRate( alfSliceParam );
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfSliceParam& alfSliceParam )
{
  int len = 1   // filter_type
    	    + 1   // alf_coefficients_delta_flag
    	    + lengthTruncatedUnary( 0, 3 )    // chroma_idc = 0, it is signalled when ALF is enabled for luma
    	    + getTBlength( alfSliceParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES );   //numLumaFilters

  if( alfSliceParam.numLumaFilters > 1 )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      len += getTBlength( (int)alfSliceParam.filterCoeffDeltaIdx[i], alfSliceParam.numLumaFilters );  //filter_coeff_delta[i]
    }
  }
  return len;
}

int EncAdaptiveLoopFilter::lengthTruncatedUnary( int symbol, int maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return 0;
  }

  bool codeLast = ( maxSymbol > symbol );
  int bins = 0;
  int numBins = 0;
  while( symbol-- )
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if( codeLast )
  {
    bins <<= 1;
    numBins++;
  }

  return numBins;
}

int EncAdaptiveLoopFilter::getTBlength( int uiSymbol, const int uiMaxSymbol )
{
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  assert( uiVal <= uiMaxSymbol );
  assert( ( uiVal << 1 ) > uiMaxSymbol );
  assert( uiSymbol < uiMaxSymbol );
  int b = uiMaxSymbol - uiVal;
  assert( b < uiVal );
  if( uiSymbol < uiVal - b )
  {
    return uiThresh;
  }
  else
  {
    return uiThresh + 1;
  }
}

int EncAdaptiveLoopFilter::getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins )
{
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !codedVarBins[ind] )
    {
      continue;
    }
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( pDiffQFilterCoeffIntPP[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Coding parameters
  int len = kMin           //min_golomb_order
    	    + maxGolombIdx   //golomb_order_increase_flag
    	    + numFilters;    //filter_coefficient_flag[i]

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( codedVarBins[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        len += lengthGolomb( abs( pDiffQFilterCoeffIntPP[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] ); // alf_coeff_luma_delta[i][j]
      }
    }
  }

  return len;
}

int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode )
{
  int ratePredMode0 = getCostFilterCoeff( alfShape, filterSet, numFilters );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( ind == 0 )
    {
      memcpy( filterCoeffDiff[ind], filterSet[ind], sizeof( int ) * alfShape.numCoeff );
    }
    else
    {
      for( int i = 0; i < alfShape.numCoeff; i++ )
      {
        filterCoeffDiff[ind][i] = filterSet[ind][i] - filterSet[ind - 1][i];
      }
    }
  }

  int ratePredMode1 = getCostFilterCoeff( alfShape, filterCoeffDiff, numFilters );

  predMode = ( ratePredMode1 < ratePredMode0 && numFilters > 1 ) ? 1 : 0;

  return ( numFilters > 1 ? 1 : 0 )        // coeff_delta_pred_mode_flag
       + ( predMode ? ratePredMode1 : ratePredMode0 ); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
}

int EncAdaptiveLoopFilter::getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( pDiffQFilterCoeffIntPP[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Coding parameters
  int len = kMin           //min_golomb_order
          + maxGolombIdx;  //golomb_order_increase_flag

  // Filter coefficients
  len += lengthFilterCoeffs( alfShape, numFilters, pDiffQFilterCoeffIntPP, m_kMinTab );  // alf_coeff_luma_delta[i][j]

  return len;
}

int EncAdaptiveLoopFilter::lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab )
{
  int bitCnt = 0;

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitCnt += lengthGolomb( abs( FilterCoeff[ind][i] ), kMinTab[alfShape.golombIdx[i]] );
    }
  }
  return bitCnt;
}

double EncAdaptiveLoopFilter::getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins )
{
  static int bitsVarBin[MAX_NUM_ALF_CLASSES];

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( m_filterCoeffSet[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    bitsVarBin[ind] = 0;
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitsVarBin[ind] += lengthGolomb( abs( m_filterCoeffSet[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] );
    }
  }

  double distForce0 = getDistCoeffForce0( codedVarBins, errorTabForce0Coeff, bitsVarBin, numFilters );

  return distForce0;
}

int EncAdaptiveLoopFilter::getGolombKMin( AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB] )
{
  int kStart;
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );

  int minBitsKStart = MAX_INT;
  int minKStart = -1;

  for( int k = 1; k < 8; k++ )
  {
    int bitsKStart = 0; kStart = k;
    for( int scanPos = 0; scanPos < maxGolombIdx; scanPos++ )
    {
      int kMin = kStart;
      int minBits = bitsCoeffScan[scanPos][kMin];

      if( bitsCoeffScan[scanPos][kStart + 1] < minBits )
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if( bitsKStart < minBitsKStart )
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for( int scanPos = 0; scanPos < maxGolombIdx; scanPos++ )
  {
    int kMin = kStart;
    int minBits = bitsCoeffScan[scanPos][kMin];

    if( bitsCoeffScan[scanPos][kStart + 1] < minBits )
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  return minKStart;
}

double EncAdaptiveLoopFilter::getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters )
{
  double distForce0 = 0;
  std::memset( codedVarBins, 0, sizeof( *codedVarBins ) * MAX_NUM_ALF_CLASSES );

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    double costDiff = errorForce0CoeffTab[filtIdx][0] - ( errorForce0CoeffTab[filtIdx][1] + m_lambda[COMPONENT_Y] * bitsVarBin[filtIdx] );
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }

  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc( int uiCode )
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return ( uiLength >> 1 ) + ( ( uiLength + 1 ) >> 1 );
}

int EncAdaptiveLoopFilter::lengthGolomb( int coeffVal, int k )
{
  int m = 2 << ( k - 1 );
  int q = coeffVal / m;
  if( coeffVal != 0 )
  {
    return q + 2 + k;
  }
  else
  {
    return q + 1 + k;
  }
}

double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2] )
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      if( filterIndices[classIdx] == filtIdx )
      {
        tmpCov += cov[classIdx];
      }
    }

    // Find coeffcients
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterCoeffQuant, tmpCov.E, tmpCov.y, alfShape.numCoeff, alfShape.weights, m_NUM_BITS );
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];

    // store coeff
    memcpy( m_filterCoeffSet[filtIdx], m_filterCoeffQuant, sizeof( int )*alfShape.numCoeff );
  }
  return error;
}

double EncAdaptiveLoopFilter::deriveCoeffQuant( int *filterCoeffQuant, double **E, double *y, const int numCoeff, std::vector<int>& weights, const int bitDepth, const bool bChroma )
{
  const int factor = 1 << ( bitDepth - 1 );
  static int filterCoeffQuantMod[MAX_NUM_ALF_LUMA_COEFF];
  static double filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  gnsSolveByChol( E, y, filterCoeff, numCoeff );
  roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );
  const int targetCoeffSumInt = 0;
  int quantCoeffSum = 0;
  for( int i = 0; i < numCoeff; i++ )
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
  }

  int count = 0;
  while( quantCoeffSum != targetCoeffSumInt && count < 10 )
  {
    int sign = quantCoeffSum > targetCoeffSumInt ? 1 : -1;
    int diff = ( quantCoeffSum - targetCoeffSumInt ) * sign;

    double errMin = MAX_DOUBLE;
    int minInd = -1;

    for( int k = 0; k < numCoeff; k++ )
    {
      if( weights[k] <= diff )
      {
        memcpy( filterCoeffQuantMod, filterCoeffQuant, sizeof( int ) * numCoeff );

        filterCoeffQuantMod[k] -= sign;
        double error = calcErrorForCoeffs( E, y, filterCoeffQuantMod, numCoeff, bitDepth );

        if( error < errMin )
        {
          errMin = error;
          minInd = k;
        }
      }
    }

    if( minInd != -1 )
    {
      filterCoeffQuant[minInd] -= sign;
    }

    quantCoeffSum = 0;
    for( int i = 0; i < numCoeff; i++ )
    {
      quantCoeffSum += weights[i] * filterCoeffQuant[i];
    }
    ++count;
  }
  if( count == 10 )
  {
    memset( filterCoeffQuant, 0, sizeof( int ) * numCoeff );
  }

  //512 -> 1, (64+32+4+2)->0.199
  int max_value = 512 + 64 + 32 + 4 + 2;  
  int min_value = -max_value;             
  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
    filterCoeff[i] = filterCoeffQuant[i] / double( factor );
  }

  quantCoeffSum = 0;
  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }
  filterCoeffQuant[numCoeff - 1] = -quantCoeffSum;
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);

  double error = calcErrorForCoeffs( E, y, filterCoeffQuant, numCoeff, bitDepth );
  return error;
}

double EncAdaptiveLoopFilter::calcErrorForCoeffs( double **E, double *y, int *coeff, const int numCoeff, const int bitDepth )
{
  double factor = 1 << ( bitDepth - 1 );
  double error = 0;

  for( int i = 0; i < numCoeff; i++ )   //diagonal
  {
    double sum = 0;
    for( int j = i + 1; j < numCoeff; j++ )
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[i][j] * coeff[j];
    }
    error += ( ( E[i][i] * coeff[i] + sum * 2 ) / factor - 2 * y[i] ) * coeff[i];
  }

  return error / factor;
}

void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor )
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int( filterCoeff[i] * sign * factor + 0.5 ) * sign;
  }
}

void EncAdaptiveLoopFilter::mergeClasses( AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] )
{
  static bool availableClass[MAX_NUM_ALF_CLASSES];
  static uint8_t indexList[MAX_NUM_ALF_CLASSES];
  static uint8_t indexListTemp[MAX_NUM_ALF_CLASSES];
  int numRemaining = numClasses;

  memset( filterIndices, 0, sizeof( short ) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES );

  for( int i = 0; i < numClasses; i++ )
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i] = i;
    availableClass[i] = true;
    covMerged[i] = cov[i];
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  while( numRemaining > 2 )
  {
    double errorMin = std::numeric_limits<double>::max();
    int bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for( int i = 0; i < numClasses - 1; i++ )
    {
      if( availableClass[i] )
      {
        for( int j = i + 1; j < numClasses; j++ )
        {
          if( availableClass[j] )
          {
            double error1 = calculateError( covMerged[i] );
            double error2 = calculateError( covMerged[j] );

            tmpCov.add( covMerged[i], covMerged[j] );
            double error = calculateError( tmpCov ) - error1 - error2;

            if( error < errorMin )
            {
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    availableClass[bestToMergeIdx2] = false;

    for( int i = 0; i < numClasses; i++ )
    {
      if( indexList[i] == bestToMergeIdx2 )
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if( numRemaining <= numClasses )
    {
      std::memcpy( indexListTemp, indexList, sizeof( uint8_t ) * numClasses );

      bool exist = false;
      int ind = 0;

      for( int j = 0; j < numClasses; j++ )
      {
        exist = false;
        for( int i = 0; i < numClasses; i++ )
        {
          if( indexListTemp[i] == j )
          {
            exist = true;
            break;
          }
        }

        if( exist )
        {
          for( int i = 0; i < numClasses; i++ )
          {
            if( indexListTemp[i] == j )
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStats( ChannelType channel, int iShapeIdx )
{
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  for( int i = 0; i < numClasses; i++ )
  {
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset();
  }
  if( isLuma( channel ) )
  {
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], numClasses );
  }
  else
  {
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], numClasses );
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], numClasses );
  }
}

void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses )
{
  for( int i = 0; i < m_numCTUsInPic; i++ )
  {
    if( ctbEnableFlags[i] )
    {
      for( int j = 0; j < numClasses; j++ )
      {
        frameCov[j] += ctbCov[i][j];
      }
    }
  }
}

void EncAdaptiveLoopFilter::deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv )
{
  int ctuRsAddr = 0;
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );

  // init CTU stats buffers
  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[toChannelType( compID )].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
        {
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset();
        }
      }
    }
  }

  // init Frame stats buffers
  const int numberOfChannels = getNumberValidChannels( m_chromaFormat );
  for( int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++ )
  {
    const ChannelType channelID = ChannelType( channelIdx );
    const int numClasses = isLuma( channelID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[channelIdx].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset();
      }
    }
  }

  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
      const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );

      for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
      {
        const ComponentID compID = ComponentID( compIdx );
        const CompArea& compArea = area.block( compID );

        int  recStride = recYuv.get( compID ).stride;
        Pel* rec = recYuv.get( compID ).bufAt( compArea );

        int  orgStride = orgYuv.get( compID ).stride;
        Pel* org = orgYuv.get( compID ).bufAt( compArea );

        ChannelType chType = toChannelType( compID );

        for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
        {
          getBlkStats( m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea );

          const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

          for( int classIdx = 0; classIdx < numClasses; classIdx++ )
          {
            m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
          }
        }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStats( AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area )
{
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF];

  int transposeIdx = 0;
  int classIdx = 0;

  for( int i = 0; i < area.height; i++ )
  {
    for( int j = 0; j < area.width; j++ )
    {
      std::memset( ELocal, 0, shape.numCoeff * sizeof( int ) );
      if( classifier )
      {
        AlfClassifier& cl = classifier[area.y + i][area.x + j];
        transposeIdx = cl.transposeIdx;
        classIdx = cl.classIdx;
      }

      int yLocal = org[j] - rec[j];
      calcCovariance( ELocal, rec + j, recStride, shape.pattern.data(), shape.filterLength >> 1, transposeIdx );
      for( int k = 0; k < shape.numCoeff; k++ )
      {
        for( int l = k; l < shape.numCoeff; l++ )
        {
          alfCovariace[classIdx].E[k][l] += ELocal[k] * ELocal[l];
        }
        alfCovariace[classIdx].y[k] += ELocal[k] * yLocal;
      }
      alfCovariace[classIdx].pixAcc += yLocal * yLocal;
    }
    org += orgStride;
    rec += recStride;
  }

  int numClasses = classifier ? MAX_NUM_ALF_CLASSES : 1;
  for( classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    for( int k = 1; k < shape.numCoeff; k++ )
    {
      for( int l = 0; l < k; l++ )
      {
        alfCovariace[classIdx].E[k][l] = alfCovariace[classIdx].E[l][k];
      }
    }
  }
}

void EncAdaptiveLoopFilter::calcCovariance( int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx )
{
  int k = 0;

  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;

      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++ )
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
    }
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for( int i = -halfFilterLength - j; i <= halfFilterLength + j; i++ )
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
    }
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;

      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j-- )
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
    }
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i-- )
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
    }
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
  }
  ELocal[filterPattern[k++]] += rec[0];
}



double EncAdaptiveLoopFilter::calculateError( AlfCovariance& cov )
{
  static double c[MAX_NUM_ALF_COEFF];

  gnsSolveByChol( cov.E, cov.y, c, cov.numCoeff );

  double sum = 0;
  for( int i = 0; i < cov.numCoeff; i++ )
  {
    sum += c[i] * cov.y[i];
  }

  return cov.pixAcc - sum;
}

//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
int EncAdaptiveLoopFilter::gnsCholeskyDec( double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq )
{
  static double invDiag[MAX_NUM_ALF_COEFF];  /* Vector of the inverse of diagonal entries of outMatr */

  for( int i = 0; i < numEq; i++ )
  {
    for( int j = i; j < numEq; j++ )
    {
      /* Compute the scaling factor */
      double scale = inpMatr[i][j];
      if( i > 0 )
      {
        for( int k = i - 1; k >= 0; k-- )
        {
          scale -= outMatr[k][j] * outMatr[k][i];
        }
      }

      /* Compute i'th row of outMatr */
      if( i == j )
      {
        if( scale <= REG_SQR ) // if(scale <= 0 )  /* If inpMatr is singular */
        {
          return 0;
        }
        else              /* Normal operation */
          invDiag[i] = 1.0 / ( outMatr[i][i] = sqrt( scale ) );
      }
      else
      {
        outMatr[i][j] = scale * invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }
    }
  }
  return 1; /* Signal that Cholesky factorization is successfully performed */
}

void EncAdaptiveLoopFilter::gnsTransposeBacksubstitution( double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order )
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
  for( int i = 1; i < order; i++ )
  {         /* For the rows 1..order-1           */

    double sum = 0; //Holds backsubstitution from already handled rows

    for( int j = 0; j < i; j++ ) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = ( rhs[i] - sum ) / U[i][i];       /* i'th component of solution vect.  */
  }
}

void EncAdaptiveLoopFilter::gnsBacksubstitution( double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A )
{
  size--;
  A[size] = z[size] / R[size][size];

  for( int i = size - 1; i >= 0; i-- )
  {
    double sum = 0;

    for( int j = i + 1; j <= size; j++ )
    {
      sum += R[i][j] * A[j];
    }

    A[i] = ( z[i] - sum ) / R[i][i];
  }
}

int EncAdaptiveLoopFilter::gnsSolveByChol( double **LHS, double *rhs, double *x, int numEq )
{
  static double aux[MAX_NUM_ALF_COEFF];     /* Auxiliary vector */
  static double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF];    /* Upper triangular Cholesky factor of LHS */
  int res = 1;  // Signal that Cholesky factorization is successfully performed

                /* The equation to be solved is LHSx = rhs */

                /* Compute upper triangular U such that U'*U = LHS */
  if( gnsCholeskyDec( LHS, U, numEq ) ) /* If Cholesky decomposition has been successful */
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
    * Solve U'*aux = rhs for aux
    */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution( U, aux, numEq, x );

  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for( int i = 0; i < numEq; i++ )
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
    res = gnsCholeskyDec( LHS, U, numEq );

    if( !res )
    {
      std::memset( x, 0, sizeof( double )*numEq );
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* Solve U*x = aux for x */
    gnsBacksubstitution( U, aux, numEq, x );
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////
void EncAdaptiveLoopFilter::setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, bool val )
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    alfSlicePara.enabledFlag[COMPONENT_Y] = val;
  }
  else
  {
    alfSlicePara.enabledFlag[COMPONENT_Cb] = alfSlicePara.enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags )
{
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
  for( int compId = compIDFirst; compId <= compIDLast; compId++ )
  {
    alfSlicePara.enabledFlag[compId] = false;
    for( int i = 0; i < m_numCTUsInPic; i++ )
    {
      if( ctuFlags[compId][i] )
      {
        alfSlicePara.enabledFlag[compId] = true;
        break;
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memcpy( ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof( uint8_t ) * m_numCTUsInPic );
    memcpy( ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val )
{
  if( isLuma( channel ) )
  {
    memset( ctuFlags[COMPONENT_Y], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memset( ctuFlags[COMPONENT_Cb], val, sizeof( uint8_t ) * m_numCTUsInPic );
    memset( ctuFlags[COMPONENT_Cr], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
}
#elif JEM_TOOLS
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/dtrace_codingstruct.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// ====================================================================================================================
// Constants
// ====================================================================================================================


#define ALF_NUM_OF_REDESIGN 1

// ====================================================================================================================
// Tables
// ====================================================================================================================

const int EncAdaptiveLoopFilter::m_aiSymmetricArray9x9[81] =
{
   0,  1,  2,  3,  4,  5,  6,  7,  8,
   9, 10, 11, 12, 13, 14, 15, 16, 17,
  18, 19, 20, 21, 22, 23, 24, 25, 26,
  27, 28, 29, 30, 31, 32, 33, 34, 35,
  36, 37, 38, 39, 40, 39, 38, 37, 36,
  35, 34, 33, 32, 31, 30, 29, 28, 27,
  26, 25, 24, 23, 22, 21, 20, 19, 18,
  17, 16, 15, 14, 13, 12, 11, 10,  9,
   8,  7,  6,  5,  4,  3,  2,  1,  0
};

const int EncAdaptiveLoopFilter::m_aiSymmetricArray7x7[49] =
{
  0,  1,  2,  3,  4,  5,  6,
  7,  8,  9, 10, 11, 12, 13,
  14, 15, 16, 17, 18, 19, 20,
  21, 22, 23, 24, 23, 22, 21,
  20, 19, 18, 17, 16, 15, 14,
  13, 12, 11, 10,  9,  8,  7,
  6,  5,  4,  3,  2,  1,  0,
};

const int EncAdaptiveLoopFilter::m_aiSymmetricArray5x5[25] =
{
  0,  1,  2,  3,  4,
  5,  6,  7,  8,  9,
  10, 11, 12, 11, 10,
  9,  8,  7,  6,  5,
  4,  3,  2,  1,  0,
};

const int EncAdaptiveLoopFilter::m_aiSymmetricArray9x7[63] =
{
   0,  1,  2,  3,  4,  5,  6,  7,  8,
   9, 10, 11, 12, 13, 14, 15, 16, 17,
  18, 19, 20, 21, 22, 23, 24, 25, 26,
  27, 28, 29, 30, 31, 30, 29, 28, 27,
  26, 25, 24, 23, 22, 21, 20, 19, 18,
  17, 16, 15, 14, 13, 12, 11, 10,  9,
   8,  7,  6,  5,  4,  3,  2,  1,  0
};

const int EncAdaptiveLoopFilter::m_aiTapPos9x9_In9x9Sym[21] =
{
                  0,  1,  2,
              3,  4,  5,  6,  7,
          8,  9, 10, 11, 12, 13, 14,
     15, 16, 17, 18, 19, 20
};

const int EncAdaptiveLoopFilter::m_aiTapPos7x7_In9x9Sym[14] =
{
                  1,
              4,  5,  6,
          9, 10, 11, 12, 13,
     16, 17, 18, 19, 20
};

const int EncAdaptiveLoopFilter::m_aiTapPos5x5_In9x9Sym[8]  =
{

            5,
       10, 11, 12,
   17, 18, 19, 20
};

const int* EncAdaptiveLoopFilter::m_iTapPosTabIn9x9Sym[m_NO_TEST_FILT] =
{
  m_aiTapPos9x9_In9x9Sym, m_aiTapPos7x7_In9x9Sym, m_aiTapPos5x5_In9x9Sym
};

// ====================================================================================================================
// Constructor / destructor
// ====================================================================================================================



EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
{
  m_eSliceType          = I_SLICE;
  m_iPicNalReferenceIdc = 0;

  m_pcBestAlfParam = nullptr;
  m_pcTempAlfParam = nullptr;

  m_EGlobalSym    = nullptr;
  m_yGlobalSym    = nullptr;
  m_pixAcc        = nullptr;
  m_E_temp        = nullptr;
  m_y_temp        = nullptr;
  m_E_merged      = nullptr;
  m_y_merged      = nullptr;
  m_pixAcc_merged = nullptr;
  m_varImg        = nullptr;


  m_filterCoeff          = nullptr;
  m_pdDoubleAlfCoeff     = nullptr;
  m_filterCoeffQuantMod  = nullptr;
  m_filterCoeffQuant     = nullptr;
  m_diffFilterCoeffQuant = nullptr;
  m_FilterCoeffQuantTemp = nullptr;
  m_filterCoeffSymQuant  = nullptr;

  m_ppdAlfCorr     = nullptr;
  m_CABACEstimator = nullptr;
  m_CABACDataStore = nullptr;
  m_pSlice         = nullptr;

  m_dLambdaLuma   = 0;
  m_dLambdaChroma = 0;
  m_isDec         = false;

  m_is9x9Alloc    = 0;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncAdaptiveLoopFilter::create( const int iPicWidth, int iPicHeight, const ChromaFormat chromaFormatIDC, const int uiMaxCUWidth, const uint32_t uiMaxCUHeight, const uint32_t uiMaxCUDepth, const int nInputBitDepth, const int nInternalBitDepth, const int numberOfCTUs )
{
  AdaptiveLoopFilter::create( iPicWidth, iPicHeight, chromaFormatIDC, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth, nInputBitDepth, nInternalBitDepth, numberOfCTUs );

  m_bestPelBuf.create( chromaFormatIDC, Area(0, 0, iPicWidth, iPicHeight ), uiMaxCUWidth, 0, 0, false );
  m_tempPelBuf.create( chromaFormatIDC, Area(0, 0, iPicWidth, iPicHeight ), uiMaxCUWidth, 0, 0, false );

  m_maskBuf.buf = (uint32_t *) calloc( iPicWidth*iPicHeight, sizeof(uint32_t ) );
  m_maskBuf.width   = iPicWidth;
  m_maskBuf.height  = iPicHeight;
  m_maskBuf.stride  = iPicWidth;

  m_maskBestBuf.buf = (uint32_t *)calloc(iPicWidth*iPicHeight, sizeof(uint32_t));
  m_maskBestBuf.width = iPicWidth;
  m_maskBestBuf.height = iPicHeight;
  m_maskBestBuf.stride = iPicWidth;


  m_varImg          = m_varImgMethods;

  m_pcBestAlfParam = new ALFParam;
  m_pcTempAlfParam = new ALFParam;
  allocALFParam(m_pcBestAlfParam);
  allocALFParam(m_pcTempAlfParam);

  // init qc_filter
  initMatrix4D_double( &m_EGlobalSym, m_NO_TEST_FILT,  m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH, m_MAX_SQR_FILT_LENGTH);
  initMatrix3D_double( &m_yGlobalSym, m_NO_TEST_FILT, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  m_pixAcc = (double *) calloc(m_NO_VAR_BINS, sizeof(double));

  initMatrix_double( &m_E_temp, m_MAX_SQR_FILT_LENGTH, m_MAX_SQR_FILT_LENGTH);//
  m_y_temp = (double *) calloc(m_MAX_SQR_FILT_LENGTH, sizeof(double));//
  initMatrix3D_double(&m_E_merged, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH, m_MAX_SQR_FILT_LENGTH);//
  initMatrix_double(&m_y_merged, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH); //
  m_pixAcc_merged = (double *) calloc(m_NO_VAR_BINS, sizeof(double));//

  initMatrix_int(&m_filterCoeffSymQuant, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  m_filterCoeffQuantMod = (int *) calloc(m_MAX_SQR_FILT_LENGTH, sizeof(int));//
  m_filterCoeff = (double *) calloc(m_MAX_SQR_FILT_LENGTH, sizeof(double));//
  m_filterCoeffQuant = (int *) calloc(m_MAX_SQR_FILT_LENGTH, sizeof(int));//
  initMatrix_int(&m_diffFilterCoeffQuant, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);//
  initMatrix_int(&m_FilterCoeffQuantTemp, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);//
#if JVET_C0038_NO_PREV_FILTERS
  initMatrix_int(&m_imgY_preFilter, iPicHeight, iPicWidth);
#endif

  initMatrix_double( &E_temp2D, m_MAX_SQR_FILT_LENGTH, m_MAX_SQR_FILT_LENGTH );
  y_temp1D = new double[m_MAX_SQR_FILT_LENGTH];

  initMatrix3D_double( &E_temp3D, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH, m_MAX_SQR_FILT_LENGTH );
  initMatrix_double( &y_temp2D, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH );
  pixAcc_temp1D = ( double * ) calloc( m_NO_VAR_BINS, sizeof( double ) );
  initMatrix_int( &FilterCoeffQuantTemp, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH );

  initMatrix_double( &y_temp9x9, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH );
}


void EncAdaptiveLoopFilter::init( CodingStructure& cs, CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder )
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator( cs.slice->getSPS() );
  m_CABACDataStore = cabacDataStore;
  m_eSliceType = cs.slice->getSliceType();
  m_iPicNalReferenceIdc = cs.picture->referenced ? 1 :0;

  // copy the clip ranges
  m_clpRngs = cs.slice->clpRngs();
  m_isGALF  = cs.sps->getSpsNext().getGALFEnabled();

  xInitParam();
}

void EncAdaptiveLoopFilter::destroy()
{
  if( !m_wasCreated )
    return;

  xUninitParam();

  m_bestPelBuf.destroy();
  m_tempPelBuf.destroy();

  free( m_maskBuf.buf );
  m_maskBuf.buf = nullptr;

  free(m_maskBestBuf.buf);
  m_maskBestBuf.buf = nullptr;

  m_CABACEstimator = NULL;
  m_CABACDataStore = nullptr;

  freeALFParam(m_pcBestAlfParam);
  freeALFParam(m_pcTempAlfParam);
  delete m_pcBestAlfParam;
  delete m_pcTempAlfParam;
  m_pcBestAlfParam = nullptr;
  m_pcTempAlfParam = nullptr;

  // delete qc filters
  destroyMatrix4D_double(m_EGlobalSym, m_NO_TEST_FILT,  m_NO_VAR_BINS);
  destroyMatrix3D_double(m_yGlobalSym, m_NO_TEST_FILT);
  destroyMatrix_int(m_filterCoeffSymQuant);

  free(m_pixAcc);

  destroyMatrix3D_double(m_E_merged, m_NO_VAR_BINS);
  destroyMatrix_double(m_y_merged);
  destroyMatrix_double(m_E_temp);
  free(m_pixAcc_merged);

  free(m_filterCoeffQuantMod);
  free(m_y_temp);

  free(m_filterCoeff);
  free(m_filterCoeffQuant);
  destroyMatrix_int(m_diffFilterCoeffQuant);
  destroyMatrix_int(m_FilterCoeffQuantTemp);
#if JVET_C0038_NO_PREV_FILTERS
  destroyMatrix_int(m_imgY_preFilter);
#endif


  destroyMatrix_double( E_temp2D );
  delete[] y_temp1D;
  y_temp1D = nullptr;

  destroyMatrix3D_double( E_temp3D, m_NO_VAR_BINS );
  free( pixAcc_temp1D );
  destroyMatrix_int( FilterCoeffQuantTemp );

  destroyMatrix_double( y_temp2D );

  if( m_is9x9Alloc )
  {
    destroyMatrix_double( y_temp9x9 );
  }

  AdaptiveLoopFilter::destroy();
}

/**
 \param pcAlfParam           ALF parameter
 \param dLambda              lambda value for RD cost computation
 \retval ruiDist             distortion
 \retval ruiBits             required bits
 \retval ruiMaxAlfCtrlDepth  optimal partition depth
 */
void EncAdaptiveLoopFilter::ALFProcess(CodingStructure& cs, ALFParam* pcAlfParam, double dLambdaLuma, double dLambdaChroma )
{
#if COM16_C806_ALF_TEMPPRED_NUM
  const int tidx = cs.slice->getTLayer();
  CHECK( tidx >= E0104_ALF_MAX_TEMPLAYERID, " index out of range");
  ALFParam *pcStoredAlfPara = cs.slice->isIntra() ? NULL : m_acStoredAlfPara[tidx];
  int iStoredAlfParaNum = m_storedAlfParaNum[tidx];
#endif
  CHECK( cs.pcv->lumaHeight != m_img_height     , "wrong parameter set" );
  CHECK( cs.pcv->lumaWidth  != m_img_width      , "wrong parameter set" );
  CHECK( cs.pcv->sizeInCtus != m_uiNumCUsInFrame, "wrong parameter set" );

  int tap, num_coef;

  // set global variables
  tap      = m_ALF_MAX_NUM_TAP;
  int tapV = AdaptiveLoopFilter::ALFTapHToTapV(tap);
  num_coef = (tap * tapV + 1) >> 1;
  num_coef = num_coef + (m_isGALF?0:1); // DC offset

  resetALFParam( pcAlfParam );
  resetALFParam( m_pcBestAlfParam );
  resetALFParam( m_pcTempAlfParam );

  if( m_isGALF )
    xInitFixedFilters();

  // set lambda
  m_dLambdaLuma   = dLambdaLuma;
  m_dLambdaChroma = dLambdaChroma;
  m_pSlice        = cs.slice;
  const PelUnitBuf orgUnitBuf = cs.getOrgBuf();

  PelUnitBuf recUnitBuf = cs.getRecoBuf();

  m_tmpRecExtBuf.copyFrom( recUnitBuf );
  PelUnitBuf recExtBuf = m_tmpRecExtBuf.getBuf( cs.area );
  recExtBuf.extendBorderPel( m_FILTER_LENGTH >> 1 );

  const PelUnitBuf cRecExtBuf = recExtBuf;
  // set min cost
#if DISTORTION_TYPE_BUGFIX
  uint64_t uiMinRate = std::numeric_limits<uint64_t>::max();
  uint64_t uiMinDist = std::numeric_limits<uint64_t>::max();
#else
  uint64_t uiMinRate = MAX_INT;
  uint64_t uiMinDist = MAX_INT;
#endif
  double dMinCost = MAX_DOUBLE;

#if DISTORTION_TYPE_BUGFIX
  uint64_t ruiBits = std::numeric_limits<uint64_t>::max();
  uint64_t ruiDist = std::numeric_limits<uint64_t>::max();
#else
  uint64_t ruiBits = MAX_INT;
  uint64_t ruiDist = MAX_INT;
#endif

  uint64_t  uiOrigRate;
  uint64_t  uiOrigDist;
  double  dOrigCost;

  // calc original cost
  xCalcRDCostLuma( orgUnitBuf, cRecExtBuf, NULL, uiOrigRate, uiOrigDist, dOrigCost );

  m_pcBestAlfParam->alf_flag        = 0;
  m_pcBestAlfParam->cu_control_flag = 0;

  // initialize temp_alfps
  m_pcTempAlfParam->alf_flag        = 1;
  m_pcTempAlfParam->tapH            = tap;
  m_pcTempAlfParam->tapV            = tapV;
  m_pcTempAlfParam->num_coeff       = num_coef;
  m_pcTempAlfParam->chroma_idc      = 0;
  m_pcTempAlfParam->cu_control_flag = 0;


  // adaptive in-loop wiener filtering
  xEncALFLuma( orgUnitBuf, cRecExtBuf, recUnitBuf, uiMinRate, uiMinDist, dMinCost, cs.slice );

  if( !m_isGALF || !m_pSlice->isIntra())
  // cu-based filter on/off control
  xCheckCUAdaptation( cs, orgUnitBuf, cRecExtBuf, recUnitBuf, uiMinRate, uiMinDist, dMinCost );

  // adaptive tap-length
  xFilterTypeDecision( cs, orgUnitBuf, recExtBuf, recUnitBuf, uiMinRate, uiMinDist, dMinCost, cs.slice);

  // compute RD cost
  xCalcRDCostLuma( orgUnitBuf, recUnitBuf, m_pcBestAlfParam, uiMinRate, uiMinDist, dMinCost );

  // compare RD cost to non-ALF case
  if( dMinCost < dOrigCost )
  {
    m_pcBestAlfParam->alf_flag = 1;
    ruiBits = uiMinRate;
    ruiDist = uiMinDist;
  }
  else
  {
    m_pcBestAlfParam->alf_flag        = 0;
    m_pcBestAlfParam->cu_control_flag = 0;

    uiMinRate = uiOrigRate;
    uiMinDist = uiOrigDist;
    dMinCost  = dOrigCost;
    recUnitBuf.get( COMPONENT_Y ).copyFrom( recExtBuf.get( COMPONENT_Y ) );

    ruiBits = uiOrigRate;
    ruiDist = uiOrigDist;
  }

  // if ALF works
  if( m_pcBestAlfParam->alf_flag )
  {
    // do additional ALF process for chroma
    xEncALFChroma( uiMinRate, orgUnitBuf, recExtBuf, recUnitBuf, ruiDist, ruiBits , cs.slice );
  }
  // copy to best storage
  copyALFParam( pcAlfParam, m_pcBestAlfParam);


#if COM16_C806_ALF_TEMPPRED_NUM
  if (pcStoredAlfPara != NULL && iStoredAlfParaNum > 0)
  {
    m_bestPelBuf.copyFrom(recUnitBuf);
    //test stored ALF coefficients of both luma and chroma.
    for (uint32_t iAlfIdx = 0; iAlfIdx < iStoredAlfParaNum && iAlfIdx < C806_ALF_TEMPPRED_NUM; iAlfIdx++)
    {
      recUnitBuf.copyFrom(recExtBuf);
      ALFParam& pcStoredAlf = pcStoredAlfPara[iAlfIdx];

      if (xFilteringLumaChroma(cs, &pcStoredAlf, orgUnitBuf, recExtBuf, recUnitBuf, ruiBits, ruiDist, dMinCost, iAlfIdx, cs.slice))
      {
        pcAlfParam->temporalPredFlag = false;
        copyALFParam(pcAlfParam, m_pcBestAlfParam);
        pcAlfParam->temporalPredFlag = true;
        pcAlfParam->prevIdx = iAlfIdx;
        if (m_pcBestAlfParam->cu_control_flag)
        {
          pcAlfParam->alf_max_depth = m_pcBestAlfParam->alf_max_depth;
        }
      }
    }
    recUnitBuf.copyFrom(m_bestPelBuf);
  }


#endif

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_ALF, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "ALF" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}


// ====================================================================================================================
// LUMA
// ====================================================================================================================
void EncAdaptiveLoopFilter::xEncALFLuma( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiMinRate, uint64_t& ruiMinDist, double& rdMinCost, const Slice* pSlice)
{
  m_updateMatrix = true;
#if JVET_C0038_NO_PREV_FILTERS
  bFindBestFixedFilter = false;
#endif

  AlfFilterType filtType = ALF_FILTER_SYM_9;

  ALFParam   cFrmAlfParam;
  ALFParam* pcAlfParam = NULL;

  pcAlfParam = &(cFrmAlfParam);
  allocALFParam(pcAlfParam);
  resetALFParam(pcAlfParam);

  pcAlfParam->alf_flag          = 1;
  pcAlfParam->chroma_idc        = 0;
  pcAlfParam->cu_control_flag   = 0;
  pcAlfParam->filterType        = filtType;
  pcAlfParam->tapH              = AdaptiveLoopFilter::m_FilterTapsOfType[ filtType ];
  pcAlfParam->tapV              = AdaptiveLoopFilter::ALFTapHToTapV( pcAlfParam->tapH);
  pcAlfParam->num_coeff         = AdaptiveLoopFilter::ALFTapHToNumCoeff(pcAlfParam->tapH);
  pcAlfParam->filters_per_group = AdaptiveLoopFilter::m_NO_FILTERS;
  pcAlfParam->forceCoeff0       = 0;
  pcAlfParam->predMethod        = 0;

  xSetInitialMask( recExtBuf.get(COMPONENT_Y) );
  xStoreInBlockMatrix(orgUnitBuf, recExtBuf, filtType);
//  xFindFilterCoeffsLuma(orgUnitBuf, recExtBuf, filtType);
  if( m_isGALF )
  {
    xCheckFilterMergingGalf(cFrmAlfParam
  #if JVET_C0038_NO_PREV_FILTERS
      , orgUnitBuf, recExtBuf
  #endif
      );
    xFilterFrame_enGalf(m_tempPelBuf, recExtBuf, filtType
  #if COM16_C806_ALF_TEMPPRED_NUM
      , &cFrmAlfParam, true
  #endif
      , pSlice->clpRng( COMPONENT_Y )
      );
  }
  else
  {
    xCheckFilterMergingAlf(cFrmAlfParam
  #if JVET_C0038_NO_PREV_FILTERS
      , orgUnitBuf, recExtBuf
  #endif
      );
    xFilterFrame_enAlf(m_tempPelBuf, recExtBuf, filtType
  #if COM16_C806_ALF_TEMPPRED_NUM
      , &cFrmAlfParam, true
  #endif
      , pSlice->clpRng( COMPONENT_Y )
      );
  }


  PelBuf recLuma = recUnitBuf.get(COMPONENT_Y);
  const CPelBuf tmpLuma = m_tempPelBuf.get(COMPONENT_Y);
  recLuma.copyFrom( tmpLuma );

  copyALFParam( m_pcBestAlfParam, &cFrmAlfParam);

  xCalcRDCostLuma( orgUnitBuf,  m_tempPelBuf, pcAlfParam, ruiMinRate, ruiMinDist, rdMinCost );
  freeALFParam(&cFrmAlfParam);
}


// ====================================================================================================================
// CHROMA
// ====================================================================================================================

void EncAdaptiveLoopFilter::xstoreInBlockMatrixForChroma(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, int tap, int chroma_idc)
{
  Pel* ImgOrg;
  Pel* ImgDec;
  int i, j, k, l, varInd = 0, ii, jj;
  int x, y, yLocal;
  int fl = tap / 2;
  int flV = AdaptiveLoopFilter::ALFFlHToFlV(fl);
  int sqrFiltLength = AdaptiveLoopFilter::ALFTapHToNumCoeff(tap);
  int fl2 = 9 / 2; //extended size at each side of the frame
  int ELocal[m_MAX_SQR_FILT_LENGTH];
  int filtType = 0; //for chroma
  int iImgHeight = recExtBuf.get(COMPONENT_Cb).height;
  int iImgWidth = recExtBuf.get(COMPONENT_Cb).width;
  double **E, *yy;

  const int *p_pattern = m_patternTab[filtType];

  memset(m_pixAcc, 0, sizeof(double)*m_NO_VAR_BINS);
  {
    memset(m_yGlobalSym[filtType][varInd], 0, sizeof(double)*m_MAX_SQR_FILT_LENGTH);
    for (k = 0; k<sqrFiltLength; k++)
    {
      memset(m_EGlobalSym[filtType][varInd][k], 0, sizeof(double)*m_MAX_SQR_FILT_LENGTH);
    }
  }
  for (int iColorIdx = 0; iColorIdx < 2; iColorIdx++)
  {
    if ((iColorIdx == 0 && chroma_idc < 2) || (iColorIdx == 1 && (chroma_idc & 0x01) == 0))
    {
      continue;
    }
    ImgOrg = (Pel*)orgUnitBuf.get(ComponentID(iColorIdx + 1)).buf;
    ImgDec = (Pel*)recExtBuf.get(ComponentID(iColorIdx + 1)).buf;
    int Stride = recExtBuf.get(ComponentID(iColorIdx + 1)).stride;
    int StrideO = orgUnitBuf.get(ComponentID(iColorIdx + 1)).stride;

    for (i = 0, y = fl2; i < iImgHeight; i++, y++)
    {
      for (j = 0, x = fl2; j < iImgWidth; j++, x++)
      {
        varInd = 0;
        k = 0;
        memset(ELocal, 0, sqrFiltLength*sizeof(int));
        for (ii = -flV; ii < 0; ii++)
        {
          for (jj = -fl - ii; jj <= fl + ii; jj++)
          {
            ELocal[p_pattern[k++]] += (ImgDec[(i + ii)*Stride + (j + jj)] + ImgDec[(i - ii)*Stride + (j - jj)]);
          }
        }
        int iOffset = i * Stride + j;
        int iOffsetO = i * StrideO + j;
        for (jj = -fl; jj<0; jj++)
        {
          ELocal[p_pattern[k++]] += (ImgDec[iOffset + jj] + ImgDec[iOffset - jj]);
        }
        ELocal[p_pattern[k++]] += ImgDec[iOffset];
        yLocal = ImgOrg[iOffsetO];
        m_pixAcc[varInd] += (yLocal*yLocal);
        E = m_EGlobalSym[filtType][varInd];
        yy = m_yGlobalSym[filtType][varInd];
        for (k = 0; k<sqrFiltLength; k++)
        {
          for (l = k; l<sqrFiltLength; l++)
          {
            E[k][l] += (double)(ELocal[k] * ELocal[l]);
          }
          yy[k] += (double)(ELocal[k] * yLocal);
        }
      }
    }
  }
  // Matrix EGlobalSeq is symmetric, only part of it is calculated
  double **pE = m_EGlobalSym[filtType][varInd];
  for (k = 1; k < sqrFiltLength; k++)
  {
    for (l = 0; l < k; l++)
    {
      pE[k][l] = pE[l][k];
    }
  }
}

#if JVET_C0038_NO_PREV_FILTERS
double EncAdaptiveLoopFilter::xTestFixedFilterFast(double ***A, double **b, double *pixAcc, double *filterCoeffSym, double *filterCoeffDefault, int varInd)
{

  int j;
  double seFilt = 0, seOrig = 0, error, filterCoeffTemp[m_MAX_SQR_FILT_LENGTH];
  int sqrFiltLength = (m_MAX_SQR_FILT_LENGTH / 2 + 1);

  for (j = 0; j < sqrFiltLength; j++)
  {
    filterCoeffTemp[j] = filterCoeffSym[j] - filterCoeffDefault[j];
  }
  seOrig = pixAcc[varInd];
  seFilt = pixAcc[varInd] + xCalcErrorForGivenWeights(A[varInd], b[varInd], filterCoeffTemp, sqrFiltLength);

  error = 0;
  if (seFilt < seOrig)
  {
    error += seFilt;
  }
  else
  {
    error += seOrig;
  }
  return(error);
}

double EncAdaptiveLoopFilter::xTestFixedFilter(const Pel *imgY_Rec, const Pel *imgY_org, const Pel* imgY_append, int usePrevFilt[], int noVarBins, int orgStride, int recStride, int filtType)
{

  int i, j, varInd, pixelInt, temp;
  double seFilt[m_NO_VAR_BINS] ={0}, seOrig[m_NO_VAR_BINS] ={0}, error;
  int fl = m_FILTER_LENGTH / 2;
  int offset = (1 << (m_NUM_BITS - 2));
  const ClpRng clpRng = m_clpRngs.comp[ COMPONENT_Y ];

  for (i = fl; i < m_img_height + fl; i++)
  {
    for (j = fl; j < m_img_width + fl; j++)
    {
      if (m_maskBuf.at(j - fl, i - fl))
      {
        varInd = m_varImg[(i - fl)][(j - fl)];
        pixelInt = xFilterPixel(imgY_append, &varInd, m_filterCoeffFinal, NULL, i, j, fl, recStride, (AlfFilterType)filtType);
        pixelInt = ((pixelInt + offset) >> (m_NUM_BITS - 1));
        pixelInt = ClipPel( pixelInt, clpRng );

        int iOffsetOrg = (i - fl)*orgStride + (j - fl);
        int iOffsetRec = (i - fl)*recStride + (j - fl);
        temp = pixelInt - imgY_org[iOffsetOrg];
        seFilt[varInd] += temp*temp;
        temp = imgY_Rec[iOffsetRec] - imgY_org[iOffsetOrg];
        seOrig[varInd] += temp*temp;
      }
    }
  }

  error = 0;
  for (i = 0; i< noVarBins; i++)
  {
    if (seFilt[i] < seOrig[i])
    {
      usePrevFilt[i] = 1;
      error += seFilt[i];
    }
    else
    {
      usePrevFilt[i] = 0;
      error += seOrig[i];
    }
  }
  return(error);
}

void EncAdaptiveLoopFilter::xPreFilterFr(int** imgY_preFilter, const Pel* imgY_rec, const Pel * imgY_org, const Pel* imgY_append, int usePrevFilt[], int Stride, int filtType)
{

  int i, j;
  int fl = m_FILTER_LENGTH / 2;

  int temp = 0, pixelInt = 0, offset = (1 << (m_NUM_BITS - 2));
  const ClpRng clpRng = m_clpRngs.comp[COMPONENT_Y];


  for (i = fl; i < m_img_height + fl; i++)
  {
    for (j = fl; j < m_img_width + fl; j++)
    {
      int varInd = m_varImg[(i - fl)][(j - fl)];
      int varIndAfterMapping = selectTransposeVarInd(varInd, &temp);
      if (m_maskBuf.at(j - fl, i - fl) && usePrevFilt[varIndAfterMapping] > 0)
      {
        pixelInt = xFilterPixel(imgY_append, &varInd, m_filterCoeffFinal, NULL, i, j, fl, Stride, (AlfFilterType)filtType);
        pixelInt = ((pixelInt + offset) >> (m_NUM_BITS - 1));
        imgY_preFilter[(i - fl)][(j - fl)] = ClipPel(pixelInt, clpRng);
      }
      else
      {
        imgY_preFilter[(i - fl)][(j - fl)] = imgY_rec[(i - fl)*Stride + (j - fl)];
      }
    }
  }
  }
#endif
void EncAdaptiveLoopFilter::xEncALFChroma(uint64_t uiLumaRate, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiDist, uint64_t& ruiBits, const Slice* pSlice)
{
  // restriction for non-referenced B-slice
  if( m_eSliceType == B_SLICE && m_iPicNalReferenceIdc == 0 )
  {
    return;
  }

  int tap, num_coef;

  // set global variables
  tap         = m_ALF_MAX_NUM_TAP_C;
  if( m_isGALF )
  {
    num_coef = AdaptiveLoopFilter::ALFTapHToNumCoeff(tap);
  }
  else
  {
    num_coef    = (tap*tap+1)>>1;
    num_coef    = num_coef + 1; // DC offset
  }

  // set min cost
  uint64_t uiMinRate = uiLumaRate;
#if DISTORTION_TYPE_BUGFIX
  uint64_t uiMinDist = std::numeric_limits<uint64_t>::max();
#else
  uint64_t uiMinDist = MAX_INT;
#endif
  double dMinCost  = MAX_DOUBLE;

  // calc original cost
  copyALFParam( m_pcTempAlfParam, m_pcBestAlfParam );
  xCalcRDCostChroma( orgUnitBuf, recUnitBuf, m_pcTempAlfParam, uiMinRate, uiMinDist, dMinCost );

  // initialize temp_alfps
  m_pcTempAlfParam->chroma_idc = 3;
  m_pcTempAlfParam->tap_chroma       = tap;
  m_pcTempAlfParam->num_coeff_chroma = num_coef;

  // Adaptive in-loop wiener filtering for chroma
  xFilteringFrameChroma(orgUnitBuf, recExtBuf, recUnitBuf );

  // filter on/off decision for chroma
  uint64_t uiFiltDistCb = xCalcSSD( orgUnitBuf, recUnitBuf, COMPONENT_Cb);
  uint64_t uiFiltDistCr = xCalcSSD( orgUnitBuf, recUnitBuf, COMPONENT_Cr);
  uint64_t uiOrgDistCb  = xCalcSSD( orgUnitBuf, recExtBuf, COMPONENT_Cb);
  uint64_t uiOrgDistCr  = xCalcSSD( orgUnitBuf, recExtBuf, COMPONENT_Cr);

  m_pcTempAlfParam->chroma_idc = 0;
  if(uiOrgDistCb > uiFiltDistCb)
  {
    m_pcTempAlfParam->chroma_idc += 2;
  }
  if(uiOrgDistCr  > uiFiltDistCr )
  {
    m_pcTempAlfParam->chroma_idc += 1;
  }

  if(m_pcTempAlfParam->chroma_idc )
  {
    if(m_pcTempAlfParam->chroma_idc !=3 )
    {
      // chroma filter re-design
      xFilteringFrameChroma( orgUnitBuf, recExtBuf, recUnitBuf );
    }

    uint64_t uiRate, uiDist;
    double dCost;
    xCalcRDCostChroma( orgUnitBuf, recUnitBuf, m_pcTempAlfParam, uiRate, uiDist, dCost );

    if( dCost < dMinCost )
    {
      copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
      if( ! m_isGALF )
      {
        predictALFCoeffChroma(m_pcBestAlfParam);
      }
      ruiBits += uiRate;
      ruiDist += uiDist;
    }
    else
    {
      m_pcBestAlfParam->chroma_idc = 0;
      if( (m_pcTempAlfParam->chroma_idc>>1) & 0x01 )
      {
        recUnitBuf.get(COMPONENT_Cb).copyFrom( recExtBuf.get(COMPONENT_Cb) );
      }
      if( m_pcTempAlfParam->chroma_idc & 0x01 )
      {
        recUnitBuf.get(COMPONENT_Cr).copyFrom( recExtBuf.get(COMPONENT_Cr) );
      }
      ruiBits += uiMinRate;
      ruiDist += uiMinDist;
    }
  }
  else
  {
    m_pcBestAlfParam->chroma_idc = 0;

    ruiBits += uiMinRate;
    ruiDist += uiMinDist;
    recUnitBuf.get(COMPONENT_Cb).copyFrom( recExtBuf.get(COMPONENT_Cb) );
    recUnitBuf.get(COMPONENT_Cr).copyFrom( recExtBuf.get(COMPONENT_Cr) );
  }
}

void EncAdaptiveLoopFilter::xInitFixedFilters()
{
  int maxFilterLength = m_MAX_SQR_FILT_LENGTH / 2 + 1;
#if JVET_C0038_NO_PREV_FILTERS
  int factor = (1 << (m_NUM_BITS - 1));
  for (int i = 0; i < maxFilterLength; i++)
  {
    for (int j = 0; j < m_NO_FILTERS*JVET_C0038_NO_PREV_FILTERS; j++)
    {
      m_filterCoeffPrev[j][i] = (double)m_ALFfilterCoeffFixed[j][i] / (double)factor;
    }
  }
#endif
  memset(m_filterCoeffDefault, 0, (maxFilterLength - 1)*sizeof(double));
  m_filterCoeffDefault[maxFilterLength - 1] = 1.0;
}

void EncAdaptiveLoopFilter::xCheckReUseFilterSet( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& dstUnitBuf, double& rdMinCost, ALFParam& filterSet, int filterSetIdx )
{
  ALFParam cFrmAlfParam;
  allocALFParam(&cFrmAlfParam);
  copyALFParam( &cFrmAlfParam, &filterSet );

  cFrmAlfParam.alf_flag         = 1;
  cFrmAlfParam.temporalPredFlag = true;
  cFrmAlfParam.prevIdx          = filterSetIdx;

  cFrmAlfParam.cu_control_flag  = 0;
  cFrmAlfParam.chroma_idc       = 0;

  double dCost  = 0;
  uint64_t uiRate = 0;
  uint64_t uiDist = 0;

  xFilterFrame_en(dstUnitBuf, recExtBuf, cFrmAlfParam, cs.slice->clpRng(COMPONENT_Y));
  uiDist = xCalcSSD(orgUnitBuf, dstUnitBuf, COMPONENT_Y);
  xCalcRDCost( uiDist, &cFrmAlfParam, uiRate, dCost );

  if (dCost < rdMinCost)
  {
    rdMinCost = dCost;
    copyALFParam(m_pcBestAlfParam, &cFrmAlfParam );
    m_bestPelBuf.get(COMPONENT_Y).copyFrom(dstUnitBuf.get(COMPONENT_Y));
  }

  cFrmAlfParam.cu_control_flag = 1;

  for( uint32_t uiDepth = 0; uiDepth < m_uiMaxTotalCUDepth; uiDepth++ )
  {
    m_tempPelBuf.get(COMPONENT_Y).copyFrom(dstUnitBuf.get(COMPONENT_Y));
    copyALFParam( m_pcTempAlfParam, &cFrmAlfParam);
    m_pcTempAlfParam->alf_max_depth = uiDepth;

    xSetCUAlfCtrlFlags( cs, orgUnitBuf, recExtBuf, m_tempPelBuf, uiDist, uiDepth, m_pcTempAlfParam); //set up varImg here
    xCalcRDCost( uiDist, m_pcTempAlfParam, uiRate, dCost );
    if (dCost < rdMinCost )
    {
      rdMinCost  = dCost;
      m_bestPelBuf.get(COMPONENT_Y).copyFrom( m_tempPelBuf.get(COMPONENT_Y) );
      copyALFParam( m_pcBestAlfParam, m_pcTempAlfParam );
    }
  }

  freeALFParam(&cFrmAlfParam);

  if( m_pcBestAlfParam->temporalPredFlag )
  {
    if( filterSet.chroma_idc != 0 )
    {
      //CHECK CHROMA
      copyALFParam( m_pcTempAlfParam, m_pcBestAlfParam );

      xFilteringFrameChroma( orgUnitBuf, recExtBuf, dstUnitBuf );

      uint64_t uiOrgDistCb  = xCalcSSD(orgUnitBuf, recExtBuf, COMPONENT_Cb);
      uint64_t uiFiltDistCb = xCalcSSD(orgUnitBuf, dstUnitBuf, COMPONENT_Cb);

      if( uiFiltDistCb < uiOrgDistCb )
      {
        m_pcBestAlfParam->chroma_idc |= 2;
        m_bestPelBuf.get(COMPONENT_Cb).copyFrom( dstUnitBuf.get(COMPONENT_Cb ) );
      }
      else
      {
        m_bestPelBuf.get(COMPONENT_Cb).copyFrom( recExtBuf.get(COMPONENT_Cb ) );
      }

      uint64_t uiOrgDistCr  = xCalcSSD(orgUnitBuf, recExtBuf, COMPONENT_Cr);
      uint64_t uiFiltDistCr = xCalcSSD(orgUnitBuf, dstUnitBuf, COMPONENT_Cr);

      if(uiOrgDistCr  > uiFiltDistCr )
      {
        m_pcBestAlfParam->chroma_idc |= 1;
        m_bestPelBuf.get(COMPONENT_Cr).copyFrom( dstUnitBuf.get(COMPONENT_Cr) );
      }
      else
      {
        m_bestPelBuf.get(COMPONENT_Cr).copyFrom( recExtBuf.get(COMPONENT_Cr) );
      }
    }
    else
    {
      m_bestPelBuf.get(COMPONENT_Cb).copyFrom( recExtBuf.get(COMPONENT_Cb) );
      m_bestPelBuf.get(COMPONENT_Cr).copyFrom( recExtBuf.get(COMPONENT_Cr) );
    }
  }
}


// ====================================================================================================================
// Private member functions
// ====================================================================================================================

void EncAdaptiveLoopFilter::xInitParam()
{
  int i, j;

  if (m_ppdAlfCorr != NULL)
  {
    for (i = 0; i < m_ALF_MAX_NUM_COEF; i++)
    {
      for (j = 0; j < m_ALF_MAX_NUM_COEF+1; j++)
      {
        m_ppdAlfCorr[i][j] = 0;
      }
    }
  }
  else
  {
    m_ppdAlfCorr = new double*[m_ALF_MAX_NUM_COEF];
    for (i = 0; i < m_ALF_MAX_NUM_COEF; i++)
    {
      m_ppdAlfCorr[i] = new double[m_ALF_MAX_NUM_COEF+1];
      for (j = 0; j < m_ALF_MAX_NUM_COEF+1; j++)
      {
        m_ppdAlfCorr[i][j] = 0;
      }
    }
  }

  if (m_pdDoubleAlfCoeff != NULL)
  {
    for (i = 0; i < m_ALF_MAX_NUM_COEF; i++)
    {
      m_pdDoubleAlfCoeff[i] = 0;
    }
  }
  else
  {
    m_pdDoubleAlfCoeff = new double[m_ALF_MAX_NUM_COEF];
    for (i = 0; i < m_ALF_MAX_NUM_COEF; i++)
    {
      m_pdDoubleAlfCoeff[i] = 0;
    }
  }
}

void EncAdaptiveLoopFilter::xUninitParam()
{
  int i;

  if (m_ppdAlfCorr != NULL)
  {
    for (i = 0; i < m_ALF_MAX_NUM_COEF; i++)
    {
      delete[] m_ppdAlfCorr[i];
      m_ppdAlfCorr[i] = NULL;
    }
    delete[] m_ppdAlfCorr;
    m_ppdAlfCorr = NULL;
  }

  if (m_pdDoubleAlfCoeff != NULL)
  {
    delete[] m_pdDoubleAlfCoeff;
    m_pdDoubleAlfCoeff = NULL;
  }
}


//********************************
// DERIVING FILTER COEFFICIENTS
//********************************

//COEFF-STORAGES:
// m_filterCoeff
// m_filterCoeffQuant
// m_filterCoeffSym
// m_filterCoeffSymQuant
// m_filterCoeffPrevSelected

//TODO needs cleanup:
//NOT USED IN THIS VERSION / Coefficents are calculated in xCheckFilterMerging
void EncAdaptiveLoopFilter::xFindFilterCoeffsLuma(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
  , ALFParam* pcAlfParam
#endif
  )
{

  int filters_per_fr, sqrFiltLength;

  double **ySym, ***ESym;

  sqrFiltLength = m_sqrFiltLengthTab[filtType];

  ESym = m_EGlobalSym[ filtType ];
  ySym = m_yGlobalSym[ filtType ];

  xStoreInBlockMatrix(orgUnitBuf, recExtBuf, filtType);

  filters_per_fr = AdaptiveLoopFilter::m_NO_FILTERS;

  memset(m_varIndTab,0,sizeof(int)*m_NO_VAR_BINS);
  for( int filtIdx = 0; filtIdx < filters_per_fr; filtIdx++)
  {
    xSolveAndQuant( m_filterCoeff, m_filterCoeffQuant, ESym[filtIdx], ySym[filtIdx], sqrFiltLength, m_weightsTab[filtType], m_NUM_BITS );
    for( int k = 0; k < sqrFiltLength; k++)
    {
      m_filterCoeffSym     [filtIdx][k] = m_filterCoeffQuant[k];
      m_filterCoeffSymQuant[filtIdx][k] = m_filterCoeffQuant[k];
    }
    m_varIndTab[filtIdx] = filtIdx;
  }

  xcalcPredFilterCoeff(filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , pcAlfParam
#endif
    );
}

double EncAdaptiveLoopFilter::xCalcFilterCoeffsGalf( double     ***EGlobalSeq,
                                                     double      **yGlobalSeq,
                                                     double       *pixAccGlobalSeq,
                                                     int           interval[m_NO_VAR_BINS],
                                                     int           filters_per_fr,
                                                     AlfFilterType filtType,
                                                     double    errorTabForce0Coeff[m_NO_VAR_BINS][2])
{
  int sqrFiltLength  = m_sqrFiltLengthTab[ filtType ];
  const int* weights = m_weightsTab[ filtType ];

  double error;
  int k;

  error = 0;
  for(int filtIdx = 0; filtIdx < filters_per_fr; filtIdx++ )
  {
    add_A_galf(m_E_temp, EGlobalSeq, interval, filtIdx, sqrFiltLength);
    add_b_galf(m_y_temp, yGlobalSeq, interval, filtIdx, sqrFiltLength);
    pixAcc_temp0D = 0;
    for (k = 0; k < m_NO_VAR_BINS; k++)
    {
      if (interval[k] == filtIdx)
      {
        pixAcc_temp0D += pixAccGlobalSeq[k];
      }
    }
    // Find coeffcients
    errorTabForce0Coeff[filtIdx][1] = pixAcc_temp0D + xSolveAndQuant(m_filterCoeff, m_filterCoeffQuant, m_E_temp, m_y_temp, sqrFiltLength, weights, m_NUM_BITS );
    errorTabForce0Coeff[filtIdx][0] = pixAcc_temp0D;
    error += errorTabForce0Coeff[filtIdx][1];

    for(k = 0; k < sqrFiltLength; k++)
    {
      m_filterCoeffSym[filtIdx][k]      = m_filterCoeffQuant[k];
      m_filterCoeffSymQuant[filtIdx][k] = m_filterCoeffQuant[k];
    }
  }
  return(error);
}

double EncAdaptiveLoopFilter::xCalcFilterCoeffs( double     ***EGlobalSeq,
                                                 double      **yGlobalSeq,
                                                 double       *pixAccGlobalSeq,
                                                 int           intervalBest[m_NO_VAR_BINS][2],
                                                 int           filters_per_fr,
                                                 AlfFilterType filtType,
                                                 int     **filterCoeffSeq,
                                                 int     **filterCoeffQuantSeq,
                                                 double    errorTabForce0Coeff[m_NO_VAR_BINS][2] )
{

  double pixAcc_temp;

  int sqrFiltLength  = m_sqrFiltLengthTab[ filtType ];
  const int* weights = m_weightsTab[ filtType ];

  double error;
  int k;

  error = 0;
  for(int filtIdx = 0; filtIdx < filters_per_fr; filtIdx++ )
  {
    add_A(m_E_temp, EGlobalSeq, intervalBest[filtIdx][0], intervalBest[filtIdx][1], sqrFiltLength);
    add_b(m_y_temp, yGlobalSeq, intervalBest[filtIdx][0], intervalBest[filtIdx][1], sqrFiltLength);
    pixAcc_temp = 0;
    for (k = intervalBest[filtIdx][0]; k <= intervalBest[filtIdx][1]; k++)
    {
      pixAcc_temp += pixAccGlobalSeq[k];
    }
    // Find coeffcients
    errorTabForce0Coeff[filtIdx][1] = pixAcc_temp + xSolveAndQuant(m_filterCoeff, m_filterCoeffQuant, m_E_temp, m_y_temp, sqrFiltLength, weights, m_NUM_BITS );
    errorTabForce0Coeff[filtIdx][0] = pixAcc_temp;
    error += errorTabForce0Coeff[filtIdx][1];

    for(k = 0; k < sqrFiltLength; k++)
    {
      filterCoeffSeq[filtIdx][k]      = m_filterCoeffQuant[k];
      filterCoeffQuantSeq[filtIdx][k] = m_filterCoeffQuant[k];
    }
  }
  return(error);
}

#if FORCE0
double EncAdaptiveLoopFilter::xCalcDistForce0(
  int           filters_per_fr,
  AlfFilterType filtType,
  int           sqrFiltLength,
  double        errorTabForce0Coeff[m_NO_VAR_BINS][2],
  double        lambda,
  int           codedVarBins[m_NO_VAR_BINS])
{
  double distForce0;
  int bitsVarBin[m_NO_VAR_BINS];

  xcollectStatCodeFilterCoeffForce0(m_filterCoeffSymQuant, filtType, sqrFiltLength,
    filters_per_fr, bitsVarBin);

  xdecideCoeffForce0(codedVarBins, &distForce0, errorTabForce0Coeff, bitsVarBin, lambda, filters_per_fr);

  return distForce0;
}
#endif

void EncAdaptiveLoopFilter::xStoreInBlockMatrix(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType)
{
  const CPelBuf orgLuma = orgUnitBuf.get(COMPONENT_Y);
  const Pel*    orgBuf = orgLuma.buf;
  const int     orgStride = orgLuma.stride;

  const CPelBuf recExtLuma = recExtBuf.get(COMPONENT_Y);
  const Pel*    recBufExt = recExtLuma.buf;
  const int     recStrideExt = recExtLuma.stride;

  if (!m_isGALF || m_updateMatrix)
  {
    int var_step_size_w = m_ALF_VAR_SIZE_W;
    int var_step_size_h = m_ALF_VAR_SIZE_H;

    int tap = m_mapTypeToNumOfTaps[filtType];

    int i, j, k, l, varInd;
    int x, y;
    int fl = tap / 2;
    int flV = AdaptiveLoopFilter::ALFFlHToFlV(fl);
    int sqrFiltLength = AdaptiveLoopFilter::ALFTapHToNumCoeff(tap);
    int fl2 = 9 / 2; //extended size at each side of the frame
    int ELocal[m_MAX_SQR_FILT_LENGTH];
    int yLocal;

    double **E, *yy;
    int count_valid = 0;


    const int *p_pattern = m_patternTab[filtType];

    memset(m_pixAcc, 0, sizeof(double) * m_NO_VAR_BINS);

    for (varInd = 0; varInd<m_NO_VAR_BINS; varInd++)
    {
      memset(m_yGlobalSym[filtType][varInd], 0, sizeof(double)*m_MAX_SQR_FILT_LENGTH);
      for (k = 0; k<sqrFiltLength; k++)
      {
        memset(m_EGlobalSym[filtType][varInd][k], 0, sizeof(double)*m_MAX_SQR_FILT_LENGTH);
      }
    }

    for (i = fl2; i < m_img_height + fl2; i++)
    {
      for (j = fl2; j < m_img_width + fl2; j++)
      {
        if (m_maskBuf.at(j - fl2, i - fl2) == 1) //[i-fl2][j-fl2]
        {
          count_valid++;
        }
      }
    }

    for (i = 0, y = fl2; i<m_img_height; i++, y++)
    {
      for (j = 0, x = fl2; j<m_img_width; j++, x++)
      {
        int condition = (m_maskBuf.at(j, i) == 0 && count_valid > 0);
        if (!condition)
        {
          k = 0;
          memset(ELocal, 0, sqrFiltLength*sizeof(int));
          if (m_isGALF)
          {
            varInd = m_varImg[i][j];
            int transpose = 0;
            int varIndMod = selectTransposeVarInd(varInd, &transpose);
            yLocal = orgBuf[(i)*orgStride + (j)] - recBufExt[(i)*recStrideExt + (j)];
            calcMatrixE(ELocal, recBufExt, p_pattern, i, j, flV, fl, transpose, recStrideExt);
            E = m_EGlobalSym[filtType][varIndMod];
            yy = m_yGlobalSym[filtType][varIndMod];

            for (k = 0; k < sqrFiltLength; k++)
            {
              for (l = k; l < sqrFiltLength; l++)
              {
                E[k][l] += (double)(ELocal[k] * ELocal[l]);
              }
              yy[k] += (double)(ELocal[k] * yLocal);
            }
            m_pixAcc[varIndMod] += (yLocal*yLocal);
          }
          else
          {
            varInd = m_varImg[i / var_step_size_h][j / var_step_size_w];
            for (int ii = -flV; ii < 0; ii++)
            {
              for (int jj = -fl - ii; jj <= fl + ii; jj++)
              {
                ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + (j + jj)] + recBufExt[(i - ii)*recStrideExt + (j - jj)]);
              }
            }
            for (int jj = -fl; jj<0; jj++)
            {
              ELocal[p_pattern[k++]] += (recBufExt[(i)*recStrideExt + (j + jj)] + recBufExt[(i)*recStrideExt + (j - jj)]);
            }
            ELocal[p_pattern[k++]] += recBufExt[(i)*recStrideExt + (j)];
            ELocal[sqrFiltLength - 1] = 1;
            yLocal = orgBuf[(i)*orgStride + (j)];

            m_pixAcc[varInd] += (yLocal*yLocal);
            E = m_EGlobalSym[filtType][varInd];
            yy = m_yGlobalSym[filtType][varInd];

            for (k = 0; k<sqrFiltLength; k++)
            {
              for (l = k; l<sqrFiltLength; l++)
                E[k][l] += (double)(ELocal[k] * ELocal[l]);
              yy[k] += (double)(ELocal[k] * yLocal);
            }
          }
        }
      }
    }
    // Matrix EGlobalSeq is symmetric, only part of it is calculated
    for (varInd = 0; varInd<m_NO_VAR_BINS; varInd++)
    {
      double **pE = m_EGlobalSym[filtType][varInd];
      for (k = 1; k<sqrFiltLength; k++)
      {
        for (l = 0; l<k; l++)
        {
          pE[k][l] = pE[l][k];
        }
      }
    }
    }
  else
  {
    CHECK(filtType == 2, "filterType has to be 0 or 1!");
    for (int varInd = 0; varInd < m_NO_VAR_BINS; varInd++)
    {
      xDeriveGlobalEyFromLgrTapFilter(m_EGlobalSym[2][varInd], m_yGlobalSym[2][varInd], m_EGlobalSym[filtType][varInd], m_yGlobalSym[filtType][varInd], m_patternMapTab[2], m_patternMapTab[filtType]);
    }
  }
}

void EncAdaptiveLoopFilter::calcMatrixE(int *ELocal, const Pel *recBufExt, const int *p_pattern, int i, int j, int flV, int fl, int transpose, int recStrideExt)
{
  int ii, jj, k = 0;

  if (transpose == 0)
  {
    for (ii = -flV; ii < 0; ii++)
    {
      for (jj = -fl - ii; jj <= fl + ii; jj++)
      {
        ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + (j + jj)] + recBufExt[(i - ii)*recStrideExt + (j - jj)]);
      }
    }
    for (jj = -fl; jj<0; jj++)
    {
      ELocal[p_pattern[k++]] += (recBufExt[(i)*recStrideExt + (j + jj)] + recBufExt[(i)*recStrideExt + (j - jj)]);
    }
    ELocal[p_pattern[k++]] += recBufExt[(i)*recStrideExt + (j)];
  }
  else if (transpose == 1)
  {
    for (jj = -flV; jj < 0; jj++)
    {
      for (ii = -fl - jj; ii <= fl + jj; ii++)
      {
        ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + (j + jj)] + recBufExt[(i - ii)*recStrideExt + (j - jj)]);
      }
    }
    for (ii = -fl; ii<0; ii++)
    {
      ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + j] + recBufExt[(i - ii)*recStrideExt + j]);
    }
    ELocal[p_pattern[k++]] += recBufExt[(i)*recStrideExt + (j)];
  }
  else if (transpose == 2)
  {
    for (ii = -flV; ii < 0; ii++)
    {
      for (jj = fl + ii; jj >= -fl - ii; jj--)
      {
        ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + (j + jj)] + recBufExt[(i - ii)*recStrideExt + (j - jj)]);
      }
    }
    for (jj = -fl; jj<0; jj++)
    {
      ELocal[p_pattern[k++]] += (recBufExt[(i)*recStrideExt + (j + jj)] + recBufExt[(i)*recStrideExt + (j - jj)]);
    }
    ELocal[p_pattern[k++]] += recBufExt[(i)*recStrideExt + (j)];
  }
  else
  {
    for (jj = -flV; jj < 0; jj++)
    {
      for (ii = fl + jj; ii >= -fl - jj; ii--)
      {
        ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + (j + jj)] + recBufExt[(i - ii)*recStrideExt + (j - jj)]);
      }
    }
    for (ii = -fl; ii<0; ii++)
    {
      ELocal[p_pattern[k++]] += (recBufExt[(i + ii)*recStrideExt + j] + recBufExt[(i - ii)*recStrideExt + j]);
    }
    ELocal[p_pattern[k++]] += recBufExt[(i)*recStrideExt + (j)];
  }
}

int EncAdaptiveLoopFilter::xFilterPixel(const Pel *ImgDec, int* varIndBeforeMapping, int **filterCoeffSym, int *pattern, int i, int j, int fl, int Stride, AlfFilterType filtNo)
{
  int varInd;
  int transpose;
  const Pel *imgY_rec = ImgDec;
  const Pel *p_imgY_pad, *p_imgY_pad0;
  varInd = selectTransposeVarInd((*varIndBeforeMapping), &transpose);
  (*varIndBeforeMapping) = varInd;
  int *coef = filterCoeffSym == NULL ? m_filterCoeffPrevSelected[varInd] : filterCoeffSym[varInd];
  int pixelInt = 0;

  if (filterCoeffSym == NULL)
  {
    if (transpose == 1)
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[38] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[30] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[39] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[32] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[22] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[31] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[40] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[37] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[29] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[33] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[21] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[23] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[13] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);
      }
      else
      {
        pixelInt += coef[36] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[28] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[37] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[34] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[20] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[29] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[33] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[24] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[12] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[21] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[23] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[14] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[4] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);
      }
    }
    else if (transpose == 3)
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[38] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[32] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[39] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[30] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[22] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[31] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[40] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[37] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[33] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[29] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[23] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[21] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[13] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);

      }
      else
      {
        pixelInt += coef[36] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[34] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[37] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[28] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[24] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[33] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[29] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[20] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[14] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[23] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[21] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[4] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);

      }
    }
    else if (transpose == 2)
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[22] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[32] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[31] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[30] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[38] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[39] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[40] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[13] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[23] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[21] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[33] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[29] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[37] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);

      }
      else
      {
        pixelInt += coef[4] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[14] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[24] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[23] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[21] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[20] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[34] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[33] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[29] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[28] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[36] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[37] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);
      }
    }
    else
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[22] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[30] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[31] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[32] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[38] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[39] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[40] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[13] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[21] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[23] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[29] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[33] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[37] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);

      }
      else
      {
        pixelInt += coef[4] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[12] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[14] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[20] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[21] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[22] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[23] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[24] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[28] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[29] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[30] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[31] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[32] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[33] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[34] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[36] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[37] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[38] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[39] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[40] * (p_imgY_pad[j - fl]);

      }
    }
  }
  else
  {
    //test prev filter
    if (transpose == 1)
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[4] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[1] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[5] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[3] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[0] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[2] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[6] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[9] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[4] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[8] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[1] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[3] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[0] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl]);

      }
      else
      {
        pixelInt += coef[16] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[9] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[17] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[15] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[4] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[18] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[14] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[8] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[1] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[19] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[3] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[0] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[20] * (p_imgY_pad[j - fl]);
      }
    }
    else if (transpose == 3)
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[4] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[3] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[5] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[1] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[0] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[2] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[6] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[9] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[8] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[4] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[3] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[1] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[0] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl]);
      }
      else
      {
        pixelInt += coef[16] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[15] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[17] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[9] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[8] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[14] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[18] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[4] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[3] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[19] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[1] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[0] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[20] * (p_imgY_pad[j - fl]);
      }
    }
    else if (transpose == 2)
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[0] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[3] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[2] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[1] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[4] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[5] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[6] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[0] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[3] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[1] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[8] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[4] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[9] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl]);

      }
      else
      {
        pixelInt += coef[0] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[3] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[1] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[8] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[4] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[15] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[14] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[9] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[16] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[17] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[18] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[19] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[20] * (p_imgY_pad[j - fl]);
      }
    }
    else
    {
      if (filtNo == 0) //5x5
      {
        pixelInt += coef[0] * (imgY_rec[(i - fl + 2)*Stride + j - fl] + imgY_rec[(i - fl - 2)*Stride + j - fl]);

        pixelInt += coef[1] * (imgY_rec[(i - fl + 1)*Stride + j - fl + 1] + imgY_rec[(i - fl - 1)*Stride + j - fl - 1]);
        pixelInt += coef[2] * (imgY_rec[(i - fl + 1)*Stride + j - fl] + imgY_rec[(i - fl - 1)*Stride + j - fl]);
        pixelInt += coef[3] * (imgY_rec[(i - fl + 1)*Stride + j - fl - 1] + imgY_rec[(i - fl - 1)*Stride + j - fl + 1]);

        pixelInt += coef[4] * (imgY_rec[(i - fl)*Stride + j - fl - 2] + imgY_rec[(i - fl)*Stride + j - fl + 2]);
        pixelInt += coef[5] * (imgY_rec[(i - fl)*Stride + j - fl - 1] + imgY_rec[(i - fl)*Stride + j - fl + 1]);
        pixelInt += coef[6] * (imgY_rec[(i - fl)*Stride + j - fl]);
      }
      else if (filtNo == 1) //7x7
      {
        pixelInt += coef[0] * (imgY_rec[(i - fl + 3)*Stride + j - fl] + imgY_rec[(i - fl - 3)*Stride + j - fl]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[1] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[3] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[4] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[8] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[9] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl]);
      }
      else
      {
        pixelInt += coef[0] * (imgY_rec[(i - fl + 4)*Stride + j - fl] + imgY_rec[(i - fl - 4)*Stride + j - fl]);
        p_imgY_pad = imgY_rec + (i - fl + 3)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 3)*Stride;
        pixelInt += coef[1] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[2] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[3] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);

        p_imgY_pad = imgY_rec + (i - fl + 2)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 2)*Stride;
        pixelInt += coef[4] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[5] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[6] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[7] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[8] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);

        p_imgY_pad = imgY_rec + (i - fl + 1)*Stride;
        p_imgY_pad0 = imgY_rec + (i - fl - 1)*Stride;
        pixelInt += coef[9] * (p_imgY_pad[j - fl + 3] + p_imgY_pad0[j - fl - 3]);
        pixelInt += coef[10] * (p_imgY_pad[j - fl + 2] + p_imgY_pad0[j - fl - 2]);
        pixelInt += coef[11] * (p_imgY_pad[j - fl + 1] + p_imgY_pad0[j - fl - 1]);
        pixelInt += coef[12] * (p_imgY_pad[j - fl] + p_imgY_pad0[j - fl]);
        pixelInt += coef[13] * (p_imgY_pad[j - fl - 1] + p_imgY_pad0[j - fl + 1]);
        pixelInt += coef[14] * (p_imgY_pad[j - fl - 2] + p_imgY_pad0[j - fl + 2]);
        pixelInt += coef[15] * (p_imgY_pad[j - fl - 3] + p_imgY_pad0[j - fl + 3]);

        p_imgY_pad = imgY_rec + (i - fl)*Stride;
        pixelInt += coef[16] * (p_imgY_pad[j - fl + 4] + p_imgY_pad[j - fl - 4]);
        pixelInt += coef[17] * (p_imgY_pad[j - fl + 3] + p_imgY_pad[j - fl - 3]);
        pixelInt += coef[18] * (p_imgY_pad[j - fl + 2] + p_imgY_pad[j - fl - 2]);
        pixelInt += coef[19] * (p_imgY_pad[j - fl + 1] + p_imgY_pad[j - fl - 1]);
        pixelInt += coef[20] * (p_imgY_pad[j - fl]);
      }
    }
  }
  return(pixelInt);
}


void EncAdaptiveLoopFilter::xcalcPredFilterCoeff(AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
  , ALFParam* alfParam
#endif
  )
{
  if( m_isGALF )
  {
    int varInd, i, k;

    const int *patternMap = m_patternMapTab[filtType];
    const int *weights = AdaptiveLoopFilter::m_weightsTab[2];
    int quantCoeffSum = 0;
    int factor = (1 << (AdaptiveLoopFilter::m_NUM_BITS - 1));

    for (varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
    {
      k=0;
      quantCoeffSum = 0;

      for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
      {
        if (m_pattern9x9Sym_Quart[i]>0 && patternMap[i]>0)
        {
          m_filterCoeffPrevSelected[varInd][i] = (m_filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i] - 1] + m_filterCoeffSym[m_varIndTab[varInd]][patternMap[i] - 1]);
          if (i != (m_MAX_SQR_FILT_LENGTH - 1))
          {
            quantCoeffSum += (weights[m_pattern9x9Sym_Quart[i] - 1] * m_filterCoeffPrevSelected[varInd][i]);
          }
          k++;
        }
        else if (m_pattern9x9Sym_Quart[i] > 0)
        {
          m_filterCoeffPrevSelected[varInd][i] = m_filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i] - 1];
          if (i != (m_MAX_SQR_FILT_LENGTH - 1))
          {
            quantCoeffSum += (weights[m_pattern9x9Sym_Quart[i] - 1] * m_filterCoeffPrevSelected[varInd][i]);
          }
        }
        else
        {
          m_filterCoeffPrevSelected[varInd][i] = 0;
        }
      }
      m_filterCoeffPrevSelected[varInd][m_MAX_SQR_FILT_LENGTH - 1] = factor - quantCoeffSum;
    }

  #if COM16_C806_ALF_TEMPPRED_NUM

    patternMap = m_patternMapTab[2];
    for (int varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
    {
      int k = 0;
      for (int i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
      {
        if (patternMap[i] > 0)
        {
          alfParam->alfCoeffLuma[varInd][k] = m_filterCoeffPrevSelected[varInd][i];
          k++;
        }
      }
    }
  #endif
  }
  else
  {
    const int* patternMap = m_patternMapTab[filtType];
    for( int varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
    {
      int k=0;
      for( int i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
      {
        if (patternMap[i]>0)
        {
          m_filterCoeffPrevSelected[varInd][i]=m_filterCoeffSym[m_varIndTab[varInd]][k];
          alfParam->alfCoeffLuma[varInd][k] = m_filterCoeffPrevSelected[varInd][i];
          k++;
        }
        else
        {
          m_filterCoeffPrevSelected[varInd][i]=0;
        }
      }
    }
  }
}

double EncAdaptiveLoopFilter::xSolveAndQuant(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, int sqrFiltLength, const int *weights, int bit_depth, bool bChroma )
{
  double error;

  int factor = (1<<(bit_depth-1)), i;
  int quantCoeffSum, minInd, targetCoeffSumInt, k, diff;
  double errMin;

  gnsSolveByChol(E, y, filterCoeff, sqrFiltLength);

  if( m_isGALF )
  {
    roundFiltCoeff(filterCoeffQuant, filterCoeff, sqrFiltLength, factor);
    if (bChroma)
    {
      targetCoeffSumInt = factor;
    }
    else
    targetCoeffSumInt = 0;
  }
  else
  {
    double targetCoeffSum = 0;
    for( i=0; i<sqrFiltLength; i++)
    {
      targetCoeffSum+=(weights[i]*filterCoeff[i]*factor);
    }
    targetCoeffSumInt=ROUND(targetCoeffSum);

    roundFiltCoeff(filterCoeffQuant, filterCoeff, sqrFiltLength, factor);
  }

  quantCoeffSum=0;
  for (i=0; i<sqrFiltLength; i++)
  {
    quantCoeffSum += weights[i]*filterCoeffQuant[i];
  }

  int count=0;
  while(quantCoeffSum!=targetCoeffSumInt && count < 10)
  {
    if (quantCoeffSum>targetCoeffSumInt)
    {
      diff=quantCoeffSum-targetCoeffSumInt;
      errMin=0; minInd=-1;
      for (k=0; k<sqrFiltLength; k++)
      {
        if (weights[k]<=diff)
        {
          for (i=0; i<sqrFiltLength; i++)
          {
            m_filterCoeffQuantMod[i]=filterCoeffQuant[i];
          }
          m_filterCoeffQuantMod[k]--;
          for (i=0; i<sqrFiltLength; i++)
          {
            filterCoeff[i]=(double)m_filterCoeffQuantMod[i]/(double)factor;
          }
          error = xCalcErrorForGivenWeights(E, y, filterCoeff, sqrFiltLength);
          if (error<errMin || minInd==-1)
          {
            errMin=error;
            minInd=k;
          }
        } // if (weights(k)<=diff){
      } // for (k=0; k<sqrFiltLength; k++){
      filterCoeffQuant[minInd]--;
    }
    else
    {
      diff=targetCoeffSumInt-quantCoeffSum;
      errMin=0; minInd=-1;
      for (k=0; k<sqrFiltLength; k++)
      {
        if (weights[k]<=diff)
        {
          for (i=0; i<sqrFiltLength; i++)
          {
            m_filterCoeffQuantMod[i]=filterCoeffQuant[i];
          }
          m_filterCoeffQuantMod[k]++;
          for (i=0; i<sqrFiltLength; i++)
          {
            filterCoeff[i]=(double)m_filterCoeffQuantMod[i]/(double)factor;
          }
          error = xCalcErrorForGivenWeights(E, y, filterCoeff, sqrFiltLength);
          if (error<errMin || minInd==-1)
          {
            errMin=error;
            minInd=k;
          }
        } // if (weights(k)<=diff){
      } // for (k=0; k<sqrFiltLength; k++){
      filterCoeffQuant[minInd]++;
    }

    quantCoeffSum=0;
    for (i=0; i<sqrFiltLength; i++)
    {
      quantCoeffSum+=weights[i]*filterCoeffQuant[i];
    }
  }
  if( count == 10 )
  {
    for (i=0; i<sqrFiltLength; i++)
    {
      filterCoeffQuant[i] = 0;
    }
  }

  if( m_isGALF )
  {
    quantCoeffSum = 0;
    for (i = 0; i<sqrFiltLength - 1; i++)
    {
      quantCoeffSum += weights[i] * filterCoeffQuant[i];
    }
    if (bChroma)
    {
      filterCoeffQuant[sqrFiltLength - 1] = factor - quantCoeffSum;
    }
    else
    {
      filterCoeffQuant[sqrFiltLength - 1] = -quantCoeffSum;
    }
  }

#if DISTORTION_LAMBDA_BUGFIX
  int max_value = std::min((1 << (3 + m_nInternalBitDepth)) - 1, (1 << 14) - 1);
  int min_value = std::max(-(1 << (3 + m_nInternalBitDepth)), -(1 << 14));
#else
  int    max_value = std::min((1 << (3 + m_nInputBitDepth + m_nBitIncrement)) - 1, (1 << 14) - 1);
  int    min_value = std::max(-(1 << (3 + m_nInputBitDepth + m_nBitIncrement)), -(1 << 14));
#endif
  for (i=0; i<sqrFiltLength; i++)
  {
    filterCoeffQuant[i] = std::min( max_value , std::max( min_value , filterCoeffQuant[i] ) );
    filterCoeff[i]=(double)filterCoeffQuant[i]/(double)factor;
  }

  if( ! m_isGALF )
  {
    // Encoder-side restriction on the ALF coefficients for limiting the Clipping LUT size
    int maxSampleValue, minSampleValue;
    int sumCoef[2]    = {0, 0};
    int maxPxlVal     = m_nIBDIMax;
    int numBitsMinus1 = m_NUM_BITS-1;
    int offset        = (1<<(m_NUM_BITS-2));
    int lastCoef      = sqrFiltLength-1;
    int centerCoef    = sqrFiltLength-2;

    int *coef = filterCoeffQuant;
#if DISTORTION_LAMBDA_BUGFIX
    int clipTableMax = ((m_ALF_HM3_QC_CLIP_RANGE - m_ALF_HM3_QC_CLIP_OFFSET - 1) << (m_nInternalBitDepth - 8));
    int clipTableMin = ((-m_ALF_HM3_QC_CLIP_OFFSET) << (m_nInternalBitDepth - 8));
#else
    int clipTableMax = ((m_ALF_HM3_QC_CLIP_RANGE - m_ALF_HM3_QC_CLIP_OFFSET - 1) << m_nBitIncrement);
    int clipTableMin = ((-m_ALF_HM3_QC_CLIP_OFFSET) << m_nBitIncrement);
#endif

    for (i=0; i<centerCoef; i++)
    {
      sumCoef[coef[i]>0?0:1] += (coef[i]<<1);
    }
    sumCoef[ coef[centerCoef] > 0 ? 0 : 1 ] += coef[centerCoef];

    maxSampleValue = ( maxPxlVal * sumCoef[0] + coef[lastCoef] + offset ) >> numBitsMinus1;
    minSampleValue = ( maxPxlVal * sumCoef[1] + coef[lastCoef] + offset ) >> numBitsMinus1;

    if ( maxSampleValue > clipTableMax || minSampleValue < clipTableMin )
    {
      memset( coef, 0, sizeof(int) * sqrFiltLength );
      coef[centerCoef] = (1<<numBitsMinus1);
      for (int j=0; j<sqrFiltLength; j++)
      {
        CHECK( !(filterCoeffQuant[j]>=min_value && filterCoeffQuant[j]<=max_value), "ALF: quant coeffs out of bound" );
        filterCoeff[j]=(double)filterCoeffQuant[j]/(double)factor;
      }
    }
  }
  error=xCalcErrorForGivenWeights(E, y, filterCoeff, sqrFiltLength);
  return(error);
}


void EncAdaptiveLoopFilter::roundFiltCoeff(int *filterCoeffQuant, double *filterCoeff, int sqrFiltLength, int factor)
{
  int i;
  double diff;
  int diffInt, sign;

  for(i = 0; i < sqrFiltLength; i++)
  {
    sign               = (filterCoeff[i]>0) ?  1: -1;
    diff               = filterCoeff[i]*sign;
    diffInt            = (int)(diff*(double)factor+0.5);
    filterCoeffQuant[i] = diffInt*sign;
  }
}

void EncAdaptiveLoopFilter::xCheckFilterMergingGalf(ALFParam& alfParam
#if JVET_C0038_NO_PREV_FILTERS
  , const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf
#endif
)
{
#if JVET_C0038_NO_PREV_FILTERS
  const PelBuf orgLuma = orgUnitBuf.get(COMPONENT_Y);
  const Pel*   pOrg = orgLuma.buf;
  const int    orgStride = orgLuma.stride;

  const PelBuf recExtLuma = recExtBuf.get(COMPONENT_Y);
  const Pel*   pRecExt = recExtLuma.buf;
  const int    extStride = recExtLuma.stride;
#endif

  AlfFilterType filtType = alfParam.filterType;

  double ***ESym = m_EGlobalSym[filtType];
  double **ySym  = m_yGlobalSym[filtType];
  double *pixAcc = m_pixAcc;

  int filters_per_fr_best=0;
  int filters_per_fr, firstFilt;
  int intervalBest[m_NO_VAR_BINS][m_NO_VAR_BINS];
  int i, k, varInd;
  double  lagrangian, lagrangianMin;
#if FORCE0
  double lagrangianForce0;
#endif
  double lambda =  m_dLambdaLuma * (1<<(2*m_nBitIncrement));
  int sqrFiltLength;

  double errorForce0CoeffTab[m_NO_VAR_BINS][2];


  sqrFiltLength = m_sqrFiltLengthTab[ filtType];

  if (m_is9x9Alloc == 0)
  {
    m_is9x9Alloc = 1;
  }
  else if (!m_updateMatrix)
  {
    for (int varInd = 0; varInd < m_NO_VAR_BINS; varInd++)
    {
      xDeriveLocalEyFromLgrTapFilter(y_temp9x9[varInd], y_temp2D[varInd], m_patternMapTab[2], m_patternMapTab[filtType]);
    }
  }

  memcpy(pixAcc_temp1D, pixAcc, sizeof(double)*m_NO_VAR_BINS);

  for (varInd=0; varInd<m_NO_VAR_BINS; varInd++)
  {
    if (m_updateMatrix)
    {
      memcpy(y_temp2D[varInd], ySym[varInd], sizeof(double)*sqrFiltLength);
    }
    for (k = 0; k < sqrFiltLength; k++)
    {
      memcpy(E_temp3D[varInd][k], ESym[varInd][k], sizeof(double)*sqrFiltLength);
    }
  }
#if JVET_C0038_NO_PREV_FILTERS
  int iFixedFilters = alfParam.iAvailableFilters;
#endif

  xfindBestFilterPredictor(
#if JVET_C0038_NO_PREV_FILTERS
    E_temp3D, y_temp2D, pixAcc_temp1D, filtType, pOrg, pRecExt, orgStride, extStride,
    usePrevFiltBest, sqrFiltLength, iFixedFilters
#endif
    );

  for(i = 0; i < m_NO_VAR_BINS; i++)
  {
    memset(m_filterCoeffSym[i], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    memset(m_filterCoeffSymQuant[i], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
  }

  firstFilt=1;  lagrangianMin=0;
  filters_per_fr = m_NO_FILTERS;
  int predMode     = 0;
  int bestPredMode = 0;
  double dist;
  int coeffBits;
#if FORCE0
  double distForce0;
  int coeffBitsForce0;
  int codedVarBins[m_NO_VAR_BINS];
#endif
  // zero all variables
  memset(intervalBest, 0, sizeof(int)*m_NO_VAR_BINS);
  xMergeFiltersGreedyGalf(E_temp3D, y_temp2D, pixAcc_temp1D, intervalBest, sqrFiltLength);

  while(filters_per_fr >=1 )
  {

    dist = xCalcFilterCoeffsGalf(E_temp3D, y_temp2D, pixAcc_temp1D, intervalBest[filters_per_fr - 1], filters_per_fr, filtType,
      errorForce0CoeffTab);
#if FORCE0
    distForce0 = xCalcDistForce0(filters_per_fr, filtType, sqrFiltLength, errorForce0CoeffTab,
      lambda, codedVarBins);
#endif

    coeffBits = xCheckFilterPredictionMode(m_filterCoeffSymQuant, filters_per_fr, filtType, predMode);
#if FORCE0
    coeffBitsForce0 = xCalcBitsForce0(m_filterCoeffSymQuant, filters_per_fr, filtType, codedVarBins);
#endif

    lagrangian = dist + lambda * coeffBits;
#if FORCE0
    lagrangianForce0 = distForce0 + lambda * coeffBitsForce0;
    if (lagrangianForce0 < lagrangian)
    {
      lagrangian = lagrangianForce0;
    }
#endif
    if( lagrangian<=lagrangianMin || firstFilt==1)
    {
      firstFilt=0;
      lagrangianMin=lagrangian;

      filters_per_fr_best = filters_per_fr;
      bestPredMode        = predMode;
      memcpy(m_varIndTab, intervalBest[filters_per_fr-1], m_NO_VAR_BINS * sizeof(int));
    }
    filters_per_fr--;
  }

  dist = xCalcFilterCoeffsGalf(E_temp3D, y_temp2D, pixAcc_temp1D, m_varIndTab, filters_per_fr_best, filtType,
    errorForce0CoeffTab);
  coeffBits = xCheckFilterPredictionMode(m_filterCoeffSymQuant, filters_per_fr_best, filtType, predMode);
#if FORCE0
  distForce0 = xCalcDistForce0(filters_per_fr_best, filtType, sqrFiltLength, errorForce0CoeffTab, lambda, codedVarBins);
  coeffBitsForce0 = xCalcBitsForce0(m_filterCoeffSymQuant, filters_per_fr_best, filtType, codedVarBins);
  lagrangian = dist + lambda * coeffBits;
  lagrangianForce0 = distForce0 + lambda * coeffBitsForce0;
  alfParam.forceCoeff0 = (lagrangianForce0 < lagrangian) ? 1 : 0;
  if (alfParam.forceCoeff0)
  {
    memcpy(alfParam.codedVarBins, codedVarBins, sizeof(int)*m_NO_VAR_BINS);
  }
#endif

  CHECK( predMode != bestPredMode, "wrong re-calculation" );

  //set alfParameter
  alfParam.filters_per_group_diff = filters_per_fr_best;
  alfParam.filters_per_group      = filters_per_fr_best;
  alfParam.predMethod             = bestPredMode;

#if FORCE0
  if (alfParam.forceCoeff0)
  {
    alfParam.predMethod = 0;
    for (varInd = 0; varInd < filters_per_fr_best; varInd++)
    {
      if (codedVarBins[varInd] == 0)
      {
        memset(m_filterCoeffSym[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
        memset(m_filterCoeffSymQuant[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
      }
    }
  }
#endif
  for (int ind = 0; ind < alfParam.filters_per_group; ++ind)
  {
    for(int i = 0; i < sqrFiltLength; i++)
    {
      if (alfParam.predMethod)
      {
        alfParam.coeffmulti[ind][i] = m_diffFilterCoeffQuant[ind][i];
      }
      else
      {
        alfParam.coeffmulti[ind][i] = m_filterCoeffSymQuant[ind][i];
      }
    }
  }
  memcpy(alfParam.filterPattern, m_varIndTab, m_NO_VAR_BINS * sizeof(int));
#if JVET_C0038_NO_PREV_FILTERS
  memcpy(alfParam.PrevFiltIdx, usePrevFiltBest, m_NO_VAR_BINS * sizeof(int));

  int predPattern;

  // frame that filter is predicted from
  if (alfParam.iAvailableFilters > 0)
  {
    // prediction pattern
    predPattern = alfParam.PrevFiltIdx[0] > 0 ? 1 : 0;
    for (i = 1; i < m_NO_VAR_BINS; i++)
    {
      int iCurrPredPattern = alfParam.PrevFiltIdx[i] > 0 ? 1 : 0;
      if (iCurrPredPattern != predPattern)
      {
        predPattern = 2;
        break;
      }
    }
    alfParam.iPredPattern = predPattern;
  }
#endif

  if (m_updateMatrix)
  {
    for (int i = 0; i< m_NO_VAR_BINS; i++)
    {
      for (int j = 0; j< m_MAX_SQR_FILT_LENGTH; j++)
      {
        y_temp9x9[i][j] = y_temp2D[i][j];
      }
    }
  }
}

void EncAdaptiveLoopFilter::xCheckFilterMergingAlf(ALFParam& alfParam
#if JVET_C0038_NO_PREV_FILTERS
  , const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf
#endif
)
{
  AlfFilterType filtType = alfParam.filterType;

  double ***ESym = m_EGlobalSym[filtType];
  double **ySym  = m_yGlobalSym[filtType];
  double *pixAcc = m_pixAcc;
  int **filterCoeffSym      = m_filterCoeffSym; //TODO Why do we need this here?
  int **filterCoeffSymQuant = m_filterCoeffSymQuant;

  int filters_per_fr_best=0;
  int filters_per_fr, firstFilt;
  int interval[m_NO_VAR_BINS][2], intervalBest[m_NO_VAR_BINS][2];
  int i, k, varInd;
  double  lagrangian, lagrangianMin;
  double lambda =  m_dLambdaLuma * (1<<(2*m_nBitIncrement));
  int sqrFiltLength;

  double errorForce0CoeffTab[m_NO_VAR_BINS][2];

  sqrFiltLength = m_sqrFiltLengthTab[ filtType];

  memcpy(pixAcc_temp1D, pixAcc, sizeof(double)*m_NO_VAR_BINS);

  for (varInd=0; varInd<m_NO_VAR_BINS; varInd++)
  {
    memcpy(y_temp2D[varInd], ySym[varInd], sizeof(double)*sqrFiltLength);
    for (k = 0; k < sqrFiltLength; k++)
    {
      memcpy(E_temp3D[varInd][k], ESym[varInd][k], sizeof(double)*sqrFiltLength);
    }
  }
  for (i = 0; i < m_NO_VAR_BINS; i++)
  {
    memset(filterCoeffSym[i], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    memset(filterCoeffSymQuant[i], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
  }

  firstFilt=1;  lagrangianMin=0;
  filters_per_fr = m_NO_FILTERS;
  int predMode     = 0;
  int bestPredMode = 0;

  while(filters_per_fr >=1 )
  {
    xMergeFiltersGreedy(E_temp3D, y_temp2D, pixAcc_temp1D, interval, sqrFiltLength, filters_per_fr);
    double dist = xCalcFilterCoeffs(E_temp3D, y_temp2D, pixAcc_temp1D, interval, filters_per_fr, filtType,
      filterCoeffSym, filterCoeffSymQuant, errorForce0CoeffTab);

    int coeffBits = xCheckFilterPredictionMode(filterCoeffSymQuant, filters_per_fr, filtType, predMode);
    lagrangian = dist + lambda * coeffBits;
    if( lagrangian<=lagrangianMin || firstFilt==1)
    {
      firstFilt=0;
      lagrangianMin=lagrangian;

      filters_per_fr_best = filters_per_fr;
      bestPredMode        = predMode;
      memcpy(intervalBest, interval, m_NO_VAR_BINS*2*sizeof(int));
    }
    filters_per_fr--;
  }
  xCalcFilterCoeffs( E_temp3D, y_temp2D, pixAcc_temp1D, intervalBest, filters_per_fr_best, filtType,
                     filterCoeffSym, filterCoeffSymQuant , errorForce0CoeffTab);
  xCheckFilterPredictionMode(filterCoeffSymQuant, filters_per_fr_best, filtType, predMode);
  CHECK( predMode != bestPredMode, "wrong re-calculation" );

  //set alfParameter
  alfParam.filters_per_group_diff = filters_per_fr_best;
  alfParam.filters_per_group      = filters_per_fr_best;
  alfParam.predMethod             = bestPredMode;

  for (int ind = 0; ind < alfParam.filters_per_group; ++ind)
  {
    for(int i = 0; i < sqrFiltLength; i++)
    {
      if (alfParam.predMethod)
      {
        alfParam.coeffmulti[ind][i] = m_diffFilterCoeffQuant[ind][i];
      }
      else
      {
        alfParam.coeffmulti[ind][i] = m_filterCoeffSymQuant[ind][i];
      }
    }
  }
  memset(m_varIndTab, 0, sizeof(int)*m_NO_VAR_BINS);
  if( filters_per_fr_best > 2 )
  {
    alfParam.filterMode = ALF_MULTIPLE_FILTERS;
    memset(alfParam.filterPattern, 0, m_NO_VAR_BINS * sizeof(int));
    for(int filtIdx = 0; filtIdx < filters_per_fr_best; filtIdx++)
    {
      for(k = intervalBest[filtIdx][0]; k <= intervalBest[filtIdx][1]; k++)
      {
        m_varIndTab[k] = filtIdx;
      }
      if( filtIdx > 0 )
      {
        alfParam.filterPattern[ intervalBest[filtIdx][0] ] = 1;
      }
    }
  }
  else if( filters_per_fr_best == 2 )
  {
    alfParam.filterMode        = ALF_TWO_FILTERS;
    alfParam.startSecondFilter = intervalBest[1][0];
    for( k = intervalBest[1][0]; k <= intervalBest[1][1]; k++ )
    {
      m_varIndTab[k] = 1;
    }
  }
  else
  {
    alfParam.filterMode = ALF_ONE_FILTER;
  }
}

void EncAdaptiveLoopFilter::xMergeFiltersGreedyGalf(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int intervalBest[m_NO_VAR_BINS][m_NO_VAR_BINS], int sqrFiltLength)
{
  int first, ind, ind1, ind2, noRemaining, i, j, exist, indexList[m_NO_VAR_BINS], indexListTemp[m_NO_VAR_BINS], available[m_NO_VAR_BINS], bestToMerge[2];
  double error, error1, error2, errorMin;

  noRemaining = m_NO_VAR_BINS;
  for (ind = 0; ind<m_NO_VAR_BINS; ind++)
  {
    intervalBest[noRemaining - 1][ind] = ind;
    indexList[ind] = ind;
    available[ind] = 1;
    m_pixAcc_merged[ind] = pixAccGlobalSeq[ind];

    memcpy(m_y_merged[ind], yGlobalSeq[ind], sizeof(double)*sqrFiltLength);
    for (i = 0; i < sqrFiltLength; i++)
    {
      memcpy(m_E_merged[ind][i], EGlobalSeq[ind][i], sizeof(double)*sqrFiltLength);
    }
  }
  for (ind = 0; ind<m_NO_VAR_BINS; ind++)
  {
    intervalBest[0][ind] = 0;
  }

  // Try merging different matrices

  while (noRemaining > 2)
  {
    errorMin = 0; first = 1; bestToMerge[0] = 0; bestToMerge[1] = 1;
    for (ind1 = 0; ind1<m_NO_VAR_BINS - 1; ind1++)
    {
      if (available[ind1])
      {
        for (ind2 = ind1 + 1; ind2<m_NO_VAR_BINS; ind2++)
        {
          if (available[ind2])
          {
            error1 = calculateErrorAbs(m_E_merged[ind1], m_y_merged[ind1], m_pixAcc_merged[ind1], sqrFiltLength);
            error2 = calculateErrorAbs(m_E_merged[ind2], m_y_merged[ind2], m_pixAcc_merged[ind2], sqrFiltLength);

            pixAcc_temp0D = m_pixAcc_merged[ind1] + m_pixAcc_merged[ind2];
            for (i = 0; i<sqrFiltLength; i++) {
              y_temp1D[i] = m_y_merged[ind1][i] + m_y_merged[ind2][i];
              for (j = 0; j<sqrFiltLength; j++) {
                E_temp2D[i][j] = m_E_merged[ind1][i][j] + m_E_merged[ind2][i][j];
              }
            }
            error = calculateErrorAbs(E_temp2D, y_temp1D, pixAcc_temp0D, sqrFiltLength) - error1 - error2;

            if (error<errorMin || first == 1)
            {
              errorMin = error;
              bestToMerge[0] = ind1;
              bestToMerge[1] = ind2;
              first = 0;
            }
          }
        }
      }
    }

    ind1 = bestToMerge[0];
    ind2 = bestToMerge[1];

    m_pixAcc_merged[ind1] += m_pixAcc_merged[ind2];
    for (i = 0; i<sqrFiltLength; i++)
    {
      m_y_merged[ind1][i] += m_y_merged[ind2][i];
      for (j = 0; j<sqrFiltLength; j++)
      {
        m_E_merged[ind1][i][j] += m_E_merged[ind2][i][j];
      }
    }

    available[ind2] = 0;

    for (i = 0; i<m_NO_VAR_BINS; i++)
    {
      if (indexList[i] == ind2)
      {
        indexList[i] = ind1;
      }
    }
    noRemaining--;
    if (noRemaining <= m_NO_VAR_BINS)
    {
      for (i = 0; i<m_NO_VAR_BINS; i++)
      {
        indexListTemp[i] = indexList[i];
      }

      exist = 0; ind = 0;
      for (j = 0; j<m_NO_VAR_BINS; j++)
      {
        exist = 0;
        for (i = 0; i<m_NO_VAR_BINS; i++)
        {
          if (indexListTemp[i] == j)
          {
            exist = 1;
            break;
          }
        }

        if (exist)
        {
          for (i = 0; i<m_NO_VAR_BINS; i++)
          {
            if (indexListTemp[i] == j)
            {
              intervalBest[noRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

double EncAdaptiveLoopFilter::xMergeFiltersGreedy(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int intervalBest[m_NO_VAR_BINS][2], int sqrFiltLength, int noIntervals)
{
  int first, ind, ind1, ind2, i, j, bestToMerge ;
  double error, error1, error2, errorMin;

  if (noIntervals == m_NO_FILTERS)
  {
    noRemaining=m_NO_VAR_BINS;
    for (ind=0; ind<m_NO_VAR_BINS; ind++)
    {
      indexList[ind]=ind;
      available[ind]=1;
      m_pixAcc_merged[ind]=pixAccGlobalSeq[ind];
      memcpy(m_y_merged[ind],yGlobalSeq[ind],sizeof(double)*sqrFiltLength);
      for (i=0; i<sqrFiltLength; i++)
      {
        memcpy(m_E_merged[ind][i],EGlobalSeq[ind][i],sizeof(double)*sqrFiltLength);
      }
    }
  }
  // Try merging different matrices
  if (noIntervals == m_NO_FILTERS)
  {
    for (ind=0; ind<m_NO_VAR_BINS; ind++)
    {
      error_tab[ind]=calculateErrorAbs(m_E_merged[ind], m_y_merged[ind], m_pixAcc_merged[ind], sqrFiltLength);
    }
    for (ind=0; ind<m_NO_VAR_BINS-1; ind++)
    {
      ind1=indexList[ind];
      ind2=indexList[ind+1];

      error1=error_tab[ind1];
      error2=error_tab[ind2];

      pixAcc_temp0D=m_pixAcc_merged[ind1]+m_pixAcc_merged[ind2];
      for (i=0; i<sqrFiltLength; i++)
      {
        m_y_temp[i]=m_y_merged[ind1][i]+m_y_merged[ind2][i];
        for (j=0; j<sqrFiltLength; j++)
        {
          m_E_temp[i][j]=m_E_merged[ind1][i][j]+m_E_merged[ind2][i][j];
        }
      }
      error_comb_tab[ind1]=calculateErrorAbs(m_E_temp, m_y_temp, pixAcc_temp0D, sqrFiltLength)-error1-error2;
    }
  }

  while( noRemaining > noIntervals )
  {
    errorMin=0; first=1;
    bestToMerge = 0;
    for (ind=0; ind<noRemaining-1; ind++)
    {
      error = error_comb_tab[indexList[ind]];
      if ((error<errorMin || first==1))
      {
        errorMin=error;
        bestToMerge=ind;
        first=0;
      }
    }
    ind1=indexList[bestToMerge];
    ind2=indexList[bestToMerge+1];
    m_pixAcc_merged[ind1]+=m_pixAcc_merged[ind2];
    for (i=0; i<sqrFiltLength; i++)
    {
      m_y_merged[ind1][i]+=m_y_merged[ind2][i];
      for (j=0; j<sqrFiltLength; j++)
      {
        m_E_merged[ind1][i][j]+=m_E_merged[ind2][i][j];
      }
    }
    available[ind2] = 0;

    //update error tables
    error_tab[ind1]=error_comb_tab[ind1]+error_tab[ind1]+error_tab[ind2];
    if (indexList[bestToMerge] > 0)
    {
      ind1=indexList[bestToMerge-1];
      ind2=indexList[bestToMerge];
      error1=error_tab[ind1];
      error2=error_tab[ind2];
      pixAcc_temp0D=m_pixAcc_merged[ind1]+m_pixAcc_merged[ind2];
      for (i=0; i<sqrFiltLength; i++)
      {
        m_y_temp[i]=m_y_merged[ind1][i]+m_y_merged[ind2][i];
        for (j=0; j<sqrFiltLength; j++)
        {
          m_E_temp[i][j]=m_E_merged[ind1][i][j]+m_E_merged[ind2][i][j];
        }
      }
      error_comb_tab[ind1]=calculateErrorAbs(m_E_temp, m_y_temp, pixAcc_temp0D, sqrFiltLength)-error1-error2;
    }
    if (indexList[bestToMerge+1] < m_NO_VAR_BINS-1)
    {
      ind1=indexList[bestToMerge];
      ind2=indexList[bestToMerge+2];
      error1=error_tab[ind1];
      error2=error_tab[ind2];
      pixAcc_temp0D=m_pixAcc_merged[ind1]+m_pixAcc_merged[ind2];
      for (i=0; i<sqrFiltLength; i++)
      {
        m_y_temp[i]=m_y_merged[ind1][i]+m_y_merged[ind2][i];
        for (j=0; j<sqrFiltLength; j++)
        {
          m_E_temp[i][j]=m_E_merged[ind1][i][j]+m_E_merged[ind2][i][j];
        }
      }
      error_comb_tab[ind1]=calculateErrorAbs(m_E_temp, m_y_temp, pixAcc_temp0D, sqrFiltLength)-error1-error2;
    }

    ind=0;
    for (i=0; i<m_NO_VAR_BINS; i++)
    {
      if (available[i]==1)
      {
        indexList[ind]=i;
        ind++;
      }
    }
    noRemaining--;
  }

  errorMin = 0;
  for (ind=0; ind<noIntervals; ind++)
  {
    errorMin+=error_tab[indexList[ind]];
  }

  for (ind=0; ind < noIntervals-1; ind++)
  {
    intervalBest[ind][0]=indexList[ind]; intervalBest[ind][1]=indexList[ind+1]-1;
  }

  intervalBest[noIntervals-1][0] = indexList[noIntervals-1];
  intervalBest[noIntervals-1][1] = m_NO_VAR_BINS-1;

  return(errorMin);
}



//********************************
// CLASSIFICATION
//********************************

void EncAdaptiveLoopFilter::xSetInitialMask( const CPelBuf& recBufExt )
{
  xClassify(m_varImg, recBufExt, 9 / 2, m_VAR_SIZE);
  m_maskBuf.fill( 1 );
}



//********************************
// CU Adaptation
//********************************
void EncAdaptiveLoopFilter::xCheckCUAdaptation( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiMinRate, uint64_t& ruiMinDist, double& rdMinCost )
{
  ALFParam cFrmAlfParam;
  allocALFParam(&cFrmAlfParam);
  copyALFParam( &cFrmAlfParam, m_pcBestAlfParam );
  int    Height = orgUnitBuf.get(COMPONENT_Y).height;
  int    Width = orgUnitBuf.get(COMPONENT_Y).width;

  for( uint32_t uiDepth = 0; uiDepth < m_uiMaxTotalCUDepth; uiDepth++ )
  {
    int nBlkSize = ( cs.slice->getSPS()->getMaxCUHeight() * cs.slice->getSPS()->getMaxCUWidth() ) >> ( uiDepth << 1 );
    int nPicSize = orgUnitBuf.get(COMPONENT_Y).width * orgUnitBuf.get(COMPONENT_Y).height;
    if( ( nBlkSize << 4 ) > nPicSize )
    {
      // block is too large
      continue;
    }

    m_tempPelBuf.get( COMPONENT_Y).copyFrom( recUnitBuf.get(COMPONENT_Y) );
    copyALFParam( m_pcTempAlfParam, &cFrmAlfParam);

    m_pcTempAlfParam->cu_control_flag = 1;
    m_pcTempAlfParam->alf_max_depth = uiDepth;

    for (uint32_t uiRD = 0; uiRD <= ALF_NUM_OF_REDESIGN; uiRD++)
    {
      if (uiRD)
      {
        // re-design filter coefficients
        xReDesignFilterCoeff( orgUnitBuf, recExtBuf, m_tempPelBuf,
#if COM16_C806_ALF_TEMPPRED_NUM
          m_pcTempAlfParam,
#endif
          cs.slice->clpRng(COMPONENT_Y) );
      }

      uint64_t uiRate, uiDist;
      double dCost;
      xSetCUAlfCtrlFlags( cs, orgUnitBuf, recExtBuf, m_tempPelBuf, uiDist, uiDepth, m_pcTempAlfParam ); //set up varImg here
      xCalcRDCost( uiDist, m_pcTempAlfParam, uiRate, dCost );

      if (dCost < rdMinCost )
      {
        rdMinCost = dCost;
        ruiMinDist = uiDist;
        ruiMinRate = uiRate;
        m_bestPelBuf.get(COMPONENT_Y).copyFrom( m_tempPelBuf.get(COMPONENT_Y) );
        copyALFParam( m_pcBestAlfParam, m_pcTempAlfParam );
        if( m_isGALF )
        {
          for (int i = 0; i < Height; i++)
          {
            for (int j = 0; j < Width; j++)
            {
              m_maskBestBuf.at(j, i) = m_maskBuf.at(j, i);
            }
          }
        }
      }
    }
  }

  if (m_pcBestAlfParam->cu_control_flag)
  {
    recUnitBuf.get(COMPONENT_Y).copyFrom( m_bestPelBuf.get(COMPONENT_Y) );
    if( m_isGALF )
    {
      for (int i = 0; i < Height; i++)
      {
        for (int j = 0; j < Width; j++)
        {
          m_maskBuf.at(j, i) = m_maskBestBuf.at(j, i);
        }
      }
    }
  }
  freeALFParam(&cFrmAlfParam);
}


void EncAdaptiveLoopFilter::xSetCUAlfCtrlFlags( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiDist, uint32_t uiAlfCtrlDepth, ALFParam *pAlfParam )
{
  ruiDist = 0;
  pAlfParam->num_alf_cu_flag = 0;

  const SPS* sps = cs.slice->getSPS();

  const unsigned widthInCtus  = cs.pcv->widthInCtus;
  const unsigned maxCUSize    = sps->getMaxCUWidth();
  const unsigned alfCtrlSize  = maxCUSize >> uiAlfCtrlDepth;
  const unsigned imgWidth     = orgUnitBuf.get(COMPONENT_Y).width;
  const unsigned imgHeight    = orgUnitBuf.get(COMPONENT_Y).height;

  Partitioner* partitioner = PartitionerFactory::get( *cs.slice );

  for( uint32_t uiCTUAddr = 0; uiCTUAddr < cs.pcv->sizeInCtus ; uiCTUAddr++ )
  {
    const unsigned  ctuXPosInCtus         = uiCTUAddr % widthInCtus;
    const unsigned  ctuYPosInCtus         = uiCTUAddr / widthInCtus;

    Position ctuPos ( ctuXPosInCtus * maxCUSize, ctuYPosInCtus * maxCUSize );
    UnitArea ctuArea( cs.area.chromaFormat, Area( ctuPos.x, ctuPos.y, maxCUSize, maxCUSize ) );

    for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
    {
      const Position&    cuPos    = currCU.lumaPos();
      const int          qtDepth  = currCU.qtDepth;
      const unsigned     qtSize   = maxCUSize >> qtDepth;
      const Position     qtPos0   = Position( (cuPos.x / qtSize)      * qtSize     , (cuPos.y / qtSize)      * qtSize );
      const Position     ctrlPos0 = Position( (cuPos.x / alfCtrlSize) * alfCtrlSize, (cuPos.y / alfCtrlSize) * alfCtrlSize );

      if( qtDepth >= uiAlfCtrlDepth && cuPos == ctrlPos0 )
      {
        UnitArea ctrlArea( cs.area.chromaFormat, Area( cuPos.x, cuPos.y, std::min( alfCtrlSize, imgWidth-cuPos.x ), std::min( alfCtrlSize, imgHeight-cuPos.y ) ) );
        xSetCUAlfCtrlFlag( cs, ctrlArea, orgUnitBuf, recExtBuf, recUnitBuf, ruiDist, pAlfParam );
      }
      else if ( (qtDepth < uiAlfCtrlDepth) && cuPos == qtPos0 )
      {
        UnitArea ctrlArea( cs.area.chromaFormat, Area( cuPos.x, cuPos.y, std::min( qtSize, imgWidth-cuPos.x ), std::min( qtSize, imgHeight-cuPos.y ) ) );
        xSetCUAlfCtrlFlag( cs, ctrlArea, orgUnitBuf, recExtBuf, recUnitBuf, ruiDist, pAlfParam );
      }
    }
  }

  delete partitioner;
}


void EncAdaptiveLoopFilter::xSetCUAlfCtrlFlag( CodingStructure& cs, const UnitArea alfCtrlArea, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiDist, ALFParam *pAlfParam)
{

  const Position& blkPos   = alfCtrlArea.lumaPos();
  const Size&     blkSize  = alfCtrlArea.lumaSize();

  CPelBuf orgBufCUs    = orgUnitBuf.get(COMPONENT_Y).subBuf( blkPos, blkSize );
  CPelBuf recBufCUs    =  recExtBuf.get(COMPONENT_Y).subBuf( blkPos, blkSize );

  CPelBuf cfiltBufCUs  = recUnitBuf.get(COMPONENT_Y).subBuf( blkPos, blkSize );
  UIntBuf  maskBufCUs  = m_maskBuf.subBuf( blkPos, blkSize );

  uint64_t uiRecSSD  = xCalcSSD( orgBufCUs, recBufCUs  );
  uint64_t uiFiltSSD = xCalcSSD( orgBufCUs, cfiltBufCUs );

  uint32_t filterFlag = !!(uiFiltSSD < uiRecSSD );
  maskBufCUs.fill( filterFlag );
  ruiDist += filterFlag ? uiFiltSSD : uiRecSSD;
  pAlfParam->alf_cu_flag[pAlfParam->num_alf_cu_flag] = 0!=filterFlag;
  pAlfParam->num_alf_cu_flag++;

  PelBuf filtBufCUs = recUnitBuf.get(COMPONENT_Y).subBuf( blkPos, blkSize );
  if( !filterFlag )
  {
    filtBufCUs.copyFrom( recBufCUs );
  }
}

//********************************
// FILTER FRAME
//********************************
void EncAdaptiveLoopFilter::xFilterFrame_en(PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, ALFParam& alfParam, const ClpRng& clpRng)
{
  AlfFilterType filtType = alfParam.filterType;
  xDecodeFilter( &alfParam );

  if( m_isGALF )
  {
    xFilterFrame_enGalf(recDstBuf, recExtBuf, filtType,
#if COM16_C806_ALF_TEMPPRED_NUM
    &alfParam, true,
#endif
    clpRng );
  }
  else
  {
    xFilterFrame_enAlf(recDstBuf, recExtBuf, filtType,
#if COM16_C806_ALF_TEMPPRED_NUM
    &alfParam, true,
#endif
    clpRng );
  }
}

//using filter coefficients in m_filterCoeffSym
void EncAdaptiveLoopFilter::xFilterFrame_enGalf(PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType,
  #if COM16_C806_ALF_TEMPPRED_NUM
    ALFParam *alfParam, bool updateFilterCoef,
#endif
  const ClpRng& clpRng )
{
  const PelBuf recExtLuma = recExtBuf.get(COMPONENT_Y);
  const Pel*   srcBuf = recExtLuma.buf;
  const int    srcStride = recExtLuma.stride;

  PelBuf filtFrLuma = recDstBuf.get(COMPONENT_Y);
  Pel*   dstBuf = filtFrLuma.buf;
  const int    dstStride = filtFrLuma.stride;
  const Pel* imgY_rec = srcBuf;

  int i, j, y, x;

  int fl;
  int pixelInt;
  int offset = (1<<(m_NUM_BITS - 2));

#if COM16_C806_ALF_TEMPPRED_NUM
  if (updateFilterCoef)
  {
#endif
    xcalcPredFilterCoeff(filtType
#if COM16_C806_ALF_TEMPPRED_NUM
      , alfParam
#endif
      );
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif
  fl=m_FILTER_LENGTH/2;
  for (y=0, i = fl; i < m_img_height+fl; i++, y++)
  {
    for (x=0, j = fl; j < m_img_width+fl; j++, x++)
    {
      int varInd = m_varImg[y][x];
      AlfFilterType filtTypeBig = (AlfFilterType)2;
      pixelInt = xFilterPixel(imgY_rec, &varInd, NULL, NULL, i, j, fl, srcStride, filtTypeBig);
      pixelInt = (int)((pixelInt+offset) >> (m_NUM_BITS - 1));
      dstBuf[y*dstStride + x] = ClipPel(pixelInt, clpRng);
    }
  }
}

//using filter coefficients in m_filterCoeffSym
void EncAdaptiveLoopFilter::xFilterFrame_enAlf(PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType,
  #if COM16_C806_ALF_TEMPPRED_NUM
    ALFParam *alfParam, bool updateFilterCoef,
#endif
  const ClpRng& clpRng )
{
  const PelBuf recExtLuma = recExtBuf.get(COMPONENT_Y);
  const Pel*   srcBuf = recExtLuma.buf;
  const int    srcStride = recExtLuma.stride;

  PelBuf filtFrLuma = recDstBuf.get(COMPONENT_Y);
  Pel*   dstBuf = filtFrLuma.buf;
  const int    dstStride = filtFrLuma.stride;

  const Pel* imgY_rec = srcBuf;
  const Pel* p_imgY_pad, *p_imgY_pad0;

  int var_step_size_w = m_ALF_VAR_SIZE_W;
  int var_step_size_h = m_ALF_VAR_SIZE_H;

  int i, j, y, x;

  int fl;
  int pixelInt;
  int offset = (1<<(m_NUM_BITS - 2));

  int sqrFiltLength;
  sqrFiltLength = m_MAX_SQR_FILT_LENGTH;
#if COM16_C806_ALF_TEMPPRED_NUM
  if (updateFilterCoef)
  {
#endif
    xcalcPredFilterCoeff(filtType
#if COM16_C806_ALF_TEMPPRED_NUM
      , alfParam
#endif
      );
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif
  fl=m_FILTER_LENGTH/2;
  for (y=0, i = fl; i < m_img_height+fl; i++, y++)
  {
    for (x=0, j = fl; j < m_img_width+fl; j++, x++)
    {
      int varInd = m_varImg[y / var_step_size_h][x / var_step_size_w];
      int *coef = m_filterCoeffPrevSelected[varInd];

      pixelInt  = m_filterCoeffPrevSelected[varInd][sqrFiltLength-1];

      if( filtType == ALF_FILTER_SYM_5 )
      {
        pixelInt += coef[22]* (imgY_rec[(i-fl+2)*srcStride + j-fl]+imgY_rec[(i-fl-2)*srcStride + j-fl]);

        pixelInt += coef[30]* (imgY_rec[(i-fl+1)*srcStride + j-fl+1]+imgY_rec[(i-fl-1)*srcStride + j-fl-1]);
        pixelInt += coef[31]* (imgY_rec[(i-fl+1)*srcStride + j-fl]  +imgY_rec[(i-fl-1)*srcStride + j-fl]);
        pixelInt += coef[32]* (imgY_rec[(i-fl+1)*srcStride + j-fl-1]+imgY_rec[(i-fl-1)*srcStride + j-fl+1]);

        pixelInt += coef[38]* (imgY_rec[(i-fl)*srcStride + j-fl-2]+imgY_rec[(i-fl)*srcStride + j-fl+2]);
        pixelInt += coef[39]* (imgY_rec[(i-fl)*srcStride + j-fl-1]+imgY_rec[(i-fl)*srcStride + j-fl+1]);
        pixelInt += coef[40]* (imgY_rec[(i-fl)*srcStride + j-fl]);
      }
      else if (filtType == ALF_FILTER_SYM_7)
      {
        pixelInt += coef[13]* (imgY_rec[(i-fl+3)*srcStride + j-fl]+imgY_rec[(i-fl-3)*srcStride + j-fl]);

        p_imgY_pad = imgY_rec + (i-fl+2)*srcStride;
        p_imgY_pad0 = imgY_rec + (i-fl-2)*srcStride;
        pixelInt += coef[21]* (p_imgY_pad[j-fl+1]+p_imgY_pad0[j-fl-1]);
        pixelInt += coef[22]* (p_imgY_pad[j-fl]+p_imgY_pad0[j-fl]);
        pixelInt += coef[23]* (p_imgY_pad[j-fl-1]+p_imgY_pad0[j-fl+1]);

        p_imgY_pad = imgY_rec + (i-fl+1)*srcStride;
        p_imgY_pad0 = imgY_rec + (i-fl-1)*srcStride;
        pixelInt += coef[29]* (p_imgY_pad[j-fl+2]+p_imgY_pad0[j-fl-2]);
        pixelInt += coef[30]* (p_imgY_pad[j-fl+1]+p_imgY_pad0[j-fl-1]);
        pixelInt += coef[31]* (p_imgY_pad[j-fl]+p_imgY_pad0[j-fl]);
        pixelInt += coef[32]* (p_imgY_pad[j-fl-1]+p_imgY_pad0[j-fl+1]);
        pixelInt += coef[33]* (p_imgY_pad[j-fl-2]+p_imgY_pad0[j-fl+2]);

        p_imgY_pad = imgY_rec + (i-fl)*srcStride;
        pixelInt += coef[37]* (p_imgY_pad[j-fl+3]+p_imgY_pad[j-fl-3]);
        pixelInt += coef[38]* (p_imgY_pad[j-fl+2]+p_imgY_pad[j-fl-2]);
        pixelInt += coef[39]* (p_imgY_pad[j-fl+1]+p_imgY_pad[j-fl-1]);
        pixelInt += coef[40]* (p_imgY_pad[j-fl]);

      }
      else if( filtType == ALF_FILTER_SYM_9 )
      {
        p_imgY_pad = imgY_rec + (i-fl+3)*srcStride;
        p_imgY_pad0 = imgY_rec + (i-fl-3)*srcStride;
        pixelInt += coef[12]* (p_imgY_pad[j-fl+1]+p_imgY_pad0[j-fl-1]);
        pixelInt += coef[13]* (p_imgY_pad[j-fl]+p_imgY_pad0[j-fl]);
        pixelInt += coef[14]* (p_imgY_pad[j-fl-1]+p_imgY_pad0[j-fl+1]);

        p_imgY_pad = imgY_rec + (i-fl+2)*srcStride;
        p_imgY_pad0 = imgY_rec + (i-fl-2)*srcStride;
        pixelInt += coef[20]* (p_imgY_pad[j-fl+2]+p_imgY_pad0[j-fl-2]);
        pixelInt += coef[21]* (p_imgY_pad[j-fl+1]+p_imgY_pad0[j-fl-1]);
        pixelInt += coef[22]* (p_imgY_pad[j-fl]+p_imgY_pad0[j-fl]);
        pixelInt += coef[23]* (p_imgY_pad[j-fl-1]+p_imgY_pad0[j-fl+1]);
        pixelInt += coef[24]* (p_imgY_pad[j-fl-2]+p_imgY_pad0[j-fl+2]);

        p_imgY_pad = imgY_rec + (i-fl+1)*srcStride;
        p_imgY_pad0 = imgY_rec + (i-fl-1)*srcStride;
        pixelInt += coef[28]* (p_imgY_pad[j-fl+3]+p_imgY_pad0[j-fl-3]);
        pixelInt += coef[29]* (p_imgY_pad[j-fl+2]+p_imgY_pad0[j-fl-2]);
        pixelInt += coef[30]* (p_imgY_pad[j-fl+1]+p_imgY_pad0[j-fl-1]);
        pixelInt += coef[31]* (p_imgY_pad[j-fl]+p_imgY_pad0[j-fl]);
        pixelInt += coef[32]* (p_imgY_pad[j-fl-1]+p_imgY_pad0[j-fl+1]);
        pixelInt += coef[33]* (p_imgY_pad[j-fl-2]+p_imgY_pad0[j-fl+2]);
        pixelInt += coef[34]* (p_imgY_pad[j-fl-3]+p_imgY_pad0[j-fl+3]);

        p_imgY_pad = imgY_rec + (i-fl)*srcStride;
        pixelInt += coef[36]* (p_imgY_pad[j-fl+4]+p_imgY_pad[j-fl-4]);
        pixelInt += coef[37]* (p_imgY_pad[j-fl+3]+p_imgY_pad[j-fl-3]);
        pixelInt += coef[38]* (p_imgY_pad[j-fl+2]+p_imgY_pad[j-fl-2]);
        pixelInt += coef[39]* (p_imgY_pad[j-fl+1]+p_imgY_pad[j-fl-1]);
        pixelInt += coef[40]* (p_imgY_pad[j-fl]);
      }
      else
      {
        THROW( "Filter Type not known!" );
      }

      pixelInt=(int)((pixelInt+offset) >> (m_NUM_BITS - 1));
      dstBuf[y*dstStride + x] = ClipPel(pixelInt, clpRng);
    }
  }
}

void EncAdaptiveLoopFilter::xfindBestFilterPredictor(
#if JVET_C0038_NO_PREV_FILTERS
  double ***E_temp, double**y_temp, double *pixAcc_temp, int filtType, const Pel* ImgOrg, const Pel* ImgDec, int orgStride, int recStride,
  int* usePrevFiltBest, int sqrFiltLength, int iFixedFilters
#endif
  )
{
#if JVET_C0038_NO_PREV_FILTERS
  int    varInd, i, j, k, yLocal, filterNo;
  int    ELocal[m_MAX_SQR_FILT_LENGTH];
  int    usePrevFilt[m_NO_VAR_BINS];

  double errorMin, error;

  int noVarBins = m_NO_VAR_BINS;
  int fl = m_flTab[filtType];
  int factor = (1 << (AdaptiveLoopFilter::m_NUM_BITS - 1));
  memset(ELocal, 0, sqrFiltLength*sizeof(int));


  if (bFindBestFixedFilter == false || m_updateMatrix)
  {

    for (varInd = 0; varInd<noVarBins; ++varInd)
    {
      errorMin = 0;
      for (i = 0; i<iFixedFilters; i++)
      {
        filterNo = varInd*iFixedFilters + i;
        error = xTestFixedFilterFast(E_temp, y_temp, pixAcc_temp, m_filterCoeffPrev[filterNo], m_filterCoeffDefault, varInd);
        if (error<errorMin || i == 0)
        {
          errorMin = error;
          usePrevFiltBest[varInd] = i;
        }
      }
    }
    for (varInd = 0; varInd<noVarBins; ++varInd)
    {
      for (i = 0; i < (m_MAX_SQR_FILT_LENGTH / 2 + 1); i++)
      {
        filterNo = (int)(varInd * iFixedFilters + usePrevFiltBest[varInd]);
        m_filterCoeffFinal[varInd][i] = (int)(m_filterCoeffPrev[filterNo][i] * factor);
      }
    }

    error = xTestFixedFilter(ImgDec, ImgOrg, ImgDec, usePrevFilt, noVarBins, orgStride, recStride, filtType);
    xPreFilterFr(m_imgY_preFilter, ImgDec, ImgOrg, ImgDec, usePrevFilt, recStride, filtType);

    for (varInd = 0; varInd<noVarBins; ++varInd)
    {
      if (usePrevFilt[varInd] == 1)
      {
        usePrevFiltBest[varInd]++;
      }
      else
      {
        usePrevFiltBest[varInd] = 0;
        for (i = 0; i < AdaptiveLoopFilter::m_MAX_SQT_FILT_SYM_LENGTH; i++)
        {
          m_filterCoeffFinal[varInd][i] = (int)(m_filterCoeffDefault[i] * factor);
        }
      }
    }
    bFindBestFixedFilter = true;
  }

  // If frNo>0 pixAcc and yGlobalSym have to be calculated again
  if (iFixedFilters)
  {
    if (m_updateMatrix)
    {
      for (varInd = 0; varInd<noVarBins; varInd++)
      {
        pixAcc_temp[varInd] = 0;
        for (k = 0; k<sqrFiltLength; k++)
        {
          y_temp[varInd][k] = 0;
        }
      }
      int transpose;
      int flV = AdaptiveLoopFilter::ALFFlHToFlV(fl);
      for (i = fl; i < m_img_height + fl; i++)
      {
        for (j = fl; j < m_img_width + fl; j++)
        {
          if (m_maskBuf.at(j - fl, i - fl))
          {
            memset(ELocal, 0, sqrFiltLength*sizeof(int));
            varInd = selectTransposeVarInd(m_varImg[i - fl][j - fl], &transpose);
            int pos = (i - fl)*orgStride + (j - fl);
            yLocal = ImgOrg[pos] - m_imgY_preFilter[i - fl][j - fl];
            calcMatrixE(ELocal, ImgDec, m_patternTab[filtType], i - fl, j - fl, flV, fl, transpose, recStride);
            for (k = 0; k<sqrFiltLength; k++)
            {
              y_temp[varInd][k] += (double)(ELocal[k] * yLocal);
            }
            pixAcc_temp[varInd] += (yLocal*yLocal);
          }
        }
      }
    }
  }
#else
  int varInd, i;
  int factor = (1 << (AdaptiveLoopFilter::m_NUM_BITS - 1));

  for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    for (i = 0; i < AdaptiveLoopFilter::m_MAX_SQT_FILT_SYM_LENGTH; i++)
    {
      m_filterCoeffFinal[varInd][i] = (int)(m_filterCoeffDefault[i] * factor);
    }
  }
#endif
}

//********************************
// CHOLESKY
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
int EncAdaptiveLoopFilter::gnsCholeskyDec(double **inpMatr, double outMatr[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], int noEq)
{
  int
  i, j, k;     /* Looping Variables */
  double
  scale;       /* scaling factor for each row */
  double
  invDiag[m_MAX_SQR_FILT_LENGTH];  /* Vector of the inverse of diagonal entries of outMatr */


  /*
   *  Cholesky decomposition starts
   */

  for(i = 0; i < noEq; i++)
  {
    for(j = i; j < noEq; j++)
    {
      /* Compute the scaling factor */
      scale=inpMatr[i][j];
      if ( i > 0) for( k = i - 1 ; k >= 0 ; k--)
        scale -= outMatr[k][j] * outMatr[k][i];

      /* Compute i'th row of outMatr */
      if(i==j)
      {
        if(scale <= REG_SQR ) // if(scale <= 0 )  /* If inpMatr is singular */
        {
          return(0);
        }
        else              /* Normal operation */
          invDiag[i] =  1.0/(outMatr[i][i]=sqrt(scale));
      }
      else
      {
        outMatr[i][j] = scale*invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }
    }
  }
  return(1); /* Signal that Cholesky factorization is successfully performed */
}

void EncAdaptiveLoopFilter::gnsTransposeBacksubstitution(double U[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], double rhs[], double x[], int order)
{
  int
  i,j;              /* Looping variables */
  double
  sum;              /* Holds backsubstitution from already handled rows */

  /* Backsubstitution starts */
  x[0] = rhs[0]/U[0][0];               /* First row of U'                   */
  for (i = 1; i < order; i++)
  {         /* For the rows 1..order-1           */

    for (j = 0, sum = 0.0; j < i; j++) /* Backsubst already solved unknowns */
      sum += x[j]*U[j][i];

    x[i]=(rhs[i] - sum)/U[i][i];       /* i'th component of solution vect.  */
  }
}



void EncAdaptiveLoopFilter::gnsBacksubstitution(double R[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], double z[m_MAX_SQR_FILT_LENGTH], int R_size, double A[m_MAX_SQR_FILT_LENGTH])
{
  int i, j;
  double sum;

  R_size--;

  A[R_size] = z[R_size] / R[R_size][R_size];

  for (i = R_size-1; i >= 0; i--)
  {
    for (j = i+1, sum = 0.0; j <= R_size; j++)
      sum += R[i][j] * A[j];

    A[i] = (z[i] - sum) / R[i][i];
  }
}

int EncAdaptiveLoopFilter::gnsSolveByChol( double **LHS, double *rhs, double *x, int noEq)
{
  double aux[m_MAX_SQR_FILT_LENGTH];     /* Auxiliary vector */
  double U[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH];    /* Upper triangular Cholesky factor of LHS */
  int  i, singular;          /* Looping variable */

  // Initialize to avoid compiler warnings
  for (int i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
  {
    for (int j = 0; j < m_MAX_SQR_FILT_LENGTH; j++)
    {
      U[i][j] = 0.0;
    }
  }

  /* The equation to be solved is LHSx = rhs */

  /* Compute upper triangular U such that U'*U = LHS */
  if(gnsCholeskyDec(LHS, U, noEq)) /* If Cholesky decomposition has been successful */
  {
    singular=1;
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
     * Solve U'*aux = rhs for aux
     */
    gnsTransposeBacksubstitution(U, rhs, aux, noEq);

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution(U, aux, noEq, x);

  }
  else /* LHS was singular */
  {
    singular=0;

    /* Regularize LHS */
    for(i=0; i<noEq; i++)
      LHS[i][i] += REG;
    /* Compute upper triangular U such that U'*U = regularized LHS */
    singular = gnsCholeskyDec(LHS, U, noEq);
    if (singular == 0)
    {
      memset(x, 0, sizeof(double)*noEq);
      return 0;
    }
    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution(U, rhs, aux, noEq);

    /* Solve U*x = aux for x */
    gnsBacksubstitution(U, aux, noEq, x);
  }
  return(singular);
}


//////////////////////////////////////////////////////////////////////////////////////////

void EncAdaptiveLoopFilter::add_A_galf(double **Amerged, double ***A, int interval[], int filtNo, int size)
{
  int i, j, ind;

  for (i = 0; i < size; i++)
  {
    for (j = 0; j < size; j++)
    {
      Amerged[i][j] = 0;
      for (ind = 0; ind < m_NO_VAR_BINS; ind++)
      {
        if (interval[ind] == filtNo)
        {
          Amerged[i][j] += A[ind][i][j];
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::add_b_galf(double *bmerged, double **b, int interval[], int filtNo, int size)
{
  int i, ind;

  for (i = 0; i < size; i++)
  {
    bmerged[i] = 0;
    for (ind = 0; ind < m_NO_VAR_BINS; ind++)
    {
      if (interval[ind] == filtNo)
      {
        bmerged[i] += b[ind][i];
      }
    }
  }
}

void EncAdaptiveLoopFilter::add_A(double **Amerged, double ***A, int start, int stop, int size)
{
  int
  i, j, ind;          /* Looping variable */

  for (i=0; i<size; i++)
  {
    for (j=0; j<size; j++)
    {
      Amerged[i][j]=0;
      for (ind=start; ind<=stop; ind++)
      {
        Amerged[i][j]+=A[ind][i][j];
      }
    }
  }
}

void EncAdaptiveLoopFilter::add_b(double *bmerged, double **b, int start, int stop, int size)
{
  int
  i, ind;          /* Looping variable */

  for (i=0; i<size; i++)
  {
    bmerged[i]=0;
    for (ind=start; ind<=stop; ind++)
    {
      bmerged[i]+=b[ind][i];
    }
  }
}

double EncAdaptiveLoopFilter::xCalcErrorForGivenWeights(double **E, double *y, double *w, int size )
{
  int i, j;
  double error, sum=0;

  error=0;
  for (i=0; i<size; i++)   //diagonal
  {
    sum=0;
    for (j=i+1; j<size; j++)
      sum+=(E[j][i]+E[i][j])*w[j];
    error+=(E[i][i]*w[i]+sum-2*y[i])*w[i];
  }

  return(error);
}

double EncAdaptiveLoopFilter::calculateErrorAbs(double **A, double *b, double y, int size)
{
  int i;
  double error, sum;
  double c[m_MAX_SQR_FILT_LENGTH];

  gnsSolveByChol(A, b, c, size);

  sum=0;
  for (i=0; i<size; i++)
  {
    sum+=c[i]*b[i];
  }
  error=y-sum;

  return(error);
}



int EncAdaptiveLoopFilter::xCheckFilterPredictionMode( int **pDiffQFilterCoeffIntPP, int filters_per_group, AlfFilterType filtType, int& predMode )
{
  int sqrFiltLength = m_sqrFiltLengthTab[ filtType ];
  int bit_ct0 = xcodeFilterCoeff( pDiffQFilterCoeffIntPP, filters_per_group, filtType );
  for( int ind = 0; ind < filters_per_group; ++ind )
  {
    if( ind == 0)
    {
      for( int i = 0; i < sqrFiltLength; i++)
      {
        m_diffFilterCoeffQuant[ind][i] = pDiffQFilterCoeffIntPP[ind][i];
      }
    }
    else
    {
      for( int i = 0; i < sqrFiltLength; i++)
      {
        m_diffFilterCoeffQuant[ind][i] = pDiffQFilterCoeffIntPP[ind][i] - pDiffQFilterCoeffIntPP[ind-1][i];
      }
    }
  }
  int bit_pred1  = xcodeFilterCoeff( m_diffFilterCoeffQuant, filters_per_group, filtType);
  predMode = !!( bit_pred1 < bit_ct0 );
  return 1 + (predMode ? bit_pred1 :  bit_ct0);
}

#if FORCE0
int EncAdaptiveLoopFilter::xCalcBitsForce0(int **pDiffQFilterCoeffIntPP, int filters_per_group, AlfFilterType filtType, int *codedVarBins)
{
  int sqrFiltLength = m_sqrFiltLengthTab[filtType];
  int ind, i, j, len;
  i = 0;
  for (ind = 0; ind < filters_per_group; ind++)
  {
    if (codedVarBins[ind] == 1)
    {
      for (j = 0; j < sqrFiltLength; j++)

        m_FilterCoeffQuantTemp[ind][j] = pDiffQFilterCoeffIntPP[ind][j];

      i++;
    }
    else
    {
      for (j = 0; j < sqrFiltLength; j++)
      {
        m_FilterCoeffQuantTemp[ind][j] = 0;
      }
    }
  }

  len = 1;
  len += xcodeFilterCoeffForce0(m_FilterCoeffQuantTemp, filters_per_group, filtType, codedVarBins);
  return (len);
}
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
void EncAdaptiveLoopFilter::xcopyFilterCoeff(int filtNo, int **filterCoeff)
{
  int varInd, i, k;
  const int *patternMap = m_patternMapTab[filtNo];
  for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    k = 0;
    for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
    {
      if (patternMap[i]>0)
      {
        m_filterCoeffPrevSelected[varInd][i] = filterCoeff[varInd][k];
        k++;
      }
      else
      {
        m_filterCoeffPrevSelected[varInd][i] = 0;
      }
    }
  }
}
#endif

int EncAdaptiveLoopFilter::xcodeFilterCoeff(int **pDiffQFilterCoeffIntPP, int filters_per_group, AlfFilterType filtType)
{
  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal, len = 0,
    kMinTab[m_MAX_SQR_FILT_LENGTH], bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB],
    minKStart, minBitsKStart, bitsKStart;

  const int * pDepthInt = AdaptiveLoopFilter::m_pDepthIntTab[filtType];
  int sqrFiltLength = m_sqrFiltLengthTab[filtType];
  sqrFiltLength = sqrFiltLength - !!m_isGALF;

  maxScanVal = 0;
  for(i = 0; i < sqrFiltLength; i++)
  {
    maxScanVal = std::max(maxScanVal, pDepthInt[i]);
  }

  // vlc for all
  memset(bitsCoeffScan, 0, m_MAX_SCAN_VAL * m_MAX_EXP_GOLOMB * sizeof(int));
  int sumCoeffs = 0;
  for(ind=0; ind<filters_per_group; ++ind)
  {
    for(i = 0; i < sqrFiltLength; i++)
    {
      scanPos=pDepthInt[i]-1;
      coeffVal=abs(pDiffQFilterCoeffIntPP[ind][i]);
      sumCoeffs+=abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (k=1; k<15; k++)
      {
        bitsCoeffScan[scanPos][k]+=lengthGolomb(coeffVal, k);
      }
    }
  }

  minBitsKStart = 0;
  minKStart = -1;
  for(k = 1; k < 8; k++)
  {
    bitsKStart = 0;
    kStart = k;
    for(scanPos = 0; scanPos < maxScanVal; scanPos++)
    {
      kMin = kStart;
      minBits = bitsCoeffScan[scanPos][kMin];

      if(bitsCoeffScan[scanPos][kStart+1] < minBits)
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if((bitsKStart < minBitsKStart) || (k == 1))
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for(scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
    kMin = kStart;
    minBits = bitsCoeffScan[scanPos][kMin];

    if(bitsCoeffScan[scanPos][kStart+1] < minBits)
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  // Coding parameters
  //  len += lengthFilterCodingParams(minKStart, maxScanVal, kMinTab, createBitstream);
  len += (3 + maxScanVal);

  // Filter coefficients
  len += lengthFilterCoeffs(sqrFiltLength, filters_per_group, pDepthInt, pDiffQFilterCoeffIntPP, kMinTab );

  return len;
}

#if FORCE0
int   EncAdaptiveLoopFilter::xcodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int filters_per_group, AlfFilterType filtType, int codedVarBins[])
{
  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal, len = 0,
    kMinTab[m_MAX_SQR_FILT_LENGTH], bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB],
    minKStart, minBitsKStart, bitsKStart;

  const int * pDepthInt = AdaptiveLoopFilter::m_pDepthIntTab[filtType];
  int sqrFiltLength = m_sqrFiltLengthTab[filtType] - 1;
  maxScanVal = 0;
  for (i = 0; i < sqrFiltLength; i++)
  {
    maxScanVal = std::max(maxScanVal, pDepthInt[i]);
  }

  // vlc for all
  memset(bitsCoeffScan, 0, m_MAX_SCAN_VAL * m_MAX_EXP_GOLOMB * sizeof(int));
  for (ind = 0; ind<filters_per_group; ++ind)
  {
    if (!codedVarBins[ind])
    {
      continue;
    }
    for (i = 0; i < sqrFiltLength; i++)
    {
      scanPos = pDepthInt[i] - 1;
      coeffVal = abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (k = 1; k<15; k++)
      {
        bitsCoeffScan[scanPos][k] += lengthGolomb(coeffVal, k);
      }
    }
  }

  minBitsKStart = 0;
  minKStart = -1;
  for (k = 1; k < 8; k++)
  {
    bitsKStart = 0;
    kStart = k;
    for (scanPos = 0; scanPos < maxScanVal; scanPos++)
    {
      kMin = kStart;
      minBits = bitsCoeffScan[scanPos][kMin];

      if (bitsCoeffScan[scanPos][kStart + 1] < minBits)
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if ((bitsKStart < minBitsKStart) || (k == 1))
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for (scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
    kMin = kStart;
    minBits = bitsCoeffScan[scanPos][kMin];

    if (bitsCoeffScan[scanPos][kStart + 1] < minBits)
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  // Coding parameters
  len += (3 + maxScanVal);
  len += filters_per_group;
  // Filter coefficients
  for (ind = 0; ind<filters_per_group; ++ind)
  {
    if (codedVarBins[ind] == 1)
    {
      for (i = 0; i < sqrFiltLength; i++)
      {
        scanPos = pDepthInt[i] - 1;
        len += lengthGolomb(abs(pDiffQFilterCoeffIntPP[ind][i]), kMinTab[scanPos]);
      }
    }
  }

  return len;
}
#endif


int EncAdaptiveLoopFilter::lengthGolomb(int coeffVal, int k)
{
  int m = 2 << (k - 1);
  int q = coeffVal / m;
  if(coeffVal != 0)
    return(q + 2 + k);
  else
    return(q + 1 + k);
}


int EncAdaptiveLoopFilter::lengthFilterCoeffs( int sqrFiltLength, int filters_per_group, const int pDepthInt[],
                                               int **FilterCoeff, int kMinTab[])
{
  int ind, scanPos, i;
  int bit_cnt = 0;

  for(ind = 0; ind < filters_per_group; ++ind)
  {
    for(i = 0; i < sqrFiltLength; i++)
    {
      scanPos = pDepthInt[i] - 1;
      bit_cnt += lengthGolomb(abs(FilterCoeff[ind][i]), kMinTab[scanPos]);
    }
  }
  return bit_cnt;
}


//###################################
// Filter Type Decsion
//###################################

void EncAdaptiveLoopFilter::xFilterTypeDecision(  CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiMinRate, uint64_t& ruiMinDist, double& rdMinCost, const Slice*  slice)
{

  // restriction for non-referenced B-slice
  if (m_eSliceType == B_SLICE && m_iPicNalReferenceIdc == 0)
  {
    return;
  }

  uint64_t uiRate, uiDist;
  double dCost;

  bool bChanged = false;
  const int maxListSize = 3;
  const static int filterTypeList[2][maxListSize] = {{ ALF_FILTER_SYM_5, ALF_FILTER_SYM_7, ALF_FILTER_SYM_9 }, { ALF_FILTER_SYM_9, ALF_FILTER_SYM_7, ALF_FILTER_SYM_5 }};

  for( int n = 0; n < maxListSize; n++)
  {
    int filtTypeIdx = filterTypeList[!!m_isGALF][n];

    if( m_isGALF )
    {
      if( m_eSliceType == I_SLICE && filtTypeIdx == ALF_FILTER_SYM_9)
      {
        continue;
      }
      if (filtTypeIdx != ALF_FILTER_SYM_9 && !m_pcTempAlfParam->cu_control_flag)
      {
        m_updateMatrix = false;
      }
    }
    AlfFilterType filtType = (AlfFilterType) filtTypeIdx;
    copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
    m_pcTempAlfParam->filterType = filtType;
    m_pcTempAlfParam->tapH      = m_mapTypeToNumOfTaps[filtType];
    m_pcTempAlfParam->tapV      = AdaptiveLoopFilter::ALFTapHToTapV    (m_pcTempAlfParam->tapH);
    m_pcTempAlfParam->num_coeff = AdaptiveLoopFilter::ALFTapHToNumCoeff(m_pcTempAlfParam->tapH);

    if (m_pcTempAlfParam->cu_control_flag)
    {
       xReDesignFilterCoeff( orgUnitBuf, recExtBuf, m_tempPelBuf,
#if COM16_C806_ALF_TEMPPRED_NUM
        m_pcTempAlfParam,
#endif
        cs.slice->clpRng(COMPONENT_Y));
       xSetCUAlfCtrlFlags( cs, orgUnitBuf, recExtBuf, m_tempPelBuf, uiDist, m_pcTempAlfParam->alf_max_depth, m_pcTempAlfParam );
       xCalcRDCostLuma( orgUnitBuf,  m_tempPelBuf, m_pcTempAlfParam, uiRate, uiDist, dCost );
    }
    else
    {
      m_maskBuf.fill(1); //TODO why is this needed?
      xReDesignFilterCoeff( orgUnitBuf, recExtBuf, m_tempPelBuf,
#if COM16_C806_ALF_TEMPPRED_NUM
        m_pcTempAlfParam,
#endif
        cs.slice->clpRng(COMPONENT_Y));
      xCalcRDCostLuma( orgUnitBuf,  m_tempPelBuf, m_pcTempAlfParam, uiRate, uiDist, dCost );
    }

    if (dCost < rdMinCost )
    {
      rdMinCost = dCost;
      ruiMinDist = uiDist;
      ruiMinRate = uiRate;
      m_bestPelBuf.get(COMPONENT_Y).copyFrom( m_tempPelBuf.get(COMPONENT_Y));
      copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
      bChanged = true;
    }
  }

  if( bChanged )
  {
   recUnitBuf.get(COMPONENT_Y).copyFrom( m_bestPelBuf.get(COMPONENT_Y) );
  }
  copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
}

//redesign using m_pcTempAlfParameter
void EncAdaptiveLoopFilter::xReDesignFilterCoeff( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& dstUnitBuf,
#if COM16_C806_ALF_TEMPPRED_NUM
  ALFParam* alfParam,
#endif
  const ClpRng& clpRng )
{
  AlfFilterType filtType = m_pcTempAlfParam->filterType;
  xStoreInBlockMatrix(orgUnitBuf, recExtBuf, filtType);
//  xFindFilterCoeffsLuma(orgUnitBuf, recExtBuf, filtType);

  if( m_isGALF )
  {
    xCheckFilterMergingGalf( *m_pcTempAlfParam
  #if JVET_C0038_NO_PREV_FILTERS
      , orgUnitBuf, recExtBuf
  #endif
      );
    xFilterFrame_enGalf(dstUnitBuf, recExtBuf, filtType,
   #if COM16_C806_ALF_TEMPPRED_NUM
      alfParam, true,
  #endif
     clpRng );
  }
  else
  {
    xCheckFilterMergingAlf( *m_pcTempAlfParam
  #if JVET_C0038_NO_PREV_FILTERS
      , orgUnitBuf, recExtBuf
  #endif
      );
    xFilterFrame_enAlf(dstUnitBuf, recExtBuf, filtType,
   #if COM16_C806_ALF_TEMPPRED_NUM
      alfParam, true,
  #endif
     clpRng );
  }
}


void EncAdaptiveLoopFilter::xCalcRDCost( const uint64_t uiDist, ALFParam* pAlfParam, uint64_t& ruiRate,  double& rdCost )
{
  if(pAlfParam != NULL)
  {
    m_CABACEstimator->initCtxModels( *m_pSlice, m_CABACDataStore );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->alf( *pAlfParam, m_eSliceType, m_isGALF );
    ruiRate = m_CABACEstimator->getEstFracBits() >> SCALE_BITS;
  }
  else
  {
    ruiRate = 1;
  }
  rdCost = (double)(ruiRate) * m_dLambdaLuma + (double)(uiDist);
}


void EncAdaptiveLoopFilter::xCalcRDCostLuma( const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recBuf, ALFParam* pAlfParam, uint64_t& ruiRate, uint64_t& ruiDist, double& rdCost )
{
  if(pAlfParam != NULL)
  {
    m_CABACEstimator->initCtxModels( *m_pSlice, m_CABACDataStore );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->alf( *pAlfParam, m_eSliceType, m_isGALF );
    ruiRate = m_CABACEstimator->getEstFracBits() >> SCALE_BITS;
  }
  else
  {
    ruiRate = 1;
  }
  ruiDist = xCalcSSD( orgUnitBuf, recBuf, COMPONENT_Y);
  rdCost  = (double)(ruiRate) * m_dLambdaLuma + (double)(ruiDist);
}

uint64_t EncAdaptiveLoopFilter::xCalcSSD(const CPelUnitBuf& OrgBuf, const CPelUnitBuf& CmpBuf,  const ComponentID compId)
{
  return xCalcSSD(OrgBuf.get(compId), CmpBuf.get(compId));
}

uint64_t EncAdaptiveLoopFilter::xCalcSSD(const CPelBuf& refBuf, const CPelBuf& cmpBuf)
{
  int iWidth = refBuf.width;
  int iHeight = refBuf.height;
  int orgStride = refBuf.stride;
  int cmpStride = cmpBuf.stride;
  const Pel* pOrg = refBuf.buf;
  const Pel* pCmp = cmpBuf.buf;

  uint64_t uiSSD = 0;
  int x, y;

  uint32_t uiShift = m_nBitIncrement<<1;
  int iTemp;

  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iTemp = pOrg[x] - pCmp[x]; uiSSD += ( iTemp * iTemp ) >> uiShift;
    }
    pOrg += orgStride;
    pCmp += cmpStride;
  }
  return uiSSD;;
}



//#####################################
//   CHROMA RELATED
//####################################

void EncAdaptiveLoopFilter::xCalcRDCostChroma( const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recBuf, ALFParam* pAlfParam, uint64_t& ruiRate, uint64_t& ruiDist, double& rdCost )
{
  if( pAlfParam->chroma_idc )
  {
    int* piTmpCoef = nullptr;
    if( ! m_isGALF )
    {
      piTmpCoef = new int[m_ALF_MAX_NUM_COEF_C];
      memcpy(piTmpCoef, pAlfParam->coeff_chroma, sizeof(int)*pAlfParam->num_coeff_chroma);
      predictALFCoeffChroma(pAlfParam); //TODO!!!!
    }

    m_CABACEstimator->initCtxModels( *m_pSlice, m_CABACDataStore );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->alf( *pAlfParam, m_eSliceType, m_isGALF );

    ruiRate = m_CABACEstimator->getEstFracBits() >> SCALE_BITS;
    if( ! m_isGALF )
    {
      memcpy(pAlfParam->coeff_chroma, piTmpCoef, sizeof(int)*pAlfParam->num_coeff_chroma);
      delete[] piTmpCoef;
      piTmpCoef = NULL;
    }
  }

  ruiDist = 0;
  ruiDist += xCalcSSD(orgUnitBuf, recBuf, COMPONENT_Cb);
  ruiDist += xCalcSSD(orgUnitBuf, recBuf, COMPONENT_Cr);
  rdCost  = (double)(ruiRate) * m_dLambdaChroma + (double)(ruiDist);
}


void EncAdaptiveLoopFilter::xFilteringFrameChroma(  const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf )
{
  int tap;
  int* qh;

  tap  = m_pcTempAlfParam->tap_chroma;
  qh   = m_pcTempAlfParam->coeff_chroma;

  if( m_isGALF )
  {
    if (m_pcTempAlfParam->chroma_idc)
    {
      int filtType = 0;
      if (tap == 9) filtType = 2;
      else if (tap == 7) filtType = 1;

      xstoreInBlockMatrixForChroma(orgUnitBuf, recExtBuf, tap, m_pcTempAlfParam->chroma_idc);
      //static double pixAcc_temp;
      int sqrFiltLength = m_sqrFiltLengthTab[filtType];
      const int* weights = m_weightsTab[filtType];
      int  bit_depth = m_NUM_BITS;
      // Find coeffcients
      xSolveAndQuant(m_filterCoeff, qh, m_EGlobalSym[filtType][0], m_yGlobalSym[filtType][0], sqrFiltLength, weights, bit_depth, true);
  #if COM16_C806_ALF_TEMPPRED_NUM
      memcpy(m_pcTempAlfParam->alfCoeffChroma, qh, sizeof(int)*sqrFiltLength);
  #endif
    }
  }
  else
  {
    int N = m_pcTempAlfParam->num_coeff_chroma;
    // initialize correlation
    for(int i=0; i<N; i++)
      memset(m_ppdAlfCorr[i], 0, sizeof(double)*(N+1));

    if ((m_pcTempAlfParam->chroma_idc>>1)&0x01)
    {
      xCalcCorrelationFunc( orgUnitBuf.get(COMPONENT_Cb), recExtBuf.get(COMPONENT_Cb), tap );
    }
    if ((m_pcTempAlfParam->chroma_idc)&0x01)
    {
      xCalcCorrelationFunc( orgUnitBuf.get(COMPONENT_Cr), recExtBuf.get(COMPONENT_Cb), tap );
    }

    int err_code = xGauss(m_ppdAlfCorr, N);

    if(err_code)
    {
      xClearFilterCoefInt(qh, N);
    }
    else
    {
      for(int i=0; i<N; i++)
        m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][N];

#if DISTORTION_LAMBDA_BUGFIX
      xQuantFilterCoefChroma(m_pdDoubleAlfCoeff, qh, tap, m_nInternalBitDepth);
#else
      xQuantFilterCoefChroma(m_pdDoubleAlfCoeff, qh, tap, m_nInputBitDepth + m_nBitIncrement);
#endif
    }
  }

  if( m_isGALF )
  {
    if (m_pcTempAlfParam->chroma_idc)
    {
      initVarForChroma(m_pcTempAlfParam, true);
    }
    if ((m_pcTempAlfParam->chroma_idc >> 1) & 0x01)
    {
      xFrameChromaGalf(m_pcTempAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cb);
    }
    if ((m_pcTempAlfParam->chroma_idc) & 0x01)
    {
      xFrameChromaGalf(m_pcTempAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cr);
    }
  }
  else
  {
    if ((m_pcTempAlfParam->chroma_idc >> 1) & 0x01)
    {
      xFrameChromaAlf( m_pcTempAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cb);
    }
    if ((m_pcTempAlfParam->chroma_idc) & 0x01)
    {
      xFrameChromaAlf( m_pcTempAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cr);
    }
  }

  if (m_pcTempAlfParam->chroma_idc<3)
  {
    if (m_pcTempAlfParam->chroma_idc==1)
    {
      recUnitBuf.get(COMPONENT_Cb).copyFrom(recExtBuf.get(COMPONENT_Cb));
    }
    if (m_pcTempAlfParam->chroma_idc==2)
    {
      recUnitBuf.get(COMPONENT_Cr).copyFrom(recExtBuf.get(COMPONENT_Cr));
    }
  }

}

#if FORCE0
void EncAdaptiveLoopFilter::xcollectStatCodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int filtType, int sqrFiltLength,
  int filters_per_group, int bitsVarBin[])
{
  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal,
    kMinTab[m_MAX_SQR_FILT_LENGTH], bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB],
    minKStart, minBitsKStart, bitsKStart;

  const int * pDepthInt = AdaptiveLoopFilter::m_pDepthIntTab[filtType];

  sqrFiltLength = sqrFiltLength - !!m_isGALF;

  maxScanVal = 0;
  for (i = 0; i<sqrFiltLength; i++)
  {
    maxScanVal = std::max(maxScanVal, pDepthInt[i]);
  }

  // vlc for all
  memset(bitsCoeffScan, 0, m_MAX_SCAN_VAL * m_MAX_EXP_GOLOMB * sizeof(int));
  for (ind = 0; ind<filters_per_group; ++ind)
  {
    for (i = 0; i < sqrFiltLength; i++)
    {
      scanPos = pDepthInt[i] - 1;
      coeffVal = abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (k = 1; k<15; k++)
      {
        bitsCoeffScan[scanPos][k] += lengthGolomb(coeffVal, k);
      }
    }
  }

  minBitsKStart = 0;
  minKStart = -1;
  for (k = 1; k<8; k++)
  {
    bitsKStart = 0; kStart = k;
    for (scanPos = 0; scanPos<maxScanVal; scanPos++)
    {
      kMin = kStart; minBits = bitsCoeffScan[scanPos][kMin];

      if (bitsCoeffScan[scanPos][kStart + 1]<minBits)
      {
        kMin = kStart + 1; minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if (bitsKStart<minBitsKStart || k == 1)
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for (scanPos = 0; scanPos<maxScanVal; scanPos++)
  {
    kMin = kStart; minBits = bitsCoeffScan[scanPos][kMin];

    if (bitsCoeffScan[scanPos][kStart + 1]<minBits)
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  for (ind = 0; ind<filters_per_group; ++ind)
  {
    bitsVarBin[ind] = 0;
    for (i = 0; i < sqrFiltLength; i++)
    {
      scanPos = pDepthInt[i] - 1;
      bitsVarBin[ind] += lengthGolomb(abs(pDiffQFilterCoeffIntPP[ind][i]), kMinTab[scanPos]);
    }
  }
}

void EncAdaptiveLoopFilter::xdecideCoeffForce0(int codedVarBins[m_NO_VAR_BINS], double *distForce0, double errorForce0CoeffTab[m_NO_VAR_BINS][2], int bitsVarBin[m_NO_VAR_BINS], double lambda, int filters_per_fr)
{
  int filtNo;
  int ind;

  *distForce0 = 0;
  for (ind = 0; ind< m_NO_VAR_BINS; ind++)
  {
    codedVarBins[ind] = 0;
  }

  for (filtNo = 0; filtNo<filters_per_fr; filtNo++)
  {
    double lagrangianDiff = errorForce0CoeffTab[filtNo][0] - (errorForce0CoeffTab[filtNo][1] + lambda*bitsVarBin[filtNo]);
    codedVarBins[filtNo] = (lagrangianDiff>0) ? 1 : 0;
    *distForce0 += errorForce0CoeffTab[filtNo][codedVarBins[filtNo] ? 1 : 0];
  }
}
#endif

void EncAdaptiveLoopFilter::xCalcCorrelationFunc(const CPelBuf& rcOrgBuf, const CPelBuf& rcCmpBuf, int iTap)
{
  int iTapV   = AdaptiveLoopFilter::ALFTapHToTapV(iTap);
  int N       = (iTap * iTapV + 1) >> 1;
  int offsetV = iTapV >> 1;
  int offset = iTap>>1;

  const int* pFiltPos;

  switch(iTap)
  {
    case 5:
      pFiltPos = m_aiSymmetricArray5x5;
      break;
    case 7:
      pFiltPos = m_aiSymmetricArray7x7;
      break;
    case 9:
      pFiltPos = m_aiSymmetricArray9x7;
      break;
    default:
      pFiltPos = m_aiSymmetricArray9x7;
      THROW( "ALF: wrong number of taps for chroma" );
      break;
  }

  Pel* pTerm = new Pel[N];

  const Pel* pCmp = rcCmpBuf.buf;
  const Pel* pOrg = rcOrgBuf.buf;
  const int iWidth  = rcOrgBuf.width;
  const int iHeight = rcOrgBuf.height;
  const int iOrgStride = rcOrgBuf.stride;
  const int iCmpStride = rcCmpBuf.stride;
  int i, j;
  for (int y = 0; y < iHeight; y++)
  {
    for (int x = 0; x < iWidth; x++)
    {
      i = 0;
      ::memset(pTerm, 0, sizeof(Pel)*N);
      for (int yy = y - offsetV; yy <= y + offsetV; yy++)
      {
        for(int xx=x-offset; xx<=x+offset; xx++)
        {
          pTerm[pFiltPos[i]] += pCmp[xx + yy*iCmpStride];
          i++;
        }
      }

      for(j=0; j<N; j++)
      {
        m_ppdAlfCorr[j][j] += pTerm[j]*pTerm[j];
        for(i=j+1; i<N; i++)
          m_ppdAlfCorr[j][i] += pTerm[j]*pTerm[i];

        // DC offset
        m_ppdAlfCorr[j][N]   += pTerm[j];
        m_ppdAlfCorr[j][N+1] += pOrg[x+y*iOrgStride]*pTerm[j];
      }
      // DC offset
      for(i=0; i<N; i++)
        m_ppdAlfCorr[N][i] += pTerm[i];
      m_ppdAlfCorr[N][N]   += 1;
      m_ppdAlfCorr[N][N+1] += pOrg[x+y*iOrgStride];
    }
  }
  for(j=0; j<N-1; j++)
  {
    for(i=j+1; i<N; i++)
      m_ppdAlfCorr[i][j] = m_ppdAlfCorr[j][i];
  }

  delete[] pTerm;
  pTerm = NULL;
}

int EncAdaptiveLoopFilter::xGauss(double **a, int N)
{
  int i, j, k;
  double t;

  for(k=0; k<N; k++)
  {
    if (a[k][k] <0.000001)
      return 1;
  }

  for(k=0; k<N-1; k++)
  {
    for(i=k+1;i<N; i++)
    {
      t=a[i][k]/a[k][k];
      for(j=k+1; j<=N; j++)
      {
        a[i][j] -= t * a[k][j];
        if(i==j && fabs(a[i][j])<0.000001) return 1;
      }
    }
  }
  for(i=N-1; i>=0; i--)
  {
    t = a[i][N];
    for(j=i+1; j<N; j++)
      t -= a[i][j] * a[j][N];
    a[i][N] = t / a[i][i];
  }
  return 0;
}

void EncAdaptiveLoopFilter::xClearFilterCoefInt(int* qh, int N)
{
  // clear
  memset( qh, 0, sizeof( int ) * N );

  // center pos
  qh[N-2]  = 1<<m_ALF_NUM_BIT_SHIFT;
}


void EncAdaptiveLoopFilter::xQuantFilterCoefChroma(double* h, int* qh, int tap, int bit_depth)
{
  int i, N;
  int max_value, min_value;
  double dbl_total_gain;
  int total_gain, q_total_gain;
  int upper, lower;
  double *dh;
  int    *nc;
  const int    *pFiltMag;

  switch(tap)
  {
    case 5:
      pFiltMag = m_aiSymmetricMag5x5;
      break;
    case 7:
      pFiltMag = m_aiSymmetricMag7x7;
      break;
    case 9:
      pFiltMag = m_aiSymmetricMag9x7;
      break;
    default:
      pFiltMag = m_aiSymmetricMag9x7;
      THROW( "ALF:CHROMA: wrong number of taps" );
      break;
  }

  int tapV = AdaptiveLoopFilter::ALFTapHToTapV(tap);
  N = (tap * tapV + 1) >> 1;

  dh = new double[N];
  nc = new int[N];

  max_value =   (1<<(1+m_ALF_NUM_BIT_SHIFT))-1;
  min_value = 0-(1<<(1+m_ALF_NUM_BIT_SHIFT));

  dbl_total_gain=0.0;
  q_total_gain=0;
  for(i=0; i<N; i++)
  {
    if(h[i]>=0.0)
      qh[i] =  (int)( h[i]*(1<<m_ALF_NUM_BIT_SHIFT)+0.5);
    else
      qh[i] = -(int)(-h[i]*(1<<m_ALF_NUM_BIT_SHIFT)+0.5);

    dh[i] = (double)qh[i]/(double)(1<<m_ALF_NUM_BIT_SHIFT) - h[i];
    dh[i]*=pFiltMag[i];
    dbl_total_gain += h[i]*pFiltMag[i];
    q_total_gain   += qh[i]*pFiltMag[i];
    nc[i] = i;
  }

  // modification of quantized filter coefficients
  total_gain = (int)(dbl_total_gain*(1<<m_ALF_NUM_BIT_SHIFT)+0.5);

  if( q_total_gain != total_gain )
  {
    xFilterCoefQuickSort(dh, nc, 0, N-1);
    if( q_total_gain > total_gain )
    {
      upper = N-1;
      while( q_total_gain > total_gain+1 )
      {
        i = nc[upper%N];
        qh[i]--;
        q_total_gain -= pFiltMag[i];
        upper--;
      }
      if( q_total_gain == total_gain+1 )
      {
        if(dh[N-1]>0)
          qh[N-1]--;
        else
        {
          i=nc[upper%N];
          qh[i]--;
          qh[N-1]++;
        }
      }
    }
    else if( q_total_gain < total_gain )
    {
      lower = 0;
      while( q_total_gain < total_gain-1 )
      {
        i=nc[lower%N];
        qh[i]++;
        q_total_gain += pFiltMag[i];
        lower++;
      }
      if( q_total_gain == total_gain-1 )
      {
        if(dh[N-1]<0)
          qh[N-1]++;
        else
        {
          i=nc[lower%N];
          qh[i]++;
          qh[N-1]--;
        }
      }
    }
  }

  // set of filter coefficients
  for(i=0; i<N; i++)
  {
    qh[i] = std::max(min_value,std::min(max_value, qh[i]));
  }

  // DC offset
#if DISTORTION_LAMBDA_BUGFIX
  max_value = std::min((1 << (3 + m_nInternalBitDepth)) - 1, (1 << 14) - 1);
  min_value = std::max(-(1 << (3 + m_nInternalBitDepth)), -(1 << 14));
#else
  max_value = std::min((1 << (3 + m_nInputBitDepth + m_nBitIncrement)) - 1, (1 << 14) - 1);
  min_value = std::max(-(1 << (3 + m_nInputBitDepth + m_nBitIncrement)), -(1 << 14));
#endif

  qh[N] =  (h[N]>=0.0)? (int)( h[N]*(1<<(m_ALF_NUM_BIT_SHIFT-bit_depth+8)) + 0.5) : -(int)(-h[N]*(1<<(m_ALF_NUM_BIT_SHIFT-bit_depth+8)) + 0.5);
  qh[N] = std::max(min_value,std::min(max_value, qh[N]));

  delete[] dh;
  dh = NULL;

  delete[] nc;
  nc = NULL;
}

void EncAdaptiveLoopFilter::xFilterCoefQuickSort( double *coef_data, int *coef_num, int upper, int lower )
{
  double mid, tmp_data;
  int i, j, tmp_num;

  i = upper;
  j = lower;
  mid = coef_data[(lower+upper)>>1];
  do
  {
    while( coef_data[i] < mid ) i++;
    while( mid < coef_data[j] ) j--;
    if( i <= j )
    {
      tmp_data = coef_data[i];
      tmp_num  = coef_num[i];
      coef_data[i] = coef_data[j];
      coef_num[i]  = coef_num[j];
      coef_data[j] = tmp_data;
      coef_num[j]  = tmp_num;
      i++;
      j--;
    }
  } while( i <= j );
  if ( upper < j ) xFilterCoefQuickSort(coef_data, coef_num, upper, j);
  if ( i < lower ) xFilterCoefQuickSort(coef_data, coef_num, i, lower);
}

#if COM16_C806_ALF_TEMPPRED_NUM
bool EncAdaptiveLoopFilter::xFilteringLumaChroma(CodingStructure& cs, ALFParam *pAlfParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, uint64_t& ruiMinRate, uint64_t& ruiMinDist, double& rdMinCost, int uiIndex, const Slice* pSlice)
{
  uint64_t uiRate, uiDist = 0;
  double dCost;

  //uint32_t   uiTmpMaxDepth = pAlfParam->alf_max_depth;
  //uint32_t   uiTmpAlfCtrlFlag = pAlfParam->cu_control_flag;
  m_pcTempAlfParam->temporalPredFlag = true;
  copyALFParam(m_pcTempAlfParam, pAlfParam);
  xcopyFilterCoeff(m_pcTempAlfParam->filterType, m_pcTempAlfParam->alfCoeffLuma);
  m_pcTempAlfParam->cu_control_flag = 0;
  m_pcTempAlfParam->prevIdx = uiIndex;
  m_pcTempAlfParam->alf_flag = 1;
  m_pcTempAlfParam->chroma_idc = 0;
  m_pcBestAlfParam->temporalPredFlag = false;
  m_varImg = m_varImgMethods;

  if( m_isGALF )
  {
    xFilterFrame_enGalf(recUnitBuf, recExtBuf, m_pcTempAlfParam->filterType, m_pcTempAlfParam, false, cs.slice->clpRng(COMPONENT_Y));
  }
  else
  {
    xFilterFrame_enAlf(recUnitBuf, recExtBuf, m_pcTempAlfParam->filterType,  m_pcTempAlfParam, false, cs.slice->clpRng(COMPONENT_Y));
  }

  uiDist = xCalcSSD(orgUnitBuf, recUnitBuf, COMPONENT_Y);
  xCalcRDCost(uiDist, m_pcTempAlfParam, uiRate, dCost);
  if (dCost < rdMinCost)
  {
    rdMinCost = dCost;
    ruiMinDist = uiDist;
    ruiMinRate = uiRate;
    m_pcBestAlfParam->temporalPredFlag = true;
    copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
    m_pcBestAlfParam->prevIdx = uiIndex;
    m_pcBestAlfParam->alf_flag = 1;
    m_pcBestAlfParam->cu_control_flag = 0;

    //if (m_pcBestAlfParam->tap >= m_ALF_MIN_NUM_TAP) // fixed a bug with HM-3
    {
      m_bestPelBuf.get(COMPONENT_Y).copyFrom(recUnitBuf.get(COMPONENT_Y));
    }
  }

  m_pcTempAlfParam->cu_control_flag = 1;
  uint32_t uiBestDepth = 0;
  bool bChanged = false;

  for (uint32_t uiDepth = 0; uiDepth < m_uiMaxTotalCUDepth; uiDepth++)
  {
    m_pcTempAlfParam->alf_max_depth = uiDepth;
    m_tempPelBuf.get(COMPONENT_Y).copyFrom(recUnitBuf.get(COMPONENT_Y));
    uint64_t uiTmpRate, uiTmpDist;
    double dTmpCost;
    //m_pcPicYuvTmp: filtered signal, pcPicDec: orig reconst
    xSetCUAlfCtrlFlags(cs, orgUnitBuf, recExtBuf, m_tempPelBuf, uiTmpDist, uiDepth, m_pcTempAlfParam);
    xCalcRDCost(uiTmpDist, m_pcTempAlfParam, uiTmpRate, dTmpCost);
    if (dTmpCost < rdMinCost)
    {
      bChanged = true;
      uiBestDepth = uiDepth;
      rdMinCost = dTmpCost;
      ruiMinDist = uiTmpDist;
      ruiMinRate = uiTmpRate;
      m_bestPelBuf.get(COMPONENT_Y).copyFrom(m_tempPelBuf.get(COMPONENT_Y));
      m_pcBestAlfParam->temporalPredFlag = false;
      copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
      m_pcBestAlfParam->temporalPredFlag = true;
    }
  }

  if (bChanged)
  {
    m_pcBestAlfParam->alf_flag = true;
    m_pcBestAlfParam->prevIdx = uiIndex;
    m_pcBestAlfParam->cu_control_flag = true;
    m_pcBestAlfParam->alf_max_depth = uiBestDepth;
  }

  if (!m_pcBestAlfParam->temporalPredFlag)
  {
    return false;
  }

  m_pcBestAlfParam->chroma_idc = 0;

  if (pAlfParam->chroma_idc != 0)
  {
    memcpy(pAlfParam->coeff_chroma, pAlfParam->alfCoeffChroma, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
    initVarForChroma(pAlfParam, true);
    xFrameChromaGalf(pAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cb);
    xFrameChromaGalf(pAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cr);

    uint64_t uiDistOrg;

    uiDist    = xCalcSSD(orgUnitBuf, recUnitBuf, COMPONENT_Cb);
    uiDistOrg = xCalcSSD(orgUnitBuf, recExtBuf, COMPONENT_Cb);

    if (uiDist < uiDistOrg)
    {
      m_pcBestAlfParam->chroma_idc |= 2;
      m_bestPelBuf.get(COMPONENT_Cb).copyFrom(recUnitBuf.get(COMPONENT_Cb));
    }
    else
    {
      m_bestPelBuf.get(COMPONENT_Cb).copyFrom(recExtBuf.get(COMPONENT_Cb));
    }
    uiDist    = xCalcSSD(orgUnitBuf, recUnitBuf,COMPONENT_Cr);
    uiDistOrg = xCalcSSD(orgUnitBuf, recExtBuf, COMPONENT_Cr);

    if (uiDist < uiDistOrg)
    {
      m_pcBestAlfParam->chroma_idc |= 1;
      m_bestPelBuf.get(COMPONENT_Cr).copyFrom(recUnitBuf.get(COMPONENT_Cr));
    }
    else
    {
      m_bestPelBuf.get(COMPONENT_Cr).copyFrom(recExtBuf.get(COMPONENT_Cr));
    }
  }
  else
  {
    m_bestPelBuf.get(COMPONENT_Cb).copyFrom(recExtBuf.get(COMPONENT_Cb));
    m_bestPelBuf.get(COMPONENT_Cr).copyFrom(recExtBuf.get(COMPONENT_Cr));
  }

  return true;
}
#endif

void EncAdaptiveLoopFilter::xDeriveGlobalEyFromLgrTapFilter(double **E0, double *y0, double **E1, double *y1, const int *pattern0, const int *pattern1)
{
  int i, j, l, k = 0;
  for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
  {
    if (pattern0[i] > 0)
    {
      if (pattern1[i] > 0)
      {
        l = 0;
        y1[pattern1[i] - 1] = y0[k];
        for (j = 0; j < m_MAX_SQR_FILT_LENGTH; j++)
        {
          if (pattern0[j] > 0)
          {
            if (pattern1[j] > 0)
            {
              E1[pattern1[i] - 1][pattern1[j] - 1] = E0[k][l];
            }
            l++;
          }
        }
      }
      k++;
    }
  }
}
void EncAdaptiveLoopFilter::xDeriveLocalEyFromLgrTapFilter(double *y0, double *y1, const int *pattern0, const int *pattern1)
{
  int i, k = 0;
  for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
  {
    if (pattern0[i] > 0)
    {
      if (pattern1[i] > 0)
      {
        y1[pattern1[i] - 1] = y0[k];
      }
      k++;
    }
  }
}


#endif

