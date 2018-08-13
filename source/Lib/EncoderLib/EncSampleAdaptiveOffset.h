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

/**
 \file     EncSampleAdaptiveOffset.h
 \brief    estimation part of sample adaptive offset class (header)
 */

#ifndef __ENCSAMPLEADAPTIVEOFFSET__
#define __ENCSAMPLEADAPTIVEOFFSET__

#include "CommonLib/SampleAdaptiveOffset.h"

#include "CABACWriter.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct SAOStatData //data structure for SAO statistics
{
  Int64 diff[MAX_NUM_SAO_CLASSES];
  Int64 count[MAX_NUM_SAO_CLASSES];

  SAOStatData(){}
  ~SAOStatData(){}
  Void reset()
  {
    ::memset(diff, 0, sizeof(Int64)*MAX_NUM_SAO_CLASSES);
    ::memset(count, 0, sizeof(Int64)*MAX_NUM_SAO_CLASSES);
  }
  const SAOStatData& operator=(const SAOStatData& src)
  {
    ::memcpy(diff, src.diff, sizeof(Int64)*MAX_NUM_SAO_CLASSES);
    ::memcpy(count, src.count, sizeof(Int64)*MAX_NUM_SAO_CLASSES);
    return *this;
  }
  const SAOStatData& operator+= (const SAOStatData& src)
  {
    for(Int i=0; i< MAX_NUM_SAO_CLASSES; i++)
    {
      diff[i] += src.diff[i];
      count[i] += src.count[i];
    }
    return *this;
  }
};

class EncSampleAdaptiveOffset : public SampleAdaptiveOffset
{
public:
  EncSampleAdaptiveOffset();
  virtual ~EncSampleAdaptiveOffset();

  //interface
  Void createEncData(Bool isPreDBFSamplesUsed, UInt numCTUsPic);
  Void destroyEncData();
#if JEM_TOOLS
  Void initCABACEstimator( CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice );
#else
  Void initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice );
#endif
#if K0238_SAO_GREEDY_MERGE_ENCODING
  Void SAOProcess(CodingStructure& cs, Bool* sliceEnabled, const Double *lambdas, const Bool bTestSAODisableAtPictureLevel, const Double saoEncodingRate, const Double saoEncodingRateChroma, Bool isPreDBFSamplesUsed, bool isGreedymergeEncoding);
#else
  Void SAOProcess(CodingStructure& cs, Bool* sliceEnabled, const Double *lambdas, const Bool bTestSAODisableAtPictureLevel, const Double saoEncodingRate, const Double saoEncodingRateChroma, Bool isPreDBFSamplesUsed);
#endif

  Void disabledRate( CodingStructure& cs, SAOBlkParam* reconParams, const Double saoEncodingRate, const Double saoEncodingRateChroma );
  Void getPreDBFStatistics(CodingStructure& cs);
private: //methods

  Void deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos, Bool& isLeftAvail, Bool& isAboveAvail, Bool& isAboveLeftAvail) const;
  Void getStatistics(std::vector<SAOStatData**>& blkStats, PelUnitBuf& orgYuv, PelUnitBuf& srcYuv, CodingStructure& cs, Bool isCalculatePreDeblockSamples = false);
  Void decidePicParams(const Slice& slice, Bool* sliceEnabled, const Double saoEncodingRate, const Double saoEncodingRateChroma);
#if K0238_SAO_GREEDY_MERGE_ENCODING
  Void decideBlkParams(CodingStructure& cs, Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, PelUnitBuf& srcYuv, PelUnitBuf& resYuv, SAOBlkParam* reconParams, SAOBlkParam* codedParams, const Bool bTestSAODisableAtPictureLevel, const Double saoEncodingRate, const Double saoEncodingRateChroma, const bool isGreedymergeEncoding);
#else
  Void decideBlkParams(CodingStructure& cs, Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, PelUnitBuf& srcYuv, PelUnitBuf& resYuv, SAOBlkParam* reconParams, SAOBlkParam* codedParams, const Bool bTestSAODisableAtPictureLevel, const Double saoEncodingRate, const Double saoEncodingRateChroma);
#endif
  Void getBlkStats(const ComponentID compIdx, const Int channelBitDepth, SAOStatData* statsDataTypes, Pel* srcBlk, Pel* orgBlk, Int srcStride, Int orgStride, Int width, Int height, Bool isLeftAvail,  Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isCalculatePreDeblockSamples);
  Void deriveModeNewRDO(const BitDepths &bitDepths, Int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, Double& modeNormCost );
  Void deriveModeMergeRDO(const BitDepths &bitDepths, Int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, Double& modeNormCost );
  Int64 getDistortion(const Int channelBitDepth, Int typeIdc, Int typeAuxInfo, Int* offsetVal, SAOStatData& statData);
  Void deriveOffsets(ComponentID compIdx, const Int channelBitDepth, Int typeIdc, SAOStatData& statData, Int* quantOffsets, Int& typeAuxInfo);
  inline Int64 estSaoDist(Int64 count, Int64 offset, Int64 diffSum, Int shift);
  inline Int estIterOffset(Int typeIdx, Double lambda, Int offsetInput, Int64 count, Int64 diffSum, Int shift, Int bitIncrease, Int64& bestDist, Double& bestCost, Int offsetTh );
  Void addPreDBFStatistics(std::vector<SAOStatData**>& blkStats);
private: //members
  //for RDO
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
  Double                 m_lambda[MAX_NUM_COMPONENT];
  const double           FracBitsScale = 1.0 / double( 1 << SCALE_BITS );

  //statistics
  std::vector<SAOStatData**>         m_statData; //[ctu][comp][classes]
  std::vector<SAOStatData**>         m_preDBFstatData;
  Double                 m_saoDisabledRate[MAX_NUM_COMPONENT][MAX_TLAYER];
  Int                    m_skipLinesR[MAX_NUM_COMPONENT][NUM_SAO_NEW_TYPES];
  Int                    m_skipLinesB[MAX_NUM_COMPONENT][NUM_SAO_NEW_TYPES];
};


//! \}

#endif
