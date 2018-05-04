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

/**
 \file     EncSampleAdaptiveOffset.cpp
 \brief       estimation part of sample adaptive offset class
 */
#include "EncSampleAdaptiveOffset.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/CodingStructure.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup EncoderLib
//! \{


#define SAOCtx(c) SubCtx( Ctx::Sao, c )


//! rounding with IBDI
inline Double xRoundIbdi2(Int bitDepth, Double x)
{
  return ((x)>0) ? (Int)(((Int)(x)+(1<<(bitDepth-8-1)))/(1<<(bitDepth-8))) : ((Int)(((Int)(x)-(1<<(bitDepth-8-1)))/(1<<(bitDepth-8))));
}

inline Double xRoundIbdi(Int bitDepth, Double x)
{
  return (bitDepth > 8 ? xRoundIbdi2(bitDepth, (x)) : ((x)>=0 ? ((Int)((x)+0.5)) : ((Int)((x)-0.5)))) ;
}


EncSampleAdaptiveOffset::EncSampleAdaptiveOffset()
{
  m_CABACEstimator = NULL;
}

EncSampleAdaptiveOffset::~EncSampleAdaptiveOffset()
{
  destroyEncData();
}

Void EncSampleAdaptiveOffset::createEncData(Bool isPreDBFSamplesUsed, UInt numCTUsPic)
{
  //statistics
  const UInt sizeInCtus = numCTUsPic;
  m_statData.resize( sizeInCtus );
  for(UInt i=0; i< sizeInCtus; i++)
  {
    m_statData[i] = new SAOStatData*[MAX_NUM_COMPONENT];
    for(UInt compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      m_statData[i][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
    }
  }
  if(isPreDBFSamplesUsed)
  {
    m_preDBFstatData.resize( sizeInCtus );
    for(UInt i=0; i< sizeInCtus; i++)
    {
      m_preDBFstatData[i] = new SAOStatData*[MAX_NUM_COMPONENT];
      for(UInt compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        m_preDBFstatData[i][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
      }
    }

  }

  ::memset(m_saoDisabledRate, 0, sizeof(m_saoDisabledRate));

  for(Int typeIdc=0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++)
  {
    m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
    m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

    m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
    m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;

    if(isPreDBFSamplesUsed)
    {
      switch(typeIdc)
      {
      case SAO_TYPE_EO_0:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 3;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 1;
        }
        break;
      case SAO_TYPE_EO_90:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 2;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;
        }
        break;
      case SAO_TYPE_EO_135:
      case SAO_TYPE_EO_45:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;
        }
        break;
      case SAO_TYPE_BO:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 2;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 3;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 1;
        }
        break;
      default:
        {
          THROW("Not a supported type");
        }
      }
    }
  }
}

Void EncSampleAdaptiveOffset::destroyEncData()
{
  for(UInt i=0; i< m_statData.size(); i++)
  {
    for(UInt compIdx=0; compIdx< MAX_NUM_COMPONENT; compIdx++)
    {
      delete[] m_statData[i][compIdx];
    }
    delete[] m_statData[i];
  }
  m_statData.clear();


  for(Int i=0; i< m_preDBFstatData.size(); i++)
  {
    for(Int compIdx=0; compIdx< MAX_NUM_COMPONENT; compIdx++)
    {
      delete[] m_preDBFstatData[i][compIdx];
    }
    delete[] m_preDBFstatData[i];
  }
  m_preDBFstatData.clear();
}

#if JEM_TOOLS
Void EncSampleAdaptiveOffset::initCABACEstimator( CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
#else
Void EncSampleAdaptiveOffset::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
#endif
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_CtxCache       = ctxCache;
#if JEM_TOOLS
  m_CABACEstimator->initCtxModels( *pcSlice, cabacDataStore );
#else
  m_CABACEstimator->initCtxModels( *pcSlice );
#endif
  m_CABACEstimator->resetBits();
}


Void EncSampleAdaptiveOffset::SAOProcess(CodingStructure& cs, Bool* sliceEnabled, const Double *lambdas, const Bool bTestSAODisableAtPictureLevel, const Double saoEncodingRate, const Double saoEncodingRateChroma, Bool isPreDBFSamplesUsed )
{
  PelUnitBuf org = cs.getOrgBuf();
  PelUnitBuf res = cs.getRecoBuf();
  PelUnitBuf src = m_tempBuf;
  memcpy(m_lambda, lambdas, sizeof(m_lambda));

  src.copyFrom(res);

  //collect statistics
  getStatistics(m_statData, org, src, cs);
  if(isPreDBFSamplesUsed)
  {
    addPreDBFStatistics(m_statData);
  }

  //slice on/off
  decidePicParams(*cs.slice, sliceEnabled, saoEncodingRate, saoEncodingRateChroma);

  //block on/off
  std::vector<SAOBlkParam> reconParams(cs.pcv->sizeInCtus);
  decideBlkParams(cs, sliceEnabled, m_statData, src, res, &reconParams[0], cs.picture->getSAO(), bTestSAODisableAtPictureLevel, saoEncodingRate, saoEncodingRateChroma);

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "SAO" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );

  xPCMLFDisableProcess(cs);
}


Void EncSampleAdaptiveOffset::getPreDBFStatistics(CodingStructure& cs)
{
  PelUnitBuf org = cs.getOrgBuf();
  PelUnitBuf rec = cs.getRecoBuf();
  getStatistics(m_preDBFstatData, org, rec, cs, true);
}

Void EncSampleAdaptiveOffset::addPreDBFStatistics(std::vector<SAOStatData**>& blkStats)
{
  const UInt numCTUsPic = (UInt)blkStats.size();
  for(UInt n=0; n< numCTUsPic; n++)
  {
    for(UInt compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      for(UInt typeIdc=0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++)
      {
        blkStats[n][compIdx][typeIdc] += m_preDBFstatData[n][compIdx][typeIdc];
      }
    }
  }
}

Void EncSampleAdaptiveOffset::getStatistics(std::vector<SAOStatData**>& blkStats, PelUnitBuf& orgYuv, PelUnitBuf& srcYuv, CodingStructure& cs, Bool isCalculatePreDeblockSamples)
{
  Bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues& pcv = *cs.pcv;
  const Int numberOfComponents = getNumberValidComponents(pcv.chrFormat);

  size_t lineBufferSize = pcv.maxCUWidth + 1;
  if (m_signLineBuf1.size() != lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  int ctuRsAddr = 0;
  for( UInt yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( UInt xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const UInt width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const UInt height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

      deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail );

      //NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      //For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth  < pcv.lumaWidth );
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        const ComponentID compID = ComponentID(compIdx);
        const CompArea& compArea = area.block( compID );

        Int  srcStride  = srcYuv.get(compID).stride;
        Pel* srcBlk     = srcYuv.get(compID).bufAt( compArea );

        Int  orgStride  = orgYuv.get(compID).stride;
        Pel* orgBlk     = orgYuv.get(compID).bufAt( compArea );

        getBlkStats(compID, cs.sps->getBitDepth(toChannelType(compID)), blkStats[ctuRsAddr][compID]
                  , srcBlk, orgBlk, srcStride, orgStride, compArea.width, compArea.height
                  , isLeftAvail,  isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail
                  , isCalculatePreDeblockSamples
                  );
      }
      ctuRsAddr++;
    }
  }
}

Void EncSampleAdaptiveOffset::decidePicParams(const Slice& slice, Bool* sliceEnabled, const Double saoEncodingRate, const Double saoEncodingRateChroma)
{
  if ( slice.getPendingRasInit() )
  { // reset
    for (Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      for (Int tempLayer = 1; tempLayer < MAX_TLAYER; tempLayer++)
      {
        m_saoDisabledRate[compIdx][tempLayer] = 0.0;
      }
    }
  }

  const Int picTempLayer = slice.getDepth();

  //decide sliceEnabled[compIdx]
  const Int numberOfComponents = m_numberOfComponents;
  for (Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    sliceEnabled[compIdx] = false;
  }

  for (Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    // reset flags & counters
    sliceEnabled[compIdx] = true;

    if (saoEncodingRate>0.0)
    {
      if (saoEncodingRateChroma>0.0)
      {
        // decide slice-level on/off based on previous results
        if( (picTempLayer > 0)
          && (m_saoDisabledRate[compIdx][picTempLayer-1] > ((compIdx==COMPONENT_Y) ? saoEncodingRate : saoEncodingRateChroma)) )
        {
          sliceEnabled[compIdx] = false;
        }
      }
      else
      {
        // decide slice-level on/off based on previous results
        if( (picTempLayer > 0)
          && (m_saoDisabledRate[COMPONENT_Y][0] > saoEncodingRate) )
        {
          sliceEnabled[compIdx] = false;
        }
      }
    }
  }
}

Int64 EncSampleAdaptiveOffset::getDistortion(const Int channelBitDepth, Int typeIdc, Int typeAuxInfo, Int* invQuantOffset, SAOStatData& statData)
{
  Int64 dist        = 0;
  Int shift         = 2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth - 8);

  switch(typeIdc)
  {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
      {
        for (Int offsetIdx=0; offsetIdx<NUM_SAO_EO_CLASSES; offsetIdx++)
        {
          dist += estSaoDist( statData.count[offsetIdx], invQuantOffset[offsetIdx], statData.diff[offsetIdx], shift);
        }
      }
      break;
    case SAO_TYPE_BO:
      {
        for (Int offsetIdx=typeAuxInfo; offsetIdx<typeAuxInfo+4; offsetIdx++)
        {
          Int bandIdx = offsetIdx % NUM_SAO_BO_CLASSES ;
          dist += estSaoDist( statData.count[bandIdx], invQuantOffset[bandIdx], statData.diff[bandIdx], shift);
        }
      }
      break;
    default:
      {
        THROW("Not a supported type");
      }
  }

  return dist;
}

inline Int64 EncSampleAdaptiveOffset::estSaoDist(Int64 count, Int64 offset, Int64 diffSum, Int shift)
{
  return (( count*offset*offset-diffSum*offset*2 ) >> shift);
}


inline Int EncSampleAdaptiveOffset::estIterOffset(Int typeIdx, Double lambda, Int offsetInput, Int64 count, Int64 diffSum, Int shift, Int bitIncrease, Int64& bestDist, Double& bestCost, Int offsetTh )
{
  Int iterOffset, tempOffset;
  Int64 tempDist, tempRate;
  Double tempCost, tempMinCost;
  Int offsetOutput = 0;
  iterOffset = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can be used to measure the exact rate here.
  tempMinCost = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = (typeIdx == SAO_TYPE_BO) ? (abs((Int)iterOffset)+2) : (abs((Int)iterOffset)+1);
    if (abs((Int)iterOffset)==offsetTh) //inclusive
    {
      tempRate --;
    }
    // Do the dequantization before distortion calculation
    tempOffset  = iterOffset << bitIncrease;
    tempDist    = estSaoDist( count, tempOffset, diffSum, shift);
    tempCost    = ((Double)tempDist + lambda * (Double) tempRate);
    if(tempCost < tempMinCost)
    {
      tempMinCost = tempCost;
      offsetOutput = iterOffset;
      bestDist = tempDist;
      bestCost = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset-1):(iterOffset+1);
  }
  return offsetOutput;
}

Void EncSampleAdaptiveOffset::deriveOffsets(ComponentID compIdx, const Int channelBitDepth, Int typeIdc, SAOStatData& statData, Int* quantOffsets, Int& typeAuxInfo)
{
  Int bitDepth = channelBitDepth;
  Int shift    = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepth-8);
  Int offsetTh = SampleAdaptiveOffset::getMaxOffsetQVal(channelBitDepth);  //inclusive

  ::memset(quantOffsets, 0, sizeof(Int)*MAX_NUM_SAO_CLASSES);

  //derive initial offsets
  Int numClasses = (typeIdc == SAO_TYPE_BO)?((Int)NUM_SAO_BO_CLASSES):((Int)NUM_SAO_EO_CLASSES);
  for(Int classIdx=0; classIdx< numClasses; classIdx++)
  {
    if( (typeIdc != SAO_TYPE_BO) && (classIdx==SAO_CLASS_EO_PLAIN)  )
    {
      continue; //offset will be zero
    }

    if(statData.count[classIdx] == 0)
    {
      continue; //offset will be zero
    }

    quantOffsets[classIdx] = (Int) xRoundIbdi(bitDepth, (Double)( statData.diff[classIdx]<<(bitDepth-8))
                                                                  /
                                                          (Double)( statData.count[classIdx]<< m_offsetStepLog2[compIdx])
                                               );
    quantOffsets[classIdx] = Clip3(-offsetTh, offsetTh, quantOffsets[classIdx]);
  }

  // adjust offsets
  switch(typeIdc)
  {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
      {
        Int64 classDist;
        Double classCost;
        for(Int classIdx=0; classIdx<NUM_SAO_EO_CLASSES; classIdx++)
        {
          if(classIdx==SAO_CLASS_EO_FULL_VALLEY && quantOffsets[classIdx] < 0)
          {
            quantOffsets[classIdx] =0;
          }
          if(classIdx==SAO_CLASS_EO_HALF_VALLEY && quantOffsets[classIdx] < 0)
          {
            quantOffsets[classIdx] =0;
          }
          if(classIdx==SAO_CLASS_EO_HALF_PEAK   && quantOffsets[classIdx] > 0)
          {
            quantOffsets[classIdx] =0;
          }
          if(classIdx==SAO_CLASS_EO_FULL_PEAK   && quantOffsets[classIdx] > 0)
          {
            quantOffsets[classIdx] =0;
          }

          if( quantOffsets[classIdx] != 0 ) //iterative adjustment only when derived offset is not zero
          {
            quantOffsets[classIdx] = estIterOffset( typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], classDist , classCost , offsetTh );
          }
        }

        typeAuxInfo =0;
      }
      break;
    case SAO_TYPE_BO:
      {
        Int64  distBOClasses[NUM_SAO_BO_CLASSES];
        Double costBOClasses[NUM_SAO_BO_CLASSES];
        ::memset(distBOClasses, 0, sizeof(Int64)*NUM_SAO_BO_CLASSES);
        for(Int classIdx=0; classIdx< NUM_SAO_BO_CLASSES; classIdx++)
        {
          costBOClasses[classIdx]= m_lambda[compIdx];
          if( quantOffsets[classIdx] != 0 ) //iterative adjustment only when derived offset is not zero
          {
            quantOffsets[classIdx] = estIterOffset( typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], distBOClasses[classIdx], costBOClasses[classIdx], offsetTh );
          }
        }

        //decide the starting band index
        Double minCost = MAX_DOUBLE, cost;
        for(Int band=0; band< NUM_SAO_BO_CLASSES- 4+ 1; band++)
        {
          cost  = costBOClasses[band  ];
          cost += costBOClasses[band+1];
          cost += costBOClasses[band+2];
          cost += costBOClasses[band+3];

          if(cost < minCost)
          {
            minCost = cost;
            typeAuxInfo = band;
          }
        }
        //clear those unused classes
        Int clearQuantOffset[NUM_SAO_BO_CLASSES];
        ::memset(clearQuantOffset, 0, sizeof(Int)*NUM_SAO_BO_CLASSES);
        for(Int i=0; i< 4; i++)
        {
          Int band = (typeAuxInfo+i)%NUM_SAO_BO_CLASSES;
          clearQuantOffset[band] = quantOffsets[band];
        }
        ::memcpy(quantOffsets, clearQuantOffset, sizeof(Int)*NUM_SAO_BO_CLASSES);
      }
      break;
    default:
      {
        THROW("Not a supported type");
      }

  }


}

Void EncSampleAdaptiveOffset::deriveModeNewRDO(const BitDepths &bitDepths, Int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, Double& modeNormCost )
{
  Double minCost, cost;
  uint64_t previousFracBits;
  const Int numberOfComponents = m_numberOfComponents;

  Int64 dist[MAX_NUM_COMPONENT], modeDist[MAX_NUM_COMPONENT];
  SAOOffset testOffset[MAX_NUM_COMPONENT];
  Int invQuantOffset[MAX_NUM_SAO_CLASSES];
  for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
  {
    modeDist[comp] = 0;
  }

  //pre-encode merge flags
  modeParam[COMPONENT_Y].modeIdc = SAO_MODE_OFF;
  const TempCtx ctxStartBlk   ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  m_CABACEstimator->sao_block_pars( modeParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), true );
  const TempCtx ctxStartLuma  ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBestLuma   ( m_CtxCache );

    //------ luma --------//
  {
    const ComponentID compIdx = COMPONENT_Y;
    //"off" case as initial cost
    modeParam[compIdx].modeIdc = SAO_MODE_OFF;
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_offset_pars( modeParam[compIdx], compIdx, sliceEnabled[compIdx], bitDepths.recon[CHANNEL_TYPE_LUMA] );
    modeDist[compIdx] = 0;
    minCost= m_lambda[compIdx]*(FracBitsScale*(Double)m_CABACEstimator->getEstFracBits());
    ctxBestLuma = SAOCtx( m_CABACEstimator->getCtx() );
    if(sliceEnabled[compIdx])
    {
      for(Int typeIdc=0; typeIdc< NUM_SAO_NEW_TYPES; typeIdc++)
      {
        testOffset[compIdx].modeIdc = SAO_MODE_NEW;
        testOffset[compIdx].typeIdc = typeIdc;

        //derive coded offset
        deriveOffsets(compIdx, bitDepths.recon[CHANNEL_TYPE_LUMA], typeIdc, blkStats[ctuRsAddr][compIdx][typeIdc], testOffset[compIdx].offset, testOffset[compIdx].typeAuxInfo);

        //inversed quantized offsets
        invertQuantOffsets(compIdx, typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset, testOffset[compIdx].offset);

        //get distortion
        dist[compIdx] = getDistortion(bitDepths.recon[CHANNEL_TYPE_LUMA], testOffset[compIdx].typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][compIdx][typeIdc]);

        //get rate
        m_CABACEstimator->getCtx() = SAOCtx( ctxStartLuma );
        m_CABACEstimator->resetBits();
        m_CABACEstimator->sao_offset_pars( testOffset[compIdx], compIdx, sliceEnabled[compIdx], bitDepths.recon[CHANNEL_TYPE_LUMA] );
        double rate = FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
        cost = (Double)dist[compIdx] + m_lambda[compIdx]*rate;
        if(cost < minCost)
        {
          minCost = cost;
          modeDist[compIdx] = dist[compIdx];
          modeParam[compIdx]= testOffset[compIdx];
          ctxBestLuma = SAOCtx( m_CABACEstimator->getCtx() );
        }
      }
    }
    m_CABACEstimator->getCtx() = SAOCtx( ctxBestLuma );
  }

  //------ chroma --------//
//"off" case as initial cost
  cost = 0;
  previousFracBits = 0;
  m_CABACEstimator->resetBits();
  for(UInt componentIndex = COMPONENT_Cb; componentIndex < numberOfComponents; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    modeParam[component].modeIdc = SAO_MODE_OFF;
    modeDist [component]         = 0;
    m_CABACEstimator->sao_offset_pars( modeParam[component], component, sliceEnabled[component], bitDepths.recon[CHANNEL_TYPE_CHROMA] );
    const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
    cost += m_lambda[component] * FracBitsScale * double( currentFracBits - previousFracBits );
    previousFracBits = currentFracBits;
  }

  minCost = cost;

  //doesn't need to store cabac status here since the whole CTU parameters will be re-encoded at the end of this function

  for(Int typeIdc=0; typeIdc< NUM_SAO_NEW_TYPES; typeIdc++)
  {
    m_CABACEstimator->getCtx() = SAOCtx( ctxBestLuma );
    m_CABACEstimator->resetBits();
    previousFracBits = 0;
    cost = 0;

    for(UInt componentIndex = COMPONENT_Cb; componentIndex < numberOfComponents; componentIndex++)
    {
      const ComponentID component = ComponentID(componentIndex);
      if(!sliceEnabled[component])
      {
        testOffset[component].modeIdc = SAO_MODE_OFF;
        dist[component]= 0;
        continue;
      }
      testOffset[component].modeIdc = SAO_MODE_NEW;
      testOffset[component].typeIdc = typeIdc;

      //derive offset & get distortion
      deriveOffsets(component, bitDepths.recon[CHANNEL_TYPE_CHROMA], typeIdc, blkStats[ctuRsAddr][component][typeIdc], testOffset[component].offset, testOffset[component].typeAuxInfo);
      invertQuantOffsets(component, typeIdc, testOffset[component].typeAuxInfo, invQuantOffset, testOffset[component].offset);
      dist[component] = getDistortion(bitDepths.recon[CHANNEL_TYPE_CHROMA], typeIdc, testOffset[component].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][component][typeIdc]);
      m_CABACEstimator->sao_offset_pars( testOffset[component], component, sliceEnabled[component], bitDepths.recon[CHANNEL_TYPE_CHROMA] );
      const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
      cost += dist[component] + (m_lambda[component] * FracBitsScale * double(currentFracBits - previousFracBits));
      previousFracBits = currentFracBits;
    }

    if(cost < minCost)
    {
      minCost = cost;
      for(UInt componentIndex = COMPONENT_Cb; componentIndex < numberOfComponents; componentIndex++)
      {
        modeDist[componentIndex]  = dist[componentIndex];
        modeParam[componentIndex] = testOffset[componentIndex];
      }
    }

  } // SAO_TYPE loop

  //----- re-gen rate & normalized cost----//
  modeNormCost = 0;
  for(UInt componentIndex = COMPONENT_Y; componentIndex < numberOfComponents; componentIndex++)
  {
    modeNormCost += (Double)modeDist[componentIndex] / m_lambda[componentIndex];
  }

  m_CABACEstimator->getCtx() = SAOCtx( ctxStartBlk );
  m_CABACEstimator->resetBits();
  m_CABACEstimator->sao_block_pars( modeParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), false );
  modeNormCost += FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
}

Void EncSampleAdaptiveOffset::deriveModeMergeRDO(const BitDepths &bitDepths, Int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, Double& modeNormCost )
{
  modeNormCost = MAX_DOUBLE;

  Double cost;
  SAOBlkParam testBlkParam;
  const Int numberOfComponents = m_numberOfComponents;

  const TempCtx ctxStart  ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBest   ( m_CtxCache );

  for(Int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    if(mergeList[mergeType] == NULL)
    {
      continue;
    }

    testBlkParam = *(mergeList[mergeType]);
    //normalized distortion
    Double normDist=0;
    for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      testBlkParam[compIdx].modeIdc = SAO_MODE_MERGE;
      testBlkParam[compIdx].typeIdc = mergeType;

      SAOOffset& mergedOffsetParam = (*(mergeList[mergeType]))[compIdx];

      if( mergedOffsetParam.modeIdc != SAO_MODE_OFF)
      {
        //offsets have been reconstructed. Don't call inversed quantization function.
        normDist += (((Double)getDistortion(bitDepths.recon[toChannelType(ComponentID(compIdx))], mergedOffsetParam.typeIdc, mergedOffsetParam.typeAuxInfo, mergedOffsetParam.offset, blkStats[ctuRsAddr][compIdx][mergedOffsetParam.typeIdc]))
                       /m_lambda[compIdx] );
      }
    }

    //rate
    m_CABACEstimator->getCtx() = SAOCtx( ctxStart );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_block_pars( testBlkParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), false );
    double rate = FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
    cost = normDist+rate;

    if(cost < modeNormCost)
    {
      modeNormCost = cost;
      modeParam    = testBlkParam;
      ctxBest      = SAOCtx( m_CABACEstimator->getCtx() );
    }
  }
  if( modeNormCost < MAX_DOUBLE )
  {
    m_CABACEstimator->getCtx() = SAOCtx( ctxBest );
  }
}

Void EncSampleAdaptiveOffset::decideBlkParams(CodingStructure& cs, Bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, PelUnitBuf& srcYuv, PelUnitBuf& resYuv,
                                               SAOBlkParam* reconParams, SAOBlkParam* codedParams, const Bool bTestSAODisableAtPictureLevel,
                                               const Double saoEncodingRate, const Double saoEncodingRateChroma)
{
  const PreCalcValues& pcv = *cs.pcv;
  Bool allBlksDisabled = true;
  const UInt numberOfComponents = m_numberOfComponents;
  for(UInt compId = COMPONENT_Y; compId < numberOfComponents; compId++)
  {
    if (sliceEnabled[compId])
    {
      allBlksDisabled = false;
    }
  }

  const TempCtx ctxPicStart ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );

  SAOBlkParam modeParam;
  Double minCost, modeCost;


  Double totalCost = 0; // Used if bTestSAODisableAtPictureLevel==true

  int ctuRsAddr = 0;
  for( UInt yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( UInt xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const UInt width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const UInt height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( pcv.chrFormat, Area( xPos , yPos, width, height) );

      if(allBlksDisabled)
      {
        codedParams[ctuRsAddr].reset();
        continue;
      }

      const TempCtx  ctxStart ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
      TempCtx        ctxBest  ( m_CtxCache );

      //get merge list
      SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
      getMergeList(cs, ctuRsAddr, reconParams, mergeList);

      minCost = MAX_DOUBLE;
      for(Int mode=1; mode < NUM_SAO_MODES; mode++)
      {
        if( mode > 1 )
        {
          m_CABACEstimator->getCtx() = SAOCtx( ctxStart );
        }
        switch(mode)
        {
        case SAO_MODE_NEW:
          {
            deriveModeNewRDO(cs.sps->getBitDepths(), ctuRsAddr, mergeList, sliceEnabled, blkStats, modeParam, modeCost );
          }
          break;
        case SAO_MODE_MERGE:
          {
            deriveModeMergeRDO(cs.sps->getBitDepths(), ctuRsAddr, mergeList, sliceEnabled, blkStats , modeParam, modeCost );
          }
          break;
        default:
          {
            THROW( "Not a supported SAO mode." );
          }
        }

        if(modeCost < minCost)
        {
          minCost                = modeCost;
          codedParams[ctuRsAddr] = modeParam;
          ctxBest                = SAOCtx( m_CABACEstimator->getCtx() );
        }
      } //mode

      totalCost += minCost;

      m_CABACEstimator->getCtx() = SAOCtx( ctxBest );

      //apply reconstructed offsets
      reconParams[ctuRsAddr] = codedParams[ctuRsAddr];
      reconstructBlkSAOParam(reconParams[ctuRsAddr], mergeList);


      offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
      ctuRsAddr++;
    } //ctuRsAddr
  }

  if (!allBlksDisabled && (totalCost >= 0) && bTestSAODisableAtPictureLevel) //SAO has not beneficial in this case - disable it
  {
    for( ctuRsAddr = 0; ctuRsAddr < pcv.sizeInCtus; ctuRsAddr++)
    {
      codedParams[ctuRsAddr].reset();
    }

    for (UInt componentIndex = 0; componentIndex < MAX_NUM_COMPONENT; componentIndex++)
    {
      sliceEnabled[componentIndex] = false;
    }
    m_CABACEstimator->getCtx() = SAOCtx(ctxPicStart);
  }

  EncSampleAdaptiveOffset::disabledRate( cs, reconParams, saoEncodingRate, saoEncodingRateChroma );
}

Void EncSampleAdaptiveOffset::disabledRate( CodingStructure& cs, SAOBlkParam* reconParams, const Double saoEncodingRate, const Double saoEncodingRateChroma )
{
  if (saoEncodingRate > 0.0)
  {
    const PreCalcValues& pcv = *cs.pcv;
    const UInt numberOfComponents = m_numberOfComponents;
    Int picTempLayer = cs.slice->getDepth();
    Int numCtusForSAOOff[MAX_NUM_COMPONENT];

    for (Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      numCtusForSAOOff[compIdx] = 0;
      for( int ctuRsAddr=0; ctuRsAddr< pcv.sizeInCtus; ctuRsAddr++)
      {
        if( reconParams[ctuRsAddr][compIdx].modeIdc == SAO_MODE_OFF)
        {
          numCtusForSAOOff[compIdx]++;
        }
      }
    }
    if (saoEncodingRateChroma > 0.0)
    {
      for (Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        m_saoDisabledRate[compIdx][picTempLayer] = (Double)numCtusForSAOOff[compIdx]/(Double)pcv.sizeInCtus;
      }
    }
    else if (picTempLayer == 0)
    {
      m_saoDisabledRate[COMPONENT_Y][0] = (Double)(numCtusForSAOOff[COMPONENT_Y]+numCtusForSAOOff[COMPONENT_Cb]+numCtusForSAOOff[COMPONENT_Cr])/(Double)(pcv.sizeInCtus *3);
    }
  }
}

Void EncSampleAdaptiveOffset::getBlkStats(const ComponentID compIdx, const Int channelBitDepth, SAOStatData* statsDataTypes
                        , Pel* srcBlk, Pel* orgBlk, Int srcStride, Int orgStride, Int width, Int height
                        , Bool isLeftAvail,  Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail
                        , Bool isCalculatePreDeblockSamples
                        )
{
  Int x,y, startX, startY, endX, endY, edgeType, firstLineStartX, firstLineEndX;
  SChar signLeft, signRight, signDown;
  Int64 *diff, *count;
  Pel *srcLine, *orgLine;
  Int* skipLinesR = m_skipLinesR[compIdx];
  Int* skipLinesB = m_skipLinesB[compIdx];

  for(Int typeIdx=0; typeIdx< NUM_SAO_NEW_TYPES; typeIdx++)
  {
    SAOStatData& statsData= statsDataTypes[typeIdx];
    statsData.reset();

    srcLine = srcBlk;
    orgLine = orgBlk;
    diff    = statsData.diff;
    count   = statsData.count;
    switch(typeIdx)
    {
    case SAO_TYPE_EO_0:
      {
        diff +=2;
        count+=2;
        endY   = (isBelowAvail) ? (height - skipLinesB[typeIdx]) : height;
        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail  ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 ;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 : (isRightAvail ? width : (width - 1))
                                                 ;
        for (y=0; y<endY; y++)
        {
          signLeft = (SChar)sgn(srcLine[startX] - srcLine[startX-1]);
          for (x=startX; x<endX; x++)
          {
            signRight =  (SChar)sgn(srcLine[x] - srcLine[x+1]);
            edgeType  =  signRight + signLeft;
            signLeft  = -signRight;

            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;
          }
          srcLine  += srcStride;
          orgLine  += orgStride;
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = isLeftAvail  ? 0 : 1;
            endX   = isRightAvail ? width : (width -1);

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              signLeft = (SChar)sgn(srcLine[startX] - srcLine[startX-1]);
              for (x=startX; x<endX; x++)
              {
                signRight =  (SChar)sgn(srcLine[x] - srcLine[x+1]);
                edgeType  =  signRight + signLeft;
                signLeft  = -signRight;

                diff [edgeType] += (orgLine[x] - srcLine[x]);
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
            }
          }
        }
      }
      break;
    case SAO_TYPE_EO_90:
      {
        diff +=2;
        count+=2;
        SChar *signUpLine = &m_signLineBuf1[0];

        startX = (!isCalculatePreDeblockSamples) ? 0
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : width)
                                                 ;
        startY = isAboveAvail ? 0 : 1;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : width)
                                                 : width
                                                 ;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);
        if (!isAboveAvail)
        {
          srcLine += srcStride;
          orgLine += orgStride;
        }

        Pel* srcLineAbove = srcLine - srcStride;
        for (x=startX; x<endX; x++)
        {
          signUpLine[x] = (SChar)sgn(srcLine[x] - srcLineAbove[x]);
        }

        Pel* srcLineBelow;
        for (y=startY; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x=startX; x<endX; x++)
          {
            signDown  = (SChar)sgn(srcLine[x] - srcLineBelow[x]);
            edgeType  = signDown + signUpLine[x];
            signUpLine[x]= -signDown;

            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = 0;
            endX   = width;

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x=startX; x<endX; x++)
              {
                edgeType = sgn(srcLine[x] - srcLineBelow[x]) + sgn(srcLine[x] - srcLineAbove[x]);
                diff [edgeType] += (orgLine[x] - srcLine[x]);
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
            }
          }
        }

      }
      break;
    case SAO_TYPE_EO_135:
      {
        diff +=2;
        count+=2;
        SChar *signUpLine, *signDownLine, *signTmpLine;

        signUpLine  = &m_signLineBuf1[0];
        signDownLine= &m_signLineBuf2[0];

        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail  ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 ;

        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]): (width - 1))
                                                 : (isRightAvail ? width : (width - 1))
                                                 ;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);

        //prepare 2nd line's upper sign
        Pel* srcLineBelow = srcLine + srcStride;
        for (x=startX; x<endX+1; x++)
        {
          signUpLine[x] = (SChar)sgn(srcLineBelow[x] - srcLine[x-1]);
        }

        //1st line
        Pel* srcLineAbove = srcLine - srcStride;
        firstLineStartX = (!isCalculatePreDeblockSamples) ? (isAboveLeftAvail ? 0    : 1) : startX;
        firstLineEndX   = (!isCalculatePreDeblockSamples) ? (isAboveAvail     ? endX : 1) : endX;
        for(x=firstLineStartX; x<firstLineEndX; x++)
        {
          edgeType = sgn(srcLine[x] - srcLineAbove[x-1]) - signUpLine[x+1];
          diff [edgeType] += (orgLine[x] - srcLine[x]);
          count[edgeType] ++;
        }
        srcLine  += srcStride;
        orgLine  += orgStride;


        //middle lines
        for (y=1; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x=startX; x<endX; x++)
          {
            signDown = (SChar)sgn(srcLine[x] - srcLineBelow[x+1]);
            edgeType = signDown + signUpLine[x];
            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;

            signDownLine[x+1] = -signDown;
          }
          signDownLine[startX] = (SChar)sgn(srcLineBelow[startX] - srcLine[startX-1]);

          signTmpLine  = signUpLine;
          signUpLine   = signDownLine;
          signDownLine = signTmpLine;

          srcLine += srcStride;
          orgLine += orgStride;
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = isLeftAvail  ? 0     : 1 ;
            endX   = isRightAvail ? width : (width -1);

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x=startX; x< endX; x++)
              {
                edgeType = sgn(srcLine[x] - srcLineBelow[x+1]) + sgn(srcLine[x] - srcLineAbove[x-1]);
                diff [edgeType] += (orgLine[x] - srcLine[x]);
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
            }
          }
        }
      }
      break;
    case SAO_TYPE_EO_45:
      {
        diff +=2;
        count+=2;
        SChar *signUpLine = &m_signLineBuf1[1];

        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail  ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 ;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 : (isRightAvail ? width : (width - 1))
                                                 ;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);

        //prepare 2nd line upper sign
        Pel* srcLineBelow = srcLine + srcStride;
        for (x=startX-1; x<endX; x++)
        {
          signUpLine[x] = (SChar)sgn(srcLineBelow[x] - srcLine[x+1]);
        }


        //first line
        Pel* srcLineAbove = srcLine - srcStride;
        firstLineStartX = (!isCalculatePreDeblockSamples) ? (isAboveAvail ? startX : endX)
                                                          : startX
                                                          ;
        firstLineEndX   = (!isCalculatePreDeblockSamples) ? ((!isRightAvail && isAboveRightAvail) ? width : endX)
                                                          : endX
                                                          ;
        for(x=firstLineStartX; x<firstLineEndX; x++)
        {
          edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) - signUpLine[x-1];
          diff [edgeType] += (orgLine[x] - srcLine[x]);
          count[edgeType] ++;
        }

        srcLine += srcStride;
        orgLine += orgStride;

        //middle lines
        for (y=1; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for(x=startX; x<endX; x++)
          {
            signDown = (SChar)sgn(srcLine[x] - srcLineBelow[x-1]);
            edgeType = signDown + signUpLine[x];

            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;

            signUpLine[x-1] = -signDown;
          }
          signUpLine[endX-1] = (SChar)sgn(srcLineBelow[endX-1] - srcLine[endX]);
          srcLine  += srcStride;
          orgLine  += orgStride;
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = isLeftAvail  ? 0     : 1 ;
            endX   = isRightAvail ? width : (width -1);

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x=startX; x<endX; x++)
              {
                edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + sgn(srcLine[x] - srcLineAbove[x+1]);
                diff [edgeType] += (orgLine[x] - srcLine[x]);
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
            }
          }
        }
      }
      break;
    case SAO_TYPE_BO:
      {
        startX = (!isCalculatePreDeblockSamples)?0
                                                :( isRightAvail?(width- skipLinesR[typeIdx]):width)
                                                ;
        endX   = (!isCalculatePreDeblockSamples)?(isRightAvail ? (width - skipLinesR[typeIdx]) : width )
                                                :width
                                                ;
        endY = isBelowAvail ? (height- skipLinesB[typeIdx]) : height;
        Int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
        for (y=0; y< endY; y++)
        {
          for (x=startX; x< endX; x++)
          {

            Int bandIdx= srcLine[x] >> shiftBits;
            diff [bandIdx] += (orgLine[x] - srcLine[x]);
            count[bandIdx] ++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = 0;
            endX   = width;

            for(y= 0; y< skipLinesB[typeIdx]; y++)
            {
              for (x=startX; x< endX; x++)
              {
                Int bandIdx= srcLine[x] >> shiftBits;
                diff [bandIdx] += (orgLine[x] - srcLine[x]);
                count[bandIdx] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;

            }

          }
        }
      }
      break;
    default:
      {
        THROW("Not a supported SAO type");
      }
    }
  }
}

Void EncSampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos, Bool& isLeftAvail, Bool& isAboveAvail, Bool& isAboveLeftAvail) const
{
#if HEVC_TILES_WPP
  Bool isLoopFiltAcrossTilePPS = cs.pps->getLoopFilterAcrossTilesEnabledFlag();
#endif

  const int width = cs.pcv->maxCUWidth;
  const int height = cs.pcv->maxCUHeight;
  const CodingUnit* cuCurr = cs.getCU(pos, CH_L);
  const CodingUnit* cuLeft = cs.getCU(pos.offset(-width, 0), CH_L);
  const CodingUnit* cuAbove = cs.getCU(pos.offset(0, -height), CH_L);
  const CodingUnit* cuAboveLeft = cs.getCU(pos.offset(-width, -height), CH_L);

  {
    isLeftAvail      = (cuLeft != NULL)      ? ( !CU::isSameSlice(*cuCurr, *cuLeft)      ? cuCurr->slice->getLFCrossSliceBoundaryFlag() : true ) : false;
    isAboveAvail     = (cuAbove != NULL)     ? ( !CU::isSameSlice(*cuCurr, *cuAbove)     ? cuCurr->slice->getLFCrossSliceBoundaryFlag() : true ) : false;
    isAboveLeftAvail = (cuAboveLeft != NULL) ? ( !CU::isSameSlice(*cuCurr, *cuAboveLeft) ? cuCurr->slice->getLFCrossSliceBoundaryFlag() : true ) : false;
  }

#if HEVC_TILES_WPP
  if (!isLoopFiltAcrossTilePPS)
  {
    isLeftAvail      = (!isLeftAvail)      ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail)     ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
  }
#endif
}

//! \}
