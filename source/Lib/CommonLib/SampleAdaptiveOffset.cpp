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

/** \file     SampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "SampleAdaptiveOffset.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "CodingStructure.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup CommonLib
//! \{

SAOOffset::SAOOffset()
{
  reset();
}

SAOOffset::~SAOOffset()
{

}

Void SAOOffset::reset()
{
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(Int)* MAX_NUM_SAO_CLASSES);
}

const SAOOffset& SAOOffset::operator= (const SAOOffset& src)
{
  modeIdc = src.modeIdc;
  typeIdc = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(Int)* MAX_NUM_SAO_CLASSES);

  return *this;
}


SAOBlkParam::SAOBlkParam()
{
  reset();
}

SAOBlkParam::~SAOBlkParam()
{

}

Void SAOBlkParam::reset()
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam& SAOBlkParam::operator= (const SAOBlkParam& src)
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx] = src.offsetParam[compIdx];
  }
  return *this;
}




SampleAdaptiveOffset::SampleAdaptiveOffset()
{
}


SampleAdaptiveOffset::~SampleAdaptiveOffset()
{
  destroy();

  m_signLineBuf1.clear();
  m_signLineBuf2.clear();
}

Void SampleAdaptiveOffset::create( Int picWidth, Int picHeight, ChromaFormat format, UInt maxCUWidth, UInt maxCUHeight, UInt maxCUDepth, UInt lumaBitShift, UInt chromaBitShift )
{
  //temporary picture buffer
  UnitArea picArea(format, Area(0, 0, picWidth, picHeight));

  m_tempBuf.destroy();
  m_tempBuf.create( picArea );

  //bit-depth related
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_offsetStepLog2  [compIdx] = isLuma(ComponentID(compIdx))? lumaBitShift : chromaBitShift;
  }
  m_numberOfComponents = getNumberValidComponents(format);
}

Void SampleAdaptiveOffset::destroy()
{
  m_tempBuf.destroy();
}

Void SampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, Int typeIdc, Int typeAuxInfo, Int* dstOffsets, Int* srcOffsets)
{
  Int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(Int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(Int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)
  {
    for(Int i=0; i< 4; i++)
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO
  {
    for(Int i=0; i< NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    CHECK(dstOffsets[SAO_CLASS_EO_PLAIN] != 0, "EO offset is not '0'"); //keep EO plain offset as zero
  }

}

Int SampleAdaptiveOffset::getMergeList(CodingStructure& cs, Int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const PreCalcValues& pcv = *cs.pcv;

  Int ctuX = ctuRsAddr % pcv.widthInCtus;
  Int ctuY = ctuRsAddr / pcv.widthInCtus;
  const CodingUnit& cu = *cs.getCU(Position(ctuX*pcv.maxCUWidth, ctuY*pcv.maxCUHeight), CH_L);
  Int mergedCTUPos;
  Int numValidMergeCandidates = 0;

  for(Int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE:
      {
        if(ctuY > 0)
        {
          mergedCTUPos = ctuRsAddr- pcv.widthInCtus;
          if(cs.getCURestricted(Position(ctuX*pcv.maxCUWidth, (ctuY-1)*pcv.maxCUHeight), cu, cu.chType))
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    case SAO_MERGE_LEFT:
      {
        if(ctuX > 0)
        {
          mergedCTUPos = ctuRsAddr- 1;
          if(cs.getCURestricted(Position((ctuX-1)*pcv.maxCUWidth, ctuY*pcv.maxCUHeight), cu, cu.chType))
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    default:
      {
        THROW("not a supported merge type");
      }
    }

    mergeList[mergeType]=mergeCandidate;
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}


Void SampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const Int numberOfComponents = m_numberOfComponents;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];

    if(offsetParam.modeIdc == SAO_MODE_OFF)
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW:
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE:
      {
        SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        CHECK(mergeTarget == NULL, "Merge target does not exist");

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        THROW("Not a supported mode");
      }
    }
  }
}

Void SampleAdaptiveOffset::xReconstructBlkSAOParams(CodingStructure& cs, SAOBlkParam* saoBlkParams)
{
  for(UInt compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;
  }

  const UInt numberOfComponents = getNumberValidComponents(cs.pcv->chrFormat);

  for(Int ctuRsAddr=0; ctuRsAddr< cs.pcv->sizeInCtus; ctuRsAddr++)
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(cs, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);

    for(UInt compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if(saoBlkParams[ctuRsAddr][compIdx].modeIdc != SAO_MODE_OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}


Void SampleAdaptiveOffset::offsetBlock(const Int channelBitDepth, const ClpRng& clpRng, Int typeIdx, Int* offset
                                          , const Pel* srcBlk, Pel* resBlk, Int srcStride, Int resStride,  Int width, Int height
                                          , Bool isLeftAvail,  Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isBelowLeftAvail, Bool isBelowRightAvail)
{
  Int x,y, startX, startY, endX, endY, edgeType;
  Int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  SChar signLeft, signRight, signDown;

  const Pel* srcLine = srcBlk;
        Pel* resLine = resBlk;

  switch(typeIdx)
  {
  case SAO_TYPE_EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = (SChar)sgn(srcLine[startX] - srcLine[startX-1]);
        for (x=startX; x< endX; x++)
        {
          signRight = (SChar)sgn(srcLine[x] - srcLine[x+1]);
          edgeType =  signRight + signLeft;
          signLeft  = -signRight;

          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine  += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_90:
    {
      offset += 2;
      SChar *signUpLine = &m_signLineBuf1[0];

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height-1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      const Pel* srcLineAbove= srcLine- srcStride;
      for (x=0; x< width; x++)
      {
        signUpLine[x] = (SChar)sgn(srcLine[x] - srcLineAbove[x]);
      }

      const Pel* srcLineBelow;
      for (y=startY; y<endY; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=0; x< width; x++)
        {
          signDown  = (SChar)sgn(srcLine[x] - srcLineBelow[x]);
          edgeType = signDown + signUpLine[x];
          signUpLine[x]= -signDown;

          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_135:
    {
      offset += 2;
      SChar *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = &m_signLineBuf1[0];
      signDownLine= &m_signLineBuf2[0];

      startX = isLeftAvail ? 0 : 1 ;
      endX   = isRightAvail ? width : (width-1);

      //prepare 2nd line's upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX; x< endX+1; x++)
      {
        signUpLine[x] = (SChar)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }

      //1st line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];

        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine  += srcStride;
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=startX; x<endX; x++)
        {
          signDown =  (SChar)sgn(srcLine[x] - srcLineBelow[x+ 1]);
          edgeType =  signDown + signUpLine[x];
          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

          signDownLine[x+1] = -signDown;
        }
        signDownLine[startX] = (SChar)sgn(srcLineBelow[startX] - srcLine[startX-1]);

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

      }
    }
    break;
  case SAO_TYPE_EO_45:
    {
      offset += 2;
      SChar *signUpLine = &m_signLineBuf1[1];

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);

      //prepare 2nd line upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (SChar)sgn(srcLineBelow[x] - srcLine[x+1]);
      }


      //first line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveAvail ? startX : (width -1 );
      firstLineEndX   = isAboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for(x= startX; x< endX; x++)
        {
          signDown =  (SChar)sgn(srcLine[x] - srcLineBelow[x-1]);
          edgeType =  signDown + signUpLine[x];
          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
          signUpLine[x-1] = -signDown;
        }
        signUpLine[endX-1] = (SChar)sgn(srcLineBelow[endX-1] - srcLine[endX]);
        srcLine  += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

      }
    }
    break;
  case SAO_TYPE_BO:
    {
      const Int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x++)
        {
          resLine[x] = ClipPel<int>(srcLine[x] + offset[srcLine[x] >> shiftBits], clpRng );
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  default:
    {
      THROW("Not a supported SAO types\n");
    }
  }
}

Void SampleAdaptiveOffset::offsetCTU( const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs)
{
  const UInt numberOfComponents = getNumberValidComponents( area.chromaFormat );
  Bool bAllOff=true;
  for( UInt compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAO_MODE_OFF)
    {
      bAllOff=false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  Bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail;

  //block boundary availability
  deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

  const size_t lineBufferSize = area.Y().width + 1;
  if (m_signLineBuf1.size() < lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID compID = ComponentID(compIdx);
    const CompArea& compArea = area.block(compID);
    SAOOffset& ctbOffset     = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      Int  srcStride    = src.get(compID).stride;
      const Pel* srcBlk = src.get(compID).bufAt(compArea);
      Int  resStride    = res.get(compID).stride;
      Pel* resBlk       = res.get(compID).bufAt(compArea);

      offsetBlock( cs.sps->getBitDepth(toChannelType(compID)),
                   cs.slice->clpRng(compID),
                   ctbOffset.typeIdc, ctbOffset.offset
                  , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
                  );
    }
  } //compIdx
}

Void SampleAdaptiveOffset::SAOProcess( CodingStructure& cs, SAOBlkParam* saoBlkParams
                                      )
{
  CHECK(!saoBlkParams, "No parameters present");

  xReconstructBlkSAOParams(cs, saoBlkParams);

  const UInt numberOfComponents = getNumberValidComponents(cs.area.chromaFormat);
  Bool bAllDisabled = true;
  for (UInt compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_picSAOEnabled[compIdx])
    {
      bAllDisabled = false;
    }
  }
  if (bAllDisabled)
  {
    return;
  }

  const PreCalcValues& pcv = *cs.pcv;
  PelUnitBuf rec = cs.getRecoBuf();
  m_tempBuf.copyFrom( rec );

  int ctuRsAddr = 0;
  for( UInt yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( UInt xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const UInt width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const UInt height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

      offsetCTU( area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs);
      ctuRsAddr++;
    }
  }

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "SAO" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );

  xPCMLFDisableProcess(cs);
}

Void SampleAdaptiveOffset::xPCMLFDisableProcess(CodingStructure& cs)
{
  const PreCalcValues& pcv = *cs.pcv;
  const Bool bPCMFilter = (cs.sps->getUsePCM() && cs.sps->getPCMFilterDisableFlag()) ? true : false;

  if( bPCMFilter || cs.pps->getTransquantBypassEnabledFlag() )
  {
    for( UInt yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
    {
      for( UInt xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
      {
        UnitArea ctuArea( cs.area.chromaFormat, Area( xPos, yPos, pcv.maxCUWidth, pcv.maxCUHeight ) );

        // CU-based deblocking
        xPCMCURestoration(cs, ctuArea);
      }
    }
  }
}

Void SampleAdaptiveOffset::xPCMCURestoration(CodingStructure& cs, const UnitArea &ctuArea)
{
  const SPS& sps = *cs.sps;

  for( auto &cu : cs.traverseCUs( ctuArea, CH_L ) )
  {
    // restore PCM samples
    if( ( cu.ipcm && sps.getPCMFilterDisableFlag() ) || CU::isLosslessCoded( cu ) )
    {
      const UInt numComponents = m_numberOfComponents;

      for( UInt comp = 0; comp < numComponents; comp++ )
      {
        xPCMSampleRestoration( cu, ComponentID( comp ) );
      }
    }
  }
}

Void SampleAdaptiveOffset::xPCMSampleRestoration(CodingUnit& cu, const ComponentID compID)
{
  const CompArea& ca = cu.block(compID);

  if( CU::isLosslessCoded( cu ) && !cu.ipcm )
  {
    for( auto &currTU : CU::traverseTUs( cu ) )
    {
      const CPelBuf& pcmBuf = currTU.getPcmbuf( compID );
             PelBuf dstBuf  = cu.cs->getRecoBuf( currTU.block(compID) );

      dstBuf.copyFrom( pcmBuf );
    }

    return;
  }

  const TransformUnit& tu = *cu.firstTU; CHECK( cu.firstTU != cu.lastTU, "Multiple TUs present in a PCM CU" );
  const CPelBuf& pcmBuf   = tu.getPcmbuf( compID );
         PelBuf dstBuf    = cu.cs->getRecoBuf( ca );
  const SPS &sps = *cu.cs->sps;
  const UInt uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));

  for (UInt y = 0; y < ca.height; y++)
  {
    for (UInt x = 0; x < ca.width; x++)
    {
      dstBuf.at(x,y) = (pcmBuf.at(x,y) << uiPcmLeftShiftBit);
    }
  }
}

Void SampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos,
  Bool& isLeftAvail,
  Bool& isRightAvail,
  Bool& isAboveAvail,
  Bool& isBelowAvail,
  Bool& isAboveLeftAvail,
  Bool& isAboveRightAvail,
  Bool& isBelowLeftAvail,
  Bool& isBelowRightAvail
  ) const
{
  const int width = cs.pcv->maxCUWidth;
  const int height = cs.pcv->maxCUHeight;
  const CodingUnit* cuCurr = cs.getCU(pos, CH_L);
  const CodingUnit* cuLeft = cs.getCU(pos.offset(-width, 0), CH_L);
  const CodingUnit* cuRight = cs.getCU(pos.offset(width, 0), CH_L);
  const CodingUnit* cuAbove = cs.getCU(pos.offset(0, -height), CH_L);
  const CodingUnit* cuBelow = cs.getCU(pos.offset(0, height), CH_L);
  const CodingUnit* cuAboveLeft = cs.getCU(pos.offset(-width, -height), CH_L);
  const CodingUnit* cuAboveRight = cs.getCU(pos.offset(width, -height), CH_L);
  const CodingUnit* cuBelowLeft = cs.getCU(pos.offset(-width, height), CH_L);
  const CodingUnit* cuBelowRight = cs.getCU(pos.offset(width, height), CH_L);

  // check cross slice flags
  {
    //left
    isLeftAvail       = (cuLeft != NULL)       ? ( !CU::isSameSlice(*cuCurr, *cuLeft)       ? cuCurr->slice->getLFCrossSliceBoundaryFlag()       : true ) : false;

    //above
    isAboveAvail      = (cuAbove != NULL)      ? ( !CU::isSameSlice(*cuCurr, *cuAbove)      ? cuCurr->slice->getLFCrossSliceBoundaryFlag()       : true ) : false;

    //right
    isRightAvail      = (cuRight != NULL)      ? ( !CU::isSameSlice(*cuCurr, *cuRight)      ? cuRight->slice->getLFCrossSliceBoundaryFlag()      : true ) : false;

    //below
    isBelowAvail      = (cuBelow != NULL)      ? ( !CU::isSameSlice(*cuCurr, *cuBelow)      ? cuBelow->slice->getLFCrossSliceBoundaryFlag()      : true ) : false;

    //above-left
    isAboveLeftAvail  = (cuAboveLeft != NULL)  ? ( !CU::isSameSlice(*cuCurr, *cuAboveLeft)  ? cuCurr->slice->getLFCrossSliceBoundaryFlag()       : true ) : false;

    //below-right
    isBelowRightAvail = (cuBelowRight != NULL) ? ( !CU::isSameSlice(*cuCurr, *cuBelowRight) ? cuBelowRight->slice->getLFCrossSliceBoundaryFlag() : true ) : false;

    //above-right
    isAboveRightAvail = false;
    if (cuAboveRight != NULL)
    {
      const bool bLFCrossSliceBoundaryFlag = (cuCurr->slice->getSliceCurStartCtuTsAddr() > cuAboveRight->slice->getSliceCurStartCtuTsAddr()) ? cuCurr->slice->getLFCrossSliceBoundaryFlag() : cuAboveRight->slice->getLFCrossSliceBoundaryFlag();
      isAboveRightAvail = ( !CU::isSameSlice(*cuCurr, *cuAboveRight) ) ? bLFCrossSliceBoundaryFlag : true;
    }

    //below-left
    isBelowLeftAvail = false;
    if (cuBelowLeft != NULL)
    {
      const bool bLFCrossSliceBoundaryFlag = (cuCurr->slice->getSliceCurStartCtuTsAddr() > cuBelowLeft->slice->getSliceCurStartCtuTsAddr()) ? cuCurr->slice->getLFCrossSliceBoundaryFlag() : cuBelowLeft->slice->getLFCrossSliceBoundaryFlag();
      isBelowLeftAvail = ( !CU::isSameSlice(*cuCurr, *cuBelowLeft) ) ? bLFCrossSliceBoundaryFlag : true;
    }
  }

#if HEVC_TILES_WPP
  // check cross tile flags
  const bool isLoopFilterAcrossTilePPS = cs.pps->getLoopFilterAcrossTilesEnabledFlag();
  if (!isLoopFilterAcrossTilePPS)
  {
    isLeftAvail       = (!isLeftAvail)       ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail      = (!isAboveAvail)      ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isRightAvail      = (!isRightAvail)      ? false : CU::isSameTile(*cuCurr, *cuRight);
    isBelowAvail      = (!isBelowAvail)      ? false : CU::isSameTile(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (!isAboveLeftAvail)  ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (!isAboveRightAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (!isBelowLeftAvail)  ? false : CU::isSameTile(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (!isBelowRightAvail) ? false : CU::isSameTile(*cuCurr, *cuBelowRight);
  }
#endif
}

//! \}
