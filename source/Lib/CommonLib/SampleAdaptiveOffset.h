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

/** \file     SampleAdaptiveOffset.h
    \brief    sample adaptive offset class (header)
*/

#ifndef __SAMPLEADAPTIVEOFFSET__
#define __SAMPLEADAPTIVEOFFSET__

#include "CommonDef.h"
#include "Unit.h"

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_SAO_TRUNCATED_BITDEPTH     10

// ====================================================================================================================
// Class definition
// ====================================================================================================================

template <typename T> Int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

class SampleAdaptiveOffset
{
public:
  SampleAdaptiveOffset();
  virtual ~SampleAdaptiveOffset();
  Void SAOProcess( CodingStructure& cs, SAOBlkParam* saoBlkParams
                   );
  Void create( Int picWidth, Int picHeight, ChromaFormat format, UInt maxCUWidth, UInt maxCUHeight, UInt maxCUDepth, UInt lumaBitShift, UInt chromaBitShift );
  Void destroy();
  static Int getMaxOffsetQVal(const Int channelBitDepth) { return (1<<(std::min<Int>(channelBitDepth,MAX_SAO_TRUNCATED_BITDEPTH)-5))-1; } //Table 9-32, inclusive

protected:
  Void deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos,
    Bool& isLeftAvail,
    Bool& isRightAvail,
    Bool& isAboveAvail,
    Bool& isBelowAvail,
    Bool& isAboveLeftAvail,
    Bool& isAboveRightAvail,
    Bool& isBelowLeftAvail,
    Bool& isBelowRightAvail
    ) const;

  Void offsetBlock(const Int channelBitDepth, const ClpRng& clpRng, Int typeIdx, Int* offset, const Pel* srcBlk, Pel* resBlk, Int srcStride, Int resStride,  Int width, Int height
                  , Bool isLeftAvail, Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isBelowLeftAvail, Bool isBelowRightAvail);
  Void invertQuantOffsets(ComponentID compIdx, Int typeIdc, Int typeAuxInfo, Int* dstOffsets, Int* srcOffsets);
  Void reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]);
  Int  getMergeList(CodingStructure& cs, Int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]);
  Void offsetCTU(const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs);
  Void xPCMLFDisableProcess(CodingStructure& cs);
  Void xPCMCURestoration(CodingStructure& cs, const UnitArea &ctuArea);
  Void xPCMSampleRestoration(CodingUnit& cu, const ComponentID compID);
  Void xReconstructBlkSAOParams(CodingStructure& cs, SAOBlkParam* saoBlkParams);

protected:
  UInt m_offsetStepLog2[MAX_NUM_COMPONENT]; //offset step
  PelStorage m_tempBuf;
  UInt m_numberOfComponents;

  std::vector<SChar> m_signLineBuf1;
  std::vector<SChar> m_signLineBuf2;
private:
  Bool m_picSAOEnabled[MAX_NUM_COMPONENT];
};

//! \}
#endif

