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

/** \file     WeightPrediction.h
    \brief    weighting prediction class (header)
*/

// Include files
#include "CommonDef.h"
#include "Unit.h"
#include "InterpolationFilter.h"
#include "WeightPrediction.h"
#include "CodingStructure.h"


static inline Pel weightBidir( Int w0, Pel P0, Int w1, Pel P1, Int round, Int shift, Int offset, const ClpRng& clpRng)
{
  return ClipPel( ( (w0*(P0 + IF_INTERNAL_OFFS) + w1*(P1 + IF_INTERNAL_OFFS) + round + (offset << (shift-1))) >> shift ), clpRng );
}

static inline Pel weightUnidir( Int w0, Pel P0, Int round, Int shift, Int offset, const ClpRng& clpRng)
{
  return ClipPel( ( (w0*(P0 + IF_INTERNAL_OFFS) + round) >> shift ) + offset, clpRng );
}

static inline Pel noWeightUnidir( Pel P0, Int round, Int shift, Int offset, const ClpRng& clpRng)
{
  return ClipPel( ( ((P0 + IF_INTERNAL_OFFS) + round) >> shift ) + offset, clpRng );
}

static inline Pel noWeightOffsetUnidir( Pel P0, Int round, Int shift, const ClpRng& clpRng)
{
  return ClipPel( ( ((P0 + IF_INTERNAL_OFFS) + round) >> shift ), clpRng );
}


// ====================================================================================================================
// Class definition
// ====================================================================================================================

WeightPrediction::WeightPrediction()
{
}



Void  WeightPrediction::getWpScaling(const Slice                *pcSlice,
                                     const Int                  &iRefIdx0,
                                     const Int                  &iRefIdx1,
                                           WPScalingParam      *&wp0,
                                           WPScalingParam      *&wp1,
                                     const ComponentID           maxNumComp)
{
  CHECK(iRefIdx0 < 0 && iRefIdx1 < 0, "Both picture reference list indizes smaller than '0'");

  const Bool wpBiPred        = pcSlice->getPPS()->getWPBiPred();
  const Bool bBiPred         = (iRefIdx0 >= 0 && iRefIdx1 >= 0);
  const Bool bUniPred        = !bBiPred;

  if (bUniPred || wpBiPred)
  {
    // explicit --------------------
    if (iRefIdx0 >= 0)
    {
      pcSlice->getWpScaling(REF_PIC_LIST_0, iRefIdx0, wp0);
    }
    if (iRefIdx1 >= 0)
    {
      pcSlice->getWpScaling(REF_PIC_LIST_1, iRefIdx1, wp1);
    }
  }
  else
  {
    THROW( "Unsupported WP configuration" );
  }

  if (iRefIdx0 < 0)
  {
    wp0 = NULL;
  }
  if (iRefIdx1 < 0)
  {
    wp1 = NULL;
  }

  const UInt numValidComponent = getNumberValidComponents(pcSlice->getSPS()->getChromaFormatIdc());
  const Bool bUseHighPrecisionPredictionWeighting = pcSlice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();

  if (bBiPred)
  {
    // Bi-predictive case
    for (Int yuv = 0; yuv < numValidComponent && yuv <= maxNumComp; yuv++)
    {
      const Int bitDepth = pcSlice->getSPS()->getBitDepth(toChannelType(ComponentID(yuv)));
      const Int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (bitDepth - 8));

      wp0[yuv].w = wp0[yuv].iWeight;
      wp1[yuv].w = wp1[yuv].iWeight;
      wp0[yuv].o = wp0[yuv].iOffset * offsetScalingFactor;
      wp1[yuv].o = wp1[yuv].iOffset * offsetScalingFactor;
      wp0[yuv].offset = wp0[yuv].o + wp1[yuv].o;
      wp0[yuv].shift = wp0[yuv].uiLog2WeightDenom + 1;
      wp0[yuv].round = (1 << wp0[yuv].uiLog2WeightDenom);
      wp1[yuv].offset = wp0[yuv].offset;
      wp1[yuv].shift = wp0[yuv].shift;
      wp1[yuv].round = wp0[yuv].round;
    }
  }
  else
  {
    // UniPred
    WPScalingParam *const pwp = (iRefIdx0 >= 0) ? wp0 : wp1;

    for (Int yuv = 0; yuv < numValidComponent && yuv <= maxNumComp; yuv++)
    {
      const Int bitDepth            = pcSlice->getSPS()->getBitDepth(toChannelType(ComponentID(yuv)));
      const Int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (bitDepth - 8));

      pwp[yuv].w      = pwp[yuv].iWeight;
      pwp[yuv].offset = pwp[yuv].iOffset * offsetScalingFactor;
      pwp[yuv].shift  = pwp[yuv].uiLog2WeightDenom;
      pwp[yuv].round  = (pwp[yuv].uiLog2WeightDenom >= 1) ? (1 << (pwp[yuv].uiLog2WeightDenom - 1)) : (0);
    }
  }
}

Void WeightPrediction::addWeightBi(const CPelUnitBuf          &pcYuvSrc0,
                                   const CPelUnitBuf          &pcYuvSrc1,
                                   const ClpRngs              &clpRngs,
                                   const WPScalingParam *const wp0,
                                   const WPScalingParam *const wp1,
                                         PelUnitBuf           &rpcYuvDst,
                                   const Bool                  bRoundLuma /*= true*/,
                                   const ComponentID           maxNumComp)
{
  const Bool enableRounding[MAX_NUM_COMPONENT] = { bRoundLuma, true, true };

  const UInt numValidComponent = (const UInt)pcYuvSrc0.bufs.size();

  for (Int componentIndex = 0; componentIndex < numValidComponent && componentIndex <= maxNumComp; componentIndex++)
  {
    const ComponentID compID = ComponentID(componentIndex);

    const Pel* pSrc0 = pcYuvSrc0.bufs[compID].buf;
    const Pel* pSrc1 = pcYuvSrc1.bufs[compID].buf;
          Pel* pDst  = rpcYuvDst.bufs[compID].buf;

    // Luma : --------------------------------------------
    const ClpRng& clpRng = clpRngs.comp[compID];
    const Int  w0       = wp0[compID].w;
    const Int  offset   = wp0[compID].offset;
    const Int  clipBD   = clpRng.bd;
    const Int  shiftNum = std::max<Int>(2, (IF_INTERNAL_PREC - clipBD));
    const Int  shift    = wp0[compID].shift + shiftNum;
    const Int  round    = (enableRounding[compID] && (shift > 0)) ? (1 << (shift - 1)) : 0;
    const Int  w1       = wp1[compID].w;
    const Int  iHeight  = rpcYuvDst.bufs[compID].height;
    const Int  iWidth   = rpcYuvDst.bufs[compID].width;

    const UInt iSrc0Stride = pcYuvSrc0.bufs[compID].stride;
    const UInt iSrc1Stride = pcYuvSrc1.bufs[compID].stride;
    const UInt iDstStride =  rpcYuvDst.bufs[compID].stride;

    for (Int y = iHeight - 1; y >= 0; y--)
    {
      // do it in batches of 4 (partial unroll)
      Int x = iWidth - 1;

      for (; x >= 3; )
      {
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
      }
      for (; x >= 0; x--)
      {
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng );
      }

      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst += iDstStride;
    } // y loop
  } // compID loop
}

Void  WeightPrediction::addWeightUni(const CPelUnitBuf          &pcYuvSrc0,
                                     const ClpRngs              &clpRngs,
                                     const WPScalingParam *const wp0,
                                           PelUnitBuf           &rpcYuvDst,
                                     const ComponentID           maxNumComp)
{
  const UInt numValidComponent = (const UInt)pcYuvSrc0.bufs.size();

  for (Int componentIndex = 0; componentIndex < numValidComponent && componentIndex <= maxNumComp; componentIndex++)
  {
    const ComponentID compID = ComponentID(componentIndex);

    const Pel* pSrc0 = pcYuvSrc0.bufs[compID].buf;
          Pel* pDst  = rpcYuvDst.bufs[compID].buf;

    // Luma : --------------------------------------------
    const ClpRng& clpRng    = clpRngs.comp[compID];
    const Int  w0           = wp0[compID].w;
    const Int  offset       = wp0[compID].offset;
    const Int  clipBD       = clpRng.bd;
    const Int  shiftNum     = std::max<Int>(2, (IF_INTERNAL_PREC - clipBD));
    const Int  shift        = wp0[compID].shift + shiftNum;
    const UInt iSrc0Stride  = pcYuvSrc0.bufs[compID].stride;
    const UInt iDstStride   = rpcYuvDst.bufs[compID].stride;
    const Int  iHeight      = rpcYuvDst.bufs[compID].height;
    const Int  iWidth       = rpcYuvDst.bufs[compID].width;

    if (w0 != 1 << wp0[compID].shift)
    {
      const Int  round = (shift > 0) ? (1 << (shift - 1)) : 0;
      for (Int y = iHeight - 1; y >= 0; y--)
      {
        Int x = iWidth - 1;
        for (; x >= 3; )
        {
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
        }
        for (; x >= 0; x--)
        {
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng);
        }
        pSrc0 += iSrc0Stride;
        pDst += iDstStride;
      }
    }
    else
    {
      const Int  round = (shiftNum > 0) ? (1 << (shiftNum - 1)) : 0;
      if (offset == 0)
      {
        for (Int y = iHeight - 1; y >= 0; y--)
        {
          Int x = iWidth - 1;
          for (; x >= 3; )
          {
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
          }
          for (; x >= 0; x--)
          {
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng);
          }
          pSrc0 += iSrc0Stride;
          pDst += iDstStride;
        }
      }
      else
      {
        for (Int y = iHeight - 1; y >= 0; y--)
        {
          Int x = iWidth - 1;
          for (; x >= 3; )
          {
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
          }
          for (; x >= 0; x--)
          {
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng);
          }
          pSrc0 += iSrc0Stride;
          pDst += iDstStride;
        }
      }
    }
  }
}

Void  WeightPrediction::xWeightedPredictionUni(const PredictionUnit       &pu,
                                               const CPelUnitBuf          &pcYuvSrc,
                                               const RefPicList           &eRefPicList,
                                                     PelUnitBuf           &pcYuvPred,
                                               const Int                   iRefIdx_input/* = -1*/,
                                               const ComponentID           maxNumComp)
{
  WPScalingParam  *pwp, *pwpTmp;

  Int iRefIdx = iRefIdx_input;
  if (iRefIdx < 0)
  {
    iRefIdx = pu.refIdx[eRefPicList];
  }

  CHECK(iRefIdx < 0, "Negative reference picture list index");

  if (eRefPicList == REF_PIC_LIST_0)
  {
    getWpScaling(pu.cs->slice, iRefIdx, -1, pwp, pwpTmp, maxNumComp);
  }
  else
  {
    getWpScaling(pu.cs->slice, -1, iRefIdx, pwpTmp, pwp, maxNumComp);
  }
  addWeightUni(pcYuvSrc, pu.cu->slice->clpRngs(), pwp, pcYuvPred, maxNumComp);
}

Void  WeightPrediction::xWeightedPredictionBi(const PredictionUnit       &pu,
                                              const CPelUnitBuf          &pcYuvSrc0,
                                              const CPelUnitBuf          &pcYuvSrc1,
                                                    PelUnitBuf           &rpcYuvDst,
                                              const ComponentID           maxNumComp)
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];
  WPScalingParam  *pwp0;
  WPScalingParam  *pwp1;

  CHECK( !pu.cs->pps->getWPBiPred(), "Weighted Bi-prediction disabled" );

  getWpScaling(pu.cu->slice, iRefIdx0, iRefIdx1, pwp0, pwp1, maxNumComp);

  if (iRefIdx0 >= 0 && iRefIdx1 >= 0)
  {
    addWeightBi(pcYuvSrc0, pcYuvSrc1, pu.cu->slice->clpRngs(), pwp0, pwp1, rpcYuvDst, true, maxNumComp);
  }
  else if (iRefIdx0 >= 0 && iRefIdx1 < 0)
  {
    addWeightUni(pcYuvSrc0, pu.cu->slice->clpRngs(), pwp0, rpcYuvDst, maxNumComp);
  }
  else if (iRefIdx0 < 0 && iRefIdx1 >= 0)
  {
    addWeightUni(pcYuvSrc1, pu.cu->slice->clpRngs(), pwp1, rpcYuvDst, maxNumComp);
  }
  else
  {
    THROW( "Both reference picture list indizes are negative" );
  }
}
