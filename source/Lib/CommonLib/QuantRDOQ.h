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

/** \file     QuantRDOQ.h
    \brief    RDOQ class (header)
*/

#ifndef __QUANTRDOQ__
#define __QUANTRDOQ__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "Quant.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// transform and quantization class
class QuantRDOQ : public Quant
{
public:
  QuantRDOQ( const Quant* other );
  ~QuantRDOQ();

public:
#if HEVC_USE_SCALING_LISTS
  Void setFlatScalingList   ( const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths );
  Void setScalingList       ( ScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths);
#endif
  // quantization
  Void quant                ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );

private:
#if HEVC_USE_SCALING_LISTS
  Double* xGetErrScaleCoeff              ( UInt list, UInt sizeX, UInt sizeY, Int qp ) { return m_errScale             [sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  Double& xGetErrScaleCoeffNoScalingList ( UInt list, UInt sizeX, UInt sizeY, Int qp ) { return m_errScaleNoScalingList[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  Void    xInitScalingList               ( const QuantRDOQ* other );
  Void    xDestroyScalingList            ();
  Void    xSetErrScaleCoeff              ( UInt list, UInt sizeX, UInt sizeY, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths );
#else
  Double  xGetErrScaleCoeff              ( SizeType width, SizeType height, Int qp, const Int maxLog2TrDynamicRange, const Int channelBitDepth);
#endif

  // RDOQ functions
  Void xRateDistOptQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx);

  inline UInt xGetCodedLevel  ( Double&             rd64CodedCost,
                                Double&             rd64CodedCost0,
                                Double&             rd64CodedCostSig,
                                Intermediate_Int    lLevelDouble,
                                UInt                uiMaxAbsLevel,
                                const BinFracBits*  fracBitsSig,
                                const BinFracBits&  fracBitsOne,
                                const BinFracBits&  fracBitsAbs,
#if JEM_TOOLS
                                const bool          useAltRC,
#endif
                                UShort              ui16AbsGoRice,
                                UInt                c1Idx,
                                UInt                c2Idx,
                                Int                 iQBits,
                                Double              errorScale,
                                Bool                bLast,
                                Bool                useLimitedPrefixLength,
                                const Int           maxLog2TrDynamicRange
                              ) const;
  inline Int xGetICRate  ( const UInt         uiAbsLevel,
                           const BinFracBits& fracBitsOne,
                           const BinFracBits& fracBitsAbs,
#if JEM_TOOLS
                           const bool         useAltRC,
#endif
                           const UShort       ui16AbsGoRice,
                           const UInt         c1Idx,
                           const UInt         c2Idx,
                           const Bool         useLimitedPrefixLength,
                           const Int          maxLog2TrDynamicRange
                         ) const;
  inline Double xGetRateLast         ( const int* lastBitsX, const int* lastBitsY,
                                       unsigned        PosX, unsigned   PosY                              ) const;

  inline Double xGetRateSigCoeffGroup( const BinFracBits& fracBitsSigCG,   unsigned uiSignificanceCoeffGroup ) const;

  inline Double xGetRateSigCoef      ( const BinFracBits& fracBitsSig,     unsigned uiSignificance           ) const;

  inline Double xGetICost            ( Double dRate                                                      ) const;
  inline Double xGetIEPRate          (                                                                   ) const;

private:
#if HEVC_USE_SCALING_LISTS
  Bool    m_isErrScaleListOwner;

  Double *m_errScale             [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  Double  m_errScaleNoScalingList[SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
#endif
  // temporary buffers for RDOQ
  Double m_pdCostCoeff        [MAX_TU_SIZE * MAX_TU_SIZE];
  Double m_pdCostSig          [MAX_TU_SIZE * MAX_TU_SIZE];
  Double m_pdCostCoeff0       [MAX_TU_SIZE * MAX_TU_SIZE];
  Double m_pdCostCoeffGroupSig[(MAX_TU_SIZE * MAX_TU_SIZE) >> MLS_CG_SIZE]; // even if CG size is 2 (if one of the sides is 2) instead of 4, there should be enough space
#if HEVC_USE_SIGN_HIDING
  Int    m_rateIncUp          [MAX_TU_SIZE * MAX_TU_SIZE];
  Int    m_rateIncDown        [MAX_TU_SIZE * MAX_TU_SIZE];
  Int    m_sigRateDelta       [MAX_TU_SIZE * MAX_TU_SIZE];
  TCoeff m_deltaU             [MAX_TU_SIZE * MAX_TU_SIZE];
#endif
};// END CLASS DEFINITION QuantRDOQ

//! \}

#endif // __QUANTRDOQ__
