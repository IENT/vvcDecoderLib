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

/** \file     Quant.h
    \brief    base quantization class (header)
*/

#ifndef __QUANT__
#define __QUANT__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define QP_BITS                 15

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// QP struct
struct QpParam
{
  Int Qp;
  Int per;
  Int rem;

private:

  QpParam(const Int           qpy,
          const ChannelType   chType,
          const Int           qpBdOffset,
          const Int           chromaQPOffset,
          const ChromaFormat  chFmt,
          const int           dqp );

public:

  QpParam(const TransformUnit& tu, const ComponentID &compID, const int QP = -MAX_INT);

}; // END STRUCT DEFINITION QpParam

/// transform and quantization class
class Quant
{
public:
  Quant( const Quant* other );
  virtual ~Quant();

  // initialize class
  virtual void init( UInt uiMaxTrSize,
                     bool useRDOQ = false,
                     bool useRDOQTS = false,
#if JVET_K0072
#else
#if JEM_TOOLS
                     UInt uiAltResiCompId = 0,
#endif
#endif
#if T0196_SELECTIVE_RDOQ
                     bool useSelectiveRDOQ = false
#endif
                     );

public:

  void   transformSkipQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &resiDiff, TCoeff &coeff,    const UInt &uiPos, const QpParam &cQP, const bool bUseHalfRoundingPoint);
  void   invTrSkipDeQuantOneSample  (TransformUnit &tu, const ComponentID &compID, const TCoeff &pcCoeff,  Pel &reconSample, const UInt &uiPos, const QpParam &cQP);

#if RDOQ_CHROMA_LAMBDA
  void   setLambdas              ( const Double lambdas[MAX_NUM_COMPONENT] )   { for (UInt component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  void   selectLambda            ( const ComponentID compIdx )                 { m_dLambda = m_lambdas[ MAP_CHROMA(compIdx) ]; }
  void   getLambdas              ( Double (&lambdas)[MAX_NUM_COMPONENT]) const { for (UInt component = 0; component < MAX_NUM_COMPONENT; component++) lambdas[component] = m_lambdas[component]; }
#endif
  void   setLambda               ( const Double dLambda )                      { m_dLambda = dLambda; }
  Double getLambda               () const                                      { return m_dLambda; }

#if HEVC_USE_SCALING_LISTS
  Int* getQuantCoeff             ( UInt list, Int qp, UInt sizeX, UInt sizeY ) { return m_quantCoef            [sizeX][sizeY][list][qp]; };  //!< get Quant Coefficent
  Int* getDequantCoeff           ( UInt list, Int qp, UInt sizeX, UInt sizeY ) { return m_dequantCoef          [sizeX][sizeY][list][qp]; };  //!< get DeQuant Coefficent

  void setUseScalingList         ( bool bUseScalingList){ m_scalingListEnabledFlag = bUseScalingList; };
  bool getUseScalingList         ( const UInt width, const UInt height, const bool isTransformSkip){ return m_scalingListEnabledFlag && (!isTransformSkip || ((width == 4) && (height == 4))); };

  void setScalingListDec         ( const ScalingList &scalingList);
  void processScalingListEnc     ( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc);
  void processScalingListDec     ( const Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc);

  virtual void setFlatScalingList( const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths);
  virtual void setScalingList    ( ScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths);
#endif

  // quantization
  virtual void quant             ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );
  // de-quantization
  virtual void dequant           ( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP );

#if ENABLE_SPLIT_PARALLELISM
  virtual void copyState         ( const Quant& other );
#endif

protected:

#if T0196_SELECTIVE_RDOQ
  bool xNeedRDOQ                 ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, const QpParam &cQP );
#endif

  Double   m_dLambda;
  UInt     m_uiMaxTrSize;
  bool     m_useRDOQ;
  bool     m_useRDOQTS;
#if T0196_SELECTIVE_RDOQ
  bool     m_useSelectiveRDOQ;
#endif
#if JVET_K0072
#else
#if JEM_TOOLS
  Int      m_altResiCompId;
#endif
#endif
#if HEVC_USE_SCALING_LISTS
private:
  void xInitScalingList   ( const Quant* other );
  void xDestroyScalingList();
  void xSetFlatScalingList( UInt list, UInt sizeX, UInt sizeY, Int qp );
  void xSetScalingListEnc ( ScalingList *scalingList, UInt list, UInt size, Int qp );
  void xSetScalingListDec ( const ScalingList &scalingList, UInt list, UInt size, Int qp );
#endif
#if HEVC_USE_SIGN_HIDING
private:
  void xSignBitHidingHDQ  (TCoeff* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const Int maxLog2TrDynamicRange);
#endif

private:
#if RDOQ_CHROMA_LAMBDA
  Double   m_lambdas[MAX_NUM_COMPONENT];
#endif
#if HEVC_USE_SCALING_LISTS
  bool     m_scalingListEnabledFlag;
  bool     m_isScalingListOwner;

  Int      *m_quantCoef            [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  Int      *m_dequantCoef          [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of dequantization matrix coefficient 4x4
#endif
};// END CLASS DEFINITION Quant


//! \}

#endif // __QUANT__
