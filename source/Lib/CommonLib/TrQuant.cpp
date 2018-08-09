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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"


#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "QuantRDOQ.h"
#if JVET_K0072
#include "DepQuant.h"
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

struct coeffGroupRDStats
{
  Int    iNNZbeforePos0;
  Double d64CodedLevelandDist; // distortion and level cost only
  Double d64UncodedDist;    // all zero coded block distortion
  Double d64SigCost;
  Double d64SigCost_0;
};

#if JEM_TOOLS
FwdTrans *fastFwdTrans[5][7] =
{
  { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64, fastForwardDCT2_B128 },
  { NULL,               fastForwardDCT5_B4, fastForwardDCT5_B8, fastForwardDCT5_B16, fastForwardDCT5_B32, fastForwardDCT5_B64, fastForwardDCT5_B128 },
  { NULL,               fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, fastForwardDCT8_B64, fastForwardDCT8_B128 },
  { NULL,               fastForwardDST1_B4, fastForwardDST1_B8, fastForwardDST1_B16, fastForwardDST1_B32, fastForwardDST1_B64, fastForwardDST1_B128 },
  { NULL,               fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, fastForwardDST7_B64, fastForwardDST7_B128 },
};

InvTrans *fastInvTrans[5][7] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64, fastInverseDCT2_B128 },
  { NULL,               fastInverseDCT5_B4, fastInverseDCT5_B8, fastInverseDCT5_B16, fastInverseDCT5_B32, fastInverseDCT5_B64, fastInverseDCT5_B128 },
  { NULL,               fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, fastInverseDCT8_B64, fastInverseDCT8_B128 },
  { NULL,               fastInverseDST1_B4, fastInverseDST1_B8, fastInverseDST1_B16, fastInverseDST1_B32, fastInverseDST1_B64, fastInverseDST1_B128 },
  { NULL,               fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, fastInverseDST7_B64, fastInverseDST7_B128 },
};
#endif

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
#if HEVC_USE_4x4_DSTVII
void xTrMxN ( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );
void xITrMxN( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );
#else
void xTrMxN ( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, const int maxLog2TrDynamicRange );
void xITrMxN( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, const int maxLog2TrDynamicRange );
#endif


TrQuant::TrQuant() : m_quant( nullptr ), m_fTr( xTrMxN ), m_fITr( xITrMxN )
{
  // allocate temporary buffers
  m_plTempCoeff   = (TCoeff*) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );

}

TrQuant::~TrQuant()
{
  if( m_quant )
  {
    delete m_quant;
    m_quant = nullptr;
  }

  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    xFree( m_plTempCoeff );
    m_plTempCoeff = nullptr;
  }
}

#if ENABLE_SPLIT_PARALLELISM

void TrQuant::copyState( const TrQuant& other )
{
  m_quant->copyState( *other.m_quant );
}
#endif

#if JEM_TOOLS
#if HEVC_USE_4x4_DSTVII
void xTrMxN_EMT( const Int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, Int iWidth, Int iHeight, bool useDST, const int maxLog2TrDynamicRange,
#else
void xTrMxN_EMT( const Int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, Int iWidth, Int iHeight, const int maxLog2TrDynamicRange,
#endif
  const UChar ucMode, const UChar ucTrIdx
#if !INTRA67_3MPM
  , const bool use65intraModes
#endif
  , const bool useQTBT )
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
  const Int shift_1st = ((g_aucLog2[iWidth ]) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
  const Int shift_2nd =  (g_aucLog2[iHeight])            + TRANSFORM_MATRIX_SHIFT                          + COM16_C806_TRANS_PREC;
  const UInt transformWidthIndex  = g_aucLog2[iWidth ] - 1;  //nLog2WidthMinus1, since transform start from 2-point
  const UInt transformHeightIndex = g_aucLog2[iHeight] - 1;  //nLog2HeightMinus1, since transform start from 2-point
  const Int iZeroOutThresh = JVET_C0024_ZERO_OUT_TH;

  Int iSkipWidth = 0, iSkipHeight = 0;
  if( useQTBT )
  {
    iSkipWidth  = (iWidth  > iZeroOutThresh ? iWidth  - iZeroOutThresh : 0);
    iSkipHeight = (iHeight > iZeroOutThresh ? iHeight - iZeroOutThresh : 0);
  }
  else
  if( ( ( ucMode == INTER_MODE_IDX || iWidth > iZeroOutThresh ) && ucTrIdx != DCT2_EMT && iWidth >= iZeroOutThresh ) || ( ucTrIdx == DCT2_EMT && iWidth > iZeroOutThresh ) )
  {
    iSkipWidth  = iWidth  >> 1;
    iSkipHeight = iHeight >> 1;
  }

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      block[( y * iWidth ) + x] = residual[( y * stride ) + x];
    }
  }

  TCoeff *tmp = ( TCoeff * ) alloca( iWidth * iHeight * sizeof( TCoeff ) );

  UInt  nTrIdxHor = DCT2, nTrIdxVer = DCT2;
  if( ucMode != INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    UInt  nTrSubsetHor, nTrSubsetVer;
#if INTRA67_3MPM
    nTrSubsetHor = g_aucTrSetHorz[ucMode];
    nTrSubsetVer = g_aucTrSetVert[ucMode];
#else
    if( use65intraModes )
    {
      nTrSubsetHor = g_aucTrSetHorz[ucMode];
      nTrSubsetVer = g_aucTrSetVert[ucMode];
    }
    else //we use only 35 intra modes
    {
      nTrSubsetHor = g_aucTrSetHorz35[ucMode];
      nTrSubsetVer = g_aucTrSetVert35[ucMode];
    }
#endif
    nTrIdxHor = g_aiTrSubsetIntra[nTrSubsetHor][ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetIntra[nTrSubsetVer][ucTrIdx >> 1];
  }
  if( ucMode == INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    nTrIdxHor = g_aiTrSubsetInter[ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetInter[ucTrIdx >> 1];
  }

  fastFwdTrans[nTrIdxHor][transformWidthIndex](block, tmp, shift_1st, iHeight, 0, iSkipWidth, 1);
  fastFwdTrans[nTrIdxVer][transformHeightIndex](tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1);
}

/** MxN inverse transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param coeff                 [in]  transform coefficients
*  \param residual              [out] residual block
*  \param stride                [in]  stride of the residual block
*  \param iWidth                [in]  width of transform
*  \param iHeight               [in]  height of transform
*  \param uiSkipWidth           [in]
*  \param uiSkipHeight          [in]
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]
*  \param ucMode                [in]
*  \param ucTrIdx               [in]
*  \param use65intraModes       [in]
*/

#if HEVC_USE_4x4_DSTVII
#if INTRA67_3MPM
void xITrMxN_EMT(const Int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, Int iWidth, Int iHeight, UInt uiSkipWidth, UInt uiSkipHeight, Bool useDST, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx )
#else
void xITrMxN_EMT( const Int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, Int iWidth, Int iHeight, UInt uiSkipWidth, UInt uiSkipHeight, Bool useDST, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx, bool use65intraModes )
#endif
#else
#if INTRA67_3MPM
void xITrMxN_EMT(const Int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, Int iWidth, Int iHeight, UInt uiSkipWidth, UInt uiSkipHeight, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx )
#else
void xITrMxN_EMT( const Int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, Int iWidth, Int iHeight, UInt uiSkipWidth, UInt uiSkipHeight, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx, bool use65intraModes )
#endif
#endif
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const TCoeff clipMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff clipMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const Int shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; //1 has been added to shift_1st at the expense of shift_2nd
  const Int shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
  const UInt transformWidthIndex  = g_aucLog2[iWidth ] - 1;  //nLog2WidthMinus1, since transform start from 2-point
  const UInt transformHeightIndex = g_aucLog2[iHeight] - 1;  //nLog2HeightMinus1, since transform start from 2-point

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  TCoeff *tmp   = ( TCoeff * ) alloca( iWidth * iHeight * sizeof( TCoeff ) );
  TCoeff *block = ( TCoeff * ) alloca( iWidth * iHeight * sizeof( TCoeff ) );

  UInt  nTrIdxHor = DCT2, nTrIdxVer = DCT2;
  if( ucMode != INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    UInt  nTrSubsetHor, nTrSubsetVer;
#if INTRA67_3MPM
    nTrSubsetHor = g_aucTrSetHorz[ucMode];
    nTrSubsetVer = g_aucTrSetVert[ucMode];
#else
    if( use65intraModes )
    {
      nTrSubsetHor = g_aucTrSetHorz[ucMode];
      nTrSubsetVer = g_aucTrSetVert[ucMode];
    }
    else //we use only 35 intra modes
    {
      nTrSubsetHor = g_aucTrSetHorz35[ucMode];
      nTrSubsetVer = g_aucTrSetVert35[ucMode];
    }
#endif
    nTrIdxHor = g_aiTrSubsetIntra[nTrSubsetHor][ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetIntra[nTrSubsetVer][ucTrIdx >> 1];
  }
  if( ucMode == INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    nTrIdxHor = g_aiTrSubsetInter[ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetInter[ucTrIdx >> 1];
  }

  fastInvTrans[nTrIdxVer][transformHeightIndex](coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 1, clipMinimum, clipMaximum);
  fastInvTrans[nTrIdxHor][transformWidthIndex](tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 1, clipMinimum, clipMaximum);

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      residual[( y * stride ) + x] = Pel( block[( y * iWidth ) + x] );
    }
  }
}
#endif

/** MxN forward transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param residual              [in]  residual block
*  \param stride                [in]  stride of residual block
*  \param coeff                 [out] transform coefficients
*  \param width                 [in]  width of transform
*  \param height                [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]

*/
#if HEVC_USE_4x4_DSTVII
void xTrMxN( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange )
#else
void xTrMxN( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, const int maxLog2TrDynamicRange )
#endif
{
  const int iWidth  = (int)width;
  const int iHeight = (int)height;

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const Int shift_1st = (g_aucLog2[iWidth] +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange;
  const Int shift_2nd = g_aucLog2[iHeight] + TRANSFORM_MATRIX_SHIFT;
  const Int iZeroOutThresh = JVET_C0024_ZERO_OUT_TH;

  UInt iSkipWidth  = (iWidth  > iZeroOutThresh ? iWidth  - iZeroOutThresh : 0);
  UInt iSkipHeight = (iHeight > iZeroOutThresh ? iHeight - iZeroOutThresh : 0);

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff   tmp[MAX_TU_SIZE * MAX_TU_SIZE] );

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      block[( y * iWidth ) + x] = residual[( y * stride ) + x];
    }
  }

  {
    switch (iWidth)
    {
    case 2:     fastForwardDCT2_B2( block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0 );  break;
    case 4:
      {
#if HEVC_USE_4x4_DSTVII
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastForwardDST7_B4( block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0 );
        }
        else
#endif
        {
          fastForwardDCT2_B4( block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0 );
        }
      }
      break;

    case 8:     fastForwardDCT2_B8  ( block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0 );  break;
    case 16:    fastForwardDCT2_B16 ( block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0 );  break;
    case 32:    fastForwardDCT2_B32 ( block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0 );  break;
    case 64:    fastForwardDCT2_B64 ( block, tmp, shift_1st + COM16_C806_TRANS_PREC, iHeight, 0, iSkipWidth, 0 );  break;
    case 128:   fastForwardDCT2_B128( block, tmp, shift_1st + COM16_C806_TRANS_PREC, iHeight, 0, iSkipWidth, 0 );  break;
    default:
      THROW( "Unsupported transformation size" ); break;
    }
  }

  {
    switch (iHeight)
    {
    case 2:     fastForwardDCT2_B2( tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );  break;
    case 4:
      {
#if HEVC_USE_4x4_DSTVII
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastForwardDST7_B4( tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );
        }
        else
#endif
        {
          fastForwardDCT2_B4( tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );
        }
      }
      break;

    case 8:     fastForwardDCT2_B8  ( tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );  break;
    case 16:    fastForwardDCT2_B16 ( tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );  break;
    case 32:    fastForwardDCT2_B32 ( tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );  break;
    case 64:    fastForwardDCT2_B64 ( tmp, coeff, shift_2nd + COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0 );  break;
    case 128:   fastForwardDCT2_B128( tmp, coeff, shift_2nd + COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0 );  break;
    default:
      THROW( "Unsupported transformation size" ); break;
    }
  }
}


/** MxN inverse transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param coeff                 [in]  transform coefficients
*  \param residual              [out] residual block
*  \param stride                [out] stride of the residual block
*  \param width                 [in]  width of transform
*  \param height                [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]
*/
#if HEVC_USE_4x4_DSTVII
void xITrMxN( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange )
#else
void xITrMxN( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, const int maxLog2TrDynamicRange )
#endif
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const int iWidth  = (int)width;
  const int iHeight = (int)height;


  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  UInt uiSkipWidth  = ( iWidth  > JVET_C0024_ZERO_OUT_TH ? iWidth  - JVET_C0024_ZERO_OUT_TH : 0 );
  UInt uiSkipHeight = ( iHeight > JVET_C0024_ZERO_OUT_TH ? iHeight - JVET_C0024_ZERO_OUT_TH : 0 );

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff   tmp[MAX_TU_SIZE * MAX_TU_SIZE] );

  {
    switch (iHeight)
    {
    case 2: fastInverseDCT2_B2( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum ); break;
    case 4:
      {
#if HEVC_USE_4x4_DSTVII
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDST7_B4( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum );
        }
        else
#endif
        {
          fastInverseDCT2_B4( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum );
        }
      }
      break;

    case   8: fastInverseDCT2_B8  ( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum ); break;
    case  16: fastInverseDCT2_B16 ( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum ); break;
    case  32: fastInverseDCT2_B32 ( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum ); break;
    case  64: fastInverseDCT2_B64 ( coeff, tmp, shift_1st + COM16_C806_TRANS_PREC, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum ); break;
    case 128: fastInverseDCT2_B128( coeff, tmp, shift_1st + COM16_C806_TRANS_PREC, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum ); break;
    default:
      THROW( "Unsupported transformation size" ); break;
    }
  }

  {
    switch (iWidth)
    {
    case 2: fastInverseDCT2_B2( tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
    case 4:
      {
#if HEVC_USE_4x4_DSTVII
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDST7_B4( tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
        }
        else
#endif
        {
          fastInverseDCT2_B4( tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
        }
      }
      break;

    case   8: fastInverseDCT2_B8  ( tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    case  16: fastInverseDCT2_B16 ( tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    case  32: fastInverseDCT2_B32 ( tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    case  64: fastInverseDCT2_B64 ( tmp, block, shift_2nd + COM16_C806_TRANS_PREC, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    case 128: fastInverseDCT2_B128( tmp, block, shift_2nd + COM16_C806_TRANS_PREC, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    default:
      THROW( "Unsupported transformation size" );
      break;
    }
  }

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      residual[(y * stride)+x] = Pel(block[(y * width) + x]);
    }
  }
}



Void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  m_quant->dequant( tu, dstCoeff, compID, cQP );
}

Void TrQuant::init( const Quant* otherQuant,
                    const UInt uiMaxTrSize,
                    const bool bUseRDOQ,
                    const bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                    const bool useSelectiveRDOQ,
#endif
#if JVET_K0072
#else
#if JEM_TOOLS
                    const UInt uiAltResiCompId,
#endif
#endif
                    const bool bEnc,
                    const bool useTransformSkipFast,
#if !INTRA67_3MPM
#if JEM_TOOLS
                    const bool use65IntraModes,
#endif
#endif
                    const bool rectTUs
)
{
  m_uiMaxTrSize          = uiMaxTrSize;
  m_bEnc                 = bEnc;
  m_useTransformSkipFast = useTransformSkipFast;
#if !INTRA67_3MPM
#if JEM_TOOLS
  m_use65IntraModes      = use65IntraModes;
#endif
#endif
  m_rectTUs              = rectTUs;

  delete m_quant;
  m_quant = nullptr;

#if JVET_K0072
  if( bUseRDOQ || !bEnc )
  {
    m_quant = new DepQuant( otherQuant, bEnc );
  }
#else
  if( bUseRDOQ )
  {
    m_quant = new QuantRDOQ( otherQuant );
  }
#endif
  else
    m_quant = new Quant( otherQuant );

  if( m_quant )
  {
#if JVET_K0072
    m_quant->init( uiMaxTrSize, bUseRDOQ, bUseRDOQTS, useSelectiveRDOQ );
#else
#if JEM_TOOLS
    m_quant->init( uiMaxTrSize, bUseRDOQ, bUseRDOQTS, uiAltResiCompId, useSelectiveRDOQ );
#else
    m_quant->init( uiMaxTrSize, bUseRDOQ, bUseRDOQTS, useSelectiveRDOQ );
#endif
#endif
  }
}

#if JEM_TOOLS
Void TrQuant::FwdNsstNxN( Int* src, const UInt uiMode, const UInt uiIndex, const UInt uiSize )
{
  const int   rnd = uiSize >> 1;
  const int   shl = 5;
#if ENABLE_BMS
  const int * par = /*((uiSize > 2) ?*/ g_nsstHyGTPar4x4[uiMode][uiIndex] /*: g_nsstHyGTPar2x2[uiMode][uiIndex])*/;
#else
  const int * par = (uiSize > 4) ? g_nsstHyGTPar8x8[uiMode][uiIndex] : /*((uiSize > 2) ?*/ g_nsstHyGTPar4x4[uiMode][uiIndex] /*: g_nsstHyGTPar2x2[uiMode][uiIndex])*/;
#endif
  const int   cof = 1 << (shl + 9);
  const int  kMax = (Int)(uiSize * uiSize);
  const int  kLog = g_aucLog2[kMax];
  const int  iMax = kMax >> 1;

  CHECK (uiIndex >= 4, "Invalid NSST index");
#if ENABLE_BMS
  CHECK (/*uiSize != 2 &&*/ uiSize != 4, "Invalid NSST size");
#else
  CHECK (/*uiSize != 2 &&*/ uiSize != 4 && uiSize != 8, "Invalid NSST size");
#endif

  for (int k = 0; k < kMax; k++) src[k] <<= shl;

  for (int r = 0, q = (kLog * rnd - 1); r < rnd; r++)
  {
    for (int d = 0; d < kLog; d++, q--)
    {
      const int   s = 1 << d;
      const int * p = par + ((r * kLog + d) * iMax);
      if (q > 0)
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a - t.s * b + 512) >> 10;
          src[j + s] = (t.c * b + t.s * a + 512) >> 10;
        }
      }
      else
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a - t.s * b + cof) >> (10 + shl);
          src[j + s] = (t.c * b + t.s * a + cof) >> (10 + shl);
        }
      }
    }
  }
}

Void TrQuant::InvNsstNxN( Int* src, const UInt uiMode, const UInt uiIndex, const UInt uiSize )
{
  const int   rnd = uiSize >> 1;
  const int   shl = 5;
#if ENABLE_BMS
  const int * par = /*((uiSize > 2) ?*/ g_nsstHyGTPar4x4[uiMode][uiIndex] /*: g_nsstHyGTPar2x2[uiMode][uiIndex])*/;
#else
  const int * par = (uiSize > 4) ? g_nsstHyGTPar8x8[uiMode][uiIndex] : /*((uiSize > 2) ?*/ g_nsstHyGTPar4x4[uiMode][uiIndex] /*: g_nsstHyGTPar2x2[uiMode][uiIndex])*/;
#endif
  const int   cof = 1 << (shl + 9);
  const int  kMax = (Int)(uiSize * uiSize);
  const int  kLog = g_aucLog2[kMax];
  const int  iMax = kMax >> 1;

  CHECK (uiIndex >= 4, "Invalid NSST index");
#if ENABLE_BMS
  CHECK (/*uiSize != 2 &&*/ uiSize != 4, "Invalid NSST size");
#else
  CHECK (/*uiSize != 2 &&*/ uiSize != 4 && uiSize != 8, "Invalid NSST size");
#endif

  for (int k = 0; k < kMax; k++) src[k] <<= shl;

  for (int r = rnd, q = (kLog * rnd - 1); --r >= 0; )
  {
    for (int d = kLog; --d >= 0; q--)
    {
      const int   s = 1 << d;
      const int * p = par + ((r * kLog + d) * iMax);
      if (q > 0)
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a + t.s * b + 512) >> 10;
          src[j + s] = (t.c * b - t.s * a + 512) >> 10;
        }
      }
      else
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a + t.s * b + cof) >> (10 + shl);
          src[j + s] = (t.c * b - t.s * a + cof) >> (10 + shl);
        }
      }
    }
  }
}

Void TrQuant::xInvNsst( const TransformUnit &tu, const ComponentID compID )
{
  const CompArea& area   = tu.blocks[compID];
  const UInt width       = area.width;
  const UInt height      = area.height;
  const UInt uiNSSTIdx   = tu.cu->nsstIdx;

  if( uiNSSTIdx && !tu.transformSkip[compID] && width >= 4 && height >= 4 && ( width & 3 ) == 0 && ( height & 3 ) == 0 )
  {
#if HEVC_USE_MDCS
    const UInt uiScanIdx    = TU::getCoefScanIdx( tu, compID );
#else
    const UInt uiScanIdx    = SCAN_DIAG;
#endif
#if ENABLE_BMS
    const UInt * scan       = g_scanOrder[SCAN_GROUPED_4x4][uiScanIdx][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )]; // TODO: verify with >=8x8 TUs and uiScanIdx != SCAN_DIAG
#else
    const bool whge3        = width >= 8 && height >= 8;
    const bool whgt3        = width >  8 && height >  8;
    const UInt * scan       = ( ( tu.cs->pcv->rectCUs && whge3 ) || whgt3 ) ? g_auiCoefTopLeftDiagScan8x8[gp_sizeIdxInfo->idxFrom( width )] : g_scanOrder[SCAN_GROUPED_4x4][uiScanIdx][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )];
#endif
    UInt uiIntraMode        = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

#if JEM_TOOLS
    if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
    {
      uiIntraMode = PLANAR_IDX;
    }

#endif
    CHECK( uiIntraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

    if( uiNSSTIdx < ( uiIntraMode <= DC_IDX ? 3 : 4 ) )
    {
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
      CodingStatistics::IncrementStatisticTool(CodingStatisticsClassType{ STATS__TOOL_NSST, tu.Y().width, tu.Y().height, compID });
#endif

#if ENABLE_BMS
      const Int iSbSize     = 4;
      const Int iSubGrpXMax = 1;
      const Int iSubGrpYMax = 1;
      const UChar * permut  = g_nsstHyGTPermut4x4[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1];
#else
      const Int iLog2SbSize = whge3 ? 3 : 2;
      const Int iSbSize     = whge3 ? 8 : 4;
      const Int iSubGrpXMax = Clip3( 1, 8, ( Int ) width  ) >> iLog2SbSize;
      const Int iSubGrpYMax = Clip3( 1, 8, ( Int ) height ) >> iLog2SbSize;
      const UChar * permut  = ( iSbSize == 4 ) ? g_nsstHyGTPermut4x4[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1] : g_nsstHyGTPermut8x8[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1];
#endif
      TCoeff * NSST_MATRIX  = m_tempMatrix;
      TCoeff * piNsstTemp;
      TCoeff * piCoeffTemp;

      for( Int iSubGroupX = 0; iSubGroupX < iSubGrpXMax; iSubGroupX++ )
      {
        for( Int iSubGroupY = 0; iSubGroupY < iSubGrpYMax; iSubGroupY++ )
        {
          const Int iOffsetX = iSbSize * iSubGroupX;
          const Int iOffsetY = iSbSize * iSubGroupY * width;
          Int y;
          piNsstTemp  = NSST_MATRIX; // inverse spectral rearrangement
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize * iSbSize; y++ )
          {
            piNsstTemp[permut[y]] = piCoeffTemp[scan[y]];
          }

          InvNsstNxN( NSST_MATRIX, g_NsstLut[uiIntraMode], uiNSSTIdx - 1, iSbSize );

          piNsstTemp  = NSST_MATRIX; // inverse Hyper-Givens transform
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize; y++ )
          {
            if( uiIntraMode > DIA_IDX )
            {
#if !ENABLE_BMS
              if( iSbSize == 4 )
              {
#endif
                piCoeffTemp[0] = piNsstTemp[ 0];  piCoeffTemp[1] = piNsstTemp[ 4];
                piCoeffTemp[2] = piNsstTemp[ 8];  piCoeffTemp[3] = piNsstTemp[12];
#if !ENABLE_BMS
              }
              else // ( iSbSize == 8 )
              {
                piCoeffTemp[0] = piNsstTemp[ 0];  piCoeffTemp[1] = piNsstTemp[ 8];
                piCoeffTemp[2] = piNsstTemp[16];  piCoeffTemp[3] = piNsstTemp[24];
                piCoeffTemp[4] = piNsstTemp[32];  piCoeffTemp[5] = piNsstTemp[40];
                piCoeffTemp[6] = piNsstTemp[48];  piCoeffTemp[7] = piNsstTemp[56];
              }
#endif
              piNsstTemp++;
            }
            else
            {
              ::memcpy( piCoeffTemp, piNsstTemp, iSbSize * sizeof( TCoeff ) );
              piNsstTemp += iSbSize;
            }
            piCoeffTemp += width;
          }
        }
      } // iSubGroupX
    }
  }
}

Void TrQuant::xFwdNsst( const TransformUnit &tu, const ComponentID compID )
{
  const CompArea& area   = tu.blocks[compID];
  const UInt width       = area.width;
  const UInt height      = area.height;
  const UInt uiNSSTIdx   = tu.cu->nsstIdx;

  if( uiNSSTIdx && !tu.transformSkip[compID] && width >= 4 && height >= 4 && ( width & 3 ) == 0 && ( height & 3 ) == 0 )
  {
#if HEVC_USE_MDCS
    const UInt uiScanIdx    = TU::getCoefScanIdx( tu, compID );
#else
    const UInt uiScanIdx    = SCAN_DIAG;
#endif
#if ENABLE_BMS
    const UInt * scan       = g_scanOrder[SCAN_GROUPED_4x4][uiScanIdx][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )]; // TODO: verify with >=8x8 TUs and uiScanIdx != SCAN_DIAG
#else
    const bool whge3        = width >= 8 && height >= 8;
    const bool whgt3        = width >  8 && height >  8;
    const UInt * scan       = ( ( tu.cs->pcv->rectCUs && whge3 ) || whgt3 ) ? g_auiCoefTopLeftDiagScan8x8[gp_sizeIdxInfo->idxFrom( width )] : g_scanOrder[SCAN_GROUPED_4x4][uiScanIdx][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )];
#endif
    UInt uiIntraMode        = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

#if JEM_TOOLS
    if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
    {
      uiIntraMode = PLANAR_IDX;
    }

#endif
    CHECK( uiIntraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

    if( uiNSSTIdx < ( uiIntraMode <= DC_IDX ? 3 : 4 ) )
    {
#if ENABLE_BMS
      const Int iSbSize     = 4;
      const Int iSubGrpXMax = 1;
      const Int iSubGrpYMax = 1;
      const UChar * permut  = g_nsstHyGTPermut4x4[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1];
#else
      const Int iLog2SbSize = whge3 ? 3 : 2;
      const Int iSbSize     = whge3 ? 8 : 4;
      const Int iSubGrpXMax = Clip3( 1, 8, ( Int ) width  ) >> iLog2SbSize;
      const Int iSubGrpYMax = Clip3( 1, 8, ( Int ) height ) >> iLog2SbSize;
      const UChar * permut  = ( iSbSize == 4 ) ? g_nsstHyGTPermut4x4[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1] : g_nsstHyGTPermut8x8[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1];
#endif
      TCoeff * NSST_MATRIX  = m_tempMatrix;
      TCoeff * piNsstTemp;
      TCoeff * piCoeffTemp;

      for( Int iSubGroupX = 0; iSubGroupX < iSubGrpXMax; iSubGroupX++ )
      {
        for( Int iSubGroupY = 0; iSubGroupY < iSubGrpYMax; iSubGroupY++ )
        {
          const Int iOffsetX = iSbSize * iSubGroupX;
          const Int iOffsetY = iSbSize * iSubGroupY * width;
          Int y;
          piNsstTemp  = NSST_MATRIX; // forward Hyper-Givens transform
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize; y++ )
          {
            if( uiIntraMode > DIA_IDX )
            {
#if !ENABLE_BMS
              if( iSbSize == 4 )
              {
#endif
                piNsstTemp[ 0] = piCoeffTemp[0];  piNsstTemp[ 4] = piCoeffTemp[1];
                piNsstTemp[ 8] = piCoeffTemp[2];  piNsstTemp[12] = piCoeffTemp[3];
#if !ENABLE_BMS
              }
              else // ( iSbSize == 8 )
              {
                piNsstTemp[ 0] = piCoeffTemp[0];  piNsstTemp[ 8] = piCoeffTemp[1];
                piNsstTemp[16] = piCoeffTemp[2];  piNsstTemp[24] = piCoeffTemp[3];
                piNsstTemp[32] = piCoeffTemp[4];  piNsstTemp[40] = piCoeffTemp[5];
                piNsstTemp[48] = piCoeffTemp[6];  piNsstTemp[56] = piCoeffTemp[7];
              }
#endif
              piNsstTemp++;
            }
            else
            {
              ::memcpy( piNsstTemp, piCoeffTemp, iSbSize * sizeof( TCoeff ) );
              piNsstTemp += iSbSize;
            }
            piCoeffTemp += width;
          }

          FwdNsstNxN( NSST_MATRIX, g_NsstLut[uiIntraMode], uiNSSTIdx - 1, iSbSize );

          piNsstTemp  = NSST_MATRIX; // forward spectral rearrangement
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize * iSbSize; y++ )
          {
            piCoeffTemp[scan[y]] = piNsstTemp[permut[y]];
          }
        }
      } // iSubGroupX
    }
  }
}
#endif // JEM_TOOLS


Void TrQuant::invTransformNxN( TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQP )
{
  const CompArea &area    = tu.blocks[compID];
  const UInt uiWidth      = area.width;
  const UInt uiHeight     = area.height;

#if ENABLE_BMS
  CHECK( uiWidth > tu.cs->sps->getMaxTrSize() || uiHeight > tu.cs->sps->getMaxTrSize(), "Maximal allowed transformation size exceeded!" );

#endif
  if (tu.cu->transQuantBypass)
  {
    // where should this logic go?
    const Bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
    const CCoeffBuf pCoeff    = tu.getCoeffs(compID);

    for (UInt y = 0, coefficientIndex = 0; y < uiHeight; y++)
    {
      for (UInt x = 0; x < uiWidth; x++, coefficientIndex++)
      {
        pResi.at(x, y) = rotateResidual ? pCoeff.at(pCoeff.width - x - 1, pCoeff.height - y - 1) : pCoeff.at(x, y);
      }
    }
  }
  else
  {
    CoeffBuf tempCoeff = CoeffBuf( m_plTempCoeff, area );
    xDeQuant( tu, tempCoeff, compID, cQP );

    DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

#if JEM_TOOLS
    if( tu.cs->sps->getSpsNext().getUseNSST() )
    {
      xInvNsst( tu, compID );
    }
#endif

    if( tu.transformSkip[compID] )
    {
      xITransformSkip( tempCoeff, pResi, tu, compID );
    }
    else
    {
      xIT( tu, compID, tempCoeff, pResi );
    }
  }

  //DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
  invRdpcmNxN(tu, compID, pResi);
}

Void TrQuant::invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual)
{
  const CompArea &area    = tu.blocks[compID];

  if (CU::isRDPCMEnabled(*tu.cu) && ((tu.transformSkip[compID] != 0) || tu.cu->transQuantBypass))
  {
    const UInt uiWidth  = area.width;
    const UInt uiHeight = area.height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if (tu.cu->predMode == MODE_INTRA)
    {
      const ChannelType chType = toChannelType(compID);
      const UInt uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), chType), chType);

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(tu.rdpcm[compID]);
    }

    const TCoeff pelMin = (TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax = (TCoeff) std::numeric_limits<Pel>::max();

    if (rdpcmMode == RDPCM_VER)
    {
      for (UInt uiX = 0; uiX < uiWidth; uiX++)
      {
        TCoeff accumulator = pcResidual.at(uiX, 0); // 32-bit accumulator

        for (UInt uiY = 1; uiY < uiHeight; uiY++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)
    {
      for (UInt uiY = 0; uiY < uiHeight; uiY++)
      {
        TCoeff accumulator = pcResidual.at(0, uiY);

        for (UInt uiX = 1; uiX < uiWidth; uiX++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

void TrQuant::xT( const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int iWidth, const int iHeight )
{
  const unsigned maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned channelBitDepth = tu.cs->sps->getBitDepth( toChannelType( compID ) );
#if HEVC_USE_4x4_DSTVII
  const bool     useDST          = TU::useDST( tu, compID );
#endif
#if JEM_TOOLS
  const unsigned ucMode          = getEmtMode ( tu, compID );
  const unsigned ucTrIdx         = getEmtTrIdx( tu, compID );

  if( ucTrIdx != DCT2_HEVC )
  {
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
    CodingStatistics::IncrementStatisticTool(CodingStatisticsClassType{ STATS__TOOL_EMT, UInt(iWidth), UInt(iHeight), compID });
#endif

#if INTRA67_3MPM
#if HEVC_USE_4x4_DSTVII
    xTrMxN_EMT(channelBitDepth, resi.buf, resi.stride, dstCoeff.buf, iWidth, iHeight, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx
#else
    xTrMxN_EMT(channelBitDepth, resi.buf, resi.stride, dstCoeff.buf, iWidth, iHeight, maxLog2TrDynamicRange, ucMode, ucTrIdx
#endif
#else
#if HEVC_USE_4x4_DSTVII
    xTrMxN_EMT( channelBitDepth, resi.buf, resi.stride, dstCoeff.buf, iWidth, iHeight, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx, m_use65IntraModes
#else
    xTrMxN_EMT( channelBitDepth, resi.buf, resi.stride, dstCoeff.buf, iWidth, iHeight, maxLog2TrDynamicRange, ucMode, ucTrIdx, m_use65IntraModes
#endif
#endif
      , m_rectTUs
      );
  }
  else
#endif
  {
#if HEVC_USE_4x4_DSTVII
    m_fTr     ( channelBitDepth, resi.buf, resi.stride, dstCoeff.buf, iWidth, iHeight, useDST, maxLog2TrDynamicRange );
#else
    m_fTr     ( channelBitDepth, resi.buf, resi.stride, dstCoeff.buf, iWidth, iHeight, maxLog2TrDynamicRange );
#endif
  }
}

/** Wrapper function between HM interface and core NxN inverse transform (2D)
 */
void TrQuant::xIT( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual )
{
  const unsigned maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned channelBitDepth = tu.cs->sps->getBitDepth( toChannelType( compID ) );
#if HEVC_USE_4x4_DSTVII
  const bool     useDST          = TU::useDST( tu, compID );
#endif

#if JEM_TOOLS
  const unsigned ucMode          = getEmtMode ( tu, compID );
  const unsigned ucTrIdx         = getEmtTrIdx( tu, compID );

  if( ucTrIdx != DCT2_HEVC )
  {
    Int iSkipWidth = 0, iSkipHeight = 0;

    if( m_rectTUs )
    {
      iSkipWidth  = ( pCoeff.width  > JVET_C0024_ZERO_OUT_TH ? pCoeff.width  - JVET_C0024_ZERO_OUT_TH : 0 );
      iSkipHeight = ( pCoeff.height > JVET_C0024_ZERO_OUT_TH ? pCoeff.height - JVET_C0024_ZERO_OUT_TH : 0 );
    }
    else if( ( ( ucMode == INTER_MODE_IDX || pCoeff.width == 64 ) && ucTrIdx != DCT2_EMT && pCoeff.width >= JVET_C0024_ZERO_OUT_TH ) || ( ucTrIdx == DCT2_EMT && pCoeff.width == 64 ) )
    {
      iSkipWidth  = pCoeff.width  >> 1;
      iSkipHeight = pCoeff.height >> 1;
    }
#if INTRA67_3MPM
#if HEVC_USE_4x4_DSTVII
    xITrMxN_EMT(channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, iSkipWidth, iSkipHeight, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx );
#else
    xITrMxN_EMT(channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, iSkipWidth, iSkipHeight, maxLog2TrDynamicRange, ucMode, ucTrIdx );
#endif
#else
#if HEVC_USE_4x4_DSTVII
    xITrMxN_EMT( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, iSkipWidth, iSkipHeight, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx, m_use65IntraModes );
#else
    xITrMxN_EMT( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, iSkipWidth, iSkipHeight, maxLog2TrDynamicRange, ucMode, ucTrIdx, m_use65IntraModes );
#endif
#endif
  }
  else
#endif
  {
#if HEVC_USE_4x4_DSTVII
    m_fITr     ( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height,                          useDST, maxLog2TrDynamicRange );
#else
    m_fITr     ( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height,                          maxLog2TrDynamicRange );
#endif
  }
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
Void TrQuant::xITransformSkip(const CCoeffBuf     &pCoeff,
                                    PelBuf        &pResidual,
                              const TransformUnit &tu,
                              const ComponentID   &compID)
{
  const CompArea &area      = tu.blocks[compID];
  const Int width           = area.width;
  const Int height          = area.height;
  const Int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const Int channelBitDepth = tu.cs->sps->getBitDepth(toChannelType(compID));

  Int iTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  if( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<Int>( 0, iTransformShift );
  }

  Int iWHScale = 1;
#if HM_QTBT_AS_IN_JEM_QUANT
  if( TU::needsBlockSizeTrafoScale( area ) )
  {
    iTransformShift += ADJ_QUANT_SHIFT;
    iWHScale = 181;
  }
#endif

  const Bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

  if( iTransformShift >= 0 )
  {
    const TCoeff offset = iTransformShift == 0 ? 0 : ( 1 << ( iTransformShift - 1 ) );

    for( UInt y = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) ) * iWHScale + offset ) >> iTransformShift );
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;

    for( UInt y = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) )  * iWHScale << iTransformShift );
      }
    }
  }
}

Void TrQuant::xQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  m_quant->quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
}

#if JEM_TOOLS
UChar TrQuant::getEmtTrIdx(TransformUnit tu, const ComponentID compID)
{
  UChar ucTrIdx = DCT2_HEVC;

  if( compID == COMPONENT_Y )
  {
    if( CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseIntraEMT() )
    {
      ucTrIdx = tu.cu->emtFlag ? tu.emtIdx : DCT2_EMT;
    }
    if( !CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseInterEMT() )
    {
      ucTrIdx = tu.cu->emtFlag ? tu.emtIdx : DCT2_EMT;
    }
  }
  else
  {
    if( CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseIntraEMT() )
    {
      ucTrIdx = DCT2_EMT;
    }
    if( !CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseInterEMT() )
    {
      ucTrIdx = DCT2_EMT;
    }
  }

  return ucTrIdx;
}

UChar TrQuant::getEmtMode( TransformUnit tu, const ComponentID compID )
{
  UChar ucMode = 0;

  if( isLuma( compID ) )
  {
    if( CU::isIntra( *tu.cu ) )
    {
      CodingStructure &cs      = *tu.cs;
      const PredictionUnit &pu = *cs.getPU( tu.blocks[compID].pos(), toChannelType( compID ) );
      const UInt uiChFinalMode = PU::getFinalIntraMode( pu, toChannelType( compID ) );
#if INTRA67_3MPM
      ucMode = uiChFinalMode;
#else
#if JEM_TOOLS
      ucMode                   = m_use65IntraModes ? uiChFinalMode : g_intraMode65to33AngMapping[uiChFinalMode];
#else
      ucMode                   = g_intraMode65to33AngMapping[uiChFinalMode];
#endif
#endif
    }
    else
    {
      ucMode = INTER_MODE_IDX;
    }
  }

  return ucMode;
}
#endif

Void TrQuant::transformNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx)
{
        CodingStructure &cs = *tu.cs;
  const SPS &sps            = *cs.sps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;

  const CPelBuf resiBuf     = cs.getResiBuf(rect);
        CoeffBuf rpcCoeff   = tu.getCoeffs(compID);

  RDPCMMode rdpcmMode = RDPCM_OFF;
  rdpcmNxN(tu, compID, cQP, uiAbsSum, rdpcmMode);

  if (rdpcmMode == RDPCM_OFF)
  {
    uiAbsSum = 0;

    // transform and quantize
    if (CU::isLosslessCoded(*tu.cu))
    {
      const Bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

      for( UInt y = 0; y < uiHeight; y++ )
      {
        for( UInt x = 0; x < uiWidth; x++ )
        {
          const Pel currentSample = resiBuf.at( x, y );

          if( rotateResidual )
          {
            rpcCoeff.at( uiWidth - x - 1, uiHeight - y - 1 ) = currentSample;
          }
          else
          {
            rpcCoeff.at( x, y ) = currentSample;
          }

          uiAbsSum += TCoeff( abs( currentSample ) );
        }
      }
    }
    else
    {
      CHECK( sps.getMaxTrSize() < uiWidth, "Unsupported transformation size" );

      CoeffBuf tempCoeff( m_plTempCoeff, rect );

      DTRACE_PEL_BUF( D_RESIDUALS, resiBuf, tu, tu.cu->predMode, compID );

      if( tu.transformSkip[compID] )
      {
        xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
      }
      else
      {
        xT( tu, compID, resiBuf, tempCoeff, uiWidth, uiHeight );
      }

#if JEM_TOOLS
      if( sps.getSpsNext().getUseNSST() )
      {
        xFwdNsst( tu, compID );
      }
#endif
      DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

      xQuant( tu, compID, tempCoeff, uiAbsSum, cQP, ctx );

      DTRACE_COEFF_BUF( D_TCOEFF, tu.getCoeffs( compID ), tu, tu.cu->predMode, compID );
    }
  }

#if ENABLE_BMS
  // set coded block flag (CBF)
  TU::setCbfAtDepth (tu, compID, tu.depth, uiAbsSum > 0);
#else
  TU::setCbf( tu, compID, uiAbsSum > 0 );
#endif
}

Void TrQuant::applyForwardRDPCM(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const RDPCMMode &mode)
{
  const Bool bLossless      = tu.cu->transQuantBypass;
  const UInt uiWidth        = tu.blocks[compID].width;
  const UInt uiHeight       = tu.blocks[compID].height;
  const Bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
  const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

  const CPelBuf pcResidual  = tu.cs->getResiBuf(tu.blocks[compID]);
  const CoeffBuf pcCoeff    = tu.getCoeffs(compID);

  UInt uiX = 0;
  UInt uiY = 0;

  UInt &majorAxis            = (mode == RDPCM_VER) ? uiX      : uiY;
  UInt &minorAxis            = (mode == RDPCM_VER) ? uiY      : uiX;
  const UInt  majorAxisLimit = (mode == RDPCM_VER) ? uiWidth  : uiHeight;
  const UInt  minorAxisLimit = (mode == RDPCM_VER) ? uiHeight : uiWidth;

  const Bool bUseHalfRoundingPoint = (mode != RDPCM_OFF);

  uiAbsSum = 0;

  for (majorAxis = 0; majorAxis < majorAxisLimit; majorAxis++)
  {
    TCoeff accumulatorValue = 0; // 32-bit accumulator

    for (minorAxis = 0; minorAxis < minorAxisLimit; minorAxis++)
    {
      const UInt sampleIndex        = (uiY * uiWidth) + uiX;
      const UInt coefficientIndex   = (rotateResidual ? (uiSizeMinus1-sampleIndex) : sampleIndex);
      const Pel  currentSample      = pcResidual.at(uiX, uiY);
      const TCoeff encoderSideDelta = TCoeff(currentSample) - accumulatorValue;

      Pel reconstructedDelta;

      if (bLossless)
      {
        pcCoeff.buf[coefficientIndex] = encoderSideDelta;
        reconstructedDelta            = (Pel) encoderSideDelta;
      }
      else
      {
        m_quant->transformSkipQuantOneSample(tu, compID, encoderSideDelta, pcCoeff.buf[coefficientIndex],   coefficientIndex, cQP, bUseHalfRoundingPoint);
        m_quant->invTrSkipDeQuantOneSample  (tu, compID, pcCoeff.buf[coefficientIndex], reconstructedDelta, coefficientIndex, cQP);
      }

      uiAbsSum += abs(pcCoeff.buf[coefficientIndex]);

      if (mode != RDPCM_OFF)
      {
        accumulatorValue += reconstructedDelta;
      }
    }
  }
}

Void TrQuant::rdpcmNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, RDPCMMode &rdpcmMode)
{
  if (!CU::isRDPCMEnabled(*tu.cu) || (!tu.transformSkip[compID] && !tu.cu->transQuantBypass))
  {
    rdpcmMode = RDPCM_OFF;
  }
  else if (CU::isIntra(*tu.cu))
  {
    const ChannelType chType = toChannelType(compID);
    const UInt uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(tu.blocks[compID].pos(), chType), chType);

    if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
    {
      rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);
    }
    else
    {
      rdpcmMode = RDPCM_OFF;
    }
  }
  else // not intra, need to select the best mode
  {
    const CompArea &area = tu.blocks[compID];
    const UInt uiWidth   = area.width;
    const UInt uiHeight  = area.height;

    RDPCMMode bestMode = NUMBER_OF_RDPCM_MODES;
    TCoeff    bestAbsSum = std::numeric_limits<TCoeff>::max();
    TCoeff    bestCoefficients[MAX_TU_SIZE * MAX_TU_SIZE];

    for (UInt modeIndex = 0; modeIndex < NUMBER_OF_RDPCM_MODES; modeIndex++)
    {
      const RDPCMMode mode = RDPCMMode(modeIndex);

      TCoeff currAbsSum = 0;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);

      if (currAbsSum < bestAbsSum)
      {
        bestMode = mode;
        bestAbsSum = currAbsSum;

        if (mode != RDPCM_OFF)
        {
          CoeffBuf(bestCoefficients, uiWidth, uiHeight).copyFrom(tu.getCoeffs(compID));
        }
      }
    }

    rdpcmMode = bestMode;
    uiAbsSum = bestAbsSum;

    if (rdpcmMode != RDPCM_OFF) //the TU is re-transformed and quantized if DPCM_OFF is returned, so there is no need to preserve it here
    {
      tu.getCoeffs(compID).copyFrom(CoeffBuf(bestCoefficients, uiWidth, uiHeight));
    }
  }

  tu.rdpcm[compID] = rdpcmMode;
}

Void TrQuant::xTransformSkip(const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt width          = rect.width;
  const UInt height         = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const Int channelBitDepth = sps.getBitDepth(chType);
  const Int maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  Int iTransformShift       = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if( sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<Int>( 0, iTransformShift );
  }

  Int iWHScale = 1;
#if HM_QTBT_AS_IN_JEM_QUANT
  if( TU::needsBlockSizeTrafoScale( rect ) )
  {
    iTransformShift -= ADJ_DEQUANT_SHIFT;
    iWHScale = 181;
  }
#endif

  const Bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );
  const UInt uiSizeMinus1 = ( width * height ) - 1;

  if( iTransformShift >= 0 )
  {
    for( UInt y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale ) << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << ( iTransformShift - 1 );

    for( UInt y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale + offset ) >> iTransformShift;
      }
    }
  }
}

//! \}
