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
 * \file
 * \brief Declaration of InterpolationFilter class
 */

#ifndef __INTERPOLATIONFILTER__
#define __INTERPOLATIONFILTER__

#include "CommonDef.h"
#include "CacheModel.h"

//! \ingroup CommonLib
//! \{

#define IF_INTERNAL_PREC 14 ///< Number of bits for internal precision
#define IF_FILTER_PREC    6 ///< Log2 of sum of filter taps
#define IF_INTERNAL_OFFS (1<<(IF_INTERNAL_PREC-1)) ///< Offset used internally

/**
 * \brief Interpolation filter class
 */
class InterpolationFilter
{
#if JEM_TOOLS
  static const TFilterCoeff m_lumaFilter  [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS   << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA  ]; ///< Luma filter taps
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_CHROMA]; ///< Chroma filter taps
  static const TFilterCoeff m_lumaFilterBilinear[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA_FRUC];     ///< Luma filter taps
#elif JVET_K0346
  static const TFilterCoeff m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA]; ///< Luma filter taps
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_CHROMA]; ///< Chroma filter taps
#else
  static const TFilterCoeff m_lumaFilter  [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA  ]; ///< Luma filter taps
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps
#endif
public:
  template<Bool isFirst, Bool isLast>
  static Void filterCopy( const ClpRng& clpRng, const Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height );

  template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
  static Void filter(const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff);

  template<Int N>
  Void filterHor(const ClpRng& clpRng, Pel const* src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height,               Bool isLast, TFilterCoeff const *coeff);
  template<Int N>
  Void filterVer(const ClpRng& clpRng, Pel const* src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, TFilterCoeff const *coeff);

protected:
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  static CacheModel* m_cacheModel;
#endif
public:
  InterpolationFilter();
  ~InterpolationFilter() {}

  Void( *m_filterHor[3][2][2] )( const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff );
  Void( *m_filterVer[3][2][2] )( const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff );
  Void( *m_filterCopy[2][2] )  ( const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height );

  void initInterpolationFilter( bool enable );
#ifdef TARGET_SIMD_X86
  Void initInterpolationFilterX86();
  template <X86_VEXT vext>
  Void _initInterpolationFilterX86();
#endif

#if JEM_TOOLS
  Void filterHor(const ComponentID compID, Pel const* src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac,               Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, Int nFilterIdx = 0 );
  Void filterVer(const ComponentID compID, Pel const* src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, Int nFilterIdx = 0 );
#else
  Void filterHor(const ComponentID compID, Pel const* src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac,               Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng );
  Void filterVer(const ComponentID compID, Pel const* src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng );
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void cacheAssign( CacheModel *cache ) { m_cacheModel = cache; }
#endif
};

//! \}

#endif
