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

/** \file     EncSlice.h
    \brief    slice encoder class (header)
*/

#ifndef __ENCSLICE__
#define __ENCSLICE__

// Include files
#include "EncCu.h"
#include "WeightPredAnalysis.h"
#include "RateCtrl.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

//! \ingroup EncoderLib
//! \{

class EncLib;
class EncGOP;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// slice encoder class
class EncSlice
  : public WeightPredAnalysis
{
private:
  // encoder configuration
  EncCfg*                 m_pcCfg;                              ///< encoder configuration class

  EncLib*                 m_pcLib;

  // pictures
  PicList*                m_pcListPic;                          ///< list of pictures

  // processing units
  EncGOP*                 m_pcGOPEncoder;                       ///< GOP encoder
  EncCu*                  m_pcCuEncoder;                        ///< CU encoder

  // encoder search
  InterSearch*            m_pcInterSearch;                      ///< encoder search class

  // coding tools
  CABACWriter*            m_CABACWriter;
  TrQuant*                m_pcTrQuant;                          ///< transform & quantization

  // RD optimization
  RdCost*                 m_pcRdCost;                           ///< RD cost computation
  CABACWriter*            m_CABACEstimator;
  uint64_t                  m_uiPicTotalBits;                     ///< total bits for the picture
  uint64_t                  m_uiPicDist;                          ///< total distortion for the picture
  std::vector<Double>     m_vdRdPicLambda;                      ///< array of lambda candidates
  std::vector<Double>     m_vdRdPicQp;                          ///< array of picture QP candidates (double-type for lambda)
  std::vector<Int>        m_viRdPicQp;                          ///< array of picture QP candidates (Int-type)
  RateCtrl*               m_pcRateCtrl;                         ///< Rate control manager
  UInt                    m_uiSliceSegmentIdx;
#if HEVC_DEPENDENT_SLICES
  Ctx                     m_lastSliceSegmentEndContextState;    ///< context storage for state at the end of the previous slice-segment (used for dependent slices only).
#endif
#if HEVC_TILES_WPP
  Ctx                     m_entropyCodingSyncContextState;      ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row
#endif
#if JEM_TOOLS
  CABACDataStore*         m_CABACDataStore;
#endif
  SliceType               m_encCABACTableIdx;
#if SHARP_LUMA_DELTA_QP
  Int                     m_gopID;
#endif

#if SHARP_LUMA_DELTA_QP
public:
  Int getGopId()        const { return m_gopID; }
  Double  calculateLambda( const Slice* slice, const Int GOPid, const Int depth, const Double refQP, const Double dQP, Int &iQP );
#if WCG_EXT
  void    setUpLambda( Slice* slice, const Double dLambda, Int iQP );
#endif
  
private:
#endif
#if !WCG_EXT
  void    setUpLambda( Slice* slice, const Double dLambda, Int iQP );
#endif
#if HEVC_TILES_WPP
  void    calculateBoundingCtuTsAddrForSlice( UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, bool &haveReachedTileBoundary, Picture* pcPic, const Int sliceMode, const Int sliceArgument );
#else
  void    calculateBoundingCtuTsAddrForSlice( UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, Picture* pcPic, const Int sliceMode, const Int sliceArgument );
#endif


public:
  EncSlice();
  virtual ~EncSlice();

  void    create              ( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth );
  void    destroy             ();
  void    init                ( EncLib* pcEncLib, const SPS& sps );

  /// preparation of slice encoding (reference marking, QP and lambda)
  void    initEncSlice        ( Picture*  pcPic, const Int pocLast, const Int pocCurr,
                                const Int iGOPid,   Slice*& rpcSlice, const bool isField );
  void    resetQP             ( Picture* pic, Int sliceQP, Double lambda );

  // compress and encode slice
  void    precompressSlice    ( Picture* pcPic                                     );      ///< precompress slice for multi-loop slice-level QP opt.
  void    compressSlice       ( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP );      ///< analysis stage of slice
  void    calCostSliceI       ( Picture* pcPic );

  void    encodeSlice         ( Picture* pcPic, OutputBitstream* pcSubstreams, UInt &numBinsCoded );
#if ENABLE_WPP_PARALLELISM
  static
#endif
  void    encodeCtus          ( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP, UInt startCtuTsAddr, UInt boundingCtuTsAddr, EncLib* pcEncLib );


  // misc. functions
  void    setSearchRange      ( Slice* pcSlice  );                                  ///< set ME range adaptively

  EncCu*  getCUEncoder        ()                    { return m_pcCuEncoder; }                        ///< CU encoder
  void    xDetermineStartAndBoundingCtuTsAddr  ( UInt& startCtuTsAddr, UInt& boundingCtuTsAddr, Picture* pcPic );
  UInt    getSliceSegmentIdx  ()                    { return m_uiSliceSegmentIdx;       }
  void    setSliceSegmentIdx  (UInt i)              { m_uiSliceSegmentIdx = i;          }

  SliceType getEncCABACTableIdx() const             { return m_encCABACTableIdx;        }

private:
  Double  xGetQPValueAccordingToLambda ( Double lambda );
};

//! \}

#endif // __ENCSLICE__
