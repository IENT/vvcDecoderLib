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

/** \file     InterSearch.h
    \brief    inter search class (header)
 */

#ifndef __INTERSEARCH__
#define __INTERSEARCH__

// Include files
#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/MotionInfo.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/RdCost.h"
#if JEM_TOOLS
#include "CommonLib/BilateralFilter.h"
#endif

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const UInt MAX_NUM_REF_LIST_ADAPT_SR = 2;
static const UInt MAX_IDX_ADAPT_SR          = 33;
static const UInt NUM_MV_PREDICTORS         = 3;

class EncModeCtrl;

/// encoder search class
class InterSearch : public InterPrediction, CrossComponentPrediction
{
private:
  EncModeCtrl     *m_modeCtrl;

  PelStorage      m_tmpPredStorage              [NUM_REF_PIC_LIST_01];
  PelStorage      m_tmpStorageLCU;
  PelStorage      m_tmpAffiStorage;
#if JVET_K0367_AFFINE_FIX_POINT
  Pel*            m_tmpAffiError;
  int*            m_tmpAffiDeri[2];
#else
  Int*            m_tmpAffiError;
  Double*         m_tmpAffiDeri[2];
#endif

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure **m_pSaveCS;

  ClpRng          m_lumaClpRng;

#if JEM_TOOLS
  PelStorage      m_obmcOrgMod;
#endif

protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;

#if JEM_TOOLS
  BilateralFilter*
                  m_bilateralFilter;

#endif
  // ME parameters
  Int             m_iSearchRange;
  Int             m_bipredSearchRange; // Search range for bi-prediction
  MESearchMethod  m_motionEstimationSearchMethod;
  Int             m_aaiAdaptSR                  [MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;
  DistParam       m_cDistParam;

  // Misc.
  Pel            *m_pTempPel;

  // AMVP cost computation
  UInt            m_auiMVPIdxCost               [AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1]; //th array bounds

  Mv              m_integerMv2Nx2N              [NUM_REF_PIC_LIST_01][MAX_NUM_REF];

  Bool            m_isInitialized;

#if JEM_TOOLS
  MotionInfo      m_SubPuFrucBuf                [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
#endif

public:
  InterSearch();
  virtual ~InterSearch();

  Void init                         ( EncCfg*        pcEncCfg,
                                      TrQuant*       pcTrQuant,
#if JEM_TOOLS
                                      BilateralFilter*
                                                     bilateralFilter,
#endif
                                      Int            iSearchRange,
                                      Int            bipredSearchRange,
                                      MESearchMethod motionEstimationSearchMethod,
                                      const UInt     maxCUWidth,
                                      const UInt     maxCUHeight,
                                      const UInt     maxTotalCUDepth,
                                      RdCost*        pcRdCost,
                                      CABACWriter*   CABACEstimator,
                                      CtxCache*      ctxCache
                                    );

  Void destroy                      ();

  Void setTempBuffers               (CodingStructure ****pSlitCS, CodingStructure ****pFullCS, CodingStructure **pSaveCS );

#if ENABLE_SPLIT_PARALLELISM
  Void copyState                    ( const InterSearch& other );
#endif

#if JVET_K0367_AFFINE_FIX_POINT
  void ( *m_HorizontalSobelFilter ) (Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height);

  void( *m_VerticalSobelFilter   ) (Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height);

  void( *m_EqualCoeffComputer    ) (Pel *pResidue, int residueStride, int **ppDerivate, int derivateBufStride, Int64( *pEqualCoeff )[7], int width, int height, bool b6Param);

  static void xHorizontalSobelFilter (Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height );

  static void xVerticalSobelFilter(Pel *const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height );

  static void xEqualCoeffComputer (Pel *pResidue, int residueStride, int **ppDerivate, int derivateBufStride, Int64( *pEqualCoeff )[7], int width, int height, bool b6Param);

#if ENABLE_SIMD_OPT_AFFINE_ME
#ifdef TARGET_SIMD_X86
  void initAffineMEFunctionX86();
  void _initAffineMEFunctionX86();
#endif
#endif
#endif

protected:

  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion  xPatternRefinement    ( const CPelBuf* pcPatternKey, Mv baseRefMv, Int iFrac, Mv& rcMvFrac, Bool bAllowUseOfHadamard );

   typedef struct
   {
     Int left;
     Int right;
     Int top;
     Int bottom;
   }SearchRange;

  typedef struct
  {
    SearchRange searchRange;
    const CPelBuf* pcPatternKey;
    const Pel*  piRefY;
    Int         iRefStride;
    Int         iBestX;
    Int         iBestY;
    UInt        uiBestRound;
    UInt        uiBestDistance;
    Distortion  uiBestSad;
    UChar       ucPointNr;
    Int         subShiftMode;
#if JEM_TOOLS
    unsigned    imvShift;
#endif
  } IntTZSearchStruct;

  // sub-functions for ME
  inline Void xTZSearchHelp         ( IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance );
  inline Void xTZ2PointSearch       ( IntTZSearchStruct& rcStruct );
  inline Void xTZ8PointSquareSearch ( IntTZSearchStruct& rcStruct, const Int iStartX, const Int iStartY, const Int iDist );
  inline Void xTZ8PointDiamondSearch( IntTZSearchStruct& rcStruct, const Int iStartX, const Int iStartY, const Int iDist, const Bool bCheckCornersAtDist1 );

  Distortion xGetInterPredictionError( PredictionUnit& pu, PelUnitBuf& origBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X );

public:
  /// encoder estimation - inter prediction (non-skip)

  void setModeCtrl( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl;}

  Void predInterSearch(CodingUnit& cu, Partitioner& partitioner );

  /// set ME search range
  Void setAdaptiveSearchRange       ( Int iDir, Int iRefIdx, Int iSearchRange) { CHECK(iDir >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdx>=Int(MAX_IDX_ADAPT_SR), "Invalid index"); m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange; }


protected:

  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  Void xEstimateMvPredAMVP        ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    RefPicList            eRefPicList,
                                    Int                   iRefIdx,
                                    Mv&                   rcMvPred,
                                    AMVPInfo&             amvpInfo,
                                    Bool                  bFilled = false,
                                    Distortion*           puiDistBiP = NULL
                                  );

  Void xCheckBestMVP              ( RefPicList  eRefPicList,
                                    Mv          cMv,
                                    Mv&         rcMvPred,
                                    Int&        riMVPIdx,
                                    AMVPInfo&   amvpInfo,
                                    UInt&       ruiBits,
                                    Distortion& ruiCost
#if JEM_TOOLS
                                    ,
                                    const UChar  imv
#endif
                                  );

  Distortion xGetTemplateCost     ( const PredictionUnit& pu,
                                    PelUnitBuf&           origBuf,
                                    PelUnitBuf&           predBuf,
                                    Mv                    cMvCand,
                                    Int                   iMVPIdx,
                                    Int                   iMVPNum,
                                    RefPicList            eRefPicList,
                                    Int                   iRefIdx
                                  );


  Void xCopyAMVPInfo              ( AMVPInfo*   pSrc, AMVPInfo* pDst );
  UInt xGetMvpIdxBits             ( Int iIdx, Int iNum );
  Void xGetBlkBits                ( PartSize  eCUMode, Bool bPSlice, Int iPartIdx,  UInt uiLastMode, UInt uiBlkBit[3]);

  Void xMergeEstimation           ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    Int                   iPartIdx,
                                    UInt&                 uiMergeIndex,
                                    Distortion&           ruiCost,
                                    MergeCtx &            mergeCtx
                                  );

#if JEM_TOOLS
  Void xFRUCMrgEstimation         ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    Distortion&           ruiMinCost,
                                    UChar&                ruhFRUCMode,
                                    MergeCtx&             mrgCtx
                                  );
#endif


  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------

  Void xMotionEstimation          ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    RefPicList            eRefPicList,
                                    Mv&                   rcMvPred,
                                    Int                   iRefIdxPred,
                                    Mv&                   rcMv,
                                    Int&                  riMVPIdx,
                                    UInt&                 ruiBits,
                                    Distortion&           ruiCost,
                                    const AMVPInfo&       amvpInfo,
                                    Bool                  bBi = false
                                  );

  Void xTZSearch                  ( const PredictionUnit& pu,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred,
                                    const Bool            bExtendedSettings,
                                    const Bool            bFastSettings = false
                                  );

  Void xTZSearchSelective         ( const PredictionUnit& pu,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  Void xSetSearchRange            ( const PredictionUnit& pu,
                                    const Mv&             cMvPred,
                                    const Int             iSrchRng,
                                    SearchRange&          sr
                                  );

  Void xPatternSearchFast         ( const PredictionUnit& pu,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  Void xPatternSearch             ( IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD
                                  );

  Void xPatternSearchIntRefine    ( PredictionUnit&     pu,
                                    IntTZSearchStruct&  cStruct,
                                    Mv&                 rcMv,
                                    Mv&                 rcMvPred,
                                    Int&                riMVPIdx,
                                    UInt&               ruiBits,
                                    Distortion&         ruiCost,
                                    const AMVPInfo&     amvpInfo,
                                    Double              fWeight
                                  );

  Void xPatternSearchFracDIF      ( const PredictionUnit& pu,
                                    RefPicList            eRefPicList,
                                    Int                   iRefIdx,
                                    IntTZSearchStruct&    cStruct,
                                    const Mv&             rcMvInt,
                                    Mv&                   rcMvHalf,
                                    Mv&                   rcMvQter,
                                    Distortion&           ruiCost
                                  );

#if JEM_TOOLS
  Void xPredAffineInterSearch     ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    Int                   puIdx,
                                    UInt&                 lastMode,
                                    Distortion&           affineCost,
#if JVET_K0220_ENC_CTRL
                                    Mv                    hevcMv[2][33]
#else
                                    Mv                    hevcMv[2][33],
                                    Bool                  bFastSkipBi
#endif
#if JVET_K0185_AFFINE_6PARA_ENC
                                  , Mv                    mvAffine4Para[2][33][3]
                                  , int                   refIdx4Para[2]
#endif
                                  );

  Void xAffineMotionEstimation    ( PredictionUnit& pu,
                                    PelUnitBuf&     origBuf,
                                    RefPicList      eRefPicList,
                                    Mv              acMvPred[3],
                                    Int             iRefIdxPred,
                                    Mv              acMv[3],
                                    UInt&           ruiBits,
                                    Distortion&     ruiCost,
                                    Bool            bBi = false
                                  );

  Void xEstimateAffineAMVP        ( PredictionUnit&  pu,
                                    AffineAMVPInfo&  affineAMVPInfo,
                                    PelUnitBuf&      origBuf,
                                    RefPicList       eRefPicList,
                                    Int              iRefIdx,
                                    Mv               acMvPred[3],
                                    Distortion*      puiDistBiP
                                  );

  Distortion xGetAffineTemplateCost( PredictionUnit& pu, PelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], Int iMVPIdx, Int iMVPNum, RefPicList eRefPicList, Int iRefIdx );

  Void xCopyAffineAMVPInfo        ( AffineAMVPInfo& src, AffineAMVPInfo& dst );
  Void xCheckBestAffineMVP        ( PredictionUnit &pu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList, Mv acMv[3], Mv acMvPred[3], Int& riMVPIdx, UInt& ruiBits, Distortion& ruiCost );
#endif
  Void xExtDIFUpSamplingH         ( CPelBuf* pcPattern );
  Void xExtDIFUpSamplingQ         ( CPelBuf* pcPatternKey, Mv halfPelRef );

  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------

  Void  setWpScalingDistParam     ( Int iRefIdx, RefPicList eRefPicListCur, Slice *slice );


public:

  Void encodeResAndCalcRdInterCU  (CodingStructure &cs, Partitioner &partitioner, const Bool &skipResidual);
  Void xEncodeInterResidualQT     (CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID);
  Void xEstimateInterResidualQT   (CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist = NULL);
  UInt64 xGetSymbolFracBitsInter  (CodingStructure &cs, Partitioner &partitioner);

};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
