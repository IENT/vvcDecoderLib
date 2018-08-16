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

#if JVET_K0367_AFFINE_FIX_POINT
#include "CommonLib/AffineGradientSearch.h"
#endif
#if JVET_K0076_CPR
#include "IbcHashMap.h"
#include <unordered_map>
#include <vector>
#endif
//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const uint32_t MAX_NUM_REF_LIST_ADAPT_SR = 2;
static const uint32_t MAX_IDX_ADAPT_SR          = 33;
static const uint32_t NUM_MV_PREDICTORS         = 3;
#if JVET_K0076_CPR
struct BlkRecord
{
  std::unordered_map<Mv, Distortion> bvRecord;
};
#endif
class EncModeCtrl;

/// encoder search class
#if JVET_K0367_AFFINE_FIX_POINT
class InterSearch : public InterPrediction, CrossComponentPrediction, AffineGradientSearch
#else
class InterSearch : public InterPrediction, CrossComponentPrediction
#endif
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
  int*            m_tmpAffiError;
  double*         m_tmpAffiDeri[2];
#endif

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure **m_pSaveCS;

  ClpRng          m_lumaClpRng;

#if JEM_TOOLS
  PelStorage      m_obmcOrgMod;
#endif
#if JVET_K0076_CPR
  std::unordered_map< Position, std::unordered_map< Size, BlkRecord> > m_ctuRecord;
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
  int             m_iSearchRange;
  int             m_bipredSearchRange; // Search range for bi-prediction
  MESearchMethod  m_motionEstimationSearchMethod;
  int             m_aaiAdaptSR                  [MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;
  DistParam       m_cDistParam;

  // Misc.
  Pel            *m_pTempPel;

  // AMVP cost computation
  uint32_t            m_auiMVPIdxCost               [AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1]; //th array bounds

  Mv              m_integerMv2Nx2N              [NUM_REF_PIC_LIST_01][MAX_NUM_REF];

  bool            m_isInitialized;

#if JEM_TOOLS
  MotionInfo      m_SubPuFrucBuf                [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
#endif
#if JVET_K0076_CPR
  unsigned int    m_uiNumBVs, m_uiNumBV16s;
  Mv              m_acBVs[IBC_NUM_CANDIDATES];
#endif
public:
  InterSearch();
  virtual ~InterSearch();

  void init                         ( EncCfg*        pcEncCfg,
                                      TrQuant*       pcTrQuant,
#if JEM_TOOLS
                                      BilateralFilter*
                                                     bilateralFilter,
#endif
                                      int            iSearchRange,
                                      int            bipredSearchRange,
                                      MESearchMethod motionEstimationSearchMethod,
                                      const uint32_t     maxCUWidth,
                                      const uint32_t     maxCUHeight,
                                      const uint32_t     maxTotalCUDepth,
                                      RdCost*        pcRdCost,
                                      CABACWriter*   CABACEstimator,
                                      CtxCache*      ctxCache
                                    );

  void destroy                      ();

  void setTempBuffers               (CodingStructure ****pSlitCS, CodingStructure ****pFullCS, CodingStructure **pSaveCS );
#if JVET_K0076_CPR
  void resetCtuRecord               ()             { m_ctuRecord.clear(); }
#endif
#if ENABLE_SPLIT_PARALLELISM
  void copyState                    ( const InterSearch& other );
#endif

protected:

  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion  xPatternRefinement    ( const CPelBuf* pcPatternKey, Mv baseRefMv, int iFrac, Mv& rcMvFrac, bool bAllowUseOfHadamard );

   typedef struct
   {
     int left;
     int right;
     int top;
     int bottom;
   }SearchRange;

  typedef struct
  {
    SearchRange searchRange;
    const CPelBuf* pcPatternKey;
    const Pel*  piRefY;
    int         iRefStride;
    int         iBestX;
    int         iBestY;
    uint32_t        uiBestRound;
    uint32_t        uiBestDistance;
    Distortion  uiBestSad;
    uint8_t       ucPointNr;
    int         subShiftMode;
#if JVET_K0357_AMVR
    unsigned    imvShift;
#endif
#if JVET_K0157
    bool        inCtuSearch;
    bool        zeroMV;
#endif
  } IntTZSearchStruct;

  // sub-functions for ME
  inline void xTZSearchHelp         ( IntTZSearchStruct& rcStruct, const int iSearchX, const int iSearchY, const uint8_t ucPointNr, const uint32_t uiDistance );
  inline void xTZ2PointSearch       ( IntTZSearchStruct& rcStruct );
  inline void xTZ8PointSquareSearch ( IntTZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist );
  inline void xTZ8PointDiamondSearch( IntTZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist, const bool bCheckCornersAtDist1 );

  Distortion xGetInterPredictionError( PredictionUnit& pu, PelUnitBuf& origBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X );

public:
  /// encoder estimation - inter prediction (non-skip)

  void setModeCtrl( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl;}

  void predInterSearch(CodingUnit& cu, Partitioner& partitioner );

  /// set ME search range
  void setAdaptiveSearchRange       ( int iDir, int iRefIdx, int iSearchRange) { CHECK(iDir >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdx>=int(MAX_IDX_ADAPT_SR), "Invalid index"); m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange; }

#if JVET_K0076_CPR
  bool  predIntraBCSearch           ( CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap);
  void  xIntraPatternSearch         ( PredictionUnit& pu, IntTZSearchStruct&  cStruct, Mv& rcMv, Distortion&  ruiCost, Mv* cMvSrchRngLT, Mv* cMvSrchRngRB, Mv* pcMvPred);
  void  xSetIntraSearchRange        ( PredictionUnit& pu, Mv& cMvPred, int iRoiWidth, int iRoiHeight, const int localSearchRangeX, const int localSearchRangeY, Mv& rcMvSrchRngLT, Mv& rcMvSrchRngRB);
  void  xIntraBlockCopyEstimation   ( PredictionUnit& pu, PelUnitBuf& origBuf, Mv     *pcMvPred, Mv     &rcMv, Distortion &ruiCost, const int localSearchRangeX, const int localSearchRangeY);
  void  xIntraBCSearchMVCandUpdate  ( Distortion  uiSad, int x, int y, Distortion* uiSadBestCand, Mv* cMVCand);
  int   xIntraBCSearchMVChromaRefine( PredictionUnit& pu, int iRoiWidth, int iRoiHeight, int cuPelX, int cuPelY, Distortion* uiSadBestCand, Mv*     cMVCand);
#endif
protected:

  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  void xEstimateMvPredAMVP        ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdx,
                                    Mv&                   rcMvPred,
                                    AMVPInfo&             amvpInfo,
                                    bool                  bFilled = false,
                                    Distortion*           puiDistBiP = NULL
                                  );

  void xCheckBestMVP              ( RefPicList  eRefPicList,
                                    Mv          cMv,
                                    Mv&         rcMvPred,
                                    int&        riMVPIdx,
                                    AMVPInfo&   amvpInfo,
                                    uint32_t&       ruiBits,
                                    Distortion& ruiCost
#if JVET_K0357_AMVR
                                    ,
                                    const uint8_t  imv
#endif
                                  );

  Distortion xGetTemplateCost     ( const PredictionUnit& pu,
                                    PelUnitBuf&           origBuf,
                                    PelUnitBuf&           predBuf,
                                    Mv                    cMvCand,
                                    int                   iMVPIdx,
                                    int                   iMVPNum,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdx
                                  );


  void xCopyAMVPInfo              ( AMVPInfo*   pSrc, AMVPInfo* pDst );
  uint32_t xGetMvpIdxBits             ( int iIdx, int iNum );
  void xGetBlkBits                ( PartSize  eCUMode, bool bPSlice, int iPartIdx,  uint32_t uiLastMode, uint32_t uiBlkBit[3]);

  void xMergeEstimation           ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    int                   iPartIdx,
                                    uint32_t&                 uiMergeIndex,
                                    Distortion&           ruiCost,
                                    MergeCtx &            mergeCtx
                                  );

#if JEM_TOOLS
  void xFRUCMrgEstimation         ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    Distortion&           ruiMinCost,
                                    uint8_t&                ruhFRUCMode,
                                    MergeCtx&             mrgCtx
                                  );
#endif


  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------

  void xMotionEstimation          ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    RefPicList            eRefPicList,
                                    Mv&                   rcMvPred,
                                    int                   iRefIdxPred,
                                    Mv&                   rcMv,
                                    int&                  riMVPIdx,
                                    uint32_t&                 ruiBits,
                                    Distortion&           ruiCost,
                                    const AMVPInfo&       amvpInfo,
                                    bool                  bBi = false
                                  );

  void xTZSearch                  ( const PredictionUnit& pu,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred,
                                    const bool            bExtendedSettings,
                                    const bool            bFastSettings = false
                                  );

  void xTZSearchSelective         ( const PredictionUnit& pu,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  void xSetSearchRange            ( const PredictionUnit& pu,
                                    const Mv&             cMvPred,
                                    const int             iSrchRng,
                                    SearchRange&          sr
#if JVET_K0157
                                  , IntTZSearchStruct &  cStruct
#endif
                                  );

  void xPatternSearchFast         ( const PredictionUnit& pu,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  void xPatternSearch             ( IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD
                                  );

  void xPatternSearchIntRefine    ( PredictionUnit&     pu,
                                    IntTZSearchStruct&  cStruct,
                                    Mv&                 rcMv,
                                    Mv&                 rcMvPred,
                                    int&                riMVPIdx,
                                    uint32_t&               ruiBits,
                                    Distortion&         ruiCost,
                                    const AMVPInfo&     amvpInfo,
                                    double              fWeight
                                  );

  void xPatternSearchFracDIF      ( const PredictionUnit& pu,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdx,
                                    IntTZSearchStruct&    cStruct,
                                    const Mv&             rcMvInt,
                                    Mv&                   rcMvHalf,
                                    Mv&                   rcMvQter,
                                    Distortion&           ruiCost
                                  );

#if JEM_TOOLS || JVET_K_AFFINE
  void xPredAffineInterSearch     ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    int                   puIdx,
                                    uint32_t&                 lastMode,
                                    Distortion&           affineCost,
#if JVET_K0220_ENC_CTRL
                                    Mv                    hevcMv[2][33]
#else
                                    Mv                    hevcMv[2][33],
                                    bool                  bFastSkipBi
#endif
#if JVET_K0185_AFFINE_6PARA_ENC
                                  , Mv                    mvAffine4Para[2][33][3]
                                  , int                   refIdx4Para[2]
#endif
                                  );

  void xAffineMotionEstimation    ( PredictionUnit& pu,
                                    PelUnitBuf&     origBuf,
                                    RefPicList      eRefPicList,
                                    Mv              acMvPred[3],
                                    int             iRefIdxPred,
                                    Mv              acMv[3],
                                    uint32_t&           ruiBits,
                                    Distortion&     ruiCost,
                                    bool            bBi = false
                                  );

  void xEstimateAffineAMVP        ( PredictionUnit&  pu,
                                    AffineAMVPInfo&  affineAMVPInfo,
                                    PelUnitBuf&      origBuf,
                                    RefPicList       eRefPicList,
                                    int              iRefIdx,
                                    Mv               acMvPred[3],
                                    Distortion*      puiDistBiP
                                  );

  Distortion xGetAffineTemplateCost( PredictionUnit& pu, PelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], int iMVPIdx, int iMVPNum, RefPicList eRefPicList, int iRefIdx );

  void xCopyAffineAMVPInfo        ( AffineAMVPInfo& src, AffineAMVPInfo& dst );
  void xCheckBestAffineMVP        ( PredictionUnit &pu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList, Mv acMv[3], Mv acMvPred[3], int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost );
#endif
  void xExtDIFUpSamplingH         ( CPelBuf* pcPattern );
  void xExtDIFUpSamplingQ         ( CPelBuf* pcPatternKey, Mv halfPelRef );

  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------

  void  setWpScalingDistParam     ( int iRefIdx, RefPicList eRefPicListCur, Slice *slice );
#if JVET_K0076_CPR
private:
  void  xxIntraBlockCopyHashSearch(PredictionUnit& pu, Mv* mvPred, int numMvPred, Mv &mv, int& idxMvPred, IbcHashMap& ibcHashMap);
#endif

public:

  void encodeResAndCalcRdInterCU  (CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual);
  void xEncodeInterResidualQT     (CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID);
  void xEstimateInterResidualQT   (CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist = NULL);
  uint64_t xGetSymbolFracBitsInter  (CodingStructure &cs, Partitioner &partitioner);

};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
