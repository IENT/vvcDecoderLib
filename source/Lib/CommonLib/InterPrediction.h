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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__


// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"

// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if JEM_TOOLS
#define BIO_TEMP_BUFFER_SIZE ( MAX_CU_SIZE ) * ( MAX_CU_SIZE )
#endif

class InterPrediction : public WeightPrediction
{
private:
#if JEM_TOOLS
  static const int  m_LICShift      = 5;
  static const int  m_LICRegShift   = 7;
  static const int  m_LICShiftDiff  = 12;
  int               m_LICMultApprox[64];
#endif

#if JEM_TOOLS
  int64_t m_piDotProduct1[BIO_TEMP_BUFFER_SIZE];
  int64_t m_piDotProduct2[BIO_TEMP_BUFFER_SIZE];
  int64_t m_piDotProduct3[BIO_TEMP_BUFFER_SIZE];
  int64_t m_piDotProduct5[BIO_TEMP_BUFFER_SIZE];
  int64_t m_piDotProduct6[BIO_TEMP_BUFFER_SIZE];
#endif

protected:
  InterpolationFilter  m_if;

  Pel*                 m_acYuvPred            [NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][MAX_NUM_COMPONENT];


  ChromaFormat         m_currChromaFormat;

  ComponentID          m_maxCompIDToPred;      ///< tells the predictor to only process the components up to (inklusive) this one - useful to skip chroma components during RD-search

  RdCost*              m_pcRdCost;

  Int                  m_iRefListIdx;
  
#if JEM_TOOLS
  Pel*                 m_pGradX0;
  Pel*                 m_pGradY0;
  Pel*                 m_pGradX1;
  Pel*                 m_pGradY1;

  PelStorage           m_tmpObmcBuf;

  Pel*                 m_cYuvPredTempDMVR[MAX_NUM_COMPONENT];

  UInt                 m_uiaBIOShift[64];
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel*          m_cacheModel;
#endif

  // motion compensation functions
#define BIO_FILTER_LENGTH                 6
#define BIO_FILTER_LENGTH_MINUS_1         (BIO_FILTER_LENGTH-1)
#define BIO_FILTER_HALF_LENGTH_MINUS_1    ((BIO_FILTER_LENGTH>>1)-1)

  void          xGradFilterX    ( const Pel* piRefY, Int iRefStride, Pel*  piDstY, Int iDstStride, Int iWidth, Int iHeight, Int iMVyFrac, Int iMVxFrac, const Int bitDepth );
  void          xGradFilterY    ( const Pel* piRefY, Int iRefStride, Pel*  piDstY, Int iDstStride, Int iWidth, Int iHeight, Int iMVyFrac, Int iMVxFrac, const Int bitDepth );
  inline void   gradFilter2DVer ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMv, const Int iShift );
  inline void   gradFilter2DHor ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );
  inline void   fracFilter2DHor ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );
  inline void   fracFilter2DVer ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMv, const Int iShift );
  inline void   gradFilter1DHor ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );
  inline void   gradFilter1DVer ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );

  inline int64_t  divide64        ( int64_t numer, int64_t denom);
  inline void   calcBlkGradient ( Int sx, Int sy, int64_t *arraysGx2, int64_t *arraysGxGy, int64_t *arraysGxdI, int64_t *arraysGy2, int64_t *arraysGydI, int64_t &sGx2, int64_t &sGy2, int64_t &sGxGy, int64_t &sGxdI, int64_t &sGydI, Int iWidth, Int iHeight);

  Pel  optical_flow_averaging   ( int64_t s1, int64_t s2, int64_t s3, int64_t s5, int64_t s6,
                                  Pel pGradX0, Pel pGradX1, Pel pGradY0, Pel pGradY1, Pel pSrcY0Temp, Pel pSrcY1Temp,
                                  const int shiftNum, const int offset, const int64_t limit, const int64_t denom_min_1, const int64_t denom_min_2, const ClpRng& clpRng );
  void applyBiOptFlow           ( const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, const Int &iRefIdx0, const Int &iRefIdx1, PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths);
#endif

#if JEM_TOOLS
  void xPredInterUni            ( const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi, const bool& bBIOApplied = false, const bool& bDMVRApplied = false );
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred, bool obmc = false );
#else
  void xPredInterUni            ( const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi );
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred );
#endif
  void xPredInterBlk            ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
#if JEM_TOOLS
                                  , const bool& bBIOApplied = false, const bool& bDMVRApplied = false, const Int& nFRUCMode = FRUC_MERGE_OFF, const bool& doLic = true
#endif
                                 );
  
#if JEM_TOOLS
  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, const bool& bBIOApplied = false );
  void xGetLICParams            ( const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, int& shift, int& scale, int& offset );
  void xLocalIlluComp           ( const PredictionUnit& pu, const ComponentID compID, const Picture& refPic, const Mv& mv, const bool biPred, PelBuf& dstBuf );
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bBIOApplied );
#else
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs );
#endif
#if !JEM_TOOLS && JVET_K_AFFINE
  void xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng );
#endif

  static bool xCheckIdenticalMotion( const PredictionUnit& pu );

#if JEM_TOOLS
  void xSubPuMC                 ( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X );
  void xSubblockOBMC            ( const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, Int iDir, bool bOBMCSimp );
  void xSubtractOBMC            ( PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, Int iDir, bool bOBMCSimp );
#endif
#if !JEM_TOOLS && JVET_K0346
  void xSubPuMC(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
#endif
#if JEM_TOOLS
  void xSubBlockMotionCompensation( PredictionUnit &pu, PelUnitBuf &pcYuvPred );
#endif

  void destroy();

#if JEM_TOOLS
  MotionInfo      m_SubPuMiBuf   [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
  MotionInfo      m_SubPuExtMiBuf[( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];

  std::list<MvField> m_listMVFieldCand[2];
  RefPicList m_bilatBestRefPicList;
  Pel*   m_acYuvPredFrucTemplate[2][MAX_NUM_COMPONENT];   //0: top, 1: left
  bool   m_bFrucTemplateAvailabe[2];

  bool xFrucFindBlkMv           (PredictionUnit& pu, const MergeCtx& mergeCtx );
  bool xFrucRefineSubBlkMv      (PredictionUnit& pu, const MergeCtx& mergeCtx, bool bTM);

  void xFrucCollectBlkStartMv   (PredictionUnit& pu, const MergeCtx& mergeCtx, RefPicList eTargetRefList = REF_PIC_LIST_0, Int nTargetRefIdx = -1, AMVPInfo* pInfo = NULL);
  void xFrucCollectSubBlkStartMv(PredictionUnit& pu, const MergeCtx& mergeCtx, RefPicList eRefPicList , const MvField& rMvStart , Int nSubBlkWidth , Int nSubBlkHeight, Position basePuPos);
#if DISTORTION_TYPE_BUGFIX
  Distortion xFrucFindBestMvFromList(MvField *pBestMvField, RefPicList &rBestRefPicList, PredictionUnit &pu,
                                     const MvField &rMvStart, Int nBlkWidth, Int nBlkHeight, bool bTM, bool bMvCost);
  Distortion xFrucRefineMv(MvField *pBestMvField, RefPicList eCurRefPicList, Distortion uiMinCost, Int nSearchMethod,
                           PredictionUnit &pu, const MvField &rMvStart, Int nBlkWidth, Int nBlkHeight, bool bTM,
                           bool bMvCostZero = false);
#else
  UInt xFrucFindBestMvFromList  (MvField* pBestMvField, RefPicList& rBestRefPicList, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, bool bTM, bool bMvCost);
  UInt xFrucRefineMv(MvField *pBestMvField, RefPicList eCurRefPicList, UInt uiMinCost, Int nSearchMethod,
                     PredictionUnit &pu, const MvField &rMvStart, Int nBlkWidth, Int nBlkHeight, bool bTM,
                     bool bMvCostZero = false);
#endif
#if DISTORTION_TYPE_BUGFIX
  template<Int SearchPattern>
  Distortion xFrucRefineMvSearch(MvField *pBestMvField, RefPicList eCurRefPicList, PredictionUnit &pu,
                                 const MvField &rMvStart, Int nBlkWidth, Int nBlkHeight, Distortion uiMinDist, bool bTM,
                                 Int nSearchStepShift, UInt uiMaxSearchRounds = MAX_UINT, bool bMvCostZero = false);
#else
  template<Int SearchPattern>
  UInt xFrucRefineMvSearch      (MvField* pBestMvField, RefPicList eCurRefPicList, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, UInt uiMinDist, bool bTM, Int nSearchStepShift, UInt uiMaxSearchRounds = MAX_UINT, bool bMvCostZero = false);
#endif

#if DISTORTION_TYPE_BUGFIX
  Distortion xFrucGetMvCost(const Mv &rMvStart, const Mv &rMvCur, Int nSearchRange, Int nWeighting, UInt precShift);
  Distortion xFrucGetBilaMatchCost(PredictionUnit &pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList,
                                   const MvField &rCurMvField, MvField &rPairMVField, Distortion uiMVCost);
  Distortion xFrucGetTempMatchCost(PredictionUnit &pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList,
                                   const MvField &rCurMvField, Distortion uiMVCost);
#else
  UInt xFrucGetMvCost(const Mv &rMvStart, const Mv &rMvCur, Int nSearchRange, Int nWeighting, UInt precShift);
  UInt xFrucGetBilaMatchCost    (PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, MvField& rPairMVField, UInt uiMVCost );
  UInt xFrucGetTempMatchCost    (PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, UInt uiMVCost );
#endif
  void xFrucUpdateTemplate      (PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField );


  void xFrucInsertMv2StartList  (const MvField & rMvField, std::list<MvField> & rList,bool setHighPrec);
  bool xFrucIsInList            (const MvField & rMvField, std::list<MvField> & rList);

  bool xFrucGetCurBlkTemplate   (PredictionUnit& pu, Int nCurBlkWidth , Int nCurBlkHeight);
  bool xFrucIsTopTempAvailable  (PredictionUnit& pu);
  bool xFrucIsLeftTempAvailable (PredictionUnit& pu);
  Int  xFrucGetSubBlkSize       (PredictionUnit& pu, Int nBlkWidth, Int nBlkHeight);

#if DISTORTION_TYPE_BUGFIX
  void xBIPMVRefine(PredictionUnit &pu, RefPicList eRefPicList, Int iWidth, Int iHeight, const CPelUnitBuf &pcYuvOrg,
                    UInt uiMaxSearchRounds, UInt nSearchStepShift, Distortion &uiMinCost, bool fullPel = true);
  Distortion xDirectMCCost(Int iBitDepth, Pel *pRef, UInt uiRefStride, const Pel *pOrg, UInt uiOrgStride, Int iWidth,
                           Int iHeight);
#else
  void xBIPMVRefine(PredictionUnit &pu, RefPicList eRefPicList, Int iWidth, Int iHeight, const CPelUnitBuf &pcYuvOrg,
                    UInt uiMaxSearchRounds, UInt nSearchStepShift, UInt &uiMinCost, bool fullPel = true);
  UInt xDirectMCCost            (Int iBitDepth, Pel* pRef, UInt uiRefStride, const Pel* pOrg, UInt uiOrgStride, Int iWidth, Int iHeight);
#endif
  void xPredInterLines          (const PredictionUnit& pu, const Picture* refPic, Mv &mv, PelUnitBuf &dstPic, const bool &bi, const ClpRng& clpRng );
  void xFillPredBlckAndBorder   (const PredictionUnit& pu, RefPicList eRefPicList, Int iWidth, Int iHeight, PelBuf &cTmpY );
  void xProcessDMVR             (      PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bBIOApplied);
#endif

#if !JEM_TOOLS && JVET_K0346
  MotionInfo      m_SubPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif

public:
  InterPrediction();
  virtual ~InterPrediction();

  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC);

  // inter
  void    motionCompensation  (PredictionUnit &pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
  void    motionCompensation  (PredictionUnit &pu, const RefPicList &eRefPicList = REF_PIC_LIST_X);
  void    motionCompensation  (CodingUnit &cu,     const RefPicList &eRefPicList = REF_PIC_LIST_X);

#if JEM_TOOLS
  void    subBlockOBMC        (CodingUnit      &cu);
  void    subBlockOBMC        (PredictionUnit  &pu, PelUnitBuf *pDst = nullptr, bool bOBMC4ME = false);

  bool    deriveFRUCMV        (PredictionUnit &pu);
  bool    frucFindBlkMv4Pred  (PredictionUnit& pu, RefPicList eTargetRefPicList, const Int nTargetRefIdx, AMVPInfo* pInfo = NULL);
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void    cacheAssign( CacheModel *cache );
#endif

};

//! \}

#endif // __INTERPREDICTION__
