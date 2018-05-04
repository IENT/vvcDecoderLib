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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include "CommonLib/AdaptiveLoopFilter.h"

#if JEM_TOOLS

#include "CABACWriter.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

typedef AreaBuf<UInt>  UIntBuf;

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))

/// estimation part of adaptive loop filter class
class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
private:
  static const Int m_aiSymmetricArray5x5[25];     ///< scan index for 5x5 filter
  static const Int m_aiSymmetricArray7x7[49];     ///< scan index for 7x7 filter
  static const Int m_aiSymmetricArray9x9[81];     ///< scan index for 9x9 filter

  static const Int m_aiSymmetricArray9x7[63];     ///< scan index for 9x7 filter

  static const Int  m_aiTapPos5x5_In9x9Sym[8];
  static const Int  m_aiTapPos7x7_In9x9Sym[14];
  static const Int  m_aiTapPos9x9_In9x9Sym[21];

  static const Int* m_iTapPosTabIn9x9Sym[m_NO_TEST_FILT];

  double    error_tab[m_NO_VAR_BINS];
  double    error_comb_tab[m_NO_VAR_BINS];
  int       indexList[m_NO_VAR_BINS];
  int       available[m_NO_VAR_BINS];
  int       noRemaining;

  double   *y_temp1D, **E_temp2D, pixAcc_temp0D;

  double ***E_temp3D;
  double  **y_temp2D;
  double   *pixAcc_temp1D;
  int     **FilterCoeffQuantTemp;

  int       m_is9x9Alloc;
  double  **y_temp9x9;

  int usePrevFiltBest[m_NO_VAR_BINS];

public:
           EncAdaptiveLoopFilter ();
  virtual ~EncAdaptiveLoopFilter (){}

  Void create                      ( const Int iPicWidth, Int iPicHeight, const ChromaFormat chromaFormatIDC, const Int uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth , const Int nInputBitDepth, const Int nInternalBitDepth, const Int numberOfCTUs );
  Void init                        ( CodingStructure& cs, CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder );
  Void destroy                     ();

  Void ALFProcess                  ( CodingStructure& cs, ALFParam* pcAlfParam, Double dLambdaLuma, Double dLambdaChroma);
  Void xstoreInBlockMatrixForChroma(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, Int tap, Int chroma_idc);

private:
  // init / uninit internal variables
  Void xInitParam                  ();
  Void xUninitParam                ();

  Void xEncALFLuma                 ( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate,
                                         UInt64& ruiMinDist, Double& rdMinCost, const Slice* pSlice);

  Void xEncALFChroma               ( UInt64 uiLumaRate,  const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiDist, UInt64& ruiBits, const Slice* pSlice );

  Void xCheckReUseFilterSet        ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& dstUnitBuf, Double& rdMinCost, ALFParam& filterSet, Int filterSetIdx );

  //adaptation
  Void  xSetInitialMask            ( const CPelBuf& recBufExt );
  Void  xInitFixedFilters();
  Void  xCheckCUAdaptation         ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost );
  Void  xSetCUAlfCtrlFlags         ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiDist, UInt uiAlfCtrlDepth, ALFParam *pAlfParam );
  Void  xSetCUAlfCtrlFlag          ( CodingStructure& cs, const UnitArea alfCtrlArea, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiDist, ALFParam *pAlfParam);
#if COM16_C806_ALF_TEMPPRED_NUM
  Bool xFilteringLumaChroma(CodingStructure& cs, ALFParam *pAlfParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int uiIndex, const Slice* pSlice);
  Void xcopyFilterCoeff(int filtNo, int **filterCoeff);
#endif

  //filter type decision
  Void xFilterTypeDecision         ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, const Slice*  slice);


  //derive filter coefficents
  Void   xFindFilterCoeffsLuma     (const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam* pcAlfParam
#endif
    );
  Void   xStoreInBlockMatrix(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType);

  Void calcMatrixE(int *ELocal, const Pel *recBufExt, const int *p_pattern, int i, int j, int flV, int fl, int transpose, int recStrideExt);
  Int  xFilterPixel(const Pel *ImgDec, Int* varIndBeforeMapping, Int **filterCoeffSym, Int *pattern, Int i, Int j, Int fl, Int Stride, AlfFilterType filtNo);
  Void xfindBestFilterPredictor(
#if JVET_C0038_NO_PREV_FILTERS
    ALFParam& pcAlfParam
#endif
    );
#if JVET_C0038_NO_PREV_FILTERS
  Double xTestFixedFilterFast      ( Double ***A, Double **b, Double *pixAcc, Double *filterCoeffSym, Double *filterCoeffDefault, Int varInd);
  Double xTestFixedFilter          ( const Pel *imgY_rec, const Pel *imgY_org, const Pel *imgY_append, Int usePrevFilt[], Int noVarBins, Int orgStride, Int recStride, Int filtType);
  Void   xPreFilterFr              ( Int** imgY_preFilter, const Pel* imgY_rec, const Pel * imgY_org, const Pel* imgY_append, Int usePrevFilt[], Int Stride, Int filtType);
  Void   xfindBestFilterPredictor  ( Double ***E_temp, Double**y_temp, Double *pixAcc_temp, Int filtType, const Pel* ImgOrg, const Pel* ImgDec, Int orgStride, Int recStride, Int* usePrevFiltBest, Int sqrFiltLength, Int iFixedFilters
    );
#endif

#if FORCE0
  Void xcollectStatCodeFilterCoeffForce0(Int **pDiffQFilterCoeffIntPP, Int filtType, Int sqrFiltLength, Int filters_per_group,
    int bitsVarBin[]);
  Void xdecideCoeffForce0(Int codedVarBins[m_NO_VAR_BINS], Double *distForce0, Double errorForce0CoeffTab[m_NO_VAR_BINS][2],
    Int bitsVarBin[m_NO_VAR_BINS], Double lambda, Int filters_per_fr);

  Double xCalcDistForce0(
    Int           filters_per_fr,
    AlfFilterType filtType,
    Int           sqrFiltLenght,
    Double        errorTabForce0Coeff[m_NO_VAR_BINS][2],
    Double        lambda,
    Int           codedVarBins[m_NO_VAR_BINS]);

#endif


  Double xCalcFilterCoeffsGalf(Double     ***EGlobalSeq,
                               Double      **yGlobalSeq,
                               Double       *pixAccGlobalSeq,
                               Int           interval[m_NO_VAR_BINS],
                               Int           filters_per_fr,
                               AlfFilterType filtType,
                               Double    errorTabForce0Coeff[m_NO_VAR_BINS][2]);

  Double xCalcFilterCoeffs         ( Double     ***EGlobalSeq,
                                     Double      **yGlobalSeq,
                                     Double       *pixAccGlobalSeq,
                                     Int           intervalBest[m_NO_VAR_BINS][2],
                                     Int           filters_per_fr,
                                     AlfFilterType filtType,
                                     Int     **filterCoeffSeq,
                                     Int     **filterCoeffQuantSeq,
                                     Double    errorTabForce0Coeff[m_NO_VAR_BINS][2] );
  Void xcalcPredFilterCoeff        ( AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam* alfParam
#endif
    ); //TODO rename

  Void   xCheckFilterMergingAlf(ALFParam& alfParam
#if JVET_C0038_NO_PREV_FILTERS
    , const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf
#endif
  );
  Void   xCheckFilterMergingGalf(ALFParam& alfParam
#if JVET_C0038_NO_PREV_FILTERS
    , const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf
#endif
  );
  Void xMergeFiltersGreedyGalf( Double ***EGlobalSeq, Double **yGlobalSeq, Double *pixAccGlobalSeq, int intervalBest[m_NO_VAR_BINS][m_NO_VAR_BINS], int sqrFiltLength);
  Double xMergeFiltersGreedy  (Double ***EGlobalSeq, Double **yGlobalSeq, Double *pixAccGlobalSeq, int intervalBest[m_NO_VAR_BINS][2], int sqrFiltLength, int noIntervals);
  Void   xReDesignFilterCoeff      ( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf
#if COM16_C806_ALF_TEMPPRED_NUM
   , ALFParam* alfParam
#endif
    , const ClpRng& clpRng);

  Double xSolveAndQuant( Double *filterCoeff, Int *filterCoeffQuant, Double **E, Double *y, int sqrFiltLength,const Int *weights, int bit_depth, Bool bChroma = false );
  Void   roundFiltCoeff            ( int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int factor);

  Void xDeriveGlobalEyFromLgrTapFilter(Double **E0, Double *y0, Double **E1, Double *y1, const Int *pattern0, const Int *pattern1);
  Void xDeriveLocalEyFromLgrTapFilter(Double *y0, Double *y1, const Int *pattern0, const Int *pattern1);
  Void add_A_galf                  (Double **Amerged, Double ***A, Int interval[], Int filtNo, Int size);
  Void add_b_galf                  (Double  *bmerged, Double  **b, Int interval[], Int filtNo, Int size);
  Void add_b                       ( Double  *bmerged, Double  **b, Int start, Int stop, Int size);
  Void add_A                       ( Double **Amerged, Double ***A, Int start, Int stop, Int size);

  Int  gnsSolveByChol              ( Double **LHS, double *rhs, double *x, int noEq);
  Void gnsBacksubstitution         ( Double R[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], Double z[m_MAX_SQR_FILT_LENGTH], Int R_size, Double A[m_MAX_SQR_FILT_LENGTH]);
  Void gnsTransposeBacksubstitution( Double U[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], Double rhs[], Double x[], Int order);
  Int  gnsCholeskyDec              ( Double **inpMatr, double outMatr[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], Int noEq);


  //Filtering
  Void xFilterFrame_en             (PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, ALFParam& alfParam,  const ClpRng& clpRng);
  Void xFilterFrame_enGalf         (PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam *alfParam, Bool updateFilterCoef
#endif
    , const ClpRng& clpRng);
  Void xFilterFrame_enAlf          (PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam *alfParam, Bool updateFilterCoef
#endif
    , const ClpRng& clpRng);
  Void xFilteringFrameChroma       ( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf );

  //Chroma
  Void xCalcCorrelationFunc        ( const CPelBuf& rcOrgBuf, const CPelBuf& rcCmpBuf, Int iTap );
  Int  xGauss                      ( Double **a, Int N);
  Void xClearFilterCoefInt         ( Int* qh, Int N);
  Void xQuantFilterCoefChroma      ( Double* h, Int* qh, Int tap, int bit_depth );
  Void xFilterCoefQuickSort        ( Double *coef_data, Int *coef_num, Int upper, Int lower );

  //R-D
  Void   xCalcRDCostLuma           ( const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recBuf, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  Void   xCalcRDCostChroma         ( const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recBuf, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  Void   xCalcRDCost               ( const UInt64 uiDist, ALFParam* pAlfParam, UInt64& ruiRate,  Double& rdCost );
  UInt64 xCalcSSD                  ( const CPelBuf& refBuf, const CPelBuf& cmpBuf );
  UInt64 xCalcSSD                  ( const CPelUnitBuf& OrgBuf, const CPelUnitBuf& CmpBuf, const ComponentID compId);
  Double xCalcErrorForGivenWeights ( Double** E, Double* y, Double* w, Int size );
  Double calculateErrorAbs         ( Double** A, Double* b, Double y,  Int size );

  //Coding
  Int xcodeFilterCoeff             ( int **pDiffQFilterCoeffIntPP, int filters_per_group, AlfFilterType filtType);
#if FORCE0
  Int xcodeFilterCoeffForce0       ( Int **pDiffQFilterCoeffIntPP, Int filters_per_group, AlfFilterType filtType, Int codedVarBins[]);
  Int xCalcBitsForce0              ( Int **pDiffQFilterCoeffIntPP, Int filters_per_group, AlfFilterType filtType, Int *codedVarBins);
#endif
  Int xCheckFilterPredictionMode   ( Int **pDiffQFilterCoeffIntPP, Int filters_per_group, AlfFilterType filtType, Int& predMode );
  Int lengthGolomb                 ( int coeffVal, int k);
  Int lengthFilterCoeffs           ( int sqrFiltLength, int filters_per_group, const int pDepthInt[], int **FilterCoeff, int kMinTab[] );

private:
  SliceType  m_eSliceType;
  Int        m_iPicNalReferenceIdc;

  PelStorage m_bestPelBuf;
  PelStorage m_tempPelBuf;

  bool       m_updateMatrix;
  UIntBuf    m_maskBestBuf;
#if JVET_C0038_NO_PREV_FILTERS
  Bool bFindBestFixedFilter;
#endif
  UIntBuf    m_maskBuf;

  ALFParam*  m_pcBestAlfParam;
  ALFParam*  m_pcTempAlfParam;

  Double**** m_EGlobalSym;
  Double***  m_yGlobalSym;
  Double*    m_pixAcc;

  Double**   m_E_temp;
  Double*    m_y_temp;

  Double***  m_E_merged;
  Double**   m_y_merged;
  Double*    m_pixAcc_merged;

  Pel**      m_varImg;
  Int        m_varIndTab[m_NO_VAR_BINS];

  Double*    m_filterCoeff;
  Double*    m_pdDoubleAlfCoeff;
  Int*       m_filterCoeffQuantMod;
  Int*       m_filterCoeffQuant;
  Int**      m_diffFilterCoeffQuant;
  Int**      m_FilterCoeffQuantTemp;
  Int**      m_filterCoeffSymQuant;
  Double     m_filterCoeffDefault[21];
#if JVET_C0038_NO_PREV_FILTERS
  Int**      m_imgY_preFilter;
  Double     m_filterCoeffPrev[m_NO_VAR_BINS*JVET_C0038_NO_PREV_FILTERS][21];
#endif
  Double**   m_ppdAlfCorr;

  //R-D
  CABACWriter*    m_CABACEstimator;
  CABACDataStore* m_CABACDataStore;
  Slice*          m_pSlice;
  Double          m_dLambdaLuma;
  Double          m_dLambdaChroma;

};
#endif

#endif
