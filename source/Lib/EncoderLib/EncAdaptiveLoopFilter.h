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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include "CommonLib/AdaptiveLoopFilter.h"

#if JVET_K0371_ALF
#include "CABACWriter.h"

struct AlfCovariance
{
  int numCoeff;
  double *y;
  double **E;
  double pixAcc;

  AlfCovariance() {}
  ~AlfCovariance() {}

  void create( Int size )
  {
    numCoeff = size;

    y = new double[numCoeff];
    E = new double*[numCoeff];

    for( Int i = 0; i < numCoeff; i++ )
    {
      E[i] = new double[numCoeff];
    }
  }

  void destroy()
  {
    for( int i = 0; i < numCoeff; i++ )
    {
      delete[] E[i];
      E[i] = nullptr;
    }

    delete[] E;
    E = nullptr;

    delete[] y;
    y = nullptr;
  }

  void reset()
  {
    pixAcc = 0;
    std::memset( y, 0, sizeof( *y ) * numCoeff );
    for( Int i = 0; i < numCoeff; i++ )
    {
      std::memset( E[i], 0, sizeof( *E[i] ) * numCoeff );
    }
  }

  const AlfCovariance& operator=( const AlfCovariance& src )
  {
    for( int i = 0; i < numCoeff; i++ )
    {
      std::memcpy( E[i], src.E[i], sizeof( *E[i] ) * numCoeff );
    }
    std::memcpy( y, src.y, sizeof( *y ) * numCoeff );
    pixAcc = src.pixAcc;

    return *this;
  }

  void add( const AlfCovariance& lhs, const AlfCovariance& rhs )
  {
    for( int j = 0; j < numCoeff; j++ )
    {
      for( int i = 0; i < numCoeff; i++ )
      {
        E[j][i] = lhs.E[j][i] + rhs.E[j][i];
      }
      y[j] = lhs.y[j] + rhs.y[j];
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }

  const AlfCovariance& operator+= ( const AlfCovariance& src )
  {
    for( int j = 0; j < numCoeff; j++ )
    {
      for( int i = 0; i < numCoeff; i++ )
      {
        E[j][i] += src.E[j][i];
      }
      y[j] += src.y[j];
    }
    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance& operator-= ( const AlfCovariance& src )
  {
    for( int j = 0; j < numCoeff; j++ )
    {
      for( int i = 0; i < numCoeff; i++ )
      {
        E[j][i] -= src.E[j][i];
      }
      y[j] -= src.y[j];
    }
    pixAcc -= src.pixAcc;

    return *this;
  }
};

class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
public:
  static constexpr int   m_MAX_SCAN_VAL = 11;
  static constexpr int   m_MAX_EXP_GOLOMB = 16;

private:
  AlfCovariance***       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][classIdx]
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_CHANNEL_TYPE];   // [CHANNEL][shapeIdx][classIdx]
  UChar*                 m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];

  //for RDO
  AlfSliceParam          m_alfSliceParamTemp;
  AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 1];
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
  double                 m_lambda[MAX_NUM_COMPONENT];
  const double           FracBitsScale = 1.0 / double( 1 << SCALE_BITS );

  int*                   m_filterCoeffQuant;
  int**                  m_filterCoeffSet;
  int**                  m_diffFilterCoeff;
  int                    m_kMinTab[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];

public:
  EncAdaptiveLoopFilter();
  virtual ~EncAdaptiveLoopFilter() {}

  void ALFProcess( CodingStructure& cs, const double *lambdas, AlfSliceParam& alfSliceParam );
#if JEM_TOOLS
  void initCABACEstimator( CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice );
#else
  void initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice );
#endif
  void create( const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] );
  void destroy();
  static int lengthGolomb( int coeffVal, int k );
  static int getGolombKMin( AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB] );

private:
  void   alfEncoder( CodingStructure& cs, AlfSliceParam& alfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel );

  void   copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel );
  double mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits );

  void   getFrameStats( ChannelType channel, int iShapeIdx );
  void   getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, UChar* ctbEnableFlags, const int numClasses );
  void   deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv );
  void   getBlkStats( AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area );
  void   calcCovariance( int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx );
  void   mergeClasses( AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] );

  double calculateError( AlfCovariance& cov );
  double calcErrorForCoeffs( double **E, double *y, int *coeff, const int numCoeff, const int bitDepth );
  double getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits );
  double deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2] );
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode );
  double deriveCoeffQuant( int *filterCoeffQuant, double **E, double *y, const int numCoeff, std::vector<int>& weights, const int bitDepth, const bool bChroma = false );
  double deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel, const int numClasses, const int numCoeff, double& distUnfilter );
  void   roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor );

  double getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters );
  int    lengthTruncatedUnary( int symbol, int maxSymbol );
  int    lengthUvlc( int uiCode );
  int    getNonFilterCoeffRate( AlfSliceParam& alfSliceParam );
  int    getTBlength( int uiSymbol, const int uiMaxSymbol );

  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins );
  int    getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab );
  double getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins );
  int    getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma );

  double getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel );
  double getUnfilteredDistortion( AlfCovariance* cov, const int numClasses );
  double getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff );

  // Cholesky decomposition
  int  gnsSolveByChol( double **LHS, double *rhs, double *x, int numEq );
  void gnsBacksubstitution( double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A );
  void gnsTransposeBacksubstitution( double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order );
  int  gnsCholeskyDec( double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq );

  void setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, bool val );
  void setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, UChar** ctuFlags );
  void setCtuEnableFlag( UChar** ctuFlags, ChannelType channel, UChar val );
  void copyCtuEnableFlag( UChar** ctuFlagsDst, UChar** ctuFlagsSrc, ChannelType channel );
};


#elif JEM_TOOLS

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

  void create                      ( const Int iPicWidth, Int iPicHeight, const ChromaFormat chromaFormatIDC, const Int uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth , const Int nInputBitDepth, const Int nInternalBitDepth, const Int numberOfCTUs );
  void init                        ( CodingStructure& cs, CABACDataStore* cabacDataStore, CABACEncoder* cabacEncoder );
  void destroy                     ();

  void ALFProcess                  ( CodingStructure& cs, ALFParam* pcAlfParam, Double dLambdaLuma, Double dLambdaChroma);
  void xstoreInBlockMatrixForChroma(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, Int tap, Int chroma_idc);

private:
  // init / uninit internal variables
  void xInitParam                  ();
  void xUninitParam                ();

  void xEncALFLuma                 ( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate,
                                         UInt64& ruiMinDist, Double& rdMinCost, const Slice* pSlice);

  void xEncALFChroma               ( UInt64 uiLumaRate,  const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiDist, UInt64& ruiBits, const Slice* pSlice );

  void xCheckReUseFilterSet        ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& dstUnitBuf, Double& rdMinCost, ALFParam& filterSet, Int filterSetIdx );

  //adaptation
  void  xSetInitialMask            ( const CPelBuf& recBufExt );
  void  xInitFixedFilters();
  void  xCheckCUAdaptation         ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost );
  void  xSetCUAlfCtrlFlags         ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiDist, UInt uiAlfCtrlDepth, ALFParam *pAlfParam );
  void  xSetCUAlfCtrlFlag          ( CodingStructure& cs, const UnitArea alfCtrlArea, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiDist, ALFParam *pAlfParam);
#if COM16_C806_ALF_TEMPPRED_NUM
  bool xFilteringLumaChroma(CodingStructure& cs, ALFParam *pAlfParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int uiIndex, const Slice* pSlice);
  void xcopyFilterCoeff(int filtNo, int **filterCoeff);
#endif

  //filter type decision
  void xFilterTypeDecision         ( CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, const Slice*  slice);


  //derive filter coefficents
  void   xFindFilterCoeffsLuma     (const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam* pcAlfParam
#endif
    );
  void   xStoreInBlockMatrix(const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType);

  void calcMatrixE(int *ELocal, const Pel *recBufExt, const int *p_pattern, int i, int j, int flV, int fl, int transpose, int recStrideExt);
  Int  xFilterPixel(const Pel *ImgDec, Int* varIndBeforeMapping, Int **filterCoeffSym, Int *pattern, Int i, Int j, Int fl, Int Stride, AlfFilterType filtNo);
  void xfindBestFilterPredictor(
#if JVET_C0038_NO_PREV_FILTERS
    ALFParam& pcAlfParam
#endif
    );
#if JVET_C0038_NO_PREV_FILTERS
  Double xTestFixedFilterFast      ( Double ***A, Double **b, Double *pixAcc, Double *filterCoeffSym, Double *filterCoeffDefault, Int varInd);
  Double xTestFixedFilter          ( const Pel *imgY_rec, const Pel *imgY_org, const Pel *imgY_append, Int usePrevFilt[], Int noVarBins, Int orgStride, Int recStride, Int filtType);
  void   xPreFilterFr              ( Int** imgY_preFilter, const Pel* imgY_rec, const Pel * imgY_org, const Pel* imgY_append, Int usePrevFilt[], Int Stride, Int filtType);
  void   xfindBestFilterPredictor  ( Double ***E_temp, Double**y_temp, Double *pixAcc_temp, Int filtType, const Pel* ImgOrg, const Pel* ImgDec, Int orgStride, Int recStride, Int* usePrevFiltBest, Int sqrFiltLength, Int iFixedFilters
    );
#endif

#if FORCE0
  void xcollectStatCodeFilterCoeffForce0(Int **pDiffQFilterCoeffIntPP, Int filtType, Int sqrFiltLength, Int filters_per_group,
    int bitsVarBin[]);
  void xdecideCoeffForce0(Int codedVarBins[m_NO_VAR_BINS], Double *distForce0, Double errorForce0CoeffTab[m_NO_VAR_BINS][2],
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
  void xcalcPredFilterCoeff        ( AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam* alfParam
#endif
    ); //TODO rename

  void   xCheckFilterMergingAlf(ALFParam& alfParam
#if JVET_C0038_NO_PREV_FILTERS
    , const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf
#endif
  );
  void   xCheckFilterMergingGalf(ALFParam& alfParam
#if JVET_C0038_NO_PREV_FILTERS
    , const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf
#endif
  );
  void xMergeFiltersGreedyGalf( Double ***EGlobalSeq, Double **yGlobalSeq, Double *pixAccGlobalSeq, int intervalBest[m_NO_VAR_BINS][m_NO_VAR_BINS], int sqrFiltLength);
  Double xMergeFiltersGreedy  (Double ***EGlobalSeq, Double **yGlobalSeq, Double *pixAccGlobalSeq, int intervalBest[m_NO_VAR_BINS][2], int sqrFiltLength, int noIntervals);
  void   xReDesignFilterCoeff      ( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf
#if COM16_C806_ALF_TEMPPRED_NUM
   , ALFParam* alfParam
#endif
    , const ClpRng& clpRng);

  Double xSolveAndQuant( Double *filterCoeff, Int *filterCoeffQuant, Double **E, Double *y, int sqrFiltLength,const Int *weights, int bit_depth, bool bChroma = false );
  void   roundFiltCoeff            ( int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int factor);

  void xDeriveGlobalEyFromLgrTapFilter(Double **E0, Double *y0, Double **E1, Double *y1, const Int *pattern0, const Int *pattern1);
  void xDeriveLocalEyFromLgrTapFilter(Double *y0, Double *y1, const Int *pattern0, const Int *pattern1);
  void add_A_galf                  (Double **Amerged, Double ***A, Int interval[], Int filtNo, Int size);
  void add_b_galf                  (Double  *bmerged, Double  **b, Int interval[], Int filtNo, Int size);
  void add_b                       ( Double  *bmerged, Double  **b, Int start, Int stop, Int size);
  void add_A                       ( Double **Amerged, Double ***A, Int start, Int stop, Int size);

  Int  gnsSolveByChol              ( Double **LHS, double *rhs, double *x, int noEq);
  void gnsBacksubstitution         ( Double R[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], Double z[m_MAX_SQR_FILT_LENGTH], Int R_size, Double A[m_MAX_SQR_FILT_LENGTH]);
  void gnsTransposeBacksubstitution( Double U[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], Double rhs[], Double x[], Int order);
  Int  gnsCholeskyDec              ( Double **inpMatr, double outMatr[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], Int noEq);


  //Filtering
  void xFilterFrame_en             (PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, ALFParam& alfParam,  const ClpRng& clpRng);
  void xFilterFrame_enGalf         (PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam *alfParam, bool updateFilterCoef
#endif
    , const ClpRng& clpRng);
  void xFilterFrame_enAlf          (PelUnitBuf& recDstBuf, const PelUnitBuf& recExtBuf, AlfFilterType filtType
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam *alfParam, bool updateFilterCoef
#endif
    , const ClpRng& clpRng);
  void xFilteringFrameChroma       ( const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf );

  //Chroma
  void xCalcCorrelationFunc        ( const CPelBuf& rcOrgBuf, const CPelBuf& rcCmpBuf, Int iTap );
  Int  xGauss                      ( Double **a, Int N);
  void xClearFilterCoefInt         ( Int* qh, Int N);
  void xQuantFilterCoefChroma      ( Double* h, Int* qh, Int tap, int bit_depth );
  void xFilterCoefQuickSort        ( Double *coef_data, Int *coef_num, Int upper, Int lower );

  //R-D
  void   xCalcRDCostLuma           ( const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recBuf, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  void   xCalcRDCostChroma         ( const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recBuf, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  void   xCalcRDCost               ( const UInt64 uiDist, ALFParam* pAlfParam, UInt64& ruiRate,  Double& rdCost );
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
  bool bFindBestFixedFilter;
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
