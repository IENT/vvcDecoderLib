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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "CommonDef.h"

#if JEM_TOOLS

#include "Picture.h"


const static Int ALF_NUM_OF_CLASSES = 16;

void destroyMatrix_int(int **m2D);
void initMatrix_int(int ***m2D, int d1, int d2);

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// adaptive loop filter class
class AdaptiveLoopFilter
{
public:
  static const Int m_ALF_MAX_NUM_COEF      = 42;                                    ///< maximum number of filter coefficients
  static const Int m_ALF_MAX_NUM_COEF_C    = 14;                                    ///< number of filter taps for chroma
protected:
  static const Int m_ALF_VAR_SIZE_H        = 4;
  static const Int m_ALF_VAR_SIZE_W        = 4;

  static const Int m_ALF_WIN_VERSIZE       = 32;
  static const Int m_ALF_WIN_HORSIZE       = 32;

  static const Int m_ALF_MAX_NUM_TAP       = 9;                                     ///< maximum number of filter taps (9x9)
  static const Int m_ALF_MIN_NUM_TAP       = 5;                                     ///< minimum number of filter taps (5x5)
  static const Int m_ALF_MAX_NUM_TAP_C     = 5;                                     ///< number of filter taps for chroma (5x5)

  static const Int m_ALF_MIN_NUM_COEF      = 14;                                    ///< minimum number of filter coefficients

  static const Int m_ALF_NUM_BIT_SHIFT     = 8;                                     ///< bit shift parameter for quantization of ALF param.
  static const Int m_ALF_ROUND_OFFSET      = ( 1 << ( m_ALF_NUM_BIT_SHIFT - 1 ) );  ///< rounding offset for ALF quantization

  static const Int m_VAR_SIZE              = 1;                                     ///< JCTVC-E323+E046

  static const Int m_FILTER_LENGTH         = 9;

  static const Int m_ALF_HM3_QC_CLIP_RANGE = 1024;
  static const Int m_ALF_HM3_QC_CLIP_OFFSET= 384;

public:

  static const Int m_NO_TEST_FILT          =  3;                                    ///< Filter supports (5/7/9)

  #define NO_VALS_LAGR                     5    //galf stuff
  #define NO_VALS_LAGR_SHIFT               3    //galf stuff
  static const Int m_MAX_SQT_FILT_SYM_LENGTH = ((m_FILTER_LENGTH*m_FILTER_LENGTH) / 4 + 1);

#if GALF
  static const Int m_NUM_BITS            = 10;
  static const Int m_NO_VAR_BINS         = 25;
  static const Int m_NO_FILTERS          = 25;
  static const Int m_MAX_SQR_FILT_LENGTH = ((m_FILTER_LENGTH*m_FILTER_LENGTH) / 2 + 1);
  static const Int m_SQR_FILT_LENGTH_9SYM = ((9 * 9) / 4 + 1);
  static const Int m_SQR_FILT_LENGTH_7SYM = ((7 * 7) / 4 + 1);
  static const Int m_SQR_FILT_LENGTH_5SYM = ((5 * 5) / 4 + 1);
#else
  static const Int m_NUM_BITS              = 9;
  static const Int m_NO_VAR_BINS           = 16;
  static const Int m_NO_FILTERS            = 16;
  static const Int m_MAX_SQR_FILT_LENGTH = ((m_FILTER_LENGTH*m_FILTER_LENGTH) / 2 + 2);
  static const Int m_SQR_FILT_LENGTH_9SYM = ((9 * 9) / 4 + 2 - 1);
  static const Int m_SQR_FILT_LENGTH_7SYM = ((7 * 7) / 4 + 2);
  static const Int m_SQR_FILT_LENGTH_5SYM = ((5 * 5) / 4 + 2);
#endif
  static const Int m_FilterTapsOfType[ALF_NUM_OF_FILTER_TYPES];

  static const Int m_MAX_SCAN_VAL          = 11;
  static const Int m_MAX_EXP_GOLOMB        = 16;

  // quantized filter coefficients
  static const Int m_aiSymmetricMag9x9[41];                                         ///< quantization scaling factor for 9x9 filter
  static const Int m_aiSymmetricMag7x7[25];                                         ///< quantization scaling factor for 7x7 filter
  static const Int m_aiSymmetricMag5x5[13];                                         ///< quantization scaling factor for 5x5 filter
  static const Int m_aiSymmetricMag9x7[32];                                         ///< quantization scaling factor for 9x7 filter

  // temporary picture buffer
  PelStorage   m_tmpRecExtBuf;                                                     ///< temporary picture buffer for extended reconstructed frame

public:
  static const Int* m_pDepthIntTab[m_NO_TEST_FILT];

protected:
#if JVET_C0038_NO_PREV_FILTERS
  static const Int m_ALFfilterCoeffFixed[m_NO_FILTERS*JVET_C0038_NO_PREV_FILTERS][21]; /// fixed filters used in ALF.
#endif
  static const Int depthInt9x9Cut[21];
  static const Int depthInt7x7Cut[14];
  static const Int depthInt5x5Cut[8];
  static const Int m_depthInt9x9Sym[21];
  static const Int m_depthInt7x7Sym[14];
  static const Int m_depthInt5x5Sym[8];
  // ------------------------------------------------------------------------------------------------------------------
  // For luma component
  // ------------------------------------------------------------------------------------------------------------------
#if GALF
  static const Int m_pattern9x9Sym[41];
  static const Int m_weights9x9Sym[22];
#else
  static const Int m_pattern9x9Sym[39];
  static const Int m_weights9x9Sym[21];
#endif
  static const Int m_pattern9x9Sym_Quart[42];
  static const Int m_pattern7x7Sym[25];
  static const Int m_weights7x7Sym[14];
  static const Int m_pattern7x7Sym_Quart[42];
  static const Int m_pattern5x5Sym[13];
  static const Int m_weights5x5Sym[8];
  static const Int m_pattern5x5Sym_Quart[45];
  static const Int m_pattern9x9Sym_9[39];
  static const Int m_pattern9x9Sym_7[25];
  static const Int m_pattern9x9Sym_5[13];

  static const Int m_flTab[m_NO_TEST_FILT];
  static const Int *m_patternTab[m_NO_TEST_FILT];
  static const Int *m_patternMapTab[m_NO_TEST_FILT];
  static const Int *m_weightsTab[m_NO_TEST_FILT];
  static const Int m_sqrFiltLengthTab[m_NO_TEST_FILT];
  static const Int m_mapTypeToNumOfTaps[m_NO_TEST_FILT];

  Int       m_img_height,m_img_width;
  Int       m_nInputBitDepth;
  Int       m_nInternalBitDepth;
  Int       m_nBitIncrement;
  Int       m_nIBDIMax;
  ClpRngs   m_clpRngs;
  UInt      m_uiMaxTotalCUDepth;
  UInt      m_uiMaxCUWidth;
  UInt      m_uiNumCUsInFrame; //TODO rename

  Pel**     m_imgY_var;
  Int**     m_imgY_temp;

  Int**     m_imgY_ver;
  Int**     m_imgY_hor;
  Int**     m_imgY_dig0;
  Int**     m_imgY_dig1;
  Int **    m_filterCoeffFinal;
  Pel**     m_varImgMethods;

  Int**     m_filterCoeffSym;
  Int**     m_filterCoeffPrevSelected;
  Short**   m_filterCoeffShort;
  Int**     m_filterCoeffTmp;
  Int**     m_filterCoeffSymTmp;

  Bool      m_isGALF;
  Bool      m_wasCreated;
  Bool      m_isDec;

public:


  bool      m_galf;
  unsigned  m_storedAlfParaNum[E0104_ALF_MAX_TEMPLAYERID];
  ALFParam  m_acStoredAlfPara[E0104_ALF_MAX_TEMPLAYERID][C806_ALF_TEMPPRED_NUM];
protected:
  /// ALF for luma component
  Void xALFLuma( CodingStructure& cs, ALFParam* pcAlfParam, PelUnitBuf& recSrcExt, PelUnitBuf& recDst );


  Void reconstructFilterCoeffs(ALFParam* pcAlfParam,int **pfilterCoeffSym );
  Void getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam);
  Void xFilterFrame  (PelUnitBuf& recSrcExt, PelUnitBuf& recDst, AlfFilterType filtType
    );
  Void xFilterBlkGalf(PelUnitBuf &recDst, const CPelUnitBuf& recSrcExt, const Area& blk, AlfFilterType filtType, const ComponentID compId);
  Void xFilterBlkAlf (PelBuf &recDst, const CPelBuf& recSrc, const Area& blk, AlfFilterType filtType);

  Void xClassify                 (Pel** classes, const CPelBuf& recSrcBuf, Int pad_size, Int fl);
  Void xClassifyByGeoLaplacian   (Pel** classes, const CPelBuf& srcLumaBuf, Int pad_size, Int fl, const Area& blk);
  Void xClassifyByGeoLaplacianBlk(Pel** classes, const CPelBuf& srcLumaBuf, Int pad_size, Int fl, const Area& blk);
  Int  selectTransposeVarInd     (Int varInd, Int *transpose);

  Void xClassifyByLaplacian      (Pel** classes, const CPelBuf& srcLumaBuf, Int pad_size, Int fl, const Area& blk);
  Void xClassifyByLaplacianBlk   (Pel** classes, const CPelBuf& srcLumaBuf, Int pad_size, Int fl, const Area& blk);
  Void xDecodeFilter( ALFParam* pcAlfParam );

  // memory allocation
  Void destroyMatrix_short(short **m2D);
  Void initMatrix_short(short ***m2D, int d1, int d2);
  Void destroyMatrix_Pel(Pel **m2D);
  Void destroyMatrix_int(int **m2D);
  Void initMatrix_int(int ***m2D, int d1, int d2);
  Void initMatrix_Pel(Pel ***m2D, int d1, int d2);
  Void destroyMatrix4D_double(double ****m4D, int d1, int d2);
  Void destroyMatrix3D_double(double ***m3D, int d1);
  Void destroyMatrix_double(double **m2D);
  Void initMatrix4D_double(double *****m4D, int d1, int d2, int d3, int d4);
  Void initMatrix3D_double(double ****m3D, int d1, int d2, int d3);
  Void initMatrix_double(double ***m2D, int d1, int d2);
  Void free_mem2Dpel(Pel **array2D);
  Void get_mem2Dpel(Pel ***array2D, int rows, int columns);
  Void no_mem_exit(const char *where);
  Void xError(const char *text, int code);
  Void calcVar(Pel **imgY_var, Pel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width = 0 , int start_height = 0 );
  Void xCalcVar(Pel **imgY_var, Pel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height );

  Void xCUAdaptive( CodingStructure& cs, const PelUnitBuf &recExtBuf, PelUnitBuf &recBuf, ALFParam* pcAlfParam
    );

  /// ALF for chroma component
  Void xALFChroma   ( ALFParam* pcAlfParam,const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf );
  Void xFrameChromaGalf(ALFParam* pcAlfParam, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, ComponentID compID);
  Void xFrameChromaAlf (ALFParam* pcAlfParam, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, ComponentID compID );

public:
  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() {}

  // initialize & destroy temporary buffer
  Void create( const Int iPicWidth, Int iPicHeight, const ChromaFormat chromaFormatIDC, const Int uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth, const Int nInputBitDepth, const Int nInternalBitDepth, const Int numberOfCTUs );
  Void destroy ();

  Void ALFProcess     ( CodingStructure& cs, ALFParam* pcAlfParam
                      ); ///< interface function for ALF process

  // alloc & free & set functions //TODO move to ALFParam class
  Void allocALFParam  ( ALFParam* pAlfParam );
  Void freeALFParam   ( ALFParam* pAlfParam );
  Void copyALFParam   ( ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam, Bool max_depth_copy = true );

  void storeALFParam  ( ALFParam* pAlfParam, bool isISlice, unsigned tLayer, unsigned tLayerMax );
  void loadALFParam   ( ALFParam* pAlfParam, unsigned idx, unsigned tLayer );

  Void resetALFParam  ( ALFParam* pDesAlfParam);
  Void resetALFPredParam(ALFParam *pAlfParam, Bool bIntra);
  Void setNumCUsInFrame(UInt uiNumCUsInFrame);

  // predict filter coefficients
  Void predictALFCoeffChroma  ( ALFParam* pAlfParam );                  ///< prediction of chroma ALF coefficients
  Void initVarForChroma(ALFParam* pcAlfParam, Bool bUpdatedDCCoef);

  Void refreshAlfTempPred();

  static Int ALFTapHToTapV     ( Int tapH );
  static Int ALFTapHToNumCoeff ( Int tapH );
  static Int ALFFlHToFlV       ( Int flH  );
};
#endif

#endif
