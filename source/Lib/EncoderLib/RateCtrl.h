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

/** \file     RateCtrl.h
    \brief    Rate control manager class
*/

#ifndef __ENCRATECTRL__
#define __ENCRATECTRL__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "../CommonLib/CommonDef.h"

#include <vector>
#include <algorithm>

using namespace std;

//! \ingroup EncoderLib
//! \{

#include "../EncoderLib/EncCfg.h"
#include <list>

const Int g_RCInvalidQPValue = -999;
const Int g_RCSmoothWindowSize = 40;
const Int g_RCMaxPicListSize = 32;
const Double g_RCWeightPicTargetBitInGOP    = 0.9;
const Double g_RCWeightPicRargetBitInBuffer = 1.0 - g_RCWeightPicTargetBitInGOP;
const Int g_RCIterationNum = 20;
const Double g_RCWeightHistoryLambda = 0.5;
const Double g_RCWeightCurrentLambda = 1.0 - g_RCWeightHistoryLambda;
const Int g_RCLCUSmoothWindowSize = 4;
const Double g_RCAlphaMinValue = 0.05;
const Double g_RCAlphaMaxValue = 500.0;
const Double g_RCBetaMinValue  = -3.0;
const Double g_RCBetaMaxValue  = -0.1;

#define ALPHA     6.7542;
#define BETA1     1.2517
#define BETA2     1.7860

struct TRCLCU
{
  Int m_actualBits;
  Int m_QP;     // QP of skip mode is set to g_RCInvalidQPValue
  Int m_targetBits;
  Double m_lambda;
  Double m_bitWeight;
  Int m_numberOfPixel;
  Double m_costIntra;
  Int m_targetBitsLeft;
};

struct TRCParameter
{
  Double m_alpha;
  Double m_beta;
};

class EncRCSeq
{
public:
  EncRCSeq();
  ~EncRCSeq();

public:
  void create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, bool useLCUSeparateModel, Int adaptiveBit );
  void destroy();
  void initBitsRatio( Int bitsRatio[] );
  void initGOPID2Level( Int GOPID2Level[] );
  void initPicPara( TRCParameter* picPara  = NULL );    // NULL to initial with default value
  void initLCUPara( TRCParameter** LCUPara = NULL );    // NULL to initial with default value
  void updateAfterPic ( Int bits );
  void setAllBitRatio( Double basicLambda, Double* equaCoeffA, Double* equaCoeffB );

public:
  Int  getTotalFrames()                 { return m_totalFrames; }
  Int  getTargetRate()                  { return m_targetRate; }
  Int  getFrameRate()                   { return m_frameRate; }
  Int  getGOPSize()                     { return m_GOPSize; }
  Int  getPicWidth()                    { return m_picWidth; }
  Int  getPicHeight()                   { return m_picHeight; }
  Int  getLCUWidth()                    { return m_LCUWidth; }
  Int  getLCUHeight()                   { return m_LCUHeight; }
  Int  getNumberOfLevel()               { return m_numberOfLevel; }
  Int  getAverageBits()                 { return m_averageBits; }
  Int  getLeftAverageBits()             { CHECK(!( m_framesLeft > 0 ), "No frames left"); return (Int)(m_bitsLeft / m_framesLeft); }
  bool getUseLCUSeparateModel()         { return m_useLCUSeparateModel; }

  Int  getNumPixel()                    { return m_numberOfPixel; }
  int64_t  getTargetBits()                { return m_targetBits; }
  Int  getNumberOfLCU()                 { return m_numberOfLCU; }
  Int* getBitRatio()                    { return m_bitsRatio; }
  Int  getBitRatio( Int idx )           { CHECK(!( idx<m_GOPSize), "Idx exceeds GOP size"); return m_bitsRatio[idx]; }
  Int* getGOPID2Level()                 { return m_GOPID2Level; }
  Int  getGOPID2Level( Int ID )         { CHECK(!( ID < m_GOPSize ), "Idx exceeds GOP size"); return m_GOPID2Level[ID]; }
  TRCParameter*  getPicPara()                                   { return m_picPara; }
  TRCParameter   getPicPara( Int level )                        { CHECK(!( level < m_numberOfLevel ), "Level too big"); return m_picPara[level]; }
  void           setPicPara( Int level, TRCParameter para )     { CHECK(!( level < m_numberOfLevel ), "Level too big"); m_picPara[level] = para; }
  TRCParameter** getLCUPara()                                   { return m_LCUPara; }
  TRCParameter*  getLCUPara( Int level )                        { CHECK(!( level < m_numberOfLevel ), "Level too big"); return m_LCUPara[level]; }
  TRCParameter   getLCUPara( Int level, Int LCUIdx )            { CHECK(!( LCUIdx  < m_numberOfLCU ), "LCU id exceeds number of LCU"); return getLCUPara(level)[LCUIdx]; }
  void           setLCUPara( Int level, Int LCUIdx, TRCParameter para ) { CHECK(!( level < m_numberOfLevel ), "Level too big"); CHECK(!( LCUIdx  < m_numberOfLCU ), "LCU id exceeds number of LCU"); m_LCUPara[level][LCUIdx] = para; }

  Int  getFramesLeft()                  { return m_framesLeft; }
  int64_t  getBitsLeft()                  { return m_bitsLeft; }

  Double getSeqBpp()                    { return m_seqTargetBpp; }
  Double getAlphaUpdate()               { return m_alphaUpdate; }
  Double getBetaUpdate()                { return m_betaUpdate; }

  Int    getAdaptiveBits()              { return m_adaptiveBit;  }
  Double getLastLambda()                { return m_lastLambda;   }
  void   setLastLambda( Double lamdba ) { m_lastLambda = lamdba; }

private:
  Int m_totalFrames;
  Int m_targetRate;
  Int m_frameRate;
  Int m_GOPSize;
  Int m_picWidth;
  Int m_picHeight;
  Int m_LCUWidth;
  Int m_LCUHeight;
  Int m_numberOfLevel;
  Int m_averageBits;

  Int m_numberOfPixel;
  int64_t m_targetBits;
  Int m_numberOfLCU;
  Int* m_bitsRatio;
  Int* m_GOPID2Level;
  TRCParameter*  m_picPara;
  TRCParameter** m_LCUPara;

  Int m_framesLeft;
  int64_t m_bitsLeft;
  Double m_seqTargetBpp;
  Double m_alphaUpdate;
  Double m_betaUpdate;
  bool m_useLCUSeparateModel;

  Int m_adaptiveBit;
  Double m_lastLambda;
};

class EncRCGOP
{
public:
  EncRCGOP();
  ~EncRCGOP();

public:
  void create( EncRCSeq* encRCSeq, Int numPic );
  void destroy();
  void updateAfterPicture( Int bitsCost );

private:
  Int  xEstGOPTargetBits( EncRCSeq* encRCSeq, Int GOPSize );
  void   xCalEquaCoeff( EncRCSeq* encRCSeq, Double* lambdaRatio, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize );
  Double xSolveEqua( Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize );

public:
  EncRCSeq* getEncRCSeq()        { return m_encRCSeq; }
  Int  getNumPic()                { return m_numPic;}
  Int  getTargetBits()            { return m_targetBits; }
  Int  getPicLeft()               { return m_picLeft; }
  Int  getBitsLeft()              { return m_bitsLeft; }
  Int  getTargetBitInGOP( Int i ) { return m_picTargetBitInGOP[i]; }

private:
  EncRCSeq* m_encRCSeq;
  Int* m_picTargetBitInGOP;
  Int m_numPic;
  Int m_targetBits;
  Int m_picLeft;
  Int m_bitsLeft;
};

class EncRCPic
{
public:
  EncRCPic();
  ~EncRCPic();

public:
  void create( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP, Int frameLevel, list<EncRCPic*>& listPreviousPictures );
  void destroy();

  Int    estimatePicQP    ( Double lambda, list<EncRCPic*>& listPreviousPictures );
  Int    getRefineBitsForIntra(Int orgBits);
  Double calculateLambdaIntra(Double alpha, Double beta, Double MADPerPixel, Double bitsPerPixel);
  Double estimatePicLambda( list<EncRCPic*>& listPreviousPictures, SliceType eSliceType);

  void   updateAlphaBetaIntra(Double *alpha, Double *beta);

  Double getLCUTargetBpp(SliceType eSliceType);
  Double getLCUEstLambdaAndQP(Double bpp, Int clipPicQP, Int *estQP);
  Double getLCUEstLambda( Double bpp );
  Int    getLCUEstQP( Double lambda, Int clipPicQP );

  void updateAfterCTU( Int LCUIdx, Int bits, Int QP, Double lambda, bool updateLCUParameter = true );
  void updateAfterPicture( Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType);

  void addToPictureLsit( list<EncRCPic*>& listPreviousPictures );
  Double calAverageQP();
  Double calAverageLambda();

private:
  Int xEstPicTargetBits( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP );
  Int xEstPicHeaderBits( list<EncRCPic*>& listPreviousPictures, Int frameLevel );
#if V0078_ADAPTIVE_LOWER_BOUND
  Int xEstPicLowerBound( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP );
#endif

public:
  EncRCSeq*      getRCSequence()                         { return m_encRCSeq; }
  EncRCGOP*      getRCGOP()                              { return m_encRCGOP; }

  Int  getFrameLevel()                                    { return m_frameLevel; }
  Int  getNumberOfPixel()                                 { return m_numberOfPixel; }
  Int  getNumberOfLCU()                                   { return m_numberOfLCU; }
  Int  getTargetBits()                                    { return m_targetBits; }
  Int  getEstHeaderBits()                                 { return m_estHeaderBits; }
  Int  getLCULeft()                                       { return m_LCULeft; }
  Int  getBitsLeft()                                      { return m_bitsLeft; }
  Int  getPixelsLeft()                                    { return m_pixelsLeft; }
  Int  getBitsCoded()                                     { return m_targetBits - m_estHeaderBits - m_bitsLeft; }
  Int  getLCUCoded()                                      { return m_numberOfLCU - m_LCULeft; }
#if V0078_ADAPTIVE_LOWER_BOUND
  Int  getLowerBound()                                    { return m_lowerBound; }
#endif
  TRCLCU* getLCU()                                        { return m_LCUs; }
  TRCLCU& getLCU( Int LCUIdx )                            { return m_LCUs[LCUIdx]; }
  Int  getPicActualHeaderBits()                           { return m_picActualHeaderBits; }
#if U0132_TARGET_BITS_SATURATION
  void setBitLeft(Int bits)                               { m_bitsLeft = bits; }
#endif
  void setTargetBits( Int bits )                          { m_targetBits = bits; m_bitsLeft = bits;}
  void setTotalIntraCost(Double cost)                     { m_totalCostIntra = cost; }
  void getLCUInitTargetBits();

  Int  getPicActualBits()                                 { return m_picActualBits; }
  Int  getPicActualQP()                                   { return m_picQP; }
  Double getPicActualLambda()                             { return m_picLambda; }
  Int  getPicEstQP()                                      { return m_estPicQP; }
  void setPicEstQP( Int QP )                              { m_estPicQP = QP; }
  Double getPicEstLambda()                                { return m_estPicLambda; }
  void setPicEstLambda( Double lambda )                   { m_picLambda = lambda; }

private:
  EncRCSeq* m_encRCSeq;
  EncRCGOP* m_encRCGOP;

  Int m_frameLevel;
  Int m_numberOfPixel;
  Int m_numberOfLCU;
  Int m_targetBits;
  Int m_estHeaderBits;
  Int m_estPicQP;
#if V0078_ADAPTIVE_LOWER_BOUND
  Int m_lowerBound;
#endif
  Double m_estPicLambda;

  Int m_LCULeft;
  Int m_bitsLeft;
  Int m_pixelsLeft;

  TRCLCU* m_LCUs;
  Int m_picActualHeaderBits;    // only SH and potential APS
  Double m_totalCostIntra;
  Double m_remainingCostIntra;
  Int m_picActualBits;          // the whole picture, including header
  Int m_picQP;                  // in integer form
  Double m_picLambda;
};

class RateCtrl
{
public:
  RateCtrl();
  ~RateCtrl();

public:
  void init( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int keepHierBits, bool useLCUSeparateModel, GOPEntry GOPList[MAX_GOP] );
  void destroy();
  void initRCPic( Int frameLevel );
  void initRCGOP( Int numberOfPictures );
  void destroyRCGOP();

public:
  void       setRCQP ( Int QP ) { m_RCQP = QP;   }
  Int        getRCQP () const   { return m_RCQP; }
  EncRCSeq* getRCSeq()          { CHECK( m_encRCSeq == NULL, "Object does not exist" ); return m_encRCSeq; }
  EncRCGOP* getRCGOP()          { CHECK( m_encRCGOP == NULL, "Object does not exist" ); return m_encRCGOP; }
  EncRCPic* getRCPic()          { CHECK( m_encRCPic == NULL, "Object does not exist" ); return m_encRCPic; }
  list<EncRCPic*>& getPicList() { return m_listRCPictures; }
#if U0132_TARGET_BITS_SATURATION
  bool       getCpbSaturationEnabled()  { return m_CpbSaturationEnabled;  }
  UInt       getCpbState()              { return m_cpbState;       }
  UInt       getCpbSize()               { return m_cpbSize;        }
  UInt       getBufferingRate()         { return m_bufferingRate;  }
  Int        updateCpbState(Int actualBits);
  void       initHrdParam(const HRD* pcHrd, Int iFrameRate, Double fInitialCpbFullness);
#endif

private:
  EncRCSeq* m_encRCSeq;
  EncRCGOP* m_encRCGOP;
  EncRCPic* m_encRCPic;
  list<EncRCPic*> m_listRCPictures;
  Int        m_RCQP;
#if U0132_TARGET_BITS_SATURATION
  bool       m_CpbSaturationEnabled;    // Enable target bits saturation to avoid CPB overflow and underflow
  Int        m_cpbState;                // CPB State
  UInt       m_cpbSize;                 // CPB size
  UInt       m_bufferingRate;           // Buffering rate
#endif
};

#endif


