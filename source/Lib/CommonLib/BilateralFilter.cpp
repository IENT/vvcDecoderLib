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

#include "BilateralFilter.h"

#if JEM_TOOLS

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>

const int BilateralFilter::SpatialSigmaValue = 62;

const int BilateralFilter::spatialSigmaBlockLengthOffsets[] = {20, 10, -10, 0, -10};
#if JVET_K0251_QP_EXT
const unsigned short maxPosList[46] = {6, 12, 18, 23, 29, 35, 41, 46, 52, 58, 64, 69, 75, 81, 87, 92, 98, 104, 110, 115, 121, 127, 133, 138, 144, 150, 156, 161, 167, 173, 179, 184, 190, 196, 202, 207, 213, 219, 225, 230, 236, 242, 248, 253, 259, 265 };
#else
const unsigned short maxPosList[34] = {6, 12, 18, 23, 29, 35, 41, 46, 52, 58, 64, 69, 75, 81, 87, 92, 98, 104, 110, 115, 121, 127, 133, 138, 144, 150, 156, 161, 167, 173, 179, 184, 190, 196};
#endif
BilateralFilter::BilateralFilter()
{
  int numQP = MAX_QP-18+1;
  // allocation
  m_bilateralFilterTable = new UShort*[numQP];
  for(int i = 0; i < numQP; i++)
  {
    m_bilateralFilterTable[i] = new UShort[maxPosList[i]+1];
  }

  // initialization
  for(int i = 0; i < numQP; i++)
  {
    for(int k = 0; k < (maxPosList[i]+1); k++)
    {
      m_bilateralFilterTable[i][k] = 0;
    }
  }
}

BilateralFilter::~BilateralFilter()
{
  destroy();
}

void BilateralFilter::create()
{
  createdivToMulLUTs();

  for( Int qp = 18; qp < MAX_QP + 1; qp++ )
  {
    createBilateralFilterTable( qp );
  }
}

void BilateralFilter::destroy()
{
  if( m_bilateralFilterTable )
  {
    int numQP = MAX_QP - 18 + 1;

    for( int i = 0; i < numQP; ++i )
    {
      delete[] m_bilateralFilterTable[i];
      m_bilateralFilterTable[i] = nullptr;
    }

    delete[] m_bilateralFilterTable;
    m_bilateralFilterTable = nullptr;
  }
}

void BilateralFilter::createdivToMulLUTs()
{
  UInt one = 1 << BITS_PER_DIV_LUT_ENTRY; // 1 is represented by 2^14 (not 2^14 -1)
  divToMulOneOverN[0] = one; // We can never divide by zero since the centerweight is non-zero, so we can set this value to something arbitrary.
  divToMulShift[0] = 0;

  for (UInt n=1; n<BILATERAL_FILTER_MAX_DENOMINATOR_PLUS_ONE; n++)
  {
    UInt tryLUT = one / n;

    UInt tryShift = 0;
    // Make sure the LUT entry stored does not start with (binary) zeros.
    while(tryLUT <= one)
    {
      // This value of tryLUT
      divToMulOneOverN[n] = tryLUT;
      divToMulShift[n] = tryShift;

      tryShift++;
      tryLUT = (one << tryShift) / n;
    }

    // We may need to add 1 to the LUT entry in order to make 3/3, 4/4, 5/5, ... come out right.
    UInt adiv = divToMulOneOverN[n] * n / (one << divToMulShift[n]);
    if(adiv != 1)
      divToMulOneOverN[n]++;
  }
}

void BilateralFilter::createBilateralFilterTable(int qp)
{
  Int spatialSigmaValue;
  Int intensitySigmaValue = (qp - 17) * 50;
  Int sqrtSpatialSigmaMulTwo;
  Int sqrtIntensitySigmaMulTwo = 2 * intensitySigmaValue * intensitySigmaValue;
  int centerWeightTableSize = 5;

  spatialSigmaValue = SpatialSigmaValue;;
  for (Int i = 0; i < centerWeightTableSize; i++)
  {
    sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);

    // Calculate the multiplication factor that we will use to convert the first table (with the strongest filter) to one of the
    // tables that gives weaker filtering (such as when TU = 8 or 16 or when we have inter filtering).
    Int sqrtSpatialSigmaMulTwoStrongestFiltering = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[0]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[0]);

    // multiplication factor equals exp(-1/stronger)/exp(-1/weaker)
    double centerWeightMultiplier = exp(-(10000.0 / sqrtSpatialSigmaMulTwoStrongestFiltering))/exp(-(10000.0 / sqrtSpatialSigmaMulTwo));
    m_bilateralCenterWeightTable[i] = (Int)(centerWeightMultiplier*65 + 0.5);
  }
  Int i = 0;
  sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);
  for (Int j = 0; j < (maxPosList[qp-18]+1); j++)
  {
    Int temp = j * 25;
    m_bilateralFilterTable[qp-18][j] = UShort(exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * 65 + 0.5);
  }
}

void BilateralFilter::smoothBlockBilateralFilter(unsigned uiWidth, unsigned uiHeight, short block[], int isInterBlock, int qp)
{
  Int length = (Int)std::min(uiWidth, uiHeight);
  Int rightPixel, centerPixel;
  Int rightWeight, bottomWeight, centerWeight;
  Int sumWeights[MAX_CU_SIZE];
  Int sumDelta[MAX_CU_SIZE];
  Int blockLengthIndex;

  Int dIB, dIR;

  if( length >= 16 )
  {
    blockLengthIndex = 2;
  }
  else if ( length >= 8 )
  {
    blockLengthIndex = 1;
  }
  else
  {
    blockLengthIndex = 0;
  }


  UShort *lookupTablePtr;

  centerWeight = m_bilateralCenterWeightTable[blockLengthIndex + 3 * isInterBlock];

  Int theMaxPos = maxPosList[qp-18];
  lookupTablePtr = m_bilateralFilterTable[qp-18];

  // for each pixel in block

  // These are the types of pixels:
  //
  // A BB C
  //
  // D EE F
  // D EE F
  //
  // G HH I
  //
  // If the block is larger than 4x4, the E-part is larger.
  //
  // Filter types:
  //
  // AA  BBB   CC
  // A    B     C
  //
  // D    E     F
  // DD  EEE   FF
  // D    E     F
  //
  // G    H     I
  // GG  HHH   II
  // C uses a filter of type x
  Int currentPixelDeltaSum;
  Int currentPixelSumWeights;
  Int rightPixelDeltaSum;
  Int rightPixelSumWeights;
  Int rightWeightTimesdIR;
  Int bottomWeightTimesdIB;

  Int mySignIfNeg;
  Int mySign;

  Short *blockCurrentPixelPtr = block;
  Short *blockRightPixelPtr = blockCurrentPixelPtr+1;
  Short *blockNextLinePixelPtr = blockCurrentPixelPtr + uiWidth;
  Int *sumWeightsPtr = sumWeights;
  Int *sumDeltaPtr = sumDelta;

  // A pixel. uses filter type xx
  //                           x
  //
  // No information from previous row
  // No information from previous pixel
  // Storing information to next row
  // Storing information to next pixel

  // top left pixel; i = 0, j = 0;

  centerPixel = *(blockCurrentPixelPtr);
  rightPixel = *(blockRightPixelPtr++);
  dIR = rightPixel - centerPixel;
  dIB = *(blockNextLinePixelPtr++) - centerPixel;

  rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
  bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];

  rightWeightTimesdIR = rightWeight*dIR;
  bottomWeightTimesdIB = bottomWeight*dIB;

  currentPixelSumWeights = centerWeight + rightWeight + bottomWeight;
  currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB;

  rightPixelSumWeights = rightWeight; //next pixel to the right
  rightPixelDeltaSum = rightWeightTimesdIR;

  *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
  *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom

  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

  for (Int i = 1; i < (uiWidth-1); i++)
  {
    // B pixel. uses filter type xxx
    //                            x
    //
    // No information from previous row
    // Information reused from previous pixel
    // Storing information to next row
    // Storing information to next pixel

    centerPixel = rightPixel;
    rightPixel = *(blockRightPixelPtr++);
    dIR = rightPixel - centerPixel;
    dIB = *(blockNextLinePixelPtr++) - centerPixel;

    rightWeight  = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];

    rightWeightTimesdIR = rightWeight*dIR;
    bottomWeightTimesdIB = bottomWeight*dIB;

    currentPixelSumWeights = centerWeight + rightPixelSumWeights + rightWeight + bottomWeight;
    currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB - rightPixelDeltaSum;

    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelDeltaSum = rightWeightTimesdIR; //next pixel to the right

    *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
    *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom

    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

  }

  // C pixel. uses filter type xx
  //                            x
  //
  // No information from previous row
  // Information reused from previous pixel
  // Storing information to next row
  // No information to store to next pixel

  centerPixel = rightPixel;
  blockRightPixelPtr++;
  dIB = *(blockNextLinePixelPtr++) - centerPixel;

  bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
  bottomWeightTimesdIB = bottomWeight*dIB;

  currentPixelSumWeights = centerWeight + rightPixelSumWeights + bottomWeight;
  currentPixelDeltaSum = bottomWeightTimesdIB - rightPixelDeltaSum;

  *(sumWeightsPtr) = bottomWeight; //next pixel to the bottom
  *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom

  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

  for (Int j = 1; j < (uiHeight-1); j++)
  {
    sumWeightsPtr = sumWeights;
    sumDeltaPtr = sumDelta;

    //                           x
    // D pixel. uses filter type xx
    //                           x
    //
    // Uses information from previous row
    // No information from previous pixel
    // Storing information to next row
    // Storing information to next pixel

    centerPixel = *(blockCurrentPixelPtr);
    rightPixel = *(blockRightPixelPtr++);
    dIR = rightPixel - centerPixel;
    dIB = *(blockNextLinePixelPtr++) - centerPixel;

    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];

    rightWeightTimesdIR = rightWeight*dIR;
    bottomWeightTimesdIB = bottomWeight*dIB;

    currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightWeight + bottomWeight;
    currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB - *(sumDeltaPtr);

    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelDeltaSum = rightWeightTimesdIR;

    *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
    *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom

    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

    for (Int i = 1; i < (uiWidth-1); i++)
    {
      //                            x
      // E pixel. uses filter type xxx
      //                            x
      //
      // Uses information from previous row
      // Uses information from previous pixel
      // Storing information to next row
      // No information to store to next pixel

      centerPixel = rightPixel;
      rightPixel = *(blockRightPixelPtr++);
      dIR = rightPixel - centerPixel;
      dIB = *(blockNextLinePixelPtr++) - centerPixel;

      rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
      bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];

      rightWeightTimesdIR = rightWeight*dIR;
      bottomWeightTimesdIB = bottomWeight*dIB;

      currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights + rightWeight + bottomWeight;
      currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB - rightPixelDeltaSum - *(sumDeltaPtr);

      rightPixelSumWeights = rightWeight; //next pixel to the right
      rightPixelDeltaSum = rightWeightTimesdIR;

      *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
      *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom

      mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
      mySign = 1 | mySignIfNeg;

      *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
    }

    //                            x
    // F pixel. uses filter type xx
    //                            x
    //
    // Uses information from previous row
    // Uses information from previous pixel
    // Storing information to next row
    // Storing information to next pixel

    centerPixel = rightPixel;
    blockRightPixelPtr++;
    dIB = *(blockNextLinePixelPtr++) - centerPixel;

    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
    bottomWeightTimesdIB = bottomWeight*dIB;

    currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights + bottomWeight;
    currentPixelDeltaSum = bottomWeightTimesdIB - rightPixelDeltaSum - *(sumDeltaPtr);

    *(sumWeightsPtr) = bottomWeight; //next pixel to the bottom
    *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom

    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

  }

  sumWeightsPtr = sumWeights;
  sumDeltaPtr = sumDelta;

  //                           x
  // G pixel. uses filter type xx
  //
  // Uses information from previous row
  // No information from previous pixel
  // No information to store to next row
  // Storing information to next pixel


  centerPixel = *(blockCurrentPixelPtr);
  rightPixel = *(blockRightPixelPtr++);
  dIR = rightPixel - centerPixel;

  rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
  rightWeightTimesdIR = rightWeight*dIR;

  currentPixelSumWeights = centerWeight + *(sumWeightsPtr++) + rightWeight;
  currentPixelDeltaSum = rightWeightTimesdIR - *(sumDeltaPtr++);

  rightPixelSumWeights = rightWeight; //next pixel to the right
  rightPixelDeltaSum = rightWeightTimesdIR;

  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

  for (Int i = 1; i < (uiWidth-1); i++)
  {
    //                            x
    // H pixel. uses filter type xxx
    //
    // Uses information from previous row
    // Uses information from previous pixel
    // No information to store to next row
    // Storing information to next pixel

    centerPixel = rightPixel;
    rightPixel = *(blockRightPixelPtr++);
    dIR = rightPixel - centerPixel;

    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
    rightWeightTimesdIR = rightWeight*dIR;

    currentPixelSumWeights = centerWeight + *(sumWeightsPtr++) + rightWeight + rightPixelSumWeights;
    currentPixelDeltaSum = rightWeightTimesdIR - rightPixelDeltaSum - *(sumDeltaPtr++);

    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelDeltaSum = rightWeightTimesdIR;

    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

  }

  //                            x
  // I pixel. uses filter type xx
  //
  // Uses information from previous row
  // Uses information from previous pixel
  // No information to store to next row
  // No information to store to nex pixel

  centerPixel = rightPixel;

  currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights;
  currentPixelDeltaSum = - rightPixelDeltaSum - *(sumDeltaPtr);

  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));

}

void BilateralFilter::bilateralFilterIntra( PelBuf& recoBuf, int qp)
{
  const unsigned uiWidth  = recoBuf.width;
  const unsigned uiHeight = recoBuf.height;
  const unsigned uiStride = recoBuf.stride;
  Pel             *piReco = recoBuf.buf;

  if( recoBuf.stride == recoBuf.width )
  {
    smoothBlockBilateralFilter(uiWidth, uiHeight, piReco, 0, qp);
  }
  else
  {
    for( unsigned j = 0; j < uiHeight; j++)
    {
      memcpy(tempblock + j * uiWidth, piReco + j * uiStride, uiWidth * sizeof(Pel));
    }

    smoothBlockBilateralFilter(uiWidth, uiHeight, tempblock, 0, qp);

    for( unsigned j = 0; j < uiHeight; j++)
    {
      memcpy(piReco + j * uiStride, tempblock + j * uiWidth, uiWidth * sizeof(Pel));
    }
  }
}

void BilateralFilter::bilateralFilterInter( PelBuf& resiBuf, const CPelBuf& predBuf, int qp, const ClpRng& clpRng)
{
  const unsigned uiWidth      = predBuf.width;
  const unsigned uiHeight     = predBuf.height;
  const unsigned uiPredStride = predBuf.stride;
  const unsigned uiStrideRes  = resiBuf.stride;
  const Pel *piPred           = predBuf.buf;
        Pel *piResi           = resiBuf.buf;

  for( unsigned  uiY = 0; uiY < uiHeight; ++uiY)
  {
    for( unsigned uiX = 0; uiX < uiWidth; ++uiX)
    {
      tempblock[ uiX + uiY*uiWidth ] = ClipPel( piPred[uiX] + piResi[uiX], clpRng);
    }
    piPred += uiPredStride;
    piResi += uiStrideRes;
  }

  smoothBlockBilateralFilter(uiWidth, uiHeight, tempblock, 1, qp);

  piPred = predBuf.buf;
  piResi = resiBuf.buf;

  // need to be performed if residual  is used
  // Resi' = Reco' - Pred
  for (UInt uiY = 0; uiY < uiHeight; ++uiY)
  {
    for (UInt uiX = 0; uiX < uiWidth; ++uiX)
    {
      piResi[uiX] = tempblock[ uiX + uiY*uiWidth ] - piPred[uiX];
    }
    piPred += uiPredStride;
    piResi += uiStrideRes;
  }
}

#endif
