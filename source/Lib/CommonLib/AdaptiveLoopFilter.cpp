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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "AdaptiveLoopFilter.h"

#if JVET_K0371_ALF
#include "CodingStructure.h"
#include "Picture.h"

AdaptiveLoopFilter::AdaptiveLoopFilter()
  : m_classifier( nullptr )
{
  for( int i = 0; i < NUM_DIRECTIONS; i++ )
  {
    m_laplacian[i] = nullptr;
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = nullptr;
  }

  m_deriveClassificationBlk = deriveClassificationBlk;
  m_filter5x5Blk = filterBlk<ALF_FILTER_5>;
  m_filter7x7Blk = filterBlk<ALF_FILTER_7>;

#if ENABLE_SIMD_OPT_ALF
#ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
#endif
#endif
}

void AdaptiveLoopFilter::ALFProcess( CodingStructure& cs, AlfSliceParam& alfSliceParam )
{
  if( !alfSliceParam.enabledFlag[COMPONENT_Y] && !alfSliceParam.enabledFlag[COMPONENT_Cb] && !alfSliceParam.enabledFlag[COMPONENT_Cr] )
  {
    return;
  }

  // set available filter shapes
  alfSliceParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU enable flags
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
  }
  reconstructCoeff( alfSliceParam, CHANNEL_TYPE_LUMA );
  reconstructCoeff( alfSliceParam, CHANNEL_TYPE_CHROMA );

  PelUnitBuf recYuv = cs.getRecoBuf();
  m_tempBuf.copyFrom( recYuv );
  PelUnitBuf tmpYuv = m_tempBuf.getBuf( cs.area );
  tmpYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );

  const PreCalcValues& pcv = *cs.pcv;

  int ctuIdx = 0;
  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area( xPos, yPos, width, height ) );
      if( m_ctuEnableFlag[COMPONENT_Y][ctuIdx] )
      {
        Area blk( xPos, yPos, width, height );
        deriveClassification( m_classifier, tmpYuv.get( COMPONENT_Y ), blk );

        if( alfSliceParam.lumaFilterType == ALF_FILTER_5 )
        {
          m_filter5x5Blk( m_classifier, recYuv, tmpYuv, blk, COMPONENT_Y, m_coeffFinal, m_clpRngs.comp[COMPONENT_Y] );
        }
        else if( alfSliceParam.lumaFilterType == ALF_FILTER_7 )
        {
          m_filter7x7Blk( m_classifier, recYuv, tmpYuv, blk, COMPONENT_Y, m_coeffFinal, m_clpRngs.comp[COMPONENT_Y] );
        }
        else
        {
          CHECK( 0, "Wrong ALF filter type" );
        }
      }

      for( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
      {
        ComponentID compID = ComponentID( compIdx );
        const int chromaScaleX = getComponentScaleX( compID, tmpYuv.chromaFormat );
        const int chromaScaleY = getComponentScaleY( compID, tmpYuv.chromaFormat );

        if( m_ctuEnableFlag[compIdx][ctuIdx] )
        {
          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY );

          m_filter5x5Blk( m_classifier, recYuv, tmpYuv, blk, compID, alfSliceParam.chromaCoeff, m_clpRngs.comp[compIdx] );
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilter::reconstructCoeff( AlfSliceParam& alfSliceParam, ChannelType channel, const bool bRedo )
{
  int factor = ( 1 << ( m_NUM_BITS - 1 ) );

  AlfFilterType filterType = isLuma( channel ) ? alfSliceParam.lumaFilterType : ALF_FILTER_5;
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int numCoeff = filterType == ALF_FILTER_5 ? 7 : 13;
  int numCoeffMinus1 = numCoeff - 1;
  int numFilters = isLuma( channel ) ? alfSliceParam.numLumaFilters : 1;
  short* coeff = isLuma( channel ) ? alfSliceParam.lumaCoeff : alfSliceParam.chromaCoeff;

  if( alfSliceParam.coeffDeltaPredModeFlag && isLuma( channel ) )
  {
    for( int i = 1; i < numFilters; i++ )
    {
      for( int j = 0; j < numCoeffMinus1; j++ )
      {
        coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += coeff[( i - 1 ) * MAX_NUM_ALF_LUMA_COEFF + j];
      }
    }
  }

  for( int filterIdx = 0; filterIdx < numFilters; filterIdx++ )
  {
    int sum = 0;
    for( int i = 0; i < numCoeffMinus1; i++ )
    {
      sum += ( coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + i] << 1 );
    }
    coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor - sum;
  }

  if( isChroma( channel ) )
  {
    return;
  }

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    int filterIdx = alfSliceParam.filterCoeffDeltaIdx[classIdx];
    memcpy( m_coeffFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF, coeff + filterIdx * MAX_NUM_ALF_LUMA_COEFF, sizeof( int16_t ) * numCoeff );
  }

  if( bRedo && alfSliceParam.coeffDeltaPredModeFlag )
  {
    for( int i = numFilters - 1; i > 0; i-- )
    {
      for( int j = 0; j < numCoeffMinus1; j++ )
      {
        coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] = coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] - coeff[( i - 1 ) * MAX_NUM_ALF_LUMA_COEFF + j];
      }
    }
  }
}

void AdaptiveLoopFilter::create( const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  std::memcpy( m_inputBitDepth, inputBitDepth, sizeof( m_inputBitDepth ) );
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_maxCUDepth = maxCUDepth;
  m_chromaFormat = format;

  m_numCTUsInWidth = ( m_picWidth / m_maxCUWidth ) + ( ( m_picWidth % m_maxCUWidth ) ? 1 : 0 );
  m_numCTUsInHeight = ( m_picHeight / m_maxCUHeight ) + ( ( m_picHeight % m_maxCUHeight ) ? 1 : 0 );
  m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;

  m_filterShapes[CHANNEL_TYPE_LUMA].push_back( AlfFilterShape( 5 ) );
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back( AlfFilterShape( 7 ) );
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back( AlfFilterShape( 5 ) );

  m_tempBuf.destroy();
  m_tempBuf.create( format, Area( 0, 0, picWidth, picHeight ), maxCUWidth, MAX_ALF_FILTER_LENGTH >> 1, 0, false );

  // Laplacian based activity
  for( int i = 0; i < NUM_DIRECTIONS; i++ )
  {
    m_laplacian[i] = new int*[m_CLASSIFICATION_BLK_SIZE + 5];

    for( int y = 0; y < m_CLASSIFICATION_BLK_SIZE + 5; y++ )
    {
      m_laplacian[i][y] = new int[m_CLASSIFICATION_BLK_SIZE + 5];
    }
  }

  // Classification
  m_classifier = new AlfClassifier*[picHeight];
  for( int i = 0; i < picHeight; i++ )
  {
    m_classifier[i] = new AlfClassifier[picWidth];
  }
}

void AdaptiveLoopFilter::destroy()
{
  for( int i = 0; i < NUM_DIRECTIONS; i++ )
  {
    if( m_laplacian[i] )
    {
      for( int y = 0; y < m_CLASSIFICATION_BLK_SIZE + 5; y++ )
      {
        delete[] m_laplacian[i][y];
        m_laplacian[i][y] = nullptr;
      }

      delete[] m_laplacian[i];
      m_laplacian[i] = nullptr;
    }
  }

  if( m_classifier )
  {
    for( int i = 0; i < m_picHeight; i++ )
    {
      delete[] m_classifier[i];
      m_classifier[i] = nullptr;
    }

    delete[] m_classifier;
    m_classifier = nullptr;
  }

  m_tempBuf.destroy();
}

void AdaptiveLoopFilter::deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blk )
{
  int height = blk.pos().y + blk.height;
  int width = blk.pos().x + blk.width;

  for( int i = blk.pos().y; i < height; i += m_CLASSIFICATION_BLK_SIZE )
  {
    int nHeight = std::min( i + m_CLASSIFICATION_BLK_SIZE, height ) - i;

    for( int j = blk.pos().x; j < width; j += m_CLASSIFICATION_BLK_SIZE )
    {
      int nWidth = std::min( j + m_CLASSIFICATION_BLK_SIZE, width ) - j;

      m_deriveClassificationBlk( classifier, m_laplacian, srcLuma, Area( j, i, nWidth, nHeight ), m_inputBitDepth[CHANNEL_TYPE_LUMA] + 4 );
    }
  }
}

void AdaptiveLoopFilter::deriveClassificationBlk( AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift )
{
  static const int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  const int stride = srcLuma.stride;
  const Pel* src = srcLuma.buf;
  const int maxActivity = 15;

  int fl = 2;
  int flP1 = fl + 1;
  int fl2 = 2 * fl;

  int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  int pixY;
  int height = blk.height + fl2;
  int width = blk.width + fl2;
  int posX = blk.pos().x;
  int posY = blk.pos().y;
  int startHeight = posY - flP1;

  for( int i = 0; i < height; i += 2 )
  {
    int yoffset = ( i + 1 + startHeight ) * stride - flP1;
    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];

    int* pYver = laplacian[VER][i];
    int* pYhor = laplacian[HOR][i];
    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig1 = laplacian[DIAG1][i];

    for( int j = 0; j < width; j += 2 )
    {
      pixY = j + 1 + posX;
      const Pel *pY = src1 + pixY;
      const Pel* pYdown = src0 + pixY;
      const Pel* pYup = src2 + pixY;
      const Pel* pYup2 = src3 + pixY;

      const Pel y0 = pY[0] << 1;
      const Pel y1 = pY[1] << 1;
      const Pel yup0 = pYup[0] << 1;
      const Pel yup1 = pYup[1] << 1;

      pYver[j] = abs( y0 - pYdown[0] - pYup[0] ) + abs( y1 - pYdown[1] - pYup[1] ) + abs( yup0 - pY[0] - pYup2[0] ) + abs( yup1 - pY[1] - pYup2[1] );
      pYhor[j] = abs( y0 - pY[1] - pY[-1] ) + abs( y1 - pY[2] - pY[0] ) + abs( yup0 - pYup[1] - pYup[-1] ) + abs( yup1 - pYup[2] - pYup[0] );
      pYdig0[j] = abs( y0 - pYdown[-1] - pYup[1] ) + abs( y1 - pYdown[0] - pYup[2] ) + abs( yup0 - pY[-1] - pYup2[1] ) + abs( yup1 - pY[0] - pYup2[2] );
      pYdig1[j] = abs( y0 - pYup[-1] - pYdown[1] ) + abs( y1 - pYup[0] - pYdown[2] ) + abs( yup0 - pYup2[-1] - pY[1] ) + abs( yup1 - pYup2[0] - pY[2] );

      if( j > 4 && ( j - 6 ) % 4 == 0 )
      {
        int jM6 = j - 6;
        int jM4 = j - 4;
        int jM2 = j - 2;

        pYver[jM6] += pYver[jM4] + pYver[jM2] + pYver[j];
        pYhor[jM6] += pYhor[jM4] + pYhor[jM2] + pYhor[j];
        pYdig0[jM6] += pYdig0[jM4] + pYdig0[jM2] + pYdig0[j];
        pYdig1[jM6] += pYdig1[jM4] + pYdig1[jM2] + pYdig1[j];
      }
    }
  }

  // classification block size
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  for( int i = 0; i < blk.height; i += clsSizeY )
  {
    int* pYver = laplacian[VER][i];
    int* pYver2 = laplacian[VER][i + 2];
    int* pYver4 = laplacian[VER][i + 4];
    int* pYver6 = laplacian[VER][i + 6];

    int* pYhor = laplacian[HOR][i];
    int* pYhor2 = laplacian[HOR][i + 2];
    int* pYhor4 = laplacian[HOR][i + 4];
    int* pYhor6 = laplacian[HOR][i + 6];

    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig02 = laplacian[DIAG0][i + 2];
    int* pYdig04 = laplacian[DIAG0][i + 4];
    int* pYdig06 = laplacian[DIAG0][i + 6];

    int* pYdig1 = laplacian[DIAG1][i];
    int* pYdig12 = laplacian[DIAG1][i + 2];
    int* pYdig14 = laplacian[DIAG1][i + 4];
    int* pYdig16 = laplacian[DIAG1][i + 6];

    for( int j = 0; j < blk.width; j += clsSizeX )
    {
      int sumV = pYver[j] + pYver2[j] + pYver4[j] + pYver6[j];
      int sumH = pYhor[j] + pYhor2[j] + pYhor4[j] + pYhor6[j];
      int sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j] + pYdig06[j];
      int sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j] + pYdig16[j];

      int tempAct = sumV + sumH;
      int activity = (Pel)Clip3<int>( 0, maxActivity, ( tempAct * 32 ) >> shift );
      int classIdx = th[activity];

      int hv1, hv0, d1, d0, hvd1, hvd0;

      if( sumV > sumH )
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }
      if( sumD0 > sumD1 )
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }
      if( d1*hv0 > hv1*d0 )
      {
        hvd1 = d1;
        hvd0 = d0;
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        hvd1 = hv1;
        hvd0 = hv0;
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
      }

      int directionStrength = 0;
      if( hvd1 > 2 * hvd0 )
      {
        directionStrength = 1;
      }
      if( hvd1 * 2 > 9 * hvd0 )
      {
        directionStrength = 2;
      }

      if( directionStrength )
      {
        classIdx += ( ( ( mainDirection & 0x1 ) << 1 ) + directionStrength ) * 5;
      }

      static const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + ( secondaryDirection >> 1 )];

      int yOffset = i + posY;
      int xOffset = j + posX;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;
      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] = cl3[0] = cl3[1] = cl3[2] = cl3[3] = AlfClassifier( classIdx, transposeIdx );
    }
  }
}

template<AlfFilterType filtType>
void AdaptiveLoopFilter::filterBlk( AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng )
{
  const bool bChroma = isChroma( compId );
  if( bChroma )
  {
    CHECK( filtType != 0, "Chroma needs to have filtType == 0" );
  }

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + startHeight * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

  short *coef = filterSet;

  const int shift = 9;
  const int offset = 1 << ( shift - 1 );

  int transposeIdx = 0;
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  std::vector<Pel> filterCoeff( MAX_NUM_ALF_LUMA_COEFF );

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;

  Pel* pRec0 = dst + startWidth;
  Pel* pRec1 = pRec0 + dstStride;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[startHeight + i] + startWidth;
    }

    for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
    {
      if( !bChroma )
      {
        AlfClassifier& cl = pClass[j];
        transposeIdx = cl.transposeIdx;
        coef = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      }

      if( filtType == ALF_FILTER_7 )
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10], coef[11], coef[12] };
        }
      }
      else
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[4], coef[1], coef[5], coef[3], coef[0], coef[2], coef[6] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[4], coef[5], coef[6] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[4], coef[3], coef[5], coef[1], coef[0], coef[2], coef[6] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6] };
        }
      }

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;
        pImg5 = pImgYPad5 + j + ii * srcStride;
        pImg6 = pImgYPad6 + j + ii * srcStride;

        pRec1 = pRec0 + j + ii * dstStride;

        for( int jj = 0; jj < clsSizeX; jj++ )
        {
          int sum = 0;
          if( filtType == ALF_FILTER_7 )
          {
            sum += filterCoeff[0] * ( pImg5[0] + pImg6[0] );

            sum += filterCoeff[1] * ( pImg3[+1] + pImg4[-1] );
            sum += filterCoeff[2] * ( pImg3[+0] + pImg4[+0] );
            sum += filterCoeff[3] * ( pImg3[-1] + pImg4[+1] );

            sum += filterCoeff[4] * ( pImg1[+2] + pImg2[-2] );
            sum += filterCoeff[5] * ( pImg1[+1] + pImg2[-1] );
            sum += filterCoeff[6] * ( pImg1[+0] + pImg2[+0] );
            sum += filterCoeff[7] * ( pImg1[-1] + pImg2[+1] );
            sum += filterCoeff[8] * ( pImg1[-2] + pImg2[+2] );

            sum += filterCoeff[9] * ( pImg0[+3] + pImg0[-3] );
            sum += filterCoeff[10] * ( pImg0[+2] + pImg0[-2] );
            sum += filterCoeff[11] * ( pImg0[+1] + pImg0[-1] );
            sum += filterCoeff[12] * ( pImg0[+0] );
          }
          else
          {
            sum += filterCoeff[0] * ( pImg3[+0] + pImg4[+0] );

            sum += filterCoeff[1] * ( pImg1[+1] + pImg2[-1] );
            sum += filterCoeff[2] * ( pImg1[+0] + pImg2[+0] );
            sum += filterCoeff[3] * ( pImg1[-1] + pImg2[+1] );

            sum += filterCoeff[4] * ( pImg0[+2] + pImg0[-2] );
            sum += filterCoeff[5] * ( pImg0[+1] + pImg0[-1] );
            sum += filterCoeff[6] * ( pImg0[+0] );
          }

          sum = ( sum + offset ) >> shift;
          pRec1[jj] = ClipPel( sum, clpRng );

          pImg0++;
          pImg1++;
          pImg2++;
          pImg3++;
          pImg4++;
          pImg5++;
          pImg6++;
        }
      }
    }

    pRec0 += dstStride2;
    pRec1 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}
#elif JEM_TOOLS

#include "UnitTools.h"

#include "dtrace_next.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

ALFParam::ALFParam()
{
  alf_flag          = 0;

  filterMode        = ALF_ONE_FILTER;
  startSecondFilter = 0;

  filterType        = ALF_FILTER_SYM_9;
  tapH              = 0;
  tapV              = 0;
  num_coeff         = 0;
  coeffmulti        = nullptr;
  alfCoeffLuma      = nullptr;
  alfCoeffChroma    = nullptr;

  chroma_idc        = 0;
  tap_chroma        = 0;
  num_coeff_chroma  = 0;
  coeff_chroma      = nullptr;

  cu_control_flag   = 0;
  alf_max_depth     = 0;
  num_alf_cu_flag   = 0;
  alf_cu_flag       = nullptr;

  //Coding related
  num_ctus_in_frame = 0;
  maxCodingDepth    = 0; //TODO "

  filters_per_group_diff = 0;
  filters_per_group      = 0;
  forceCoeff0            = 0;

  predMethod        = 0;
  minKStart         = 0;
  maxScanVal        = 0;

  temporalPredFlag  = 0;
  prevIdx           = 0;
};

ALFParam::~ALFParam()
{
 destroy();
}

void ALFParam::destroy()
{
   if ( coeff_chroma != nullptr)
   {
    delete[] coeff_chroma;
    coeff_chroma = nullptr;
   }

   if( coeffmulti != nullptr )
   {
    for (int i=0; i < AdaptiveLoopFilter::m_NO_VAR_BINS; i++)
    {
      delete[] coeffmulti[i];
      coeffmulti[i] = nullptr;
    }
    delete[] coeffmulti;
    coeffmulti = nullptr;
   }

   if( alf_cu_flag != nullptr)
   {
    delete[] alf_cu_flag;
    alf_cu_flag = nullptr;
   }
}


void ALFParam::reset()
{
  alf_flag         = 0;
  cu_control_flag  = 0;
  chroma_idc       = 0;
  num_coeff_chroma = 0;
#if GALF
  tap_chroma       = 5;
#if JVET_C0038_NO_PREV_FILTERS
  iAvailableFilters = JVET_C0038_NO_PREV_FILTERS;
#endif
#else
  tap_chroma = 0;
#endif
  ::memset(coeff_chroma, 0, sizeof(int)*AdaptiveLoopFilter::m_ALF_MAX_NUM_COEF_C);
  for (int i=0; i< AdaptiveLoopFilter::m_NO_VAR_BINS; i++)
  {
    ::memset(coeffmulti[i], 0, sizeof(int)*AdaptiveLoopFilter::m_ALF_MAX_NUM_COEF );
  }
  num_alf_cu_flag  = 0;
  ::memset( kMinTab , 0 , sizeof( kMinTab ) );

  temporalPredFlag  = 0;
  prevIdx           = 0;
}

void ALFParam::copyFrom(const ALFParam& src, const bool isGALF, bool max_depth_copy)
{
  alf_flag = src.alf_flag;
  cu_control_flag = src.cu_control_flag;
  chroma_idc = src.chroma_idc;

  tapH = src.tapH;
  tapV = src.tapV;
  num_coeff = src.num_coeff;
  tap_chroma = src.tap_chroma;
  num_coeff_chroma = src.num_coeff_chroma;

  ::memcpy(coeff_chroma, src.coeff_chroma, sizeof(int) * AdaptiveLoopFilter::m_ALF_MAX_NUM_COEF_C);
  filterType = src.filterType;
  ::memcpy(filterPattern, src.filterPattern, sizeof(int) * AdaptiveLoopFilter::m_NO_VAR_BINS);
  startSecondFilter = src.startSecondFilter;
  filterMode = src.filterMode;

  //Coeff send related
  filters_per_group_diff = src.filters_per_group_diff; //this can be updated using codedVarBins
  filters_per_group = src.filters_per_group; //this can be updated using codedVarBinsif


#ifdef FORCE0
  if( ! isGALF || FORCE0 )
#else
  if( ! isGALF )
#endif
  {
    ::memcpy(codedVarBins, src.codedVarBins, sizeof(int) * AdaptiveLoopFilter::m_NO_VAR_BINS);
    forceCoeff0 = src.forceCoeff0;
  }
#if JVET_C0038_NO_PREV_FILTERS
  iAvailableFilters = src.iAvailableFilters;
  iPredPattern = src.iPredPattern;
  ::memcpy(PrevFiltIdx, src.PrevFiltIdx, sizeof(int) * AdaptiveLoopFilter::m_NO_VAR_BINS);
#endif
  predMethod = src.predMethod;
  for (int i=0; i<AdaptiveLoopFilter::m_NO_VAR_BINS; i++)
  {
    ::memcpy(coeffmulti[i], src.coeffmulti[i], sizeof(int) * AdaptiveLoopFilter::m_ALF_MAX_NUM_COEF);
    // galf stuff
    ::memcpy(alfCoeffLuma[i], src.alfCoeffLuma[i], sizeof(int) * AdaptiveLoopFilter::m_ALF_MAX_NUM_COEF);
  }
  minKStart = src.minKStart;
  ::memcpy( kMinTab , src.kMinTab , sizeof( src.kMinTab ) );

  ::memcpy( mapClassToFilter , src.mapClassToFilter , sizeof( src.mapClassToFilter ) );

#if COM16_C806_ALF_TEMPPRED_NUM
  if (max_depth_copy)
  {
#endif
    alf_max_depth = src.alf_max_depth;
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
  ::memcpy(alfCoeffChroma, src.alfCoeffChroma, sizeof(int) * AdaptiveLoopFilter::m_ALF_MAX_NUM_COEF_C);
#endif
  num_alf_cu_flag = src.num_alf_cu_flag;
  ::memcpy(alf_cu_flag, src.alf_cu_flag, sizeof(bool) * src.num_alf_cu_flag);

  temporalPredFlag = src.temporalPredFlag;
  prevIdx          = src.prevIdx;
}

// ====================================================================================================================
// Tables
// ====================================================================================================================

#if JVET_C0038_NO_PREV_FILTERS
const int AdaptiveLoopFilter::m_ALFfilterCoeffFixed[m_NO_FILTERS*JVET_C0038_NO_PREV_FILTERS][21] =
{
  {0, -6, 3, -6, -8, -2, 7, 0, -5, -10, -1, 28, 64, 36, 4, -2, -1, 7, 15, 68, 130},
  {-6, -5, 6, -4, -5, 1, 10, 4, 3, -7, 4, 24, 53, 34, 11, 2, -9, 8, 20, 61, 102},
  {-8, -10, 13, -4, -13, 12, 19, 14, 0, -11, 15, 24, 38, 31, 20, 7, -9, 19, 27, 42, 60},
  {4, 8, -21, 11, 0, -18, -1, -21, -1, 9, -14, 32, 83, 37, -12, 12, 8, -28, -3, 86, 170},
  {-13, -6, 19, 0, -7, 16, 16, 19, 14, -8, 17, 16, 23, 25, 25, 18, -11, 25, 25, 27, 32},
  {-17, 1, 17, 13, 5, 13, 9, 10, 40, -3, 19, 15, 13, 14, 17, 37, -14, 18, 18, 21, 20},
  {13, -7, -1, -1, 22, -24, -2, -32, 28, -5, -26, 7, 88, 9, -34, 2, 18, -3, -5, 94, 230},
  {-2, -11, 21, -10, -19, 8, 17, 14, -15, -19, 7, 19, 44, 34, 18, -1, -3, 29, 28, 52, 90},
  {-12, -9, 9, -13, 0, 10, 19, 17, -3, -12, 13, 20, 46, 31, 19, -9, -19, 14, 28, 52, 110},
  {-5, 0, 2, -9, 8, -5, -8, -9, 28, -3, -4, 12, 58, 46, 7, -7, -8, -1, 2, 71, 162},
  {-23, -9, 23, -15, -6, 20, 27, 28, -5, -11, 23, 21, 30, 32, 33, -4, -34, 31, 36, 36, 46},
  {-27, -8, 28, -10, -2, 25, 25, 31, 3, -13, 29, 17, 19, 30, 37, 2, -34, 33, 37, 25, 18},
  {-72, 20, 38, 2, 25, 19, 28, 31, 50, 6, 23, 0, 3, 12, 38, 18, -84, 54, 33, 8, 8},
  {-7, 3, 12, 3, -4, -10, -2, -13, 5, 2, -8, 17, 64, 28, -3, 4, -6, 4, 9, 81, 154},
  {-5, -5, 12, -5, 2, 1, -9, 0, 3, -8, 3, 1, 55, 12, 4, -3, -5, 14, -12, 71, 260},
  {-7, 0, 7, -2, -5, -7, 8, 0, 3, -5, 0, 20, 56, 29, 7, 4, -8, 6, 21, 67, 124},
  {2, 1, -8, -1, -3, -14, -1, -11, 0, 0, -12, 30, 80, 38, -10, 3, 2, -11, 4, 83, 168},
  {-1, -5, 5, -8, 0, -4, -2, 2, -3, -7, -5, 11, 65, 27, 7, -9, -2, 4, 8, 77, 192},
  {-3, -6, 6, -11, -5, 2, 3, 5, -3, -6, 2, 19, 61, 41, 16, -7, -6, 0, 15, 73, 120},
  {-3, -8, 12, -14, -8, 5, 2, 13, -5, -8, 4, 15, 53, 39, 25, -8, -6, 5, 21, 66, 112},
  {-1, -12, 11, -17, -19, 10, 15, 21, -15, -18, 10, 25, 52, 36, 26, -11, -6, 20, 27, 59, 86},
  {-6, -11, 5, -9, -15, 14, 11, 17, 10, -15, 17, 28, 41, 37, 22, 7, -10, 10, 25, 49, 58},
  {-1, 1, 5, 5, 4, -16, -10, -25, 10, 2, -19, 11, 86, 19, -26, 7, 1, 4, -11, 93, 232},
  {-2, -8, 10, -2, -11, 10, -1, 3, -14, -6, 8, -3, 49, 22, 21, -2, -1, 5, 0, 80, 196},
  {-6, -12, 11, -18, -14, 15, 14, 25, -2, -16, 17, 24, 41, 37, 32, -6, -12, 15, 28, 51, 64},
  {-1, -5, 9, -15, -7, -3, 9, 14, -17, -13, -2, 16, 59, 34, 22, -18, -4, 13, 23, 71, 142},
  {-2, -4, 10, -6, 4, -3, -14, -3, 4, -5, -3, -5, 61, 10, 1, -6, -2, 10, -15, 74, 300},
  {-8, -2, 1, -30, 8, 4, 3, 28, 22, -6, 8, 18, 44, 51, 38, -19, -17, -1, 18, 56, 80},
  {-44, -26, 72, -33, -17, 46, 14, 65, -21, -20, 45, 9, 25, 17, 65, -20, -64, 89, 13, 24, 34},
  {-1, -6, 8, -4, 0, 8, -4, 9, 0, -8, 12, -8, 1, -7, 14, -3, -3, 16, -17, 16, 466},
  {-4, -2, 16, -6, -3, 2, -16, -17, 10, 0, 0, -7, 46, 40, -3, -6, -4, 11, -22, 79, 284},
  {-1, -3, 7, -18, 6, 8, -28, -12, 25, -1, 6, -2, 52, 73, 15, -16, -4, -1, -23, 71, 204},
  {-4, -1, 15, 2, 3, -2, -34, -13, 3, 0, -6, -2, 72, 19, -10, 1, -3, 14, -37, 84, 310},
  {-3, 1, 12, 0, 4, -13, -13, -17, 4, 4, -15, -5, 72, 17, -12, 0, -2, 3, -15, 97, 274},
  {2, 2, -3, -3, -2, -16, -1, -13, -5, 1, -18, 25, 72, 47, -3, -5, 3, -12, 11, 86, 176},
  {-2, 3, 7, 1, 1, -17, -9, -20, -1, 5, -20, 12, 70, 39, -13, -1, 0, -1, -6, 89, 238},
  {-2, -3, 7, -13, -2, 2, -16, 6, -1, -4, 1, 5, 50, 57, 24, -13, -4, 2, -6, 69, 194},
  {-1, -2, 5, 3, 2, -3, -7, -16, 3, -1, -6, 2, 33, 20, -11, 1, 0, 7, -17, 52, 384},
  {2, 9, -8, 12, 4, -30, -4, -39, 9, 10, -31, 32, 88, 45, -38, 12, 5, -14, 0, 86, 212},
  {13, 15, -39, 19, 1, -44, 30, -54, 7, 14, -40, 51, 90, 60, -50, 20, 17, -45, 32, 84, 150},
  {6, 2, -10, 1, -9, -18, 10, -20, 0, 3, -17, 34, 73, 51, -15, 4, 8, -20, 19, 79, 150},
  {-3, -2, 11, -1, 4, -5, -14, -11, 4, -1, -6, -10, 56, 8, -7, -2, -3, 12, -21, 71, 352},
  {-2, -3, 7, -2, 0, 4, -9, 2, -2, -4, 3, -3, 14, 9, 5, -4, -3, 12, -16, 26, 444},
  {0, -2, 5, -4, 1, -3, 3, -1, -2, -5, -2, 2, 5, 14, 5, -6, 0, 6, 1, 13, 452},
  {2, -5, 1, -5, -1, 5, 3, 7, 0, -6, 7, 5, -11, 3, 9, -3, 2, 2, 5, -11, 494},
  {-4, -2, 10, 0, -1, 8, -22, -13, 5, 0, 5, -11, 33, 28, -2, 0, -8, 17, -31, 31, 426},
  {-3, -2, 9, -1, 1, 3, -17, -3, 0, -2, 1, -10, 39, 14, 3, -3, -3, 10, -23, 58, 370},
  {-2, 1, -2, 8, -2, 18, 0, -7, -1, -1, 20, -48, 2, 13, 20, 5, -4, 0, -17, 51, 404},
  {-1, -2, 4, 0, 0, 3, -4, -3, 1, -2, 3, -11, 13, 2, 0, -2, -1, 5, -8, 22, 474},
  {-5, 2, 15, 3, 8, -15, -17, -26, 8, 6, -19, -6, 73, 23, -24, 3, -3, 5, -18, 98, 290},
  {-1, 3, 6, 3, 4, -14, -21, -26, -1, 2, -16, 11, 72, 57, -9, -4, 4, -6, -15, 92, 230},
  {2, 10, -1, 0, -1, -33, 1, -25, -4, 12, -35, 36, 74, 62, -15, -5, 6, -24, 22, 91, 166},
  {-4, -2, 13, 3, 3, 1, -20, -15, 4, -1, -2, -15, 47, 14, -10, 0, -5, 16, -28, 66, 382},
  {-4, 6, 10, 9, 7, -24, -23, -37, 11, 8, -30, 31, 80, 48, -39, 10, -2, 5, -17, 80, 254},
  {-1, 10, -4, 13, 6, -33, -4, -45, 9, 12, -37, 37, 85, 54, -44, 13, 3, -14, 3, 88, 210},
  {6, 4, -12, 8, 2, -22, 0, -37, 3, 4, -24, 31, 81, 61, -31, 7, 10, -19, 3, 83, 196},
  {-3, -3, 10, 1, 1, 4, -15, -7, 4, -2, 4, -18, 34, 1, -6, 0, -4, 15, -26, 50, 432},
  {-2, -2, 8, 0, 0, 4, -15, 0, 0, -2, 3, -11, 25, -3, 1, -1, -3, 12, -20, 34, 456},
  {1, -2, 2, -2, 0, 0, 0, 2, -2, -4, 1, 2, -2, 4, 4, -3, 2, 0, 3, 0, 500},
  {-3, -2, 11, 1, 5, -2, -11, -18, 9, 1, -1, -30, 63, -2, -14, 2, -3, 9, -24, 88, 354},
  {-3, -3, 8, 1, 1, 6, -10, -6, 3, -4, 3, -17, 24, 0, 0, -2, -3, 13, -17, 29, 466},
  {-4, -2, 10, 2, 0, 6, -15, -8, 1, -3, 5, -16, 22, 19, -3, -1, -5, 13, -20, 38, 434},
  {-1, -2, 5, 0, 0, 3, -6, -2, 0, -2, 3, -7, 7, 7, -1, -1, -2, 7, -10, 19, 478},
  {-7, 10, 17, 5, 8, -39, -12, -33, 4, 16, -38, 19, 78, 44, -32, 6, 0, -12, 3, 110, 218},
  {-1, -1, 2, 0, 0, 1, -3, 0, 0, -1, 2, -3, 3, 0, 0, 0, -1, 3, -4, 5, 508},
  {-2, -2, 7, 0, -1, 5, -11, 1, 0, -2, 5, -12, 16, -4, 1, 0, -3, 10, -17, 24, 482},
  {-5, 2, 13, 7, 9, -20, -10, -40, 8, 11, -29, 10, 62, 56, -38, 9, 0, -5, -7, 87, 272},
  {-3, 9, 1, 10, 8, -32, -1, -43, 4, 13, -37, 26, 72, 62, -37, 8, 3, -16, 6, 92, 222},
  {-6, -3, 17, 4, 1, 7, -28, -11, 2, 0, 4, -23, 46, 11, -10, 3, -7, 19, -38, 70, 396},
  {-4, -3, 11, 2, 0, 7, -18, -4, 1, -2, 7, -19, 29, -1, -4, 1, -5, 15, -28, 45, 452},
  {-5, -4, 15, 3, 6, -1, -15, -25, 11, 2, -4, -26, 55, 18, -23, 4, -5, 13, -26, 79, 368},
  {-4, -3, 12, 2, 0, 6, -14, -7, 2, -2, 5, -16, 20, 9, -6, 0, -5, 14, -20, 32, 462},
  {-7, 3, 14, 6, 10, -17, -13, -38, 15, 11, -25, -12, 82, 30, -39, 9, -4, 3, -15, 108, 270},
  {-1, -2, 5, 0, -1, 4, -6, 0, 0, -2, 5, -12, 11, -3, 1, -1, -2, 7, -10, 16, 494},
  {-6, -2, 14, 4, 0, 5, -19, -14, 1, -2, 3, -18, 28, 28, -10, -1, -5, 17, -26, 50, 418},
  {-1, -1, 3, 0, 0, 2, -3, 0, 0, -1, 2, -5, 4, -1, 0, 0, -1, 4, -4, 5, 506},
  {-2, -3, 8, 0, 2, 6, -15, -5, 6, -1, 6, -36, 56, -19, -4, 1, -2, 8, -23, 68, 410},
  {0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 0, 0, -1, 1, -1, 1, 512},
  {-3, -2, 7, 1, -1, 4, -8, -1, 0, -2, 4, -9, 8, 2, -1, 0, -4, 9, -10, 14, 496},
  {-4, -4, 11, 1, -2, 10, -13, -4, 0, -1, 8, -25, 21, 3, -3, 1, -6, 12, -15, 30, 472},
  {-2, -5, 15, -24, -4, 18, -15, -27, 41, 1, 9, -4, 43, 81, 28, -15, -4, -4, -22, 65, 162},
  {-2, -1, 0, -9, -5, -5, 8, 5, 19, -10, -3, 21, 49, 37, 19, 15, -9, 0, 16, 58, 106},
  {-7, -5, 12, -9, -7, 10, 8, 6, 38, -14, 9, 10, 34, 33, 26, 28, -12, 8, 8, 42, 76},
  {-17, -7, 19, 8, 2, 22, 1, -1, 71, -21, 18, 11, 15, 12, 20, 55, -17, 15, -2, 28, 48},
  {1, -2, 4, -4, -1, -6, -4, -16, 15, -3, -10, 18, 66, 44, -1, 13, 1, -11, -5, 75, 164},
  {-13, -10, 13, 1, -6, 21, 7, 14, 40, -18, 18, 14, 27, 29, 15, 31, -16, 17, 8, 36, 56},
  {-19, -8, 20, 17, -5, 26, -1, 1, 82, -24, 22, 10, 9, 3, 9, 77, -14, 18, -8, 21, 40},
  {-18, -9, 14, 18, -8, 22, 2, 7, 78, -25, 21, 13, 13, 2, 15, 70, -15, 19, -1, 19, 38},
  {2, -7, 4, 8, 3, -15, -3, -29, 20, -8, -6, 14, 83, 23, -30, 20, 8, -6, -11, 83, 206},
  {-8, -6, 4, -2, -8, 10, 10, 11, 36, -19, 12, 17, 33, 27, 25, 33, -16, 12, 15, 39, 62},
  {-11, -4, 6, -19, 1, 9, 19, 24, -1, -8, 6, 15, 42, 45, 28, -3, -21, 13, 20, 45, 100},
  {-34, -2, 34, -22, -2, 18, 28, 38, 0, -6, 13, 6, 27, 39, 42, 6, -49, 45, 25, 29, 42},
  {-54, 2, 61, -19, 5, 35, 21, 39, 27, -14, 23, -14, 3, 24, 48, 27, -68, 65, 25, 14, 12},
  {-2, -4, 10, -8, 4, 4, -7, -18, 27, -3, 2, -11, 46, 46, 18, 1, -2, -3, -13, 60, 218},
  {-2, 0, 4, -8, -4, -5, 6, 2, 24, -8, -2, 11, 48, 38, 19, 17, -8, -2, 13, 58, 110},
  {-8, 6, -2, -6, 16, 11, -18, -21, 107, 2, 4, -13, 12, 60, 45, 26, -20, -20, -17, 44, 96},
  {0, 1, 6, -12, 1, -8, -7, -7, 8, 3, -14, 4, 64, 42, 13, -1, -1, -8, -7, 77, 204},
  {1, 0, 1, -16, -2, -5, 6, 7, 12, -3, -8, 13, 49, 48, 29, 8, -7, -10, 14, 62, 114},
  {-2, -5, 2, -14, -5, 8, 10, 12, 40, -17, 9, 16, 32, 34, 31, 31, -14, 1, 16, 42, 58},
  {0, 1, 10, -13, -1, -7, -7, -10, 9, 4, -14, 2, 60, 49, 13, 0, -1, -8, -5, 80, 188},
  {2, -3, 5, -15, -2, 0, -2, -11, 18, 2, -6, 7, 56, 53, 12, 4, -3, -12, -2, 71, 164},
  {-1, 0, 11, -5, -1, -10, -9, -24, 14, 5, -12, 1, 73, 36, -10, 8, -1, -3, -15, 85, 228},
  {-1, -2, 0, -19, -4, 1, 12, 16, 33, -13, 3, 16, 34, 43, 33, 23, -14, -4, 20, 47, 64},
  {-4, -1, 3, -18, 0, 4, 8, 2, 63, -19, 6, 18, 25, 33, 24, 58, -16, -11, 7, 43, 62},
  {-2, 1, -1, -29, 6, -7, 17, 29, -5, 0, -12, 16, 48, 53, 38, -8, -15, 1, 18, 57, 102},
  {-26, -1, 26, -36, -2, 18, 27, 54, 11, -15, 17, 6, 21, 38, 52, 13, -52, 50, 17, 27, 22},
  {0, -4, 11, -11, 5, 0, -4, -5, 14, -7, 5, -15, 27, 20, 20, 2, -4, 6, -19, 47, 336},
  {-5, 5, -11, -19, 0, 6, 6, 6, 93, -14, 13, 16, 18, 33, 34, 42, -25, -4, 9, 31, 44},
  {-1, -3, 20, -23, 3, 3, -13, -17, 35, 3, -3, -13, 39, 69, 42, 9, -9, -19, -8, 68, 148},
  {-4, -2, 11, -5, -3, 20, -5, -68, 41, 1, 21, -10, -1, 107, 28, -13, -5, 4, -50, 53, 272},
  {-4, -3, 12, -20, 2, 30, -15, -54, 66, 4, 19, -25, 3, 110, 55, -9, -7, -12, -41, 55, 180},
  {-5, 0, 9, -28, 12, 13, -19, -17, 93, 4, 3, -10, 19, 75, 51, 5, -18, -15, -12, 49, 94},
  {-3, 0, 16, -5, 0, -5, -13, -18, 3, 9, -13, -15, 62, 39, 5, -5, -1, -2, -14, 92, 248},
  {-1, 1, 16, -13, -1, -8, -13, -10, -1, 11, -18, 0, 58, 61, 29, -6, 0, -14, -4, 83, 172},
  {2, -1, 9, -19, 0, 0, -8, -4, 16, 4, -10, -3, 39, 72, 48, 4, -2, -20, -3, 65, 134},
  {2, -4, 12, -17, -3, 7, -7, -20, 19, 3, -1, -11, 26, 90, 57, -4, -2, -14, -15, 61, 154},
  {-1, -2, 10, -2, 0, 2, -18, -14, 0, 1, -1, -10, 54, 37, 3, -4, 0, 3, -22, 65, 310},
  {6, -1, -2, -14, -1, -7, -1, -3, -3, 5, -16, 18, 59, 61, 15, -3, 3, -22, 8, 77, 154},
  {6, -1, 1, -26, -4, -7, 8, 16, 13, 2, -14, 14, 35, 56, 54, 11, -7, -22, 21, 59, 82},
  {3, 10, -13, 14, 4, -33, 2, -47, 14, 11, -32, 31, 92, 50, -51, 17, 5, -18, 7, 88, 204},
  {2, -1, 15, -22, -1, -6, -6, 1, 5, 9, -16, 2, 43, 64, 52, 4, -4, -23, 7, 68, 126},
  {0, -3, 5, -1, 0, 5, -5, -11, -2, -1, 4, -15, 16, 44, 15, -5, -2, 5, -14, 32, 378},
  {0, -5, 12, -10, 4, 2, -10, -10, 0, -1, -2, -20, 44, 61, 31, -11, 2, -2, -17, 63, 250},
  {-2, -2, 3, 8, -5, 11, -2, -44, 10, 0, 11, -15, 14, 78, 7, -4, -5, 11, -34, 48, 336},
  {-2, -2, 5, 2, -3, 15, -6, -43, 17, 1, 12, -22, 8, 105, 35, -6, -4, 1, -38, 52, 258},
  {-2, -1, 0, 5, -3, 11, 2, -29, 9, -1, 12, -24, 1, 86, 41, -7, -3, 5, -32, 40, 292},
  {1, -2, 13, -26, 1, 24, -9, -55, 64, 4, 10, -33, -3, 107, 89, 11, -7, -26, -41, 53, 162},
  {-2, -3, 11, 5, -1, 9, -28, -27, -3, 1, 3, -15, 55, 73, 5, -3, -2, 11, -44, 81, 260},
  {-1, -4, 5, 5, 1, 7, -12, -18, -1, -4, 9, -23, 36, 33, 4, -3, -1, 11, -30, 56, 372},
  {-2, -3, 9, 3, -1, 8, -11, -29, -6, -1, 6, -22, 32, 85, 16, -10, 1, 5, -29, 65, 280},
  {-2, 4, 13, -6, 1, -15, -15, -10, -17, 14, -25, 4, 64, 72, 26, -11, 3, -17, 1, 88, 168},
  {6, 1, 10, -21, -1, -16, -9, 11, -16, 14, -30, 14, 53, 74, 51, -12, 3, -23, 13, 73, 122},
  {0, -1, -2, 7, -2, 7, 1, -21, -1, -2, 8, -20, 16, 39, -6, 1, -2, 8, -20, 39, 414},
  {-3, -3, 13, 4, -1, 6, -22, -23, -3, 1, 2, -20, 50, 55, -2, -4, -1, 11, -32, 73, 310},
  {-3, 0, 15, 8, -2, -5, -30, -44, 13, 7, -10, 10, 74, 62, -35, 9, -1, 5, -31, 81, 266},
  {3, 5, -5, 10, -2, -18, -9, -48, 10, 12, -24, 26, 88, 64, -38, 13, 8, -21, -7, 90, 198},
  {-2, -6, 12, -3, -4, 16, -9, -42, 7, 1, 7, -20, 23, 116, 29, -10, 3, -1, -37, 59, 234},
  {0, -3, 4, -1, 1, 3, -4, -2, -2, -2, 4, -12, 13, 8, 9, -4, 1, 2, -8, 17, 464},
  {0, -4, 15, -8, 1, 1, -20, -9, -10, 5, -6, -18, 49, 82, 36, -10, 4, -6, -23, 72, 210},
  {-2, -3, 4, 9, -2, 12, -10, -34, -4, -1, 9, -25, 24, 93, 16, -5, -3, 10, -35, 59, 288},
  {-1, -2, -1, 9, -3, 12, 0, -27, -8, -2, 10, -31, 14, 86, -2, -2, -3, 8, -20, 53, 332},
  {0, -1, -2, 6, -1, 6, 0, -11, -4, -2, 6, -16, 0, 46, -3, 2, -3, 5, -9, 24, 426},
  {-1, -1, -3, 9, -3, 11, 2, -24, -6, -3, 12, -31, 10, 75, -9, 1, -3, 8, -17, 46, 366},
  {-4, 0, 11, 3, 0, 0, -12, -23, 0, 8, -6, -33, 66, 37, -9, -1, -1, 1, -18, 102, 270},
  {0, 0, 0, 2, -2, 3, 0, -5, -1, -1, 5, -10, 5, 11, -3, 1, -1, 2, -5, 12, 486},
  {-4, -4, 8, 10, -3, 10, -10, -32, 0, 2, 9, -38, 42, 56, -16, 1, -4, 13, -31, 84, 326},
  {0, 1, 9, 3, 1, -16, -12, -16, -23, 12, -24, 7, 66, 73, 14, -4, 10, -28, 5, 92, 172},
  {-1, -1, -1, 6, -3, 7, -1, -13, -2, -1, 8, -18, 9, 32, -9, 3, -3, 5, -11, 27, 446},
  {-6, -4, 21, 5, -2, 4, -23, -26, -12, 8, -4, -29, 54, 80, 4, -9, 1, 6, -27, 95, 240},
  {0, 0, 0, 1, -1, 2, 0, -2, 0, -1, 2, -5, 1, 6, -2, 1, -1, 1, -2, 6, 500},
  {-2, -3, 7, 4, -2, 8, -7, -22, 7, 1, 7, -27, 29, 34, -28, 8, -2, 3, -16, 58, 398},
  {-2, -1, 3, 8, -3, 9, -13, -15, -4, -3, 12, -24, 30, 29, -6, 3, -5, 16, -35, 57, 400},
  {0, 0, -3, 5, -4, 5, 5, -16, 1, -1, 10, -20, 9, 29, -14, 4, -1, 2, -8, 27, 452},
  {0, 0, -1, 4, -2, 4, -1, -9, 0, -2, 7, -14, 9, 16, -5, 2, -2, 5, -10, 20, 470},
  {-4, -2, 14, 8, -1, 2, -28, -33, 2, 9, -9, -3, 58, 74, -41, 15, 0, -8, -15, 76, 284},
  {-3, -2, 6, 12, -3, 11, -16, -31, -6, 0, 10, -23, 34, 69, -17, 6, -3, 9, -36, 67, 344},
  {-5, -1, 6, 11, -4, 15, -15, -23, -12, 0, 11, -42, 28, 93, 10, -3, -4, 14, -33, 64, 292},
  {-2, -1, -2, 12, -5, 14, -4, -24, -5, -2, 13, -36, 16, 72, -9, 3, -5, 11, -19, 46, 366},
  {-2, -3, 1, 11, -5, 14, -2, -21, -6, -2, 13, -37, 11, 66, -3, 4, -5, 9, -16, 37, 384},
  {-1, -1, 0, 6, -3, 7, -1, -11, -2, -2, 9, -22, 7, 33, -7, 3, -3, 5, -8, 24, 446},
  {-1, 7, -7, -19, -2, -1, 11, 7, 29, -7, -1, 18, 44, 34, 34, 19, -11, -9, 13, 48, 100},
  {-2, 4, 7, -7, -1, 1, -6, -3, 33, -21, 5, 1, 31, 20, 49, 57, -10, -3, 21, 38, 84},
  {-12, -17, 20, 14, -6, 22, 4, 27, 70, -16, 17, -17, 11, -5, 29, 76, 4, 31, -15, 5, 28},
  {-6, -12, 24, 12, -9, 10, 3, 27, 52, -11, 10, -19, 8, 0, 36, 84, 19, 31, -17, -2, 32},
  {-4, 7, -19, 6, 9, 2, -6, 20, 131, -10, 11, -16, -11, -10, 48, 100, -2, 3, -21, 3, 30},
  {-1, -2, 5, 6, -13, 6, 8, 13, 44, -26, 14, 3, 36, 13, 32, 61, 2, 23, 0, 13, 38},
  {-3, 1, 5, -30, 17, 28, -12, -31, 85, 2, 10, -20, -8, 58, 94, 22, -6, -39, -17, 48, 104},
  {0, -4, 2, -9, 7, -1, 15, 9, 44, -25, 15, -9, 51, 6, 37, 47, 1, 6, 1, 12, 102},
  {-1, 2, 1, -15, -3, 2, 3, 7, 24, -11, 9, 15, 40, 36, 33, 28, -11, -1, 9, 39, 100},
  {-7, -1, -6, 9, -4, 7, 3, 23, 78, -13, 14, -3, 10, 5, 39, 70, 1, 14, -10, 10, 34},
  {-9, -6, 22, -16, -7, 18, -5, 8, 93, -43, 25, -19, -9, -25, 55, 139, 10, 34, -13, -4, 16},
  {-4, -4, 10, -9, -4, 11, -1, 9, 46, -30, 21, 4, 18, 12, 38, 69, -12, 19, 3, 26, 68},
  {-16, 10, 5, -15, -3, 14, 19, 27, 30, -18, 13, 5, 24, 35, 38, 41, -34, 33, 12, 19, 34},
  {-8, 5, 3, -3, -5, 15, 2, 15, 64, -31, 18, 10, 17, 4, 46, 76, -17, 21, -3, 13, 28},
  {-4, -1, 11, -26, 1, 39, 2, -73, 86, 4, 28, -13, -12, 96, 92, -29, -9, -11, -46, 46, 150},
  {-5, 20, -10, -8, 24, 11, -19, -8, 107, 2, -6, -20, -16, 32, 71, 63, -19, -29, -13, 42, 74},
  {-1, 2, 12, -23, 1, 8, -12, -25, 61, 3, -2, 0, 29, 58, 46, 9, -6, -8, -17, 56, 130},
  {-2, 9, -5, -24, 1, -3, 14, 6, 44, 4, -8, 16, 32, 42, 46, 22, -17, -13, 11, 45, 72},
  {-1, 5, 5, -21, 1, -2, -2, -17, 64, 4, -11, 10, 33, 42, 40, 36, -9, -11, -8, 47, 102},
  {-1, 10, -2, -22, 0, 5, 7, -7, 63, -8, -3, 13, 17, 22, 54, 72, -20, -20, 13, 35, 56},
  {-1, 8, 4, -34, -4, 17, 3, -10, 95, -25, 13, 11, -2, 9, 45, 108, -15, -10, -2, 23, 46},
  {1, 2, 6, -20, -1, 0, 5, -16, 24, 2, -11, 12, 48, 52, 39, 13, -5, -18, 0, 59, 128},
  {0, 4, 7, -15, -4, -2, 2, -5, 25, -3, -7, 15, 38, 38, 37, 34, -10, -7, 6, 50, 106},
  {-2, 1, -1, -22, -10, 18, 10, 6, 66, -18, 23, 16, 12, 11, 33, 85, -9, 5, -10, 17, 50},
  {-2, 10, 4, -40, -2, 17, -1, -13, 96, -43, 28, 1, -12, -1, 35, 168, -17, -2, -10, 18, 44},
  {-5, 5, -8, -14, -8, 9, 22, 14, 41, -9, 11, 19, 24, 33, 29, 47, -14, -2, 4, 30, 56},
  {-1, 5, 6, -28, 3, 9, -2, -19, 96, -7, 0, 5, 5, 23, 71, 72, -18, -26, 4, 32, 52},
  {0, 5, 5, -28, 3, 4, 3, -18, 77, 2, -9, 7, 21, 35, 61, 35, -12, -22, -2, 47, 84},
  {-1, 16, -20, -21, 0, 5, -10, 8, 124, -18, 17, 8, 7, 21, 36, 61, -9, 7, -13, 16, 44},
  {-3, -2, 11, -13, -5, 28, 9, -98, 85, -4, 26, -2, -14, 109, 76, -34, -4, 4, -45, 42, 180},
  {-4, -2, 12, -28, 0, 44, -2, -79, 108, 0, 27, -18, -22, 97, 94, -20, -5, -16, -42, 43, 138},
  {-1, 6, 10, -18, -6, 7, 0, -21, 31, -3, -8, 10, 24, 30, 60, 61, -13, -14, 11, 49, 82},
  {0, -3, 9, -3, -1, 6, -8, -36, 18, -1, 6, -8, 31, 82, 28, -11, 0, 3, -27, 60, 222},
  {0, 0, 15, -19, 1, -1, -8, -21, 37, 7, -9, 2, 36, 55, 49, 11, -3, -19, -5, 62, 132},
  {0, 2, 13, -20, 0, -3, -5, -26, 52, 9, -10, 6, 26, 42, 57, 29, -6, -25, 1, 61, 106},
  {2, 8, -1, -24, 0, 2, 6, -22, 60, -1, -7, 18, 17, 30, 69, 58, -13, -28, 10, 39, 66},
  {2, -1, 7, -13, -4, 3, -4, -21, 12, 5, -7, 3, 49, 64, 11, -1, 0, -9, -11, 75, 192},
  {1, 1, 10, -25, 1, 1, 1, -21, 43, 4, -8, 8, 30, 56, 65, 20, -8, -21, -1, 52, 94},
  {2, -7, 13, -9, -3, 8, -6, -41, 31, 2, 3, -11, 40, 81, 37, -10, 2, -1, -29, 59, 190},
  {0, -3, 10, -4, -2, 6, -10, -28, 21, 1, 4, -12, 36, 57, 37, -3, -1, 0, -25, 50, 244},
  {1, -2, 16, -21, 2, -4, -6, -18, 36, 6, -11, -1, 37, 58, 55, 11, -3, -17, -2, 58, 122},
  {-1, -2, 16, -7, -3, 1, -9, -33, 25, 6, -4, -10, 40, 66, 42, 7, -1, -6, -19, 61, 174},
  {0, -2, 19, -17, -2, 4, -13, -32, 45, 6, -5, -4, 32, 70, 54, 8, -2, -11, -14, 54, 132},
  {-1, 0, -1, 8, -5, 11, 4, -71, 45, -4, 14, 2, 8, 90, 38, -16, -5, 13, -36, 40, 244},
  {-1, -3, 1, -2, -4, 23, 9, -84, 71, -2, 20, -6, -12, 106, 78, -13, -4, -1, -41, 32, 178},
  {-1, -3, 5, -25, -1, 36, 5, -99, 122, -1, 27, -16, -34, 106, 103, -2, -3, -11, -40, 16, 144},
  {-2, 0, 11, -66, 10, 41, -25, -72, 200, -1, 32, -38, -29, 93, 108, -13, -2, -4, -51, 8, 112},
  {-4, 2, 22, -13, -3, -8, -9, -21, 20, 12, -15, -9, 53, 45, 20, 4, -3, -8, -4, 83, 184},
  {0, -5, 12, -6, 0, 3, -8, -25, 14, -2, 4, -9, 30, 50, 16, -11, 4, 0, -20, 65, 288},
  {0, -8, 21, -15, 0, 8, -10, -32, 23, 2, 3, -11, 34, 74, 50, -9, 3, -12, -19, 63, 182},
  {8, 1, 3, -25, 6, -13, 4, -9, 25, 7, -16, 15, 39, 57, 53, 7, -6, -24, 11, 57, 112},
  {2, -3, 6, 0, -3, 0, -4, -28, 4, 1, 4, -6, 30, 71, -10, 1, 1, 1, -19, 55, 306},
  {12, -4, -7, -4, -3, -17, 8, -25, 5, 13, -27, 30, 74, 48, 11, 12, 8, -33, 2, 79, 148},
  {-1, 1, -2, 10, -5, 11, 0, -46, 12, -2, 9, -18, 19, 105, 30, -10, -2, 9, -30, 48, 236},
  {1, 0, -3, 8, -1, 4, 3, -28, 4, -4, 9, -16, 13, 65, 10, -5, -2, 8, -18, 33, 350},
  {-1, -3, 13, -5, -3, 6, -9, -28, 17, 1, 4, -15, 29, 69, 41, -9, 3, -1, -25, 56, 232},
  {-2, -4, 17, -12, -1, 6, -12, -25, 31, 8, -2, -15, 26, 84, 61, -4, 0, -11, -17, 59, 138},
  {1, -7, 21, -18, 4, -7, -9, -22, 39, 14, -11, -10, 30, 66, 80, 23, 0, -28, -2, 45, 94},
  {-2, -5, 12, -8, 1, 15, 9, -88, 107, -5, 15, -12, -12, 102, 83, -28, -4, 10, -33, 20, 158},
  {0, 2, -9, 15, -2, 5, 5, -52, 19, -3, 10, -5, 7, 87, 20, -4, -4, 9, -29, 43, 284},
  {0, -1, -3, 11, -4, 11, 7, -69, 46, -4, 15, -9, -3, 112, 50, -11, -4, 8, -31, 34, 202},
  {-2, -6, 17, -31, 1, 23, 0, -95, 173, -6, 21, -14, -18, 109, 74, -54, -2, 28, -37, 11, 128},
  {-1, 0, 2, 9, -4, 4, -6, -33, 1, -2, 15, -32, 45, 75, 6, 2, -3, 13, -32, 56, 282},
  {-5, -2, 22, -5, -4, -3, -13, -16, -2, 13, -6, -28, 55, 59, 19, -3, 0, -4, -14, 88, 210},
  {0, 0, 0, 2, 1, 0, 0, -5, -2, -2, 2, -5, 4, 17, 3, -2, 0, 4, -4, 8, 470},
  {-3, -4, 15, 1, -3, -3, -8, -20, -13, 17, -9, -24, 50, 68, 27, -9, 2, -13, -9, 98, 192},
  {0, 2, -7, 11, -2, 3, 0, -20, -6, -1, 5, -8, 9, 61, -13, 6, -3, 7, -20, 38, 388},
  {0, -1, -3, 12, -1, 5, -2, -29, 1, -3, 8, -21, 13, 86, -1, -3, -5, 15, -22, 54, 306},
  {0, 1, -3, 4, -2, 2, 3, -9, -5, -1, 5, -11, 3, 40, -3, 2, -1, 2, -5, 16, 436},
  {0, 2, -3, 8, -5, 3, 0, -21, 0, 1, 8, -9, 9, 47, -21, 8, -2, 2, -16, 36, 418},
  {0, 1, -4, 12, -5, 5, -3, -29, -2, 0, 11, -16, 21, 68, -20, 9, -3, 7, -29, 59, 348},
  {0, -6, 7, 3, -5, 8, -3, -26, 0, 3, 8, -30, 35, 59, -21, 5, 1, -1, -21, 83, 314},
  {0, 0, 0, 12, -7, 5, -4, -39, 1, 1, 11, -10, 19, 87, -30, 9, -3, 6, -29, 56, 342},
  {-1, -4, 5, 12, -3, 10, -10, -32, 5, -2, 9, -29, 16, 107, 23, -5, -3, 15, -28, 57, 228},
  {1, -4, 3, 11, -6, 9, -3, -40, 15, -3, 12, -24, 4, 120, 38, -4, -1, 9, -27, 41, 210},
  {1, 0, -6, 11, -2, 4, 2, -20, -7, -3, 9, -19, 6, 80, -7, 6, -2, 5, -14, 33, 358},
  {-4, -5, 10, 9, -2, 13, -9, -39, 19, -1, 11, -31, 6, 112, 56, -6, -1, 8, -30, 43, 194},
  {0, 1, -6, 13, -3, 6, 5, -26, -6, -4, 11, -23, 3, 106, 13, 1, -4, 9, -18, 31, 294},
  {0, 2, -8, 11, -2, 3, 3, -21, 0, -1, 9, -17, 7, 60, -11, 3, -4, 7, -15, 41, 378},
  {1, 3, -6, 5, -4, 2, 9, -10, -11, -1, 9, -17, -2, 65, -12, 5, -1, -1, -5, 26, 402},
  {0, -5, 6, -7, 0, -2, -2, -4, -4, -14, -3, 14, 49, 24, 5, 5, -1, 8, 17, 79, 182},
  {-5, -6, 11, -11, -12, 8, 7, 5, -3, -15, 14, 23, 35, 28, 17, 23, -14, 33, 32, 50, 72},
  {-5, -8, 12, -12, -18, 15, 14, 8, -2, -15, 20, 24, 33, 24, 16, 28, -6, 33, 29, 38, 56},
  {-7, -6, 23, -7, -22, 13, 4, 2, -4, -11, 9, 18, 31, 23, 7, 28, 2, 35, 24, 53, 82},
  {-2, 1, 3, -4, -3, -10, -2, -8, 0, -13, 0, 25, 56, 28, 2, 9, -6, 6, 28, 79, 134},
  {-4, -1, 9, -4, -7, -4, 2, -4, -2, -10, 4, 21, 49, 24, 3, 14, -7, 14, 26, 68, 130},
  {1, 1, 0, 9, 11, -26, -14, -28, 11, -3, -23, 29, 88, 30, -34, 13, 22, -11, -10, 94, 192},
  {-2, -6, 11, -6, -17, 8, 10, 3, -21, -18, 5, 17, 38, 26, 14, 7, -1, 30, 30, 65, 126},
  {-16, -10, 23, -8, -23, 36, 13, 11, 20, -3, 18, 17, 3, 8, 1, 76, 12, 52, 12, 13, 2},
  {0, -12, 6, -27, -4, 12, 16, 20, -6, -22, 16, 24, 41, 28, 22, 0, -24, 30, 35, 54, 94},
  {-24, -9, 41, -25, -6, 28, 11, 24, 0, -17, 28, 16, 15, 22, 35, 23, -43, 45, 38, 33, 42},
  {-35, 6, 35, -22, -5, 32, 18, 25, 21, -15, 30, 11, 6, 15, 19, 47, -56, 65, 29, 21, 18},
  {-2, -3, 11, -2, -4, 9, -6, 1, -8, -9, 3, -11, 18, 1, 5, 16, -3, 13, -4, 94, 274},
  {-4, -4, 14, -13, 0, 13, -11, -10, -3, -4, 2, 3, 27, 40, 40, 22, -16, 4, 21, 69, 132},
  {13, -23, 8, -27, 34, -5, 18, -22, 44, -27, -7, -33, 53, -17, -7, -3, 20, 10, 14, 78, 270},
  {-7, -3, 31, -6, 9, 2, -8, -34, 44, -5, 19, -4, 30, 4, 11, 61, 2, 16, 17, 28, 98},
  {-1, 1, 8, -3, 0, -8, -10, -7, -11, -10, -11, 13, 54, 33, 12, 1, -3, 5, 20, 90, 166},
  {0, -1, 5, -9, -5, -6, -2, 3, -14, -14, 2, 25, 49, 33, 20, 8, -13, 11, 34, 73, 114},
  {1, -9, 6, -19, -17, 13, 8, 15, -16, -24, 19, 27, 40, 33, 25, 13, -15, 34, 33, 54, 70},
  {1, 0, -4, -1, 1, -11, -1, -10, -8, -7, -12, 26, 67, 37, -1, 6, -1, -1, 13, 83, 158},
  {-3, -1, -1, 14, -6, 16, -3, -15, -3, 1, 18, -32, -21, 1, 36, 27, 3, -5, -14, 104, 280},
  {-2, -1, 3, 4, 5, -6, -6, -13, 0, -3, -17, -1, 60, 13, -15, 5, 1, 11, -10, 96, 264},
  {0, -15, 27, -25, -36, 30, 8, 29, -28, -36, 33, 16, 27, 22, 41, -2, -15, 70, 36, 46, 56},
  {1, -8, 10, -32, -12, 13, 10, 32, -17, -28, 27, 27, 33, 31, 41, 8, -31, 37, 40, 47, 54},
  {-2, -3, 9, -11, 0, 1, -5, 8, -12, -17, 1, 12, 45, 27, 22, -5, -11, 14, 26, 76, 162},
  {-2, -2, 6, -2, -4, 4, -1, 0, -6, -8, 6, -10, 20, -1, 12, -1, 0, 15, -22, 81, 342},
  {-5, -16, 36, -48, -32, 42, 5, 49, -16, -37, 49, 22, 8, 21, 54, 16, -27, 49, 39, 37, 20},
  {2, -9, 10, -12, 2, 7, -1, 6, 5, -14, 19, -18, 5, -13, 15, -2, -2, 28, -38, 50, 432},
  {-2, -6, 18, -7, -16, 2, 10, 9, -26, -25, -2, 12, 38, 27, 20, -20, -7, 44, 42, 79, 132},
  {0, -11, 22, -32, 6, 11, -12, 1, 19, -18, 9, 4, 23, 32, 32, 22, -17, 16, 34, 65, 100},
  {-4, -1, 9, -3, 0, 17, -13, -18, -11, 8, -1, -12, 14, 40, 48, 24, -14, -23, 1, 95, 200},
  {-3, 1, 13, -5, 1, -9, -8, -8, -2, 2, -16, 2, 48, 14, -2, 8, -5, -6, 12, 107, 224},
  {-3, -1, 9, 5, 4, -2, -17, -18, -3, 1, -15, -2, 50, 27, 1, -2, 0, -2, -11, 101, 268},
  {-3, 2, 9, 3, 4, -14, -8, -18, -10, 0, -22, 12, 54, 35, 9, -4, -1, -11, 21, 101, 194},
  {2, -2, 9, -7, 0, -11, -6, -7, -20, -2, -14, 11, 49, 41, 33, 1, -3, -11, 28, 86, 158},
  {-2, 6, -1, 9, 7, -22, -8, -31, 6, 5, -28, 21, 79, 36, -30, 7, 2, -6, 0, 88, 236},
  {3, 1, -3, 0, 3, -16, -1, -12, -17, -1, -19, 26, 62, 44, 11, 6, -1, -18, 25, 84, 158},
  {-1, 0, 9, -4, 0, -9, -7, -6, -17, -8, -12, 10, 43, 40, 30, -7, -7, 1, 33, 92, 152},
  {0, 5, -6, 14, 7, -27, -3, -41, 5, 9, -31, 25, 84, 49, -36, 20, 5, -18, 4, 89, 204},
  {-2, -3, 7, 1, 3, 0, 0, -17, 1, 3, -10, -22, 45, 5, -1, -2, -1, 3, -16, 97, 330},
  {-1, -3, 5, -2, 0, 5, -2, -1, -4, -6, 5, -8, 3, 7, 13, -3, -3, 11, -9, 27, 444},
  {-1, -4, 6, 0, 2, 5, -2, -9, -3, -2, -5, -18, 21, 14, 10, -7, -2, 6, -4, 69, 360},
  {-1, -3, -2, 8, -2, 20, -10, -19, -15, 4, 0, -10, 13, 26, 30, 14, -8, -6, -27, 86, 316},
  {1, -4, 8, -14, 13, 10, -5, -16, -33, 7, -9, -11, 17, 42, 72, 69, -37, -34, 37, 80, 126},
  {-2, -3, 11, 1, 4, -7, 0, -19, 0, 5, -20, -10, 46, 14, -5, 0, -2, -5, 0, 116, 264},
  {-4, -3, 7, -2, 3, 9, -8, -9, -4, -5, -1, -12, 16, 30, 23, -5, -6, 5, -8, 86, 288},
  {-1, -2, -2, 11, -4, 9, 5, -16, -2, -2, 22, -38, -9, 3, 34, 12, -2, 0, -16, 85, 338},
  {-3, -1, 20, -5, 4, -12, -7, -16, -11, 11, -30, 4, 41, 32, 18, 2, -1, -24, 20, 117, 194},
  {-4, -3, 11, 5, 5, -5, -6, -24, 2, 8, -20, -7, 43, 22, -10, 1, 0, -9, -5, 103, 298},
  {-4, 4, 5, 12, 6, -15, -9, -28, -16, 4, -24, 10, 52, 46, 12, -1, 5, -23, 12, 101, 214},
  {-1, -2, 5, 7, 2, -5, -10, -14, -36, 7, -18, 6, 45, 55, 44, 1, 2, -37, 24, 95, 172},
  {-3, -3, 5, 6, 1, 6, -5, -17, 0, 4, -7, -23, 31, 20, -10, 1, -2, 0, -12, 93, 342},
  {-5, 3, 10, 8, 5, -13, -14, -24, -6, 7, -24, 8, 54, 39, -3, 1, 0, -14, 5, 97, 244},
  {-5, 4, 11, 10, 7, -16, -23, -36, 10, 12, -35, 21, 76, 44, -41, 13, 0, -6, -3, 81, 264},
  {1, 15, -18, 19, 9, -48, 15, -56, 9, 15, -47, 52, 86, 62, -49, 19, 10, -34, 26, 76, 188},
  {-2, -5, 12, -1, 2, 4, -13, -6, 3, 3, 0, -22, 37, -3, -5, 1, -4, 13, -38, 92, 376},
  {-3, -5, 14, 1, 6, -8, -4, -20, 6, 9, -14, -19, 55, 3, -13, 4, -1, -5, -18, 117, 302},
  {-1, -1, 2, 1, -1, 5, -2, -3, -1, -2, 4, -13, 6, 3, 5, -3, -2, 6, -10, 37, 452},
  {-1, -2, 3, 1, 0, 2, 1, -5, -1, -2, 3, -11, 7, 2, 4, -2, -1, 2, -4, 26, 468},
  {-1, -3, 2, 5, -1, 8, -3, -10, -5, 2, -2, -17, 18, 7, 9, -1, -2, 0, -15, 75, 380},
  {-4, -4, 7, 10, 0, 4, -6, -19, -7, 4, -6, -19, 23, 25, 10, -2, -2, -1, -15, 104, 308},
  {-5, -4, 8, 12, -2, 15, -23, -15, -11, 9, -8, -21, 36, 27, 8, 7, -6, 3, -27, 99, 308},
  {-3, -3, 4, 4, -1, 7, 1, -13, 0, 0, 3, -26, 9, 16, 6, -5, -4, 9, -13, 70, 390},
  {-6, 0, 19, 6, 7, -21, -8, -33, -5, 16, -41, 17, 55, 45, -7, 1, -2, -26, 33, 111, 190},
  {-1, -2, 3, 0, -1, 4, -3, -1, 0, -1, 4, -9, 6, -1, 1, 0, -2, 4, -7, 16, 492},
  {-2, -4, 7, 3, 0, 8, -11, -8, 1, 1, 4, -26, 29, 3, -3, 1, -4, 13, -36, 82, 396},
  {-4, 0, 3, 15, 3, -4, -4, -25, -12, 11, -15, -13, 39, 27, 1, 8, 2, -19, -13, 111, 290},
  {0, -2, 2, 1, -1, 4, -2, -4, 0, -1, 6, -17, 14, -1, 1, 1, -1, 4, -14, 37, 458},
  {-6, -6, 13, 9, 2, 12, -22, -18, 0, 2, -5, -23, 37, 25, -9, 1, -3, 10, -34, 103, 336},
  {-3, 6, -3, 17, 7, -25, 2, -42, -3, 11, -35, 26, 63, 55, -30, 13, 8, -35, 22, 85, 234},
  {-3, -5, 8, 8, -1, 11, -7, -22, -1, 5, -2, -27, 32, 23, -15, 9, 1, -12, -3, 61, 392},
  {0, -1, 2, 0, 0, 2, -2, 0, 0, -1, 2, -4, 2, 0, 1, 0, -1, 2, -5, 9, 500},
  {0, -1, 1, 0, 0, 2, -1, 0, 0, 0, 1, -3, 2, -1, 1, 0, -1, 1, -2, 4, 506},
  {-5, -6, 15, 9, 9, -8, -3, -37, 6, 17, -35, -6, 50, 40, -36, 12, 4, -28, 13, 104, 282},
  {0, -2, 3, 0, -1, 5, -5, -2, 1, -1, 6, -15, 15, -4, 0, 0, -3, 9, -21, 38, 466},
  {-1, -2, 3, 2, -2, 6, -4, -7, 1, -1, 8, -23, 20, -1, 0, 0, -3, 11, -29, 61, 434},
  {-3, -5, 7, 6, -2, 10, -6, -16, 1, 1, 5, -28, 20, 13, 0, 0, -3, 8, -24, 77, 390},
  {-3, -5, 8, 4, -2, 12, -10, -9, 0, 3, 4, -29, 19, 9, 0, 1, -7, 11, -15, 55, 420},
  {-2, -4, 7, 1, -1, 8, -6, -6, 1, 0, 7, -22, 15, 0, 3, -1, -4, 7, -9, 32, 460},
  {-2, -5, 9, 0, 6, -5, 3, -20, 10, 6, -5, -28, 48, -6, -16, 7, 0, 0, -29, 103, 360},
  {2, -5, 3, -6, -7, 8, 0, -5, -18, -18, 4, 22, 36, 27, 14, 31, 0, 19, 22, 69, 116},
  {0, -7, 7, -13, -42, 29, 25, 12, -23, -11, 8, 28, 25, 15, 6, 55, 21, 61, 11, 28, 42},
  {-3, -3, 12, -15, -18, 9, 14, -5, 8, 0, 2, 16, 28, 7, 6, 65, 13, 47, 8, 32, 66},
  {1, -6, 4, -6, -11, 8, 11, -2, -21, -23, 4, 26, 32, 18, 11, 39, 6, 32, 27, 60, 92},
  {-1, 2, 4, 2, 5, -15, -8, -23, 4, -9, -16, 29, 62, 29, -14, 19, 19, 0, -4, 95, 152},
  {-1, -4, 15, -14, -51, 30, 22, 19, -37, -15, 6, 25, 14, 24, 18, 40, 19, 83, 19, 30, 28},
  {-5, -7, 21, -22, -51, 38, 18, 17, -2, 20, -5, 24, 2, 6, -10, 94, 37, 76, -8, 12, 2},
  {-9, -3, 11, -1, -32, 24, 8, -8, 53, 24, -12, 13, 8, -6, -19, 114, 24, 75, -17, 4, 10},
  {-14, 13, 17, 5, 1, 5, -3, -25, 36, 10, -40, 14, 4, 1, -15, 141, 25, 34, -17, 43, 42},
  {0, -4, 3, -1, -6, 9, 12, 7, -27, -33, 2, 11, 20, 17, 10, 3, -1, 41, 45, 79, 138},
  {-3, 2, 11, -5, -4, -8, -5, -11, 1, -13, 3, 20, 35, 13, 1, 28, 4, 13, 25, 83, 132},
  {-2, -1, 11, -7, -4, 7, -2, 4, -11, -17, -1, 9, 20, 4, 1, 32, 3, 12, 18, 87, 186},
  {-6, 5, 14, -12, -17, 11, 7, -2, -2, -22, -1, 25, 21, 7, 7, 83, 9, 36, 15, 49, 58},
  {-1, 0, 11, -17, -39, 22, 18, 3, 22, -2, 2, 18, 16, 1, -3, 99, 18, 59, 0, 14, 30},
  {-2, 0, 6, 0, -36, 21, 28, 12, -56, -30, 9, 18, 26, 16, 19, 21, 17, 69, 32, 44, 84},
  {34, -37, 1, -49, 52, -3, 9, -18, 59, -44, -6, -2, 41, -13, -12, 15, 20, 1, 7, 77, 248},
  {3, -5, 3, -5, 23, -17, -2, -11, -3, -34, -13, 13, 53, 21, 0, 4, 2, 26, 26, 90, 164},
  {4, -3, -1, -5, -7, 0, 7, 10, -36, -45, 18, 31, 38, 26, 22, 18, -11, 52, 35, 60, 86},
  {7, -10, 1, -13, -32, 24, 18, 15, -44, -36, 29, 32, 30, 17, 14, 48, 5, 66, 20, 39, 52},
  {2, -3, 10, -17, -31, 24, 11, 13, -32, -53, 39, 35, 17, 8, 8, 77, -3, 67, 30, 37, 34},
  {5, -7, 4, -14, 11, 0, 7, 9, -12, -56, 2, 19, 34, 11, -9, 26, 4, 60, 13, 77, 144},
  {-1, 1, 8, -2, -2, -1, 3, 4, -37, -39, 7, 22, 28, 13, 11, 51, -3, 45, 32, 66, 100},
  {2, -14, 13, -12, -54, 46, 28, 34, -64, -32, 28, 32, 13, 9, 17, 46, 14, 111, 9, 19, 22},
  {-1, -4, 6, 1, 0, 2, 5, 2, -18, -17, -5, 1, 22, 8, 3, 2, -3, 17, 29, 102, 208},
  {2, -6, 17, -27, -51, 44, 19, 22, -40, -53, 50, 37, 11, 0, 1, 91, 5, 82, 17, 26, 18},
  {-2, -2, 11, 2, -1, -5, -1, -3, -19, -21, 0, 14, 30, 13, 4, 9, -5, 24, 38, 94, 152},
  {-1, -3, 12, -6, 0, -4, -3, 2, -12, -9, 1, 4, 19, 6, -5, 27, -7, 5, 20, 107, 206},
  {-2, 0, 5, 2, 0, 6, 5, -2, -25, -18, -8, -1, 18, 9, 5, 27, -6, 9, 33, 109, 180},
  {0, -6, 5, 4, -14, 15, 23, 26, -66, -32, 7, 3, 2, -5, 15, 34, 3, 38, 36, 88, 160},
  {1, -21, 23, -1, -47, 31, 38, 24, -74, -52, 24, 28, 20, 18, 26, -4, 10, 96, 45, 45, 52},
  {0, -4, 1, 3, 17, 2, 11, 0, -17, -39, 4, -20, 3, -19, -14, 62, 9, 17, 10, 99, 262},
  {-8, 4, 49, -37, 46, -38, -29, -7, -2, -30, 4, 40, 7, -22, -78, 216, -26, -41, 122, 68, 36},
  {-2, -4, 7, 5, 15, -8, -3, -5, -18, -17, -21, 3, 33, 21, 5, 3, -1, 9, 29, 114, 182},
  {-1, -3, 7, 4, 15, -9, -3, -1, -22, -31, -9, 2, 28, 20, 21, -4, -2, 27, 33, 97, 174},
  {-3, 0, 6, 7, 12, -12, 3, 4, -42, -38, -6, 10, 27, 20, 28, 12, -8, 36, 44, 95, 122},
  {-1, 2, 2, 10, 6, -12, 3, 16, -67, -59, 15, 22, 25, 13, 34, 39, -10, 66, 34, 77, 82},
  {1, 5, -12, 12, 12, -25, 4, -30, 0, -7, -30, 22, 81, 39, -27, 6, 7, 7, 4, 92, 190},
  {0, 4, -1, 1, 8, -12, 4, 12, -40, -55, 11, 19, 31, 23, 32, 15, -7, 51, 31, 81, 96},
  {-2, -5, 10, 2, 11, -5, -3, -12, -4, -4, -21, -6, 38, 12, -1, -1, -3, -1, 17, 118, 232},
  {-1, -5, 2, 3, 7, 2, 2, -10, -3, -3, -16, -19, 45, 0, -6, 3, 0, 10, -10, 99, 312},
  {-2, -4, 2, 6, 8, 3, 6, -2, -20, -19, -16, -5, 26, 14, 8, -2, -1, 18, 20, 105, 222},
  {-1, -4, 3, 2, 3, 6, 1, 0, -12, -11, -2, -13, 12, 7, 14, -6, -2, 20, -8, 84, 326},
  {-1, -4, -2, 9, 0, 12, 14, 6, -41, -28, 0, -17, 8, 10, 25, 12, 0, 23, 24, 108, 196},
  {-1, -3, 0, 5, 3, 6, 4, 5, -21, -17, -2, -12, 8, 7, 16, 5, 0, 21, 1, 77, 308},
  {-1, 1, 14, -7, 18, -16, 4, 14, -51, -43, 0, 1, 21, 1, 23, 75, -2, 28, 35, 84, 114},
  {-2, -1, 1, 6, -12, 24, 9, 2, -43, -4, -7, -20, -1, 16, 36, 31, -5, 0, 21, 117, 176},
  {-1, -3, -2, 7, -2, 32, 15, -24, -63, 6, 0, -22, -15, 1, 56, 128, -45, -28, 60, 90, 132},
  {-1, -5, 9, 0, -2, 6, 5, -9, -10, 9, -21, -9, 25, 6, -6, 15, 0, -18, 10, 134, 236},
  {-1, -7, 10, 0, 4, 0, 5, -16, 3, 9, -23, -13, 33, 6, -11, 4, 0, -13, 3, 130, 266},
  {-3, -5, 7, 7, 14, -7, 4, -20, -9, -2, -31, -5, 33, 25, 6, -3, 0, -8, 33, 119, 202},
  {-6, -6, 7, 16, 20, -13, -1, -12, -25, -18, -23, 3, 32, 28, 21, -4, -1, 7, 43, 108, 160},
  {-9, -1, 11, 16, 21, -23, -3, 1, -47, -34, -12, 13, 30, 28, 32, 11, -4, 24, 47, 98, 114},
  {0, -5, 3, 3, 1, 6, 5, -11, -2, 3, -10, -23, 28, 4, -3, 0, -3, 1, -3, 93, 338},
  {-3, 0, -1, 10, 5, -2, 3, -9, -17, -2, -17, -13, 33, 14, 7, 6, 3, -6, 6, 103, 272},
  {-4, 11, 0, 12, 14, -43, -2, -50, 19, 6, -42, 31, 100, 46, -54, 10, 10, -22, 38, 66, 220},
  {-1, -4, 3, 3, 0, 9, -1, -4, -6, -2, -5, -16, 11, 7, 8, -5, -3, 12, -4, 74, 360},
  {-1, -2, 0, 2, -1, 7, -1, 0, -6, -3, 1, -12, 6, 0, 10, -2, -2, 14, -4, 36, 428},
  {-2, -8, 7, 5, 6, 3, 5, -13, -7, 2, -24, -13, 24, 16, 3, -3, -3, -4, 25, 118, 238},
  {-3, -7, 8, 7, 9, 3, 1, -10, -14, -11, -18, -14, 18, 19, 20, -7, -3, 12, 30, 111, 210},
  {-3, -7, 5, 9, 8, 5, 8, -4, -28, -16, -18, -15, 17, 15, 25, -1, -1, 15, 32, 112, 196},
  {-5, -6, 8, 11, 14, -1, -2, -7, -24, -32, -10, -5, 11, 19, 32, -4, 6, 34, 26, 106, 170},
  {-1, -3, -3, 7, -4, 12, 14, -7, -17, -4, -5, -25, 15, -1, 22, 0, 0, 1, 9, 100, 292},
  {0, -3, -14, 18, -3, 23, 11, 3, -42, -10, -9, -26, 4, 8, 36, 18, -1, -5, 38, 112, 196},
  {-4, -1, -7, 19, -10, 26, 7, -7, -53, 9, -15, -19, 2, 11, 45, 61, -17, -15, 56, 99, 138},
  {0, -3, 0, 2, -3, 7, 6, -11, 1, 3, -1, -23, 18, 3, 2, -2, -3, 3, -12, 92, 354},
  {-2, -5, 3, 6, 12, -1, 5, -14, -7, -5, -22, -11, 18, 21, 13, -8, 2, -2, 20, 109, 248},
  {-1, -9, 4, 10, 9, -1, 7, -25, 0, 6, -26, -8, 16, 27, 0, -5, 0, -14, 25, 105, 272},
  {0, -2, 2, 1, -1, 4, 0, -3, 0, 1, 3, -15, 12, -2, 1, 0, -2, 0, -7, 48, 432},
  {-1, -12, 15, -1, 7, 2, 1, -20, 6, 7, -23, -14, 27, 16, -9, -1, 3, -10, 3, 120, 280},
  {-5, -6, 4, 14, 11, -3, 11, -32, -5, 1, -24, -21, 42, 22, -3, 1, 0, -7, 22, 97, 274},
  {-1, -6, 4, 5, 2, 2, 9, -14, -5, 3, -2, -36, 35, -4, 10, 1, 2, -14, 17, 77, 342},
  {-1, -11, 8, 5, 8, 10, -9, -13, 0, 3, -10, -38, 58, 9, -30, 14, 6, -32, 49, 25, 410},
  {0, -1, 1, 0, 0, 2, -2, 0, -1, 0, 3, -7, 6, -5, 4, 0, 0, 0, -2, 16, 484},
  {0, -3, 3, 0, -1, 5, -4, -1, 0, 0, 3, -9, 1, 2, 4, -1, -3, 6, -5, 35, 448},
  {0, -6, 0, 8, -5, 15, 4, -12, -7, 4, -6, -29, 10, 14, 14, -4, -3, 1, 7, 111, 280},
  {-1, 1, -3, 5, 0, -5, 9, 6, -18, -1, -1, -14, 13, -12, 14, 11, 7, -18, 2, 114, 294},
  {-1, -1, -5, 13, -5, 4, 8, 1, -28, 10, -13, -22, 12, 3, 13, 30, 3, -22, 26, 114, 232},
  {0, -5, 2, 4, -3, 9, 2, -9, -2, 3, 5, -33, 14, 3, 11, -1, -3, 3, 4, 80, 344},
  {-2, -12, 7, 13, 8, 3, 7, -22, -12, 2, -19, -21, 22, 18, 15, -4, -1, -6, 28, 110, 244},
  {-1, -6, 8, 0, 4, -2, 8, -20, 8, 7, -16, -9, 17, 14, -16, 4, 2, -11, 0, 85, 360}
};

#endif

const int AdaptiveLoopFilter::m_FilterTapsOfType[ALF_NUM_OF_FILTER_TYPES] =
{
  5,
  7,
  9
};

#if GALF
const int AdaptiveLoopFilter::m_pattern9x9Sym[41] =
{
                 0,
              1, 2, 3,
           4, 5, 6, 7, 8,
       9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 19, 18, 17, 16,
      15, 14, 13, 12, 11, 10, 9,
           8, 7, 6, 5, 4,
              3, 2, 1,
                 0
};
const int AdaptiveLoopFilter::m_weights9x9Sym[22] =
{
           2,
        2, 2, 2,
     2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
     2, 2, 2, 2, 1, 1
};

const int AdaptiveLoopFilter::m_pattern9x9Sym_Quart[42] =
{

  0, 0, 0, 0, 1, 0, 0, 0, 0,
  0, 0, 0, 2, 3, 4, 0, 0, 0,
  0, 0, 5, 6, 7, 8, 9, 0, 0,
  0, 10, 11, 12, 13, 14, 15, 16, 0,
  17, 18, 19, 20, 21, 22
};

#else
const int AdaptiveLoopFilter::m_pattern9x9Sym[39] =
{
                   0,
               1,  2,  3,
           4,  5,  6,  7,  8,
       9, 10, 11, 12, 13, 14, 15,
      16, 17, 18, 19, 18, 17, 16,
      15, 14, 13, 12, 11, 10,  9,
           8,  7,  6,  5,  4,
               3,  2,  1,
                   0
};

const int AdaptiveLoopFilter::m_weights9x9Sym[21] =
{
               2,  2,  2,
           2,  2,  2,  2,  2,
       2,  2,  2,  2,  2,  2,  2,
   2,  2,  2,  2,  1,  1
};

const int AdaptiveLoopFilter::m_pattern9x9Sym_Quart[42] =
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  1,  2,  3,  0,  0,  0,
   0,  0,  4,  5,  6,  7,  8,  0,  0,
   0,  9, 10, 11, 12, 13, 14, 15,  0,
  16, 17, 18, 19, 20, 21
};
#endif

const int AdaptiveLoopFilter::m_pattern7x7Sym[25] =
{
               0,
           1,  2,  3,
       4,  5,  6,  7,  8,
   9, 10, 11, 12, 11, 10, 9,
       8,  7,  6,  5,  4,
           3,  2,  1,
               0
};

const int AdaptiveLoopFilter::m_weights7x7Sym[14] =
{
              2,
          2,  2,  2,
      2,  2,  2,  2,  2,
  2,  2,  2,  1,  1
};


const int AdaptiveLoopFilter::m_pattern7x7Sym_Quart[42] =
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  9,  0,  0,
   0, 10, 11, 12, 13, 14,
};

const int AdaptiveLoopFilter::m_pattern5x5Sym[13] =
{
           0,
       1,  2,  3,
   4,  5,  6,  5,  4,
       3,  2,  1,
           0
};

const int AdaptiveLoopFilter::m_weights5x5Sym[8] =
{
          2,
       2, 2, 2,
    2, 2, 1, 1
};

const int AdaptiveLoopFilter::m_pattern5x5Sym_Quart[45] =
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  0,  0,  0,
};

const int AdaptiveLoopFilter::m_pattern9x9Sym_9[39] =
{
              12, 13, 14,
          20, 21, 22, 23, 24,
      28, 29, 30, 31, 32, 33, 34,
  36, 37, 38, 39, 40, 39, 38, 37, 36,
      34, 33, 32, 31, 30, 29, 28,
          24, 23, 22, 21, 20,
              14, 13, 12,
};

const int AdaptiveLoopFilter::m_pattern9x9Sym_7[25] =
{
               13,
           21, 22, 23,
       29, 30, 31, 32, 33,
   37, 38, 39, 40, 39, 38, 37,
       33, 32, 31, 30, 29,
           23, 22, 21,
               13

};

const int AdaptiveLoopFilter::m_pattern9x9Sym_5[13] =
{
          22,
      30, 31, 32,
  38, 39, 40, 39, 38,
      32, 31, 30,
          22,
 };



//MOVE TO ENCODER: //TODO

const int AdaptiveLoopFilter::m_mapTypeToNumOfTaps[ m_NO_TEST_FILT] =
{
  5, 7, 9
};


const int* AdaptiveLoopFilter::m_patternTab[m_NO_TEST_FILT] =
{
   m_pattern5x5Sym, m_pattern7x7Sym, m_pattern9x9Sym
};


const int* AdaptiveLoopFilter::m_weightsTab[m_NO_TEST_FILT] =
{
   m_weights5x5Sym, m_weights7x7Sym, m_weights9x9Sym
};

const int AdaptiveLoopFilter::m_sqrFiltLengthTab[m_NO_TEST_FILT] =
{
   m_SQR_FILT_LENGTH_5SYM, m_SQR_FILT_LENGTH_7SYM, m_SQR_FILT_LENGTH_9SYM
};

const int* AdaptiveLoopFilter::m_patternMapTab[m_NO_TEST_FILT] =
{
    m_pattern5x5Sym_Quart, m_pattern7x7Sym_Quart, m_pattern9x9Sym_Quart
};

const int AdaptiveLoopFilter::m_flTab[m_NO_TEST_FILT] =
{
  5/2, 7/2, 9/2
};



#if GALF
const int AdaptiveLoopFilter::depthInt9x9Cut[21] =
{
  1,
  1, 2, 1,
  1, 2, 3, 2, 1,
  1, 2, 3, 4, 3, 2, 1,
  1, 2, 3, 4, 5,
};

const int AdaptiveLoopFilter::depthInt7x7Cut[14] =
{
  1,
  1, 2, 1,
  1, 2, 3, 2, 1,
  1, 2, 3, 4, 4
};


const int AdaptiveLoopFilter::depthInt5x5Cut[8] =
{
  1,
  1, 2, 1,
  1, 2, 3, 3
};
#else
const int AdaptiveLoopFilter::m_depthInt9x9Sym[21] =
{
           5, 6, 5,
        5, 6, 7, 6, 5,
     5, 6, 7, 8, 7, 6, 5,
  5, 6, 7, 8, 9, 9
};

const int AdaptiveLoopFilter::m_depthInt7x7Sym[14] =
{
           4,
        4, 5, 4,
     4, 5, 6, 5, 4,
  4, 5, 6, 7, 7
};

const int AdaptiveLoopFilter::m_depthInt5x5Sym[8] =
{
        3,
     3, 4, 3,
  3, 4, 5, 5
};
#endif

const int* AdaptiveLoopFilter::m_pDepthIntTab[m_NO_TEST_FILT] =
{
#if GALF
  depthInt5x5Cut, depthInt7x7Cut, depthInt9x9Cut
#else
  m_depthInt5x5Sym, m_depthInt7x7Sym, m_depthInt9x9Sym
#endif
};

//END MOVE TO ENCODER






// scaling factor for quantization of filter coefficients (9x9)
const int AdaptiveLoopFilter::m_aiSymmetricMag9x9[41] =
{
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 1
};

// scaling factor for quantization of filter coefficients (7x7)
const int AdaptiveLoopFilter::m_aiSymmetricMag7x7[25] =
{
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 1
};

// scaling factor for quantization of filter coefficients (5x5)
const int AdaptiveLoopFilter::m_aiSymmetricMag5x5[13] =
{
  2, 2, 2, 2, 2,
  2, 2, 2, 2, 2,
  2, 2, 1
};

const int AdaptiveLoopFilter::m_aiSymmetricMag9x7[32] =
{
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 1
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

AdaptiveLoopFilter::AdaptiveLoopFilter()
{
  m_img_height        = 0;
  m_img_width         = 0;
  m_nInputBitDepth    = 0;
  m_nInternalBitDepth = 0;
  m_nBitIncrement     = 0;
  m_nIBDIMax          = 0;
  m_uiMaxTotalCUDepth = 0;
  m_uiMaxCUWidth      = 0;
  m_uiNumCUsInFrame   = 0;

  m_varImgMethods = nullptr; //TODO remove, only m_imgY_var is used
  m_imgY_var      = nullptr;
  m_imgY_temp     = nullptr;
  m_imgY_ver      = nullptr;
  m_imgY_hor      = nullptr;
  m_imgY_dig0     = nullptr;
  m_imgY_dig1     = nullptr;
  m_filterCoeffFinal = NULL;
  m_filterCoeffSym          = nullptr;
  m_filterCoeffPrevSelected = nullptr;
  m_filterCoeffTmp          = nullptr;
  m_filterCoeffSymTmp       = nullptr;
  m_filterCoeffShort        = nullptr;

  m_isGALF        = false;
  m_wasCreated    = false;
  m_isDec           = true;


}

void AdaptiveLoopFilter:: xError(const char *text, int code)
{
  fprintf(stderr, "%s\n", text);
  exit(code);
}

void AdaptiveLoopFilter:: no_mem_exit(const char *where)
{
  char errortext[200];
  sprintf(errortext, "Could not allocate memory: %s",where);
  xError (errortext, 100);
}

void AdaptiveLoopFilter::initMatrix_Pel(Pel ***m2D, int d1, int d2)
{
  int i;

  if(!(*m2D = (Pel **) calloc(d1, sizeof(Pel *))))
    THROW("initMatrix_Pel: memory allocation problem\n");
  if(!((*m2D)[0] = (Pel *) calloc(d1 * d2, sizeof(Pel))))
    THROW("initMatrix_Pel: memory allocation problem\n");

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

void AdaptiveLoopFilter::initMatrix_int(int ***m2D, int d1, int d2)
{
  int i;

  if(!(*m2D = (int **) calloc(d1, sizeof(int *))))
    THROW("initMatrix_int: memory allocation problem\n");
  if(!((*m2D)[0] = (int *) calloc(d1 * d2, sizeof(int))))
    THROW("initMatrix_int: memory allocation problem\n");

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

void AdaptiveLoopFilter::initMatrix_short(short ***m2D, int d1, int d2)
{
  int i;

  if(!(*m2D = (short **) calloc(d1, sizeof(short *))))
    THROW("initMatrix_int: memory allocation problem\n");
  if(!((*m2D)[0] = (short *) calloc(d1 * d2, sizeof(short))))
    THROW("initMatrix_int: memory allocation problem\n");

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

void AdaptiveLoopFilter::destroyMatrix_short(short **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      THROW("destroyMatrix_int: memory free problem\n");
    free(m2D);
  }
}

void AdaptiveLoopFilter::destroyMatrix_int(int **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      THROW("destroyMatrix_int: memory free problem\n");
    free(m2D);
  }
}

void AdaptiveLoopFilter::destroyMatrix_Pel(Pel **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      THROW("destroyMatrix_Pel: memory free problem\n");
    free(m2D);
  }
}

void AdaptiveLoopFilter::get_mem2Dpel(Pel ***array2D, int rows, int columns)
{
  int i;

  if((*array2D      = (Pel**)calloc(rows,        sizeof(Pel*))) == NULL)
    no_mem_exit("get_mem2Dpel: array2D");
  if(((*array2D)[0] = (Pel* )calloc(rows*columns,sizeof(Pel ))) == NULL)
    no_mem_exit("get_mem2Dpel: array2D");

  for(i=1 ; i<rows ; i++)
    (*array2D)[i] =  (*array2D)[i-1] + columns  ;
}

void AdaptiveLoopFilter::free_mem2Dpel(Pel **array2D)
{
  if (array2D)
  {
    if (array2D[0])
      free (array2D[0]);
    else xError ("free_mem2Dpel: trying to free unused memory",100);

    free (array2D);
  }
}

void AdaptiveLoopFilter::initMatrix_double(double ***m2D, int d1, int d2)
{
  int i;

  if(!(*m2D = (double **) calloc(d1, sizeof(double *))))
    THROW("initMatrix_double: memory allocation problem\n");
  if(!((*m2D)[0] = (double *) calloc(d1 * d2, sizeof(double))))
    THROW("initMatrix_double: memory allocation problem\n");

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

void AdaptiveLoopFilter::initMatrix3D_double(double ****m3D, int d1, int d2, int d3)
{
  int  j;

  if(!((*m3D) = (double ***) calloc(d1, sizeof(double **))))
    THROW("initMatrix3D_double: memory allocation problem\n");

  for(j = 0; j < d1; j++)
    initMatrix_double((*m3D) + j, d2, d3);
}


void AdaptiveLoopFilter::initMatrix4D_double(double *****m4D, int d1, int d2, int d3, int d4)
{
  int  j;

  if(!((*m4D) = (double ****) calloc(d1, sizeof(double ***))))
    THROW("initMatrix4D_double: memory allocation problem\n");

  for(j = 0; j < d1; j++)
    initMatrix3D_double((*m4D) + j, d2, d3, d4);
}


void AdaptiveLoopFilter::destroyMatrix_double(double **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      THROW("destroyMatrix_double: memory free problem\n");
    free(m2D);
  }
  else
  {
    THROW("destroyMatrix_double: memory free problem\n");
  }
}

void AdaptiveLoopFilter::destroyMatrix3D_double(double ***m3D, int d1)
{
  int i;

  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix_double(m3D[i]);
    free(m3D);
  }
  else
  {
    THROW("destroyMatrix3D_double: memory free problem\n");
  }
}


void AdaptiveLoopFilter::destroyMatrix4D_double(double ****m4D, int d1, int d2)
{
  int  j;

  if(m4D)
  {
    for(j = 0; j < d1; j++)
      destroyMatrix3D_double(m4D[j], d2);
    free(m4D);
  }
  else
  {
    THROW("destroyMatrix4D_double: memory free problem\n");
  }
}

void AdaptiveLoopFilter::create( const int iPicWidth, const int iPicHeight, const ChromaFormat chromaFormatIDC, const int uiMaxCUWidth, const uint32_t uiMaxCUHeight, const uint32_t uiMaxCUDepth, const int nInputBitDepth, const int nInternalBitDepth, const int numberOfCTUs )
{
  m_nInputBitDepth    = nInputBitDepth;
  m_nInternalBitDepth = nInternalBitDepth;
#if DISTORTION_LAMBDA_BUGFIX
  m_nBitIncrement = DISTORTION_PRECISION_ADJUSTMENT(nInternalBitDepth);
#else
  m_nBitIncrement = nInternalBitDepth - 8;   // according to ALF on HM-3
#endif
  m_nIBDIMax          = ( 1 << m_nInternalBitDepth ) - 1;

  m_uiMaxTotalCUDepth = uiMaxCUDepth;
  m_uiMaxCUWidth      = uiMaxCUWidth;
  m_uiNumCUsInFrame   = numberOfCTUs;


  if( m_wasCreated )
  {
    CHECK( m_img_height != iPicHeight, "ALF: wrong init" );
    CHECK( m_img_width  != iPicWidth,  "ALF: wrong init" );
    return;
  }

  m_tmpRecExtBuf.create( chromaFormatIDC, Area(0, 0, iPicWidth, iPicHeight ), uiMaxCUWidth, m_FILTER_LENGTH >> 1, 0, false);

  m_img_height = iPicHeight;
  m_img_width  = iPicWidth;

  get_mem2Dpel( &m_varImgMethods, m_img_height, m_img_width );
  initMatrix_int(&m_imgY_temp, m_ALF_WIN_VERSIZE+2*m_VAR_SIZE+3, m_ALF_WIN_HORSIZE+2*m_VAR_SIZE+3);
  initMatrix_int(&m_imgY_ver, m_ALF_WIN_VERSIZE+2*m_VAR_SIZE+3, m_ALF_WIN_HORSIZE+2*m_VAR_SIZE+3);
  initMatrix_int(&m_imgY_hor, m_ALF_WIN_VERSIZE+2*m_VAR_SIZE+3, m_ALF_WIN_HORSIZE+2*m_VAR_SIZE+3);

  initMatrix_int(&m_imgY_dig0, m_ALF_WIN_VERSIZE + 2 * m_VAR_SIZE + 3, m_ALF_WIN_HORSIZE + 2 * m_VAR_SIZE + 3);
  initMatrix_int(&m_imgY_dig1, m_ALF_WIN_VERSIZE + 2 * m_VAR_SIZE + 3, m_ALF_WIN_HORSIZE + 2 * m_VAR_SIZE + 3);


  initMatrix_int(&m_filterCoeffSym, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  initMatrix_int(&m_filterCoeffPrevSelected, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  initMatrix_int(&m_filterCoeffTmp, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  initMatrix_int(&m_filterCoeffSymTmp, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  initMatrix_short(&m_filterCoeffShort, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);

  initMatrix_int(&m_filterCoeffFinal, m_NO_VAR_BINS, (m_MAX_SQR_FILT_LENGTH / 2 + 1));

  for( int k = 0; k < E0104_ALF_MAX_TEMPLAYERID; k++)
  {
    for( int i = 0; i < C806_ALF_TEMPPRED_NUM; i++)
    {
      allocALFParam(&m_acStoredAlfPara[k][i]);
    }
  }

  m_wasCreated = true;
}

void AdaptiveLoopFilter::destroy()
{
  if( !m_wasCreated )
    return;
  m_tmpRecExtBuf.destroy();

  destroyMatrix_int(m_imgY_temp);

  destroyMatrix_int(m_imgY_ver);
  destroyMatrix_int(m_imgY_hor);

  destroyMatrix_int(m_imgY_dig0);
  destroyMatrix_int(m_imgY_dig1);

  free_mem2Dpel(m_varImgMethods);

  destroyMatrix_short(m_filterCoeffShort);

  destroyMatrix_int(m_filterCoeffSym);
  destroyMatrix_int(m_filterCoeffPrevSelected);
  destroyMatrix_int(m_filterCoeffTmp);
  destroyMatrix_int(m_filterCoeffSymTmp);

  destroyMatrix_int(m_filterCoeffFinal);

  for( int k = 0; k < E0104_ALF_MAX_TEMPLAYERID; k++)
  {
    for (int i = 0; i < C806_ALF_TEMPPRED_NUM; i++)
    {
      freeALFParam(&m_acStoredAlfPara[k][i]);
    }
    m_storedAlfParaNum[k] = 0;
  }
  m_wasCreated = false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

int AdaptiveLoopFilter::ALFTapHToTapV(int tapH)
{
  return std::min<uint32_t>(tapH, 7);
}

int AdaptiveLoopFilter::ALFFlHToFlV(int flH)
{
#if GALF
  return flH;
#else
  return std::min<uint32_t>(flH, 7/2);
#endif
}

int AdaptiveLoopFilter::ALFTapHToNumCoeff(int tapH)
{
  int num_coeff;

  num_coeff = (int)(tapH*tapH)/4 + 2;
#if GALF
  num_coeff -= 1;
#else
  if (tapH == 9)
    num_coeff -= 1;
  else
    CHECK(tapH >= 9, "Filter defined only for tapH < 0");
#endif
  return num_coeff;
}

// --------------------------------------------------------------------------------------------------------------------
// allocate / free / copy functions
// --------------------------------------------------------------------------------------------------------------------

void AdaptiveLoopFilter::allocALFParam(ALFParam* pAlfParam)
{
  pAlfParam->alf_flag = 0;
  pAlfParam->cu_control_flag = 0;
  pAlfParam->alf_max_depth = 0;
#if JVET_C0038_NO_PREV_FILTERS
  pAlfParam->iAvailableFilters = JVET_C0038_NO_PREV_FILTERS;
  pAlfParam->iPredPattern = 0;
#endif

  pAlfParam->alfCoeffChroma = new int[m_ALF_MAX_NUM_COEF_C];
  ::memset(pAlfParam->alfCoeffChroma, 0, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
  pAlfParam->temporalPredFlag = false;
  pAlfParam->prevIdx = -1;

  pAlfParam->coeff_chroma = new int[m_ALF_MAX_NUM_COEF_C];
  ::memset(pAlfParam->coeff_chroma, 0, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
  pAlfParam->coeffmulti = new int*[m_NO_VAR_BINS];
  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    pAlfParam->coeffmulti[i] = new int[m_ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->coeffmulti[i], 0, sizeof(int)*m_ALF_MAX_NUM_COEF);
  }
  pAlfParam->num_ctus_in_frame = m_uiNumCUsInFrame;
  pAlfParam->maxCodingDepth = m_uiMaxTotalCUDepth;
  pAlfParam->num_alf_cu_flag  = 0;
  pAlfParam->alf_cu_flag      = new bool[(m_uiNumCUsInFrame << ((m_uiMaxTotalCUDepth-1)*2))];
  ::memset(pAlfParam->kMinTab, 0, sizeof(pAlfParam->kMinTab));

  // galf stuff
  pAlfParam->alfCoeffLuma = new int*[m_NO_VAR_BINS];
  for (int i = 0; i<m_NO_VAR_BINS; i++)
  {
    pAlfParam->alfCoeffLuma[i] = new int[m_ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->alfCoeffLuma[i], 0, sizeof(int)*m_ALF_MAX_NUM_COEF);
  }
}

void AdaptiveLoopFilter::freeALFParam(ALFParam* pAlfParam)
{
  if( pAlfParam == nullptr )
    return;

  if (pAlfParam->coeff_chroma != NULL)
  {
    delete[] pAlfParam->coeff_chroma;
    pAlfParam->coeff_chroma = NULL;
  }

  if( pAlfParam->coeffmulti != NULL )
  {
    for (int i=0; i<m_NO_VAR_BINS; i++)
    {
      delete[] pAlfParam->coeffmulti[i];
      pAlfParam->coeffmulti[i] = NULL;
    }
    delete[] pAlfParam->coeffmulti;
    pAlfParam->coeffmulti = NULL;
  }

  if(pAlfParam->alf_cu_flag != NULL)
  {
    delete[] pAlfParam->alf_cu_flag;
    pAlfParam->alf_cu_flag = NULL;
  }

  if( pAlfParam->alfCoeffLuma != NULL )
  {
    for (int i = 0; i<m_NO_VAR_BINS; i++)
    {
      delete[] pAlfParam->alfCoeffLuma[i];
      pAlfParam->alfCoeffLuma[i] = NULL;
    }
    delete[] pAlfParam->alfCoeffLuma;
    pAlfParam->alfCoeffLuma = NULL;
  }

  if( pAlfParam->alfCoeffChroma != NULL )
  {
    delete[] pAlfParam->alfCoeffChroma;
    pAlfParam->alfCoeffChroma = NULL;
  }
}

void AdaptiveLoopFilter::copyALFParam(ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam, bool max_depth_copy )
{
#if COM16_C806_ALF_TEMPPRED_NUM
  if (!pDesAlfParam->temporalPredFlag)
  {
#endif
    pDesAlfParam->alf_flag = pSrcAlfParam->alf_flag;
    pDesAlfParam->cu_control_flag = pSrcAlfParam->cu_control_flag;
    pDesAlfParam->chroma_idc = pSrcAlfParam->chroma_idc;
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif
  pDesAlfParam->tapH = pSrcAlfParam->tapH;
  pDesAlfParam->tapV = pSrcAlfParam->tapV;
  pDesAlfParam->num_coeff = pSrcAlfParam->num_coeff;
  pDesAlfParam->tap_chroma = pSrcAlfParam->tap_chroma;
  pDesAlfParam->num_coeff_chroma = pSrcAlfParam->num_coeff_chroma;

  ::memcpy(pDesAlfParam->coeff_chroma, pSrcAlfParam->coeff_chroma, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
  pDesAlfParam->filterType = pSrcAlfParam->filterType;
  ::memcpy(pDesAlfParam->filterPattern, pSrcAlfParam->filterPattern, sizeof(int)*m_NO_VAR_BINS);
  pDesAlfParam->startSecondFilter = pSrcAlfParam->startSecondFilter;
  pDesAlfParam->filterMode = pSrcAlfParam->filterMode;

  //Coeff send related
  pDesAlfParam->filters_per_group_diff = pSrcAlfParam->filters_per_group_diff; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = pSrcAlfParam->filters_per_group; //this can be updated using codedVarBinsif


#ifdef FORCE0
  if( ! m_isGALF || FORCE0 )
#else
  if( ! m_isGALF )
#endif
  {
    ::memcpy(pDesAlfParam->codedVarBins, pSrcAlfParam->codedVarBins, sizeof(int)*m_NO_VAR_BINS);
    pDesAlfParam->forceCoeff0 = pSrcAlfParam->forceCoeff0;
  }
#if JVET_C0038_NO_PREV_FILTERS
  pDesAlfParam->iAvailableFilters = pSrcAlfParam->iAvailableFilters;
  pDesAlfParam->iPredPattern = pSrcAlfParam->iPredPattern;
  ::memcpy(pDesAlfParam->PrevFiltIdx, pSrcAlfParam->PrevFiltIdx, sizeof(int) *m_NO_VAR_BINS);
#endif
  pDesAlfParam->predMethod = pSrcAlfParam->predMethod;
  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    ::memcpy(pDesAlfParam->coeffmulti[i], pSrcAlfParam->coeffmulti[i], sizeof(int)*m_ALF_MAX_NUM_COEF);
    // galf stuff
    ::memcpy(pDesAlfParam->alfCoeffLuma[i], pSrcAlfParam->alfCoeffLuma[i], sizeof(int)*m_ALF_MAX_NUM_COEF);
  }
  pDesAlfParam->minKStart = pSrcAlfParam->minKStart;
  ::memcpy( pDesAlfParam->kMinTab , pSrcAlfParam->kMinTab , sizeof( pSrcAlfParam->kMinTab ) );

  ::memcpy( pDesAlfParam->mapClassToFilter , pSrcAlfParam->mapClassToFilter , sizeof( pSrcAlfParam->mapClassToFilter ) );

#if COM16_C806_ALF_TEMPPRED_NUM
  if (max_depth_copy)
  {
#endif
    pDesAlfParam->alf_max_depth = pSrcAlfParam->alf_max_depth;
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
  ::memcpy(pDesAlfParam->alfCoeffChroma, pSrcAlfParam->alfCoeffChroma, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
  if (!pDesAlfParam->temporalPredFlag)
  {
#endif
    pDesAlfParam->num_alf_cu_flag = pSrcAlfParam->num_alf_cu_flag;
    ::memcpy(pDesAlfParam->alf_cu_flag, pSrcAlfParam->alf_cu_flag, sizeof(bool)*pSrcAlfParam->num_alf_cu_flag);
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
  if( !pDesAlfParam->temporalPredFlag )
  {
#endif
  pDesAlfParam->temporalPredFlag = pSrcAlfParam->temporalPredFlag;
  pDesAlfParam->prevIdx          = pSrcAlfParam->prevIdx;
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif
}

void AdaptiveLoopFilter::resetALFParam(ALFParam* pDesAlfParam)
{
  if( pDesAlfParam->coeffmulti == nullptr )
  {
    allocALFParam( pDesAlfParam);
  }
  pDesAlfParam->alf_flag = 0;
  pDesAlfParam->cu_control_flag = 0;
  pDesAlfParam->chroma_idc = 0;
  pDesAlfParam->tapH = 0;
  pDesAlfParam->tapV = 0;
  pDesAlfParam->num_coeff = 0;
  if( m_isGALF )
  {
    pDesAlfParam->tap_chroma = m_ALF_MAX_NUM_TAP_C;
  }
  else
  {
    pDesAlfParam->tap_chroma = 0;
    pDesAlfParam->num_coeff_chroma = 0;
  }
#if JVET_C0038_NO_PREV_FILTERS
  pDesAlfParam->iAvailableFilters = JVET_C0038_NO_PREV_FILTERS;
  pDesAlfParam->iPredPattern = 0;
  ::memset(pDesAlfParam->PrevFiltIdx, 0, sizeof(int)*m_NO_VAR_BINS);
#endif
  ::memset(pDesAlfParam->coeff_chroma, 0, sizeof(int)*m_ALF_MAX_NUM_COEF_C);

  pDesAlfParam->filterType = ALF_FILTER_SYM_5;
  ::memset(pDesAlfParam->filterPattern, 0, sizeof(int)*m_NO_VAR_BINS);
  pDesAlfParam->startSecondFilter = 0;
  pDesAlfParam->filterMode = ALF_MULTIPLE_FILTERS;
  pDesAlfParam->minKStart  = 0;
  ::memset(pDesAlfParam->kMinTab , 0 , sizeof( pDesAlfParam->kMinTab ) );

  //Coeff send related
  pDesAlfParam->filters_per_group_diff = 0; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = 0; //this can be updated using codedVarBins
#if !GALF
  ::memset(pDesAlfParam->codedVarBins, 0, sizeof(int)*m_NO_VAR_BINS);
  pDesAlfParam->forceCoeff0 = 0;
#endif
  pDesAlfParam->predMethod = 0;
  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    ::memset(pDesAlfParam->coeffmulti[i], 0, sizeof(int)*m_ALF_MAX_NUM_COEF);
#if GALF
    ::memset(pDesAlfParam->alfCoeffLuma[i], 0, sizeof(int)*m_ALF_MAX_NUM_COEF);
#endif
  }
  ::memset( pDesAlfParam->mapClassToFilter , 0 , sizeof( pDesAlfParam->mapClassToFilter ) );

  pDesAlfParam->num_alf_cu_flag  = 0;
#if COM16_C806_ALF_TEMPPRED_NUM
  ::memset(pDesAlfParam->alfCoeffChroma, 0, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
  pDesAlfParam->temporalPredFlag = false;
  pDesAlfParam->prevIdx = -1;
#else
  pDesAlfParam->temporalPredFlag = 0;
  pDesAlfParam->prevIdx          = 0;
#endif


}

// --------------------------------------------------------------------------------------------------------------------
// interface function for actual ALF process
// --------------------------------------------------------------------------------------------------------------------

/**
 \param cs            coding structure (CodingStructure) class (input/output)
 \param pcAlfParam    ALF parameter
*/
void AdaptiveLoopFilter::ALFProcess( CodingStructure& cs, ALFParam* pcAlfParam
                                    )
{


  if(!pcAlfParam->alf_flag)
  {
    return;
  }
  PelUnitBuf recUnitBuf = cs.getRecoBuf();

  // copy the clip ranges
  m_clpRngs = cs.slice->clpRngs();
  m_isGALF  = cs.sps->getSpsNext().getGALFEnabled();

  m_tmpRecExtBuf.copyFrom( recUnitBuf );
  PelUnitBuf tmpRecExt = m_tmpRecExtBuf.getBuf( cs.area );
  tmpRecExt.extendBorderPel( m_FILTER_LENGTH >> 1 );

  xALFLuma( cs, pcAlfParam,  tmpRecExt, recUnitBuf );

  if(pcAlfParam->chroma_idc)
  {
    if( m_isGALF )
    {
#if COM16_C806_ALF_TEMPPRED_NUM
      initVarForChroma(pcAlfParam, (pcAlfParam->temporalPredFlag ? true : false)
                      );
#else
      initVarForChroma(pcAlfParam, false);
#endif
    }
    else
    {
      predictALFCoeffChroma(pcAlfParam);
    }

#if COM16_C806_ALF_TEMPPRED_NUM
    memcpy(pcAlfParam->alfCoeffChroma, pcAlfParam->coeff_chroma, sizeof(int)*m_ALF_MAX_NUM_COEF_C);
#endif
    xALFChroma(pcAlfParam, tmpRecExt, recUnitBuf);
  }

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_ALF, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "ALF" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// ALF for luma
// --------------------------------------------------------------------------------------------------------------------
void AdaptiveLoopFilter::xALFLuma( CodingStructure& cs, ALFParam* pcAlfParam, PelUnitBuf& recSrcExt, PelUnitBuf& recDst )
{
  //Decode and reconst filter coefficients
  xDecodeFilter( pcAlfParam );
  m_imgY_var       = m_varImgMethods;

  if( pcAlfParam->cu_control_flag )
  {
    xCUAdaptive( cs, recSrcExt, recDst, pcAlfParam 
    );
  }
  else
  {
    xFilterFrame(recSrcExt, recDst, pcAlfParam->filterType
    );
  }
}


// --------------------------------------------------------------------------------------------------------------------
// ALF for chroma
// --------------------------------------------------------------------------------------------------------------------

void AdaptiveLoopFilter::xALFChroma( ALFParam* pcAlfParam,  const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf )
{
  if((pcAlfParam->chroma_idc>>1) & 0x01)
  {
    if( m_isGALF )
      xFrameChromaGalf(pcAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cb);
    else
      xFrameChromaAlf(pcAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cb);
  }
  else
  {
    //TODO is this needed?
    recUnitBuf.get(COMPONENT_Cb).copyFrom( recExtBuf.get(COMPONENT_Cb) );
  }

  if(pcAlfParam->chroma_idc&0x01)
  {
    if( m_isGALF )
      xFrameChromaGalf(pcAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cr);
    else
      xFrameChromaAlf(pcAlfParam, recExtBuf, recUnitBuf, COMPONENT_Cr);
  }
  else
  {
    //TODO is this needed?
    recUnitBuf.get(COMPONENT_Cr).copyFrom( recExtBuf.get(COMPONENT_Cr) );
  }
}

void AdaptiveLoopFilter::xFilterFrame(PelUnitBuf& recSrcExt, PelUnitBuf& recDst, AlfFilterType filtType
  )
{
  int i, j;
  for (i = 0; i < m_img_height; i += m_ALF_WIN_VERSIZE)
  {
    for (j = 0; j < m_img_width; j += m_ALF_WIN_HORSIZE)
    {
      const int nHeight = std::min(i + m_ALF_WIN_VERSIZE, m_img_height) - i;
      const int nWidth = std::min(j + m_ALF_WIN_HORSIZE, m_img_width) - j;
      Area blk_cur(j, i, nWidth, nHeight);

      if (m_isGALF)
      {
        xClassifyByGeoLaplacian(m_imgY_var, recSrcExt.get(COMPONENT_Y), m_FILTER_LENGTH / 2, m_VAR_SIZE, blk_cur);
        xFilterBlkGalf(recDst, recSrcExt, blk_cur, filtType, COMPONENT_Y);
      }
      else
      {
        xClassifyByLaplacian(m_imgY_var, recSrcExt.get(COMPONENT_Y), m_FILTER_LENGTH / 2, m_VAR_SIZE, blk_cur);
        xFilterBlkAlf(recDst.get(COMPONENT_Y), recSrcExt.get(COMPONENT_Y), blk_cur, filtType);
      }

    }
  }
}

void AdaptiveLoopFilter::xCUAdaptive( CodingStructure& cs, const PelUnitBuf &recExtBuf, PelUnitBuf &recBuf, ALFParam* pcAlfParam
  )
{
  const SPS*     sps            = cs.slice->getSPS();
  const unsigned widthInCtus    = cs.pcv->widthInCtus;
  const unsigned maxCUSize      = sps->getMaxCUWidth();
  const unsigned uiAlfCtrlDepth = pcAlfParam->alf_max_depth;
  const unsigned alfCtrlSize    = maxCUSize >> uiAlfCtrlDepth;

  const unsigned imgWidth       = recBuf.get(COMPONENT_Y).width;
  const unsigned imgHeight      = recBuf.get(COMPONENT_Y).height;

  Partitioner* partitioner = PartitionerFactory::get( *cs.slice );

  uint32_t indx = 0;
  for( uint32_t uiCTUAddr = 0; uiCTUAddr < cs.pcv->sizeInCtus ; uiCTUAddr++ )
  {
    const unsigned  ctuXPosInCtus         = uiCTUAddr % widthInCtus;
    const unsigned  ctuYPosInCtus         = uiCTUAddr / widthInCtus;

    Position ctuPos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    UnitArea ctuArea(cs.area.chromaFormat, Area( ctuPos.x, ctuPos.y, maxCUSize, maxCUSize ) );

    for( auto &currCU : cs.traverseCUs( ctuArea, CH_L ) )
    {
#if JVET_K0076_CPR_DT
      if (currCU.chType != CH_L)
        break;
#endif
      const Position&    cuPos   = currCU.lumaPos();
      const int          qtDepth = currCU.qtDepth;
      const unsigned     qtSize  = maxCUSize >> qtDepth;
      const Position     qtPos0  = Position((cuPos.x / qtSize) * qtSize, (cuPos.y / qtSize) * qtSize);
      const Position   ctrlPos0  = Position(cuPos.x / alfCtrlSize * alfCtrlSize, cuPos.y / alfCtrlSize * alfCtrlSize);

      if ((qtDepth >= uiAlfCtrlDepth && cuPos == ctrlPos0) || ((qtDepth < uiAlfCtrlDepth) && cuPos == qtPos0))
      {
        Area blk;
        if (qtDepth >= uiAlfCtrlDepth)
        {
          blk = Area(cuPos.x, cuPos.y, std::min(alfCtrlSize, imgWidth - cuPos.x), std::min(alfCtrlSize, imgHeight - cuPos.y));
        }
        else
        {
          blk = Area(cuPos.x, cuPos.y, std::min(qtSize, imgWidth - cuPos.x), std::min(qtSize, imgHeight - cuPos.y));
        }

        CHECK(indx >= pcAlfParam->num_alf_cu_flag, "Exceeded the number of num-cu flags");

        if (pcAlfParam->alf_cu_flag[indx] == 1)
        {
          if (m_isGALF)
          {
            xClassifyByGeoLaplacian(m_imgY_var, recExtBuf.get(COMPONENT_Y), m_FILTER_LENGTH / 2, m_VAR_SIZE, blk);
            xFilterBlkGalf(recBuf, recExtBuf, blk, pcAlfParam->filterType, COMPONENT_Y);
          }
          else
          {
            xClassifyByLaplacian(m_imgY_var, recExtBuf.get(COMPONENT_Y), m_FILTER_LENGTH / 2, m_VAR_SIZE, blk);
            xFilterBlkAlf(recBuf.get(COMPONENT_Y), recExtBuf.get(COMPONENT_Y), blk, pcAlfParam->filterType);
          }
        }
        else
        {
          recBuf.get(COMPONENT_Y).subBuf(blk.pos(), blk.size()).copyFrom(recExtBuf.get(COMPONENT_Y).subBuf(blk.pos(), blk.size()));
        }
        indx++;
      }
    }
  }

  delete partitioner;
}


// --------------------------------------------------------------------------------------------------------------------
// reconstruction of filter coefficients
// ------------------------------------

//copy filter to m_filterCoeffPrevSelected
void AdaptiveLoopFilter::getCurrentFilter( int **filterCoeffSym, ALFParam* pcAlfParam )
{
  int i, k, varInd;
  const int *patternMapTab[3]={ m_pattern5x5Sym_Quart, m_pattern7x7Sym_Quart, m_pattern9x9Sym_Quart  };
#if GALF
  int ** filterCoeffFinal;
  int factor = (1 << (AdaptiveLoopFilter::m_NUM_BITS - 1));
  int iMaxNumCoeff = (m_MAX_SQR_FILT_LENGTH / 2 + 1);

  initMatrix_int(&filterCoeffFinal, m_NO_VAR_BINS, iMaxNumCoeff);

#if !JVET_C0038_NO_PREV_FILTERS
  for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    memset(m_filterCoeffPrevSelected[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    memset(filterCoeffFinal[varInd], 0, sizeof(int)*(iMaxNumCoeff - 1));
    filterCoeffFinal[varInd][(iMaxNumCoeff - 1)] = factor;
  }
#else

  int filterNo;
  for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    memset(m_filterCoeffPrevSelected[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    memset(filterCoeffFinal[varInd], 0, sizeof(int)*iMaxNumCoeff);
  }

  if (pcAlfParam->iPredPattern)
  {
    for (varInd = 0; varInd< m_NO_VAR_BINS; ++varInd)
    {
      if (pcAlfParam->PrevFiltIdx[varInd])
      {
        int iPrevFiltIdx = pcAlfParam->PrevFiltIdx[varInd] - 1;
        for (i = 0; i < iMaxNumCoeff; i++)
        {
          filterNo = varInd*JVET_C0038_NO_PREV_FILTERS + iPrevFiltIdx;
            filterCoeffFinal[varInd][i] = m_ALFfilterCoeffFixed[filterNo][i];
        }
      }
      else
      {
        memset(filterCoeffFinal[varInd], 0, sizeof(int)*(iMaxNumCoeff - 1));
        filterCoeffFinal[varInd][iMaxNumCoeff - 1] = factor;
      }
    }
  }
  else
  {
    for (varInd = 0; varInd< m_NO_VAR_BINS; ++varInd)
    {
      memset(filterCoeffFinal[varInd], 0, (iMaxNumCoeff - 1)*sizeof(int));
      filterCoeffFinal[varInd][(iMaxNumCoeff - 1)] = factor;
    }
  }
#endif

  const int *patternMap = patternMapTab[pcAlfParam->filterType];

  for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    k = 0;
    for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
    {
      if (m_pattern9x9Sym_Quart[i] > 0 && patternMap[i]>0)
      {
        m_filterCoeffPrevSelected[varInd][i] = (filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i] - 1] + filterCoeffSym[pcAlfParam->filterPattern[varInd]][patternMap[i] - 1]);
        pcAlfParam->alfCoeffLuma[varInd][m_pattern9x9Sym_Quart[i] - 1] = m_filterCoeffPrevSelected[varInd][i];
        k++;
      }
      else if (m_pattern9x9Sym_Quart[i] > 0)
      {
        m_filterCoeffPrevSelected[varInd][i] = filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i] - 1];
        pcAlfParam->alfCoeffLuma[varInd][m_pattern9x9Sym_Quart[i] - 1] = filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i] - 1];
      }
      else
      {
        m_filterCoeffPrevSelected[varInd][i] = 0;
      }
    }
  }
  int iNumCoeffMinus1 = m_MAX_SQT_FILT_SYM_LENGTH - 1, quantCoeffSum = 0;
  const int * weights = AdaptiveLoopFilter::m_weightsTab[2];

  for (varInd = 0; varInd < m_NO_VAR_BINS; ++varInd)
  {
    quantCoeffSum = 0;
    for (i = 0; i < iNumCoeffMinus1; i++)
    {
      quantCoeffSum += weights[i] * pcAlfParam->alfCoeffLuma[varInd][i];
    }

    pcAlfParam->alfCoeffLuma[varInd][iNumCoeffMinus1] = factor - quantCoeffSum;
    m_filterCoeffPrevSelected[varInd][m_MAX_SQR_FILT_LENGTH - 1] = factor - quantCoeffSum;
  }

  destroyMatrix_int(filterCoeffFinal);
#else
  for (varInd=0; varInd <m_NO_VAR_BINS; ++varInd)
  {
    memset(m_filterCoeffPrevSelected[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
  }
  const int *patternMap=patternMapTab[pcAlfParam->filterType];
  for (varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    k=0;
    for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
    {
      if (patternMap[i] > 0)
      {
#if COM16_C806_ALF_TEMPPRED_NUM
        pcAlfParam->alfCoeffLuma[varInd][k] =
#endif
        m_filterCoeffPrevSelected[varInd][i] = filterCoeffSym[pcAlfParam->mapClassToFilter[varInd]][k];
        k++;
      }
      else
      {
        m_filterCoeffPrevSelected[varInd][i]=0;
      }
    }
  }
#endif
}



void AdaptiveLoopFilter::xDecodeFilter( ALFParam* pcAlfParam )
{
  int **pfilterCoeffSym = m_filterCoeffSym;
#if COM16_C806_ALF_TEMPPRED_NUM
  int i;
  if (pcAlfParam->temporalPredFlag)
  {
    for (i = 0; i < m_NO_VAR_BINS; i++)
    {
      memcpy(pfilterCoeffSym[i], &pcAlfParam->alfCoeffLuma[i][0], sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    }

    int k, varInd;
    const int *patternMapTab[3] ={m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart};
    {
      for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
      {
        memset(m_filterCoeffPrevSelected[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
      }
      const int *patternMap = patternMapTab[2-pcAlfParam->filterType];
      for (varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
      {
        k = 0;
        for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
        {
          if (patternMap[i]>0)
          {
            m_filterCoeffPrevSelected[varInd][i] = pcAlfParam->alfCoeffLuma[varInd][k];
            k++;
          }
          else
          {
            m_filterCoeffPrevSelected[varInd][i] = 0;
          }
        }
      }
    }
  }
  else
  {
#endif
    reconstructFilterCoeffs(pcAlfParam, pfilterCoeffSym);
    //pFilterCoeffSym -> m_filterCoeffPrevSelected
    getCurrentFilter(pfilterCoeffSym, pcAlfParam);
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif

  int *coef;
#if GALF
  int centerCoef = m_MAX_SQR_FILT_LENGTH - 1;

  for (int varInd = 0; varInd<m_NO_VAR_BINS; ++varInd)
  {
    coef = m_filterCoeffPrevSelected[varInd];
    for( int i = 0; i < centerCoef; i++)
    {
      m_filterCoeffShort[varInd][i] = (int16_t)coef[i];
    }
    m_filterCoeffShort[varInd][centerCoef] = (int16_t)coef[centerCoef];
  }
#else
  int maxPxlVal = m_nIBDIMax;
  int maxSampleValue, minSampleValue = 0;
  int clipRange[2] = { 0, 0 };
  int sumCoef[2];

  int numBitsMinus1 = m_NUM_BITS-1;
  int offset        = (1<<(m_NUM_BITS-2));
  int lastCoef      = m_MAX_SQR_FILT_LENGTH-1;
  int centerCoef    = m_MAX_SQR_FILT_LENGTH-2;

  //m_filterCoeffPrevSelected -> m_filterCoeffShort
  for(int varInd=0; varInd < m_NO_VAR_BINS; ++varInd)
  {
    coef = m_filterCoeffPrevSelected[varInd];
    sumCoef[0] = 0;
    sumCoef[1] = 0;
    for(int i = 0; i < centerCoef; i++)
    {
      CHECK( coef[i] > 32767 || coef[i] < -32768, "ALF: Coeffs out of bound" );

      m_filterCoeffShort[varInd][i]   = (int16_t) coef[i];
      sumCoef[ coef[i] > 0 ? 0 : 1 ] += (coef[i] << 1);
    }
    CHECK( coef[centerCoef] > 32767 || coef[centerCoef] < -32768, "ALF: Coeffs out of bound" );
    CHECK( coef[lastCoef]   > 32767 || coef[lastCoef]   < -32768, "ALF: Coeffs out of bound" );

    m_filterCoeffShort[varInd][centerCoef] = (int16_t)coef[centerCoef];
    m_filterCoeffShort[varInd][lastCoef]   = (int16_t)coef[lastCoef];

    sumCoef[ coef[centerCoef] > 0 ? 0 : 1 ] += coef[centerCoef];

    maxSampleValue = ( maxPxlVal * sumCoef[0] + coef[lastCoef] + offset ) >> numBitsMinus1;
    minSampleValue = ( maxPxlVal * sumCoef[1] + coef[lastCoef] + offset ) >> numBitsMinus1;

    if( clipRange[0] < maxSampleValue )
    {
      clipRange[0] = maxSampleValue;
    }
    if( clipRange[1] > minSampleValue )
    {
      clipRange[1] = minSampleValue;
    }
  }
#endif

  //TODO move to calling function
  memset( m_imgY_temp[0], 0, sizeof(int)*(m_ALF_WIN_VERSIZE+2*m_VAR_SIZE)*(m_ALF_WIN_HORSIZE+2*m_VAR_SIZE));
  m_imgY_var = m_varImgMethods;
}


void AdaptiveLoopFilter::reconstructFilterCoeffs( ALFParam* pcAlfParam, int **pfilterCoeffSym )
{
  //todo true only if not forceCoeff0
//  CHECK( pcAlfParam->forceCoeff0, "ForceCoeff0 not implemented yet" )
  pcAlfParam->filters_per_group_diff = pcAlfParam->filters_per_group;

  int i, src, ind;

  // Copy non zero filters to filterCoeffTmp
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    for(i = 0; i < pcAlfParam->num_coeff; i++)
    {
      m_filterCoeffTmp[ind][i] = pcAlfParam->coeffmulti[ind][i];
    }
  }

  // Undo prediction
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    if( (!pcAlfParam->predMethod) || (ind == 0) )
    {
      memcpy(m_filterCoeffSymTmp[ind],m_filterCoeffTmp[ind],sizeof(int)*pcAlfParam->num_coeff);
    }
    else
    {
      // Prediction
      for(i = 0; i < pcAlfParam->num_coeff; ++i)
      {
        m_filterCoeffSymTmp[ind][i] = (int)(m_filterCoeffTmp[ind][i] + m_filterCoeffSymTmp[ind - 1][i]);
      }
    }
  }

  // Add filters forced to zero
  if(pcAlfParam->forceCoeff0)
  {
#if !FORCE0
    CHECK(pcAlfParam->filters_per_group_diff >= pcAlfParam->filters_per_group, "ALF: Inconsistent for forceCoeff0 ");
#endif
    src = 0;
    for (ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
    {
      if (pcAlfParam->codedVarBins[ind])
      {
        ++src;
      }
    }
#if FORCE0
    pcAlfParam->filters_per_group_diff = src;
    CHECK(pcAlfParam->filters_per_group_diff >= pcAlfParam->filters_per_group, "ALF: Inconsistent for forceCoeff0");
#else
    CHECK(src != pcAlfParam->filters_per_group_diff, "ALF: Inconsistent for forceCoeff0")
#endif
  }
  else
  {
    CHECK(pcAlfParam->filters_per_group != pcAlfParam->filters_per_group_diff, "ALF: Inconsistent for forceCoeff0 disabled");
  }

  for (ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
  {
    memcpy(pfilterCoeffSym[ind], m_filterCoeffSymTmp[ind], sizeof(int)*pcAlfParam->num_coeff);
  }
}

void AdaptiveLoopFilter::initVarForChroma(ALFParam* pcAlfParam, bool bUpdatedDCCoef)
{
  int k, i;
  //initilization for clip operation in subfilterFrame()

  int filtNo = pcAlfParam->tap_chroma == 9 ? 2 : (pcAlfParam->tap_chroma == 7 ? 1 : 0);
  if (!bUpdatedDCCoef)
  {
    const int * weights = AdaptiveLoopFilter::m_weightsTab[filtNo];
    int quantCoeffSum = 0;
    int factor = (1 << (m_NUM_BITS - 1));
    for (i = 0; i< pcAlfParam->num_coeff_chroma - 1; i++)
    {
      quantCoeffSum += weights[i] * pcAlfParam->coeff_chroma[i];
    }
    pcAlfParam->coeff_chroma[pcAlfParam->num_coeff_chroma - 1] = factor - quantCoeffSum;
  }
  //fill in the ALF coefficients
  const int* patternMap = m_patternMapTab[filtNo];
  k = 0;
  for (i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
  {
    if (patternMap[i]>0)
    {
      m_filterCoeffShort[0][i] = pcAlfParam->coeff_chroma[k];
      k++;
    }
    else
    {
      m_filterCoeffShort[0][i] = 0;
    }
  }
}

void AdaptiveLoopFilter::predictALFCoeffChroma( ALFParam* pAlfParam )
{
  int i, sum, pred, tap, N;
  const int* pFiltMag = NULL;

  tap = pAlfParam->tap_chroma;
  switch(tap)
  {
    case 5:
      pFiltMag = m_aiSymmetricMag5x5;
      break;
    case 7:
      pFiltMag = m_aiSymmetricMag7x7;
      break;
    case 9:
      pFiltMag = m_aiSymmetricMag9x9;
      break;
    default:
      THROW( "ALF: Filter not defined");
      break;
  }
  N = (tap*tap+1)>>1;
  sum=0;
  for(i=0; i<N;i++)
  {
    sum+=pFiltMag[i]*pAlfParam->coeff_chroma[i];
  }
  pred=(1<<m_ALF_NUM_BIT_SHIFT)-(sum-pAlfParam->coeff_chroma[N-1]);
  pAlfParam->coeff_chroma[N-1]=pred-pAlfParam->coeff_chroma[N-1];
}

void AdaptiveLoopFilter::resetALFPredParam(ALFParam *pAlfParam, bool bIntra)
{
  //reset to 9x9 filter shape
  pAlfParam->filterType = (AlfFilterType)2;
  pAlfParam->tapV = 9;
  pAlfParam->num_coeff = AdaptiveLoopFilter::m_SQR_FILT_LENGTH_9SYM;
}

void AdaptiveLoopFilter::setNumCUsInFrame(uint32_t uiNumCUsInFrame)
{
  m_uiNumCUsInFrame = uiNumCUsInFrame;
}





//***********************************
// CLASSIFICATION
//***********************************

void AdaptiveLoopFilter::xClassify(Pel** classes, const CPelBuf& recSrcBuf, int pad_size, int fl)
{
  Area blk(0, 0, recSrcBuf.width, recSrcBuf.height);
#if GALF
  xClassifyByGeoLaplacian(classes, recSrcBuf, pad_size, fl, blk);
#else
  xClassifyByLaplacian(classes, recSrcBuf, pad_size, fl, blk);
#endif
}

static Pel Clip_post(int high, int val)
{
  return (Pel)(((val > high)? high: val));
}

void AdaptiveLoopFilter::xClassifyByGeoLaplacian(Pel** classes, const CPelBuf& srcLumaBuf, int pad_size, int fl, const Area& blk)
{
  int i, j;

  int end_height = blk.pos().y + blk.height;
  int end_width = blk.pos().x + blk.width;

  for (i = blk.pos().y; i < end_height; i += m_ALF_WIN_VERSIZE)
  {
    for (j = blk.pos().x; j < end_width; j += m_ALF_WIN_HORSIZE)
    {
      int nHeight = std::min(i + m_ALF_WIN_VERSIZE, end_height) - i;
      int nWidth = std::min(j + m_ALF_WIN_HORSIZE, end_width) - j;

      Area blk_cur;
      blk_cur = Area(j, i, nWidth, nHeight);
      xClassifyByGeoLaplacianBlk(classes, srcLumaBuf, pad_size, fl, blk_cur);
    }
  }
}
void AdaptiveLoopFilter::xClassifyByGeoLaplacianBlk(Pel** classes, const CPelBuf& srcLumaBuf, int pad_size, int fl, const Area& blk)
{
  const int img_stride = srcLumaBuf.stride;
  const Pel* srcExt = srcLumaBuf.buf;
  int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  fl = 2;
  int i, j;
#if DISTORTION_LAMBDA_BUGFIX
  int shift = (11 + m_nInternalBitDepth - 8);
#else
#if FULL_NBIT
  int shift = (11 + m_nBitIncrement + m_nInputBitDepth - 8);
#else
  int shift = (11 + m_nBitIncrement);
#endif
#endif
  int flplusOne = fl + 1;
  int fl2plusTwo = 2 * fl + 2;
  int var_max = 15;

  int avg_var;
  int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  int pixY;
  int iTempAct = 0;

  int imgHExtended = blk.height + fl2plusTwo;
  int imgWExtended = blk.width + fl2plusTwo;
  int start_height1 = blk.pos().y - flplusOne;

  for (i = 2; i < imgHExtended; i += 2)
  {
    int yoffset = (i - 1 + start_height1) * img_stride - flplusOne;
    const Pel *p_imgY_pad_down = &srcExt[yoffset - img_stride];
    const Pel *p_imgY_pad = &srcExt[yoffset];
    const Pel *p_imgY_pad_up = &srcExt[yoffset + img_stride];
    const Pel *p_imgY_pad_up2 = &srcExt[yoffset + img_stride * 2];
    for (j = 2; j < imgWExtended; j += 2)
    {
      pixY = j - 1 + blk.pos().x;
      m_imgY_ver[i - 2][j - 2] = abs((p_imgY_pad[pixY] << 1) - p_imgY_pad_down[pixY] - p_imgY_pad_up[pixY]) +
        abs((p_imgY_pad[pixY + 1] << 1) - p_imgY_pad_down[pixY + 1] - p_imgY_pad_up[pixY + 1]) +
        abs((p_imgY_pad_up[pixY] << 1) - p_imgY_pad[pixY] - p_imgY_pad_up2[pixY]) +
        abs((p_imgY_pad_up[pixY + 1] << 1) - p_imgY_pad[pixY + 1] - p_imgY_pad_up2[pixY + 1]);

      m_imgY_hor[i - 2][j - 2] = abs((p_imgY_pad[pixY] << 1) - p_imgY_pad[pixY + 1] - p_imgY_pad[pixY - 1]) +
        abs((p_imgY_pad[pixY + 1] << 1) - p_imgY_pad[pixY + 2] - p_imgY_pad[pixY]) +
        abs((p_imgY_pad_up[pixY] << 1) - p_imgY_pad_up[pixY + 1] - p_imgY_pad_up[pixY - 1]) +
        abs((p_imgY_pad_up[pixY + 1] << 1) - p_imgY_pad_up[pixY + 2] - p_imgY_pad_up[pixY]);
      m_imgY_dig0[i - 2][j - 2] = abs((p_imgY_pad[pixY] << 1) - p_imgY_pad_down[pixY - 1] - p_imgY_pad_up[pixY + 1]) +
        abs((p_imgY_pad[pixY + 1] << 1) - p_imgY_pad_down[pixY] - p_imgY_pad_up[pixY + 2]) +
        abs((p_imgY_pad_up[pixY] << 1) - p_imgY_pad[pixY - 1] - p_imgY_pad_up2[pixY + 1]) +
        abs((p_imgY_pad_up[pixY + 1] << 1) - p_imgY_pad[pixY] - p_imgY_pad_up2[pixY + 2]);

      m_imgY_dig1[i - 2][j - 2] = abs((p_imgY_pad[pixY] << 1) - p_imgY_pad_up[pixY - 1] - p_imgY_pad_down[pixY + 1]) +
        abs((p_imgY_pad[pixY + 1] << 1) - p_imgY_pad_up[pixY] - p_imgY_pad_down[pixY + 2]) +
        abs((p_imgY_pad_up[pixY] << 1) - p_imgY_pad_up2[pixY - 1] - p_imgY_pad[pixY + 1]) +
        abs((p_imgY_pad_up[pixY + 1] << 1) - p_imgY_pad_up2[pixY] - p_imgY_pad[pixY + 2]);
      if (j > 4)
      {
        m_imgY_ver[i - 2][j - 6] = m_imgY_ver[i - 2][j - 6] + m_imgY_ver[i - 2][j - 4] + m_imgY_ver[i - 2][j - 2];
        m_imgY_hor[i - 2][j - 6] = m_imgY_hor[i - 2][j - 6] + m_imgY_hor[i - 2][j - 4] + m_imgY_hor[i - 2][j - 2];
        m_imgY_dig0[i - 2][j - 6] = m_imgY_dig0[i - 2][j - 6] + m_imgY_dig0[i - 2][j - 4] + m_imgY_dig0[i - 2][j - 2];
        m_imgY_dig1[i - 2][j - 6] = m_imgY_dig1[i - 2][j - 6] + m_imgY_dig1[i - 2][j - 4] + m_imgY_dig1[i - 2][j - 2];
      }
    }
  }

  for (i = 0; i < blk.height; i += 2)
  {
    for (j = 0; j < blk.width; j += 2)
    {
      int sum_V = m_imgY_ver[i][j] + m_imgY_ver[i + 2][j] + m_imgY_ver[i + 4][j];
      int sum_H = m_imgY_hor[i][j] + m_imgY_hor[i + 2][j] + m_imgY_hor[i + 4][j];
      int sum_D0 = m_imgY_dig0[i][j] + m_imgY_dig0[i + 2][j] + m_imgY_dig0[i + 4][j];
      int sum_D1 = m_imgY_dig1[i][j] + m_imgY_dig1[i + 2][j] + m_imgY_dig1[i + 4][j];
      iTempAct = sum_V + sum_H;
      avg_var = (Pel)Clip3<int>(0, var_max, (iTempAct * 24) >> (shift));
      avg_var = th[avg_var];
      int HV_high, HV_low;
      int D_high, D_low;
      int HV_D_high, HV_D_low;
      if (sum_V>sum_H)
      {
        HV_high = sum_V;
        HV_low = sum_H;
        dirTempHV = 1;
      }
      else
      {
        HV_high = sum_H;
        HV_low = sum_V;
        dirTempHV = 3;
      }
      if (sum_D0 > sum_D1)
      {
        D_high = sum_D0;
        D_low = sum_D1;
        dirTempD = 0;
      }
      else
      {
        D_high = sum_D1;
        D_low = sum_D0;
        dirTempD = 2;
      }
      if (D_high*HV_low > HV_high*D_low)
      {
        HV_D_high = D_high;
        HV_D_low = D_low;
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        HV_D_high = HV_high;
        HV_D_low = HV_low;
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
      }
      avg_var += ((2 * (mainDirection)+(secondaryDirection) / 2) << NO_VALS_LAGR_SHIFT);

      if (HV_D_high > 2 * HV_D_low)
      {
        avg_var += (8 << NO_VALS_LAGR_SHIFT);
      }
      if (HV_D_high * 2 > 9 * HV_D_low)
      {
        avg_var += (8 << NO_VALS_LAGR_SHIFT);
      }
      int yOffset = (i + blk.pos().y);
      int xOffset = (j + blk.pos().x);

      classes[yOffset][xOffset] = classes[yOffset][xOffset + 1] =
        classes[yOffset + 1][xOffset] = classes[yOffset + 1][xOffset + 1] = avg_var;
    }
  }
}


int AdaptiveLoopFilter::selectTransposeVarInd(int varInd, int *transpose)
{
  int aTransTable[8] ={0, 1, 0, 2, 2, 3, 1, 3};
  int direction = varInd >> NO_VALS_LAGR_SHIFT;
  int varIndMod = varInd&((1 << NO_VALS_LAGR_SHIFT) - 1);
  int dirRatio = direction >> 3;

  direction = direction & 0x07;
  *transpose = aTransTable[direction];

  if (dirRatio)
  {
    varIndMod += ((direction & 0x02) + dirRatio)*NO_VALS_LAGR;
  }
  return(varIndMod);
}

void AdaptiveLoopFilter::xClassifyByLaplacian(Pel** classes, const CPelBuf& srcLumaBuf, int pad_size, int fl, const Area& blk)
{
  int i, j;

  int end_height = blk.pos().y + blk.height;
  int end_width = blk.pos().x + blk.width;
  for (i = blk.pos().y; i < end_height; i += m_ALF_WIN_VERSIZE)
  {
    for (j = blk.pos().x; j < end_width; j += m_ALF_WIN_HORSIZE)
    {
      int nHeight = std::min(i + m_ALF_WIN_VERSIZE, end_height) - i;
      int nWidth = std::min(j + m_ALF_WIN_HORSIZE, end_width) - j;
      Area blk_cur;
      blk_cur = Area(j, i, nWidth, nHeight);
      xClassifyByLaplacianBlk(classes, srcLumaBuf, pad_size, fl, blk_cur);
    }
  }
}

void AdaptiveLoopFilter::xClassifyByLaplacianBlk(Pel** classes, const CPelBuf& srcLumaBuf, int pad_size, int fl, const Area& blk)
{
  const int img_stride = srcLumaBuf.stride;
  const Pel* srcExt = srcLumaBuf.buf;

  static const int shift_h = (int)(log((double)m_ALF_VAR_SIZE_H) / log(2.0));
  static const int shift_w = (int)(log((double)m_ALF_VAR_SIZE_W) / log(2.0));

  int i, j;
  int *p_imgY_temp;
#if DISTORTION_LAMBDA_BUGFIX
  int shift = (11 + m_nInternalBitDepth - 8);
#else
#if FULL_NBIT
  int shift = (11 + m_nBitIncrement + m_nInputBitDepth - 8);
#else
  int shift = (11 + m_nBitIncrement);
#endif
#endif
  int fl2plusOne = (m_VAR_SIZE << 1) + 1; //3
  int pad_offset = pad_size - fl - 1;
  int var_max = m_NO_VAR_BINS - 1;
  int mult_fact_int_tab[4] = { 1,114,41,21 };
  int mult_fact_int = mult_fact_int_tab[m_VAR_SIZE];
  int avg_var;
  int vertical, horizontal;
  int direction;
  int step1 = m_NO_VAR_BINS / 3 - 1;
  int th[m_NO_VAR_BINS] = { 0, 1, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4 };
  int pixY;

  for (i = 1; i < blk.height + fl2plusOne; i++)
  {
    int yoffset = (pad_offset + i + blk.pos().y - pad_size) * img_stride + pad_offset - pad_size;
    const Pel *p_imgY_pad = &srcExt[yoffset];
    const Pel *p_imgY_pad_up = &srcExt[yoffset + img_stride];
    const Pel *p_imgY_pad_down = &srcExt[yoffset - img_stride];
    p_imgY_temp = (int*)&m_imgY_temp[i - 1][0];
    for (j = 1; j < blk.width + fl2plusOne; j++)
    {
      pixY = j + blk.pos().x;
      vertical = abs((p_imgY_pad[pixY] << 1) - p_imgY_pad_down[pixY] - p_imgY_pad_up[pixY]);
      horizontal = abs((p_imgY_pad[pixY] << 1) - p_imgY_pad[pixY + 1] - p_imgY_pad[pixY - 1]);
      m_imgY_ver[i - 1][j - 1] = vertical;
      m_imgY_hor[i - 1][j - 1] = horizontal;
      *(p_imgY_temp++) = vertical + horizontal;
    }

    for (j = 1; j < blk.width + fl2plusOne; j = j + 4)
    {
      m_imgY_temp[i - 1][j] = (m_imgY_temp[i - 1][j - 1] + m_imgY_temp[i - 1][j + 4])
        + ((m_imgY_temp[i - 1][j] + m_imgY_temp[i - 1][j + 3]) << 1)
        + ((m_imgY_temp[i - 1][j + 1] + m_imgY_temp[i - 1][j + 2]) * 3);
      m_imgY_ver[i - 1][j] = m_imgY_ver[i - 1][j] + m_imgY_ver[i - 1][j + 1] + m_imgY_ver[i - 1][j + 2] + m_imgY_ver[i - 1][j + 3];
      m_imgY_hor[i - 1][j] = m_imgY_hor[i - 1][j] + m_imgY_hor[i - 1][j + 1] + m_imgY_hor[i - 1][j + 2] + m_imgY_hor[i - 1][j + 3];
    }
  }

  for (i = 1; i < blk.height + 1; i = i + 4)
  {
    for (j = 1; j < blk.width + 1; j = j + 4)
    {
      m_imgY_temp[i - 1][j - 1] = (m_imgY_temp[i - 1][j] + m_imgY_temp[i + 4][j])
        + ((m_imgY_temp[i][j] + m_imgY_temp[i + 3][j]) << 1)
        + ((m_imgY_temp[i + 1][j] + m_imgY_temp[i + 2][j]) * 3);

      m_imgY_ver[i - 1][j - 1] = m_imgY_ver[i][j] + m_imgY_ver[i + 1][j] + m_imgY_ver[i + 2][j] + m_imgY_ver[i + 3][j];
      m_imgY_hor[i - 1][j - 1] = m_imgY_hor[i][j] + m_imgY_hor[i + 1][j] + m_imgY_hor[i + 2][j] + m_imgY_hor[i + 3][j];
      avg_var = m_imgY_temp[i - 1][j - 1] >> (shift_h + shift_w);
      avg_var = (Pel)Clip_post(var_max, (avg_var * mult_fact_int) >> shift);
      avg_var = th[avg_var];

      direction = 0;
      if (m_imgY_ver[i - 1][j - 1] > 2 * m_imgY_hor[i - 1][j - 1]) direction = 1; //vertical
      if (m_imgY_hor[i - 1][j - 1] > 2 * m_imgY_ver[i - 1][j - 1]) direction = 2; //horizontal

      avg_var = Clip_post(step1, (int)avg_var) + (step1 + 1)*direction;
      classes[(i + blk.pos().y - 1) >> shift_h][(j + blk.pos().x - 1) >> shift_w] = avg_var;
    }
  }
}

void AdaptiveLoopFilter::xFilterBlkGalf(PelUnitBuf &recDst, const CPelUnitBuf& recSrcExt, const Area& blk, AlfFilterType filtType, const ComponentID compId)
{
  const bool bChroma = compId != COMPONENT_Y;
  if( bChroma )
  {
    CHECK(filtType != 0, "Chroma needs to have filtType == 0");
  }
  filtType = bChroma ? ALF_FILTER_SYM_5 : ALF_FILTER_SYM_9;

  const CPelBuf srcLumaBuf = recSrcExt.get(compId);
         PelBuf dstLumaBuf = recDst.get(compId);

  const int srcStride = srcLumaBuf.stride;
  const int dstStride = dstLumaBuf.stride;

  const Pel* srcExt = srcLumaBuf.buf;
        Pel* dst   = dstLumaBuf.buf;

  int i, j, pixelInt;
  Pel *pImgYVar;

  const Pel *pImgYPad, *pImgYPad1,*pImgYPad2,*pImgYPad3,*pImgYPad4,*pImgYPad5,*pImgYPad6;

  int16_t *coef = m_filterCoeffShort[0];
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
  Pel *pImgYRec;
  const Pel *pImgYPad7, *pImgYPad8;

  int numBitsMinus1= m_NUM_BITS-1;
  int offset = (1<<(m_NUM_BITS-2));

  int startHeight = blk.y;
  int endHeight   = blk.y + blk.height;
  int startWidth  = blk.x;
  int endWidth    = blk.x + blk.width;

  Pel* imgYRecPost = dst;
  imgYRecPost += startHeight * dstStride;

  if (bChroma)
  {
    pImgYVar = NULL;
  }
  int transpose = 0;

  const ClpRng& clpRng = m_clpRngs.comp[compId];

  const Pel* imgYRec = srcExt;
  switch( filtType )
  {
  case ALF_FILTER_SYM_5:
    for (i = startHeight; i < endHeight; i++)
    {
      if (!bChroma)
      {
        pImgYVar = m_imgY_var[i] + startWidth;
      }
      pImgYPad = imgYRec + i   *srcStride;
      pImgYPad1 = imgYRec + (i + 1)*srcStride;
      pImgYPad2 = imgYRec + (i - 1)*srcStride;
      pImgYPad3 = imgYRec + (i + 2)*srcStride;
      pImgYPad4 = imgYRec + (i - 2)*srcStride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if (!bChroma)
        {
          int varIndMod = selectTransposeVarInd(*(pImgYVar++), &transpose);
          coef = m_filterCoeffShort[varIndMod];
        }
        pixelInt = 0;

        pImg0 = pImgYPad + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;

        if (transpose == 1)
        {
          pixelInt += coef[38] * (pImg3[+0] + pImg4[+0]);

          pixelInt += coef[30] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[39] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[32] * (pImg1[-1] + pImg2[+1]);

          pixelInt += coef[22] * (pImg0[-2] + pImg0[+2]);
          pixelInt += coef[31] * (pImg0[-1] + pImg0[+1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }
        else if (transpose == 3)
        {
          pixelInt += coef[38] * (pImg3[+0] + pImg4[+0]);

          pixelInt += coef[32] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[39] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[30] * (pImg1[-1] + pImg2[+1]);

          pixelInt += coef[22] * (pImg0[-2] + pImg0[+2]);
          pixelInt += coef[31] * (pImg0[-1] + pImg0[+1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }
        else if (transpose == 2)
        {
          pixelInt += coef[22] * (pImg3[+0] + pImg4[+0]);

          pixelInt += coef[32] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[31] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[30] * (pImg1[-1] + pImg2[+1]);

          pixelInt += coef[38] * (pImg0[-2] + pImg0[+2]);
          pixelInt += coef[39] * (pImg0[-1] + pImg0[+1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }
        else
        {
          pixelInt += coef[22] * (pImg3[+0] + pImg4[+0]);

          pixelInt += coef[30] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[31] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[32] * (pImg1[-1] + pImg2[+1]);

          pixelInt += coef[38] * (pImg0[-2] + pImg0[+2]);
          pixelInt += coef[39] * (pImg0[-1] + pImg0[+1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }

        pixelInt = (int)((pixelInt + offset) >> (numBitsMinus1));
        *(pImgYRec++) = ClipPel(pixelInt, clpRng);
      }
      imgYRecPost += dstStride;
    }
    break;

  case ALF_FILTER_SYM_7:
    for (i = startHeight; i < endHeight; i++)
    {
      if (!bChroma)
      {
        pImgYVar = m_imgY_var[i] + startWidth;
      }

      pImgYPad = imgYRec + i    *srcStride;
      pImgYPad1 = imgYRec + (i + 1)*srcStride;
      pImgYPad2 = imgYRec + (i - 1)*srcStride;
      pImgYPad3 = imgYRec + (i + 2)*srcStride;
      pImgYPad4 = imgYRec + (i - 2)*srcStride;
      pImgYPad5 = imgYRec + (i + 3)*srcStride;
      pImgYPad6 = imgYRec + (i - 3)*srcStride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if (!bChroma)
        {
          int varIndMod = selectTransposeVarInd(*(pImgYVar++), &transpose);
          coef = m_filterCoeffShort[varIndMod];
        }
        pixelInt = 0;

        pImg0 = pImgYPad + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;

        if (transpose == 1)
        {
          pixelInt += coef[37] * (pImg5[0] + pImg6[0]);

          pixelInt += coef[29] * (pImg3[+1] + pImg4[-1]);
          pixelInt += coef[38] * (pImg3[+0] + pImg4[+0]);
          pixelInt += coef[33] * (pImg3[-1] + pImg4[+1]);

          pixelInt += coef[21] * (pImg1[+2] + pImg2[-2]);
          pixelInt += coef[30] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[39] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[32] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[23] * (pImg1[-2] + pImg2[+2]);

          pixelInt += coef[13] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[22] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[31] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }
        else if (transpose == 3)
        {
          pixelInt += coef[37] * (pImg5[0] + pImg6[0]);

          pixelInt += coef[33] * (pImg3[+1] + pImg4[-1]);
          pixelInt += coef[38] * (pImg3[+0] + pImg4[+0]);
          pixelInt += coef[29] * (pImg3[-1] + pImg4[+1]);

          pixelInt += coef[23] * (pImg1[+2] + pImg2[-2]);
          pixelInt += coef[32] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[39] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[30] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[21] * (pImg1[-2] + pImg2[+2]);

          pixelInt += coef[13] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[22] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[31] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }
        else if (transpose == 2)
        {
          pixelInt += coef[13] * (pImg5[0] + pImg6[0]);

          pixelInt += coef[23] * (pImg3[+1] + pImg4[-1]);
          pixelInt += coef[22] * (pImg3[+0] + pImg4[+0]);
          pixelInt += coef[21] * (pImg3[-1] + pImg4[+1]);

          pixelInt += coef[33] * (pImg1[+2] + pImg2[-2]);
          pixelInt += coef[32] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[31] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[30] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[29] * (pImg1[-2] + pImg2[+2]);

          pixelInt += coef[37] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[38] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[39] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }
        else
        {
          pixelInt += coef[13] * (pImg5[0] + pImg6[0]);

          pixelInt += coef[21] * (pImg3[+1] + pImg4[-1]);
          pixelInt += coef[22] * (pImg3[+0] + pImg4[+0]);
          pixelInt += coef[23] * (pImg3[-1] + pImg4[+1]);

          pixelInt += coef[29] * (pImg1[+2] + pImg2[-2]);
          pixelInt += coef[30] * (pImg1[+1] + pImg2[-1]);
          pixelInt += coef[31] * (pImg1[+0] + pImg2[+0]);
          pixelInt += coef[32] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[33] * (pImg1[-2] + pImg2[+2]);

          pixelInt += coef[37] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[38] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[39] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[+0]);
        }

        pixelInt = (int)((pixelInt + offset) >> (numBitsMinus1));
        *(pImgYRec++) = ClipPel(pixelInt, clpRng);
      }
      imgYRecPost += dstStride;
    }
    break;

  case ALF_FILTER_SYM_9:
    for (i = startHeight; i < endHeight; i++)
    {
      if (!bChroma)
      {
        pImgYVar = m_imgY_var[i] + startWidth;
      }

      pImgYPad = imgYRec + i   *srcStride;
      pImgYPad1 = imgYRec + (i + 1)*srcStride;
      pImgYPad2 = imgYRec + (i - 1)*srcStride;
      pImgYPad3 = imgYRec + (i + 2)*srcStride;
      pImgYPad4 = imgYRec + (i - 2)*srcStride;
      pImgYPad5 = imgYRec + (i + 3)*srcStride;
      pImgYPad6 = imgYRec + (i - 3)*srcStride;
      pImgYPad7 = imgYRec + (i + 4)*srcStride;
      pImgYPad8 = imgYRec + (i - 4)*srcStride;
      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if (!bChroma)
        {
          int varIndMod = selectTransposeVarInd(*(pImgYVar++), &transpose);
          coef = m_filterCoeffShort[varIndMod];
        }
        pixelInt = 0;

        pImg0 = pImgYPad + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;

        if (transpose == 1)
        {
          pixelInt += coef[36] * (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[28] * (pImg5[1] + pImg6[-1]);
          pixelInt += coef[37] * (pImg5[0] + pImg6[0]);
          pixelInt += coef[34] * (pImg5[-1] + pImg6[1]);

          pixelInt += coef[20] * (pImg3[2] + pImg4[-2]);
          pixelInt += coef[29] * (pImg3[1] + pImg4[-1]);
          pixelInt += coef[38] * (pImg3[0] + pImg4[0]);
          pixelInt += coef[33] * (pImg3[-1] + pImg4[+1]);
          pixelInt += coef[24] * (pImg3[-2] + pImg4[+2]);

          pixelInt += coef[12] * (pImg1[3] + pImg2[-3]);
          pixelInt += coef[21] * (pImg1[2] + pImg2[-2]);
          pixelInt += coef[30] * (pImg1[1] + pImg2[-1]);
          pixelInt += coef[39] * (pImg1[0] + pImg2[0]);
          pixelInt += coef[32] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[23] * (pImg1[-2] + pImg2[+2]);
          pixelInt += coef[14] * (pImg1[-3] + pImg2[+3]);

          pixelInt += coef[4] * (pImg0[+4] + pImg0[-4]);
          pixelInt += coef[13] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[22] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[31] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[0]);
        }
        else if (transpose == 3)
        {
          pixelInt += coef[36] * (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[34] * (pImg5[1] + pImg6[-1]);
          pixelInt += coef[37] * (pImg5[0] + pImg6[0]);
          pixelInt += coef[28] * (pImg5[-1] + pImg6[1]);

          pixelInt += coef[24] * (pImg3[2] + pImg4[-2]);
          pixelInt += coef[33] * (pImg3[1] + pImg4[-1]);
          pixelInt += coef[38] * (pImg3[0] + pImg4[0]);
          pixelInt += coef[29] * (pImg3[-1] + pImg4[+1]);
          pixelInt += coef[20] * (pImg3[-2] + pImg4[+2]);

          pixelInt += coef[14] * (pImg1[3] + pImg2[-3]);
          pixelInt += coef[23] * (pImg1[2] + pImg2[-2]);
          pixelInt += coef[32] * (pImg1[1] + pImg2[-1]);
          pixelInt += coef[39] * (pImg1[0] + pImg2[0]);
          pixelInt += coef[30] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[21] * (pImg1[-2] + pImg2[+2]);
          pixelInt += coef[12] * (pImg1[-3] + pImg2[+3]);

          pixelInt += coef[4] * (pImg0[+4] + pImg0[-4]);
          pixelInt += coef[13] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[22] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[31] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[0]);
        }
        else if (transpose == 2)
        {
          pixelInt += coef[4] * (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[14] * (pImg5[1] + pImg6[-1]);
          pixelInt += coef[13] * (pImg5[0] + pImg6[0]);
          pixelInt += coef[12] * (pImg5[-1] + pImg6[1]);

          pixelInt += coef[24] * (pImg3[2] + pImg4[-2]);
          pixelInt += coef[23] * (pImg3[1] + pImg4[-1]);
          pixelInt += coef[22] * (pImg3[0] + pImg4[0]);
          pixelInt += coef[21] * (pImg3[-1] + pImg4[+1]);
          pixelInt += coef[20] * (pImg3[-2] + pImg4[+2]);

          pixelInt += coef[34] * (pImg1[3] + pImg2[-3]);
          pixelInt += coef[33] * (pImg1[2] + pImg2[-2]);
          pixelInt += coef[32] * (pImg1[1] + pImg2[-1]);
          pixelInt += coef[31] * (pImg1[0] + pImg2[0]);
          pixelInt += coef[30] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[29] * (pImg1[-2] + pImg2[+2]);
          pixelInt += coef[28] * (pImg1[-3] + pImg2[+3]);

          pixelInt += coef[36] * (pImg0[+4] + pImg0[-4]);
          pixelInt += coef[37] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[38] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[39] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[0]);
        }
        else
        {
          pixelInt += coef[4] * (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[12] * (pImg5[1] + pImg6[-1]);
          pixelInt += coef[13] * (pImg5[0] + pImg6[0]);
          pixelInt += coef[14] * (pImg5[-1] + pImg6[1]);

          pixelInt += coef[20] * (pImg3[2] + pImg4[-2]);
          pixelInt += coef[21] * (pImg3[1] + pImg4[-1]);
          pixelInt += coef[22] * (pImg3[0] + pImg4[0]);
          pixelInt += coef[23] * (pImg3[-1] + pImg4[+1]);
          pixelInt += coef[24] * (pImg3[-2] + pImg4[+2]);

          pixelInt += coef[28] * (pImg1[3] + pImg2[-3]);
          pixelInt += coef[29] * (pImg1[2] + pImg2[-2]);
          pixelInt += coef[30] * (pImg1[1] + pImg2[-1]);
          pixelInt += coef[31] * (pImg1[0] + pImg2[0]);
          pixelInt += coef[32] * (pImg1[-1] + pImg2[+1]);
          pixelInt += coef[33] * (pImg1[-2] + pImg2[+2]);
          pixelInt += coef[34] * (pImg1[-3] + pImg2[+3]);

          pixelInt += coef[36] * (pImg0[+4] + pImg0[-4]);
          pixelInt += coef[37] * (pImg0[+3] + pImg0[-3]);
          pixelInt += coef[38] * (pImg0[+2] + pImg0[-2]);
          pixelInt += coef[39] * (pImg0[+1] + pImg0[-1]);
          pixelInt += coef[40] * (pImg0[0]);
        }

        pixelInt = (int)((pixelInt + offset) >> (numBitsMinus1));
        *(pImgYRec++) = ClipPel(pixelInt, clpRng);
      }
      imgYRecPost += dstStride;
    }
    break;

  case ALF_NUM_OF_FILTER_TYPES:
    THROW("ALF: wrong filter Type");
    break;
  }
}

void AdaptiveLoopFilter::xFilterBlkAlf(PelBuf &recDst, const CPelBuf& recSrc, const Area& blk, AlfFilterType filtType)
{
  const int srcStride = recSrc.stride;
  const int dstStride = recDst.stride;
  const Pel* srcExt   = recSrc.buf;
        Pel* dst      = recDst.buf;

  int varStepSizeWidth  = m_ALF_VAR_SIZE_W;
  int varStepSizeHeight = m_ALF_VAR_SIZE_H;
  int shiftHeight = (int)(log((double)varStepSizeHeight)/log(2.0));
  int shiftWidth  = (int)(log((double)varStepSizeWidth)/log(2.0));

  int i, j, pixelInt;
  Pel *pImgYVar;

  const Pel *pImgYPad, *pImgYPad1,*pImgYPad2,*pImgYPad3,*pImgYPad4,*pImgYPad5,*pImgYPad6;

  int16_t *coef = m_filterCoeffShort[0];
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
  Pel *pImgYRec;

  int numBitsMinus1= m_NUM_BITS-1;
  int offset = (1<<(m_NUM_BITS-2));

  int startHeight = blk.y;
  int endHeight   = blk.y + blk.height;
  int startWidth  = blk.x;
  int endWidth    = blk.x + blk.width;

  Pel* imgYRecPost = dst;
  imgYRecPost += startHeight * dstStride;

  const ClpRng& clpRng = m_clpRngs.comp[COMPONENT_Y];

  const Pel* imgYRec = srcExt;
  switch( filtType )
  {
  case ALF_FILTER_SYM_5:
    for (i =  startHeight; i < endHeight; i++)
    {
      pImgYVar  = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      pImgYPad  = imgYRec +  i   *srcStride;
      pImgYPad1 = imgYRec + (i+1)*srcStride;
      pImgYPad2 = imgYRec + (i-1)*srcStride;
      pImgYPad3 = imgYRec + (i+2)*srcStride;
      pImgYPad4 = imgYRec + (i-2)*srcStride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[m_MAX_SQR_FILT_LENGTH-1];

        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;

        pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);

        pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);

        pixelInt += coef[38]* (pImg0[-2]+pImg0[+2]);
        pixelInt += coef[39]* (pImg0[-1]+pImg0[+1]);
        pixelInt += coef[40]* (pImg0[+0]);

        pixelInt=(int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = ClipPel(pixelInt, clpRng );
      }
      imgYRecPost += dstStride;
    }
    break;

  case ALF_FILTER_SYM_7:
    for (i =  startHeight; i < endHeight; i++)
    {
      pImgYVar = m_imgY_var[i >> shiftHeight] + (startWidth >> shiftWidth);

      pImgYPad  = imgYRec + i    *srcStride;
      pImgYPad1 = imgYRec + (i+1)*srcStride;
      pImgYPad2 = imgYRec + (i-1)*srcStride;
      pImgYPad3 = imgYRec + (i+2)*srcStride;
      pImgYPad4 = imgYRec + (i-2)*srcStride;
      pImgYPad5 = imgYRec + (i+3)*srcStride;
      pImgYPad6 = imgYRec + (i-3)*srcStride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if ((j&(varStepSizeWidth - 1)) == 0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[m_MAX_SQR_FILT_LENGTH - 1];

        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;

        pixelInt += coef[13]* (pImg5[0]+pImg6[0]);

        pixelInt += coef[21]* (pImg3[+1]+pImg4[-1]);
        pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);
        pixelInt += coef[23]* (pImg3[-1]+pImg4[+1]);

        pixelInt += coef[29]* (pImg1[+2]+pImg2[-2]);
        pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);
        pixelInt += coef[33]* (pImg1[-2]+pImg2[+2]);

        pixelInt += coef[37]* (pImg0[+3]+pImg0[-3]);
        pixelInt += coef[38]* (pImg0[+2]+pImg0[-2]);
        pixelInt += coef[39]* (pImg0[+1]+pImg0[-1]);
        pixelInt += coef[40]* (pImg0[+0]);

        pixelInt=(int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = ClipPel(pixelInt, clpRng );
      }
      imgYRecPost += dstStride;
    }
    break;

  case ALF_FILTER_SYM_9:
    for (i =  startHeight; i < endHeight; i++)
    {
      pImgYVar = m_imgY_var[i >> shiftHeight] + (startWidth >> shiftWidth);

      pImgYPad  = imgYRec +  i   *srcStride;
      pImgYPad1 = imgYRec + (i+1)*srcStride;
      pImgYPad2 = imgYRec + (i-1)*srcStride;
      pImgYPad3 = imgYRec + (i+2)*srcStride;
      pImgYPad4 = imgYRec + (i-2)*srcStride;
      pImgYPad5 = imgYRec + (i+3)*srcStride;
      pImgYPad6 = imgYRec + (i-3)*srcStride;
      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if ((j&(varStepSizeWidth - 1)) == 0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[m_MAX_SQR_FILT_LENGTH - 1];

        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;

        pixelInt += coef[12]* (pImg5[1]+pImg6[-1]);
        pixelInt += coef[13]* (pImg5[0]+pImg6[0]);
        pixelInt += coef[14]* (pImg5[-1]+pImg6[1]);

        pixelInt += coef[20]* (pImg3[2]+pImg4[-2]);
        pixelInt += coef[21]* (pImg3[1]+pImg4[-1]);
        pixelInt += coef[22]* (pImg3[0]+pImg4[0]);
        pixelInt += coef[23]* (pImg3[-1]+pImg4[+1]);
        pixelInt += coef[24]* (pImg3[-2]+pImg4[+2]);

        pixelInt += coef[28]* (pImg1[3]+pImg2[-3]);
        pixelInt += coef[29]* (pImg1[2]+pImg2[-2]);
        pixelInt += coef[30]* (pImg1[1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[0]+pImg2[0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);
        pixelInt += coef[33]* (pImg1[-2]+pImg2[+2]);
        pixelInt += coef[34]* (pImg1[-3]+pImg2[+3]);

        pixelInt += coef[36]* (pImg0[+4]+pImg0[-4]);
        pixelInt += coef[37]* (pImg0[+3]+pImg0[-3]);
        pixelInt += coef[38]* (pImg0[+2]+pImg0[-2]);
        pixelInt += coef[39]* (pImg0[+1]+pImg0[-1]);
        pixelInt += coef[40]* (pImg0[0]);

        pixelInt=(int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = ClipPel(pixelInt, clpRng );
      }
      imgYRecPost += dstStride;
    }
    break;

  case ALF_NUM_OF_FILTER_TYPES:
    THROW( "ALF: wrong filter Type");
    break;
  }
}


/**
 \param pcPicDec    picture before ALF
 \param pcPicRest   picture after  ALF
 \param qh          filter coefficient
 \param iTap        filter tap
 \param iColor      0 for Cb and 1 for Cr
 */
void AdaptiveLoopFilter::xFrameChromaGalf(ALFParam* pcAlfParam, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, ComponentID compID)
{
  int iHeight = recUnitBuf.get(compID).height;
  int iWidth = recUnitBuf.get(compID).width;
  xFilterBlkGalf(recUnitBuf, recExtBuf, Area(0, 0, iWidth, iHeight), (AlfFilterType)0, compID);
}

void AdaptiveLoopFilter::xFrameChromaAlf( ALFParam* pcAlfParam, const PelUnitBuf& recExtBuf, PelUnitBuf& recUnitBuf, ComponentID compID )
{
  int iTap = pcAlfParam->tap_chroma;
  int *qh  = pcAlfParam->coeff_chroma;

  int i, x, y, value;//, offset;
  Pel PixSum[m_ALF_MAX_NUM_COEF];

  //offset = iTap>>1;
  int iHeight = recUnitBuf.get(compID).height;
  int iWidth  = recUnitBuf.get(compID).width;

  Pel* pDec   = recExtBuf.get(compID).buf;
  int iDecStride = recExtBuf.get(compID).stride;

  Pel* pRest      = recUnitBuf.get(compID).buf;
  int iRestStride = recUnitBuf.get(compID).stride;

#if DISTORTION_LAMBDA_BUGFIX
  int iShift = m_nInternalBitDepth - 8;
#else
  int iShift = m_nInputBitDepth + m_nBitIncrement - 8;
#endif
  Pel* pTmpDec1, *pTmpDec2;
  Pel* pTmpPixSum;

  const ClpRng& clpRng( m_clpRngs.comp[compID] );

  switch(iTap)
  {
    case 5:
    {
      int iJump = iDecStride - 4;
      pDec -= iDecStride*2;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-2;
          pTmpDec2 = pTmpDec1+4+(4*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          const int N = (iTap*iTap+1)>>1;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + m_ALF_ROUND_OFFSET)>>m_ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) ClipPel( value, clpRng);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
      break;
    case 7:
    {
      int iJump = iDecStride - 6;
      pDec -= iDecStride*3;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-3;
          pTmpDec2 = pTmpDec1+6+(6*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          const int N = (iTap*iTap+1)>>1;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + m_ALF_ROUND_OFFSET)>>m_ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) ClipPel(value, clpRng);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
      break;
    case 9:
    {
      int iJump = iDecStride - 8;
      pDec -= iDecStride*4;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-4;
          pTmpDec2 = pTmpDec1+8+(8*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;


          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum =(*pTmpDec1);

          value = 0;
          const int N = (iTap*iTap+1)>>1;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + m_ALF_ROUND_OFFSET)>>m_ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) ClipPel(value, clpRng);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
      break;
    default:
      THROW( "ALF: CHROMA: wrong number of taps" );
      break;
  }
}

void AdaptiveLoopFilter::refreshAlfTempPred()
{
  ::memset( m_storedAlfParaNum, 0, sizeof(m_storedAlfParaNum));
}


void AdaptiveLoopFilter::storeALFParam( ALFParam* pAlfParam, bool isISlice, unsigned tLayer, unsigned tLayerMax )
{
  for( int k = tLayer; k <= tLayerMax; k++)
  {
    unsigned idx = m_storedAlfParaNum[k] % C806_ALF_TEMPPRED_NUM;
    m_storedAlfParaNum[k]++;
    m_acStoredAlfPara[k][idx].temporalPredFlag = false;
    copyALFParam(&m_acStoredAlfPara[k][idx], pAlfParam, false);

    if( m_isGALF )
    {
      resetALFPredParam(&m_acStoredAlfPara[k][idx], isISlice );
    }
  }
}

void AdaptiveLoopFilter::loadALFParam( ALFParam* pAlfParam, unsigned idx, unsigned tLayer )
{
  copyALFParam(pAlfParam, &m_acStoredAlfPara[tLayer][idx], false);
}

#endif
