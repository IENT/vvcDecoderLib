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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"

#include <memory.h>
#include <algorithm>

//! \ingroup CommonLib
//! \{
#if DMVR_JVET_K0217
namespace ns_SAD_POINTS_INDEXES
{
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForBottom =
  {
    {
      NOT_AVAILABLE,                  // BOTTOM       -> NOT_AVAILABLE
      CENTER,                         // TOP          -> CENTER
      BOTTOM_RIGHT,                   // RIGHT        -> BOTTOM_RIGHT
      BOTTOM_LEFT,                    // LEFT         -> BOTTOM_LEFT
      LEFT,                           // TOP_LEFT     -> LEFT
      RIGHT,                          // TOP_RIGHT    -> RIGHT
      NOT_AVAILABLE,                  // BOTTOM_LEFT  -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // BOTTOM_RIGHT -> NOT_AVAILABLE
      TOP                             // CENTER       -> TOP
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForTop =
  {
    {
      CENTER,                         // BOTTOM       -> CENTER
      NOT_AVAILABLE,                  // TOP          -> NOT_AVAILABLE
      TOP_RIGHT,                      // RIGHT        -> TOP_RIGHT
      TOP_LEFT,                       // LEFT         -> TOP_LEFT
      NOT_AVAILABLE,                  // TOP_LEFT     -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // TOP_RIGHT    -> NOT_AVAILABLE
      LEFT,                           // BOTTOM_LEFT  -> LEFT
      RIGHT,                          // BOTTOM_RIGHT -> RIGHT
      TOP                             // CENTER       -> TOP
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForLeft =
  {
    {
      BOTTOM_LEFT,                    // BOTTOM       -> BOTTOM_LEFT
      TOP_LEFT,                       // TOP          -> TOP_LEFT
      CENTER,                         // RIGHT        -> CENTER
      NOT_AVAILABLE,                  // LEFT         -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // TOP_LEFT     -> NOT_AVAILABLE
      TOP,                            // TOP_RIGHT    -> TOP
      NOT_AVAILABLE,                  // BOTTOM_LEFT  -> NOT_AVAILABLE
      BOTTOM,                         // BOTTOM_RIGHT -> BOTTOM
      LEFT                            // CENTER       -> LEFT
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForRight =
  {
    {
      BOTTOM_RIGHT,                   // BOTTOM       -> BOTTOM_RIGHT
      TOP_RIGHT,                      // TOP          -> TOP_RIGHT
      NOT_AVAILABLE,                  // RIGHT        -> NOT_AVAILABLE
      CENTER,                         // LEFT         -> CENTER
      TOP,                            // TOP_LEFT     -> TOP
      NOT_AVAILABLE,                  // TOP_RIGHT    -> NOT_AVAILABLE
      BOTTOM,                         // BOTTOM_LEFT  -> BOTTOM
      NOT_AVAILABLE,                  // BOTTOM_RIGHT -> NOT_AVAILABLE
      RIGHT                           // CENTER       -> RIGHT
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForBottomLeft =
  {
    {
      NOT_AVAILABLE,                  // BOTTOM       -> NOT_AVAILABLE
      LEFT,                           // TOP          -> LEFT
      BOTTOM,                         // RIGHT        -> BOTTOM
      NOT_AVAILABLE,                  // LEFT         -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // TOP_LEFT     -> NOT_AVAILABLE
      CENTER,                         // TOP_RIGHT    -> CENTER
      NOT_AVAILABLE,                  // BOTTOM_LEFT  -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // BOTTOM_RIGHT -> NOT_AVAILABLE
      BOTTOM_LEFT                     // CENTER       -> BOTTOM_LEFT
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForBottomRight =
  {
    {
      NOT_AVAILABLE,                  // BOTTOM       -> NOT_AVAILABLE
      RIGHT,                          // TOP          -> RIGHT
      NOT_AVAILABLE,                  // RIGHT        -> NOT_AVAILABLE
      BOTTOM,                         // LEFT         -> BOTTOM
      CENTER,                         // TOP_LEFT     -> CENTER
      NOT_AVAILABLE,                  // TOP_RIGHT    -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // BOTTOM_LEFT  -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // BOTTOM_RIGHT -> NOT_AVAILABLE
      BOTTOM_RIGHT                    // CENTER       -> BOTTOM_RIGHT
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForTopLeft =
  {
    {
      LEFT,                           // BOTTOM       -> LEFT
      NOT_AVAILABLE,                  // TOP          -> NOT_AVAILABLE
      TOP,                            // RIGHT        -> TOP
      NOT_AVAILABLE,                  // LEFT         -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // TOP_LEFT     -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // TOP_RIGHT    -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // BOTTOM_LEFT  -> NOT_AVAILABLE
      CENTER,                         // BOTTOM_RIGHT -> CENTER
      TOP_LEFT                        // CENTER       -> TOP_LEFT
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForTopRight =
  {
    {
      RIGHT,                          // BOTTOM       -> RIGHT
      NOT_AVAILABLE,                  // TOP          -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // RIGHT        -> NOT_AVAILABLE
      TOP,                            // LEFT         -> TOP
      NOT_AVAILABLE,                  // TOP_LEFT     -> NOT_AVAILABLE
      NOT_AVAILABLE,                  // TOP_RIGHT    -> NOT_AVAILABLE
      CENTER,                         // BOTTOM_LEFT  -> CENTER
      NOT_AVAILABLE,                  // BOTTOM_RIGHT -> NOT_AVAILABLE
      TOP_RIGHT                       // CENTER       -> TOP_RIGHT
    }
  };
  static const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> SADindexesArrayForCenter =
  {
    {
      NOT_AVAILABLE,                  // BOTTOM
      NOT_AVAILABLE,                  // TOP
      NOT_AVAILABLE,                  // RIGHT
      NOT_AVAILABLE,                  // LEFT
      NOT_AVAILABLE,                  // TOP_LEFT
      NOT_AVAILABLE,                  // TOP_RIGHT
      NOT_AVAILABLE,                  // BOTTOM_LEFT
      NOT_AVAILABLE,                  // BOTTOM_RIGHT
      NOT_AVAILABLE                   // CENTER
    }
  };
  static const std::array<std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT>, SAD_POINT_INDEX::COUNT> currentSADIndexesSet =
  {
    {
      SADindexesArrayForBottom,
      SADindexesArrayForTop,
      SADindexesArrayForRight,
      SADindexesArrayForLeft,
      SADindexesArrayForTopLeft,
      SADindexesArrayForTopRight,
      SADindexesArrayForBottomLeft,
      SADindexesArrayForBottomRight,
      SADindexesArrayForCenter
    }
  };
}
using namespace ns_SAD_POINTS_INDEXES;
#endif

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
, m_maxCompIDToPred ( MAX_NUM_COMPONENT )
, m_pcRdCost        ( nullptr )
#if JEM_TOOLS
, m_pGradX0         ( nullptr )
, m_pGradY0         ( nullptr )
, m_pGradX1         ( nullptr )
, m_pGradY1         ( nullptr )
#if JVET_K0485_BIO
, m_pBIOPadRef      ( nullptr )
#endif
#endif
{
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

#if DMVR_JVET_K0217
  m_filteredBlockL1[0][1] = nullptr;
  m_filteredBlockL1[1][0] = nullptr;
#endif
#if JEM_TOOLS
  m_LICMultApprox[0] = 0;
  for( int k = 1; k < 64; k++ )
  {
    m_LICMultApprox[k] = ( ( 1 << 15 ) + ( k >> 1 ) ) / k;
  }

  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t tmplt = 0; tmplt < 2; tmplt++ )
    {
      m_acYuvPredFrucTemplate[tmplt][ch] = nullptr;
    }
#if !DMVR_JVET_K0217
    m_cYuvPredTempDMVR[ch] = nullptr;
#endif
  }
#if DMVR_JVET_K0217
  m_cYuvPredTempDMVRL1 = nullptr;
  m_cYuvPredTempDMVRL0 = nullptr;
#endif
#endif
}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }
#if DMVR_JVET_K0217
  xFree(m_filteredBlockL1[0][1]);
  m_filteredBlockL1[0][1] = nullptr;
  xFree(m_filteredBlockL1[1][0]);
  m_filteredBlockL1[1][0] = nullptr;
#endif
#if JEM_TOOLS
  xFree( m_pGradX0 );   m_pGradX0 = nullptr;
  xFree( m_pGradY0 );   m_pGradY0 = nullptr;
  xFree( m_pGradX1 );   m_pGradX1 = nullptr;
  xFree( m_pGradY1 );   m_pGradY1 = nullptr;
#if JVET_K0485_BIO
  xFree(m_pBIOPadRef);  m_pBIOPadRef = nullptr;
#endif
  m_tmpObmcBuf.destroy();

  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t tmplt = 0; tmplt < 2; tmplt++ )
    {
      xFree( m_acYuvPredFrucTemplate[tmplt][ch] );
      m_acYuvPredFrucTemplate[tmplt][ch] = nullptr;
    }
#if !DMVR_JVET_K0217
    xFree( m_cYuvPredTempDMVR[ch] );
    m_cYuvPredTempDMVR[ch] = nullptr;
#endif
  }
#if DMVR_JVET_K0217
  xFree(m_cYuvPredTempDMVRL1);
  m_cYuvPredTempDMVRL1 = nullptr;

  xFree(m_cYuvPredTempDMVRL0);
  m_cYuvPredTempDMVRL0 = nullptr;
#endif
#endif
}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC )
{
  m_pcRdCost = pcRdCost;


  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;
#if DMVR_JVET_K0217
  int extWidth = MAX_CU_SIZE + m_bufferWidthExtSize + 16;
  int extHeight = MAX_CU_SIZE + m_bufferWidthExtSize + 1;
#endif
#if DMVR_JVET_K0217 & JVET_K0485_BIO 
  extWidth = extWidth > (MAX_CU_SIZE + (2 * JVET_K0485_BIO_EXTEND_SIZE + 2) + 16) ? extWidth : MAX_CU_SIZE + (2 * JVET_K0485_BIO_EXTEND_SIZE + 2) + 16;
  extHeight = extHeight > (MAX_CU_SIZE + (2 * JVET_K0485_BIO_EXTEND_SIZE + 2) + 1) ? extHeight : MAX_CU_SIZE + (2 * JVET_K0485_BIO_EXTEND_SIZE + 2) + 1;
#endif
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
#if JEM_TOOLS
#if !DMVR_JVET_K0217
#if JVET_K0485_BIO
      int extWidth  = MAX_CU_SIZE + (2 * JVET_K0485_BIO_EXTEND_SIZE + 2) + 16;
      int extHeight = MAX_CU_SIZE + (2 * JVET_K0485_BIO_EXTEND_SIZE + 2) + 1;
#else
      int extWidth  = MAX_CU_SIZE + DMVR_INTME_RANGE*2 + 16;
      int extHeight = MAX_CU_SIZE + DMVR_INTME_RANGE*2 + 1;
#endif
#endif
#else
      int extWidth  = MAX_CU_SIZE + 16;
      int extHeight = MAX_CU_SIZE + 1;
#endif
      for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
      {
        m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

        for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
        {
          m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        }
      }

      // new structure
      for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        m_acYuvPred[i][c] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
    }

#if DMVR_JVET_K0217
    m_filteredBlockL1[0][1] = (Pel*)xMalloc(Pel, extWidth * extHeight);
    m_filteredBlockL1[1][0] = (Pel*)xMalloc(Pel, extWidth * extHeight);
#endif

    m_iRefListIdx = -1;
    
#if JEM_TOOLS
    m_pGradX0     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
    m_pGradY0     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
    m_pGradX1     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
    m_pGradY1     = ( Pel* ) xMalloc( Pel, BIO_TEMP_BUFFER_SIZE );
#if JVET_K0485_BIO
    m_pBIOPadRef = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2 + 7) * (MAX_CU_SIZE + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2 + 7));
#endif

    m_tmpObmcBuf.create( UnitArea( chromaFormatIDC, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );

    for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
    {
      for( uint32_t tmplt = 0; tmplt < 2; tmplt++ )
      {
        m_acYuvPredFrucTemplate[tmplt][ch] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
#if !DMVR_JVET_K0217
      m_cYuvPredTempDMVR[ch] = ( Pel* ) xMalloc( Pel, ( MAX_CU_SIZE + DMVR_INTME_RANGE*2 ) * ( MAX_CU_SIZE + DMVR_INTME_RANGE*2 ) );
#endif
    }
#if DMVR_JVET_K0217
    m_cYuvPredTempDMVRL0 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + m_bufferWidthExtSize) * (MAX_CU_SIZE + m_bufferWidthExtSize));
    m_cYuvPredTempDMVRL1 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + m_bufferWidthExtSize) * (MAX_CU_SIZE + m_bufferWidthExtSize));
#endif
#endif
  }

#if JEM_TOOLS
#if JVET_K0485_BIO
  bioGradFilter = InterPrediction::gradFilter;
#else
  m_uiaBIOShift[0] = 0;
  for (int i = 1; i < 64; i++)
  {
    m_uiaBIOShift[i] = ((1 << 15) + i / 2) / i;
  }
#endif
#endif
#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter( true );
#endif
}

#if JEM_TOOLS || JVET_K_AFFINE
bool checkIdenticalMotion( const PredictionUnit &pu, bool checkAffine )
#else
bool checkIdenticalMotion( const PredictionUnit &pu )
#endif
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
#if JEM_TOOLS || JVET_K_AFFINE
        if( !pu.cu->affine )
#endif
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
#if JEM_TOOLS || JVET_K_AFFINE
        else
        {
          CHECK( !checkAffine, "In this case, checkAffine should be on." );
          const CMotionBuf &mb = pu.getMotionBuf();
#if JVET_K_AFFINE_BUG_FIXES
#if JVET_K0337_AFFINE_6PARA
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#else
          if ( (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
#endif
#else
          if( ( mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1] ) && ( mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1] ) && ( mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1] ) )
#endif
          {
            return true;
          }
        }
#endif
      }
    }
  }

  return false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
#if JEM_TOOLS || JVET_K_AFFINE
        if( !pu.cu->affine )
#endif
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
#if JEM_TOOLS || JVET_K_AFFINE
        else
        {
          const CMotionBuf &mb = pu.getMotionBuf();
#if JVET_K_AFFINE_BUG_FIXES
#if JVET_K0337_AFFINE_6PARA
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#else
          if ( (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) )
#endif
#else
          if( ( mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1] ) && ( mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1] ) && ( mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1] ) )
#endif
          {
            return true;
          }
        }
#endif
      }
    }
  }

  return false;
}

#if JEM_TOOLS || JVET_K0346
void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{
#if !JVET_K0346
  const SPSNext& spsNext  = pu.cs->sps->getSpsNext();
#endif

  // compute the location of the current PU
  Position puPos    = pu.lumaPos();
  Size puSize       = pu.lumaSize();

#if JVET_K0346
  int numPartLine, numPartCol, puHeight, puWidth;
#else
  int iNumPartLine, iNumPartCol, iPUHeight, iPUWidth;
#endif
#if JEM_TOOLS
  if( pu.mergeType == MRG_TYPE_FRUC )
  {
    uint32_t nRefineBlockSize = xFrucGetSubBlkSize( pu, puSize.width, puSize.height );

#if JVET_K0346
    puHeight     = std::min(puSize.width, nRefineBlockSize);
    puWidth      = std::min(puSize.height, nRefineBlockSize);
#else
    iPUHeight     = std::min( puSize.width,  nRefineBlockSize );
    iPUWidth      = std::min( puSize.height, nRefineBlockSize );
#endif
  }
  else
#endif
  {
#if JVET_K0346
    const Slice& slice = *pu.cs->slice;
    numPartLine = std::max(puSize.width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    numPartCol  = std::max(puSize.height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    puHeight    = numPartCol == 1 ? puSize.height : 1 << slice.getSubPuMvpSubblkLog2Size();
    puWidth     = numPartLine == 1 ? puSize.width : 1 << slice.getSubPuMvpSubblkLog2Size();
#else
    iNumPartLine  = std::max( puSize.width  >> spsNext.getSubPuMvpLog2Size(), 1u );
    iNumPartCol   = std::max( puSize.height >> spsNext.getSubPuMvpLog2Size(), 1u );
    iPUHeight     = iNumPartCol  == 1 ? puSize.height : 1 << spsNext.getSubPuMvpLog2Size();
    iPUWidth      = iNumPartLine == 1 ? puSize.width  : 1 << spsNext.getSubPuMvpLog2Size();
#endif
  }

  PredictionUnit subPu;

  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  // join sub-pus containing the same motion
#if JVET_K0346
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

#if JEM_TOOLS
      subPu.mvRefine = false;
#endif
      motionCompensation(subPu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
#else
  for( int y = puPos.y; y < puPos.y + puSize.height; y += iPUHeight )
  {
    for( int x = puPos.x; x < puPos.x + puSize.width; x += iPUWidth )
    {
      const MotionInfo &curMi = pu.getMotionInfo( Position{ x, y } );

      int dx = iPUWidth;
      int dy = iPUHeight;

      subPu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( x, y, dx, dy ) ) );
      subPu                   = curMi;
      PelUnitBuf subPredBuf   = predBuf.subBuf( UnitAreaRelative( pu, subPu ) );

#if JEM_TOOLS
      subPu.mvRefine = false;
#endif
      motionCompensation( subPu, subPredBuf, eRefPicList );
    }
  }
#endif
}
#endif

#if JVET_K0076_CPR_DT
void InterPrediction::xChromaMC(PredictionUnit &pu, PelUnitBuf& pcYuvPred)
{
  // separated tree, chroma
  const CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  PredictionUnit subPu;
  subPu.cs = pu.cs;
  subPu.cu = pu.cu;
  
  Picture * refPic = pu.cu->slice->getPic();
  for (int y = lumaArea.y; y < lumaArea.y + lumaArea.height; y += MIN_PU_SIZE)
  {
    for (int x = lumaArea.x; x < lumaArea.x + lumaArea.width; x += MIN_PU_SIZE)
    {
      const MotionInfo &curMi = pu.cs->picture->cs->getMotionInfo(Position{ x, y });
      
      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, MIN_PU_SIZE, MIN_PU_SIZE)));
      PelUnitBuf subPredBuf = pcYuvPred.subBuf(UnitAreaRelative(pu, subPu));
      
      xPredInterBlk(COMPONENT_Cb, subPu, refPic, curMi.mv[0], subPredBuf, false, pu.cu->slice->clpRng(COMPONENT_Cb), false, false, FRUC_MERGE_OFF, false);
      xPredInterBlk(COMPONENT_Cr, subPu, refPic, curMi.mv[0], subPredBuf, false, pu.cu->slice->clpRng(COMPONENT_Cr), false, false, FRUC_MERGE_OFF, false);
    }
  }
}
#endif

#if JEM_TOOLS
void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi, const bool& bBIOApplied /*= false*/, const bool& bDMVRApplied /*= false*/ 
#if JVET_K0076_CPR_DT
  , const bool luma, const bool chroma
#endif
)
#else
void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi )
#endif
{
  const SPS &sps = *pu.cs->sps;

  int iRefIdx = pu.refIdx[eRefPicList];
  Mv mv[3];

#if JEM_TOOLS || JVET_K_AFFINE
  if( pu.cu->affine )
  {
    CHECK( iRefIdx < 0, "iRefIdx incorrect." );

    const CMotionBuf &mb = pu.getMotionBuf();
    mv[0] = mb.at( 0,            0             ).mv[eRefPicList];
    mv[1] = mb.at( mb.width - 1, 0             ).mv[eRefPicList];
    mv[2] = mb.at( 0,            mb.height - 1 ).mv[eRefPicList];
#if !JVET_K_AFFINE_BUG_FIXES
    clipMv(mv[1], pu.cu->lumaPos(), sps);
    clipMv(mv[2], pu.cu->lumaPos(), sps);
#endif
  }
  else
#endif
  {
    mv[0] = pu.mv[eRefPicList];
  }
#if (JEM_TOOLS || JVET_K_AFFINE) && JVET_K_AFFINE_BUG_FIXES
  if ( !pu.cu->affine )
#endif
  clipMv(mv[0], pu.cu->lumaPos(), sps);


  for( uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );
#if JVET_K0076_CPR_DT
    if (compID == COMPONENT_Y && !luma)
      continue;
    if (compID != COMPONENT_Y && !chroma)
      continue;
#endif
#if JEM_TOOLS
    if( pu.cu->affine )
    {
      xPredAffineBlk( compID, pu, pu.cu->slice->getRefPic(eRefPicList, iRefIdx), mv, pcYuvPred, bi, pu.cu->slice->clpRng( compID ), bBIOApplied );
    }
    else
#elif JVET_K_AFFINE
    if ( pu.cu->affine )
    {
      xPredAffineBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv, pcYuvPred, bi, pu.cu->slice->clpRng( compID ) );
    }
    else
#endif
    {
      xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng( compID )
#if JEM_TOOLS
                     , bBIOApplied, bDMVRApplied, FRUC_MERGE_OFF, true
#endif
                    );
    }
  }
}

#if JEM_TOOLS
void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred, bool obmc)
#else
void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred)
#endif
{
  const PPS   &pps   = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;

#if JEM_TOOLS
  bool bBIOApplied = false;
  if ( pu.cs->sps->getSpsNext().getUseBIO() )
  {
    if( pu.cu->LICFlag || pu.cu->affine || obmc )
    {
      bBIOApplied = false;
    }
#if !JVET_K0485_BIO
    else if( PU::isBIOLDB( pu ) )
    {
      bBIOApplied = true;
    }
#endif
    else
    {
      const bool bBIOcheck0 = !( pps.getWPBiPred() && slice.getSliceType() == B_SLICE );
      const bool bBIOcheck1 = !( pps.getUseWP()    && slice.getSliceType() == P_SLICE );
      if( bBIOcheck0
          && bBIOcheck1
          && PU::isBiPredFromDifferentDir( pu )
        )
      {
        bBIOApplied = true;
      }
    }
  }

  bool bDMVRApplied = false;
  if ( pu.cs->sps->getSpsNext().getUseDMVR() )
  {
    if ( pu.mvRefine
        && pu.mergeFlag
        && pu.mergeType == MRG_TYPE_DEFAULT_N
        && ! pu.frucMrgMode
        && ! pu.cu->LICFlag
        && ! pu.cu->affine
        && PU::isBiPredFromDifferentDir( pu )
       )
    {
      bDMVRApplied = true;
    }
  }
#endif

  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK( pu.refIdx[refList] >= slice.getNumRefIdx( eRefPicList ), "Invalid reference index" );
    m_iRefListIdx = refList;

    PelUnitBuf pcMbBuf = ( pu.chromaFormat == CHROMA_400 ?
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y())) :
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())) );

    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      xPredInterUni ( pu, eRefPicList, pcMbBuf, true
#if JEM_TOOLS
                      , bBIOApplied, bDMVRApplied
#endif
                     );
    }
    else
    {
#if JEM_TOOLS
      if( !pu.cu->LICFlag && ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true, bBIOApplied, false );
      }
      else
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, false, bBIOApplied, false );
      }
#else
      if( ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true );
      }
      else
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, false );
      }
#endif
    }
  }

#if JEM_TOOLS
  if ( bDMVRApplied )
  {
    xProcessDMVR( pu, pcYuvPred, slice.clpRngs(), bBIOApplied );
  }
#if DMVR_JVET_LOW_LATENCY_K0217
  else if (pu.cs->sps->getSpsNext().getUseDMVR()
    && pu.mergeFlag
    && pu.mergeType == MRG_TYPE_DEFAULT_N
#if JEM_TOOLS
    && !pu.frucMrgMode
    && !pu.cu->LICFlag
#endif
    && !pu.cu->affine
    && PU::isBiPredFromDifferentDir(pu))
  {
    pu.mvd[REF_PIC_LIST_0].setZero();
  }
#endif
#endif

  CPelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())) );
  CPelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())) );
#if JEM_TOOLS
  if( !pu.cu->LICFlag && pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( !pu.cu->LICFlag && pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), bBIOApplied );
  }
#else
  if( pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs() );
  }
#endif
}

#if JVET_K0485_BIO && JEM_TOOLS
void InterPrediction::xPadRefFromFMC(const Pel* refBufPtr, int refBufStride, int width, int height, Pel* padRefPelPtr, int &padRefStride, bool isFracMC)
{
  int  widthFMC = 3 + width + 4;
  int  heightFMC = 3 + height + 4;
  int  widthPadFMC = widthFMC + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2;

  padRefStride = widthPadFMC;

  int  offsetPadFMC = 2 * widthPadFMC + 2;
  Pel* targetPadPtr = padRefPelPtr + offsetPadFMC;

  //Copy the inner area
  for (int i = 0; i<heightFMC; i++)
  {
    ::memcpy(targetPadPtr, refBufPtr, sizeof(Pel)*(widthFMC));
    refBufPtr += refBufStride;
    targetPadPtr += padRefStride;
  }

  if (isFracMC)
  {
    //Pad left and right margin
    targetPadPtr = padRefPelPtr + offsetPadFMC;
    for (int y = 0; y<heightFMC; y++)
    {
      for (int x = 0; x<JVET_K0485_BIO_EXTEND_SIZE + 1; x++)
      {
        targetPadPtr[-(x + 1)] = targetPadPtr[0];
        targetPadPtr[widthFMC + x] = targetPadPtr[widthFMC - 1];
      }
      targetPadPtr += padRefStride;
    }

    //Pad top/bottom margin
    targetPadPtr = padRefPelPtr + 2 * padRefStride;
    for (int y = 0; y < JVET_K0485_BIO_EXTEND_SIZE + 1; y++)
    {
      ::memcpy(targetPadPtr - (y + 1)*padRefStride, targetPadPtr, sizeof(Pel)*(padRefStride));
      ::memcpy(targetPadPtr + (heightFMC - 1)*padRefStride + (y + 1)*padRefStride, targetPadPtr + (heightFMC - 1)*padRefStride, sizeof(Pel)*(padRefStride));
    }
  }
}
#endif

void InterPrediction::xPredInterBlk ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
#if JEM_TOOLS
                                      , const bool& bBIOApplied /*=false*/, const bool& bDMVRApplied /*= false*/, const int& nFRUCMode /*= FRUC_MERGE_OFF*/, const bool& doLic /*= true*/
#endif
#if DMVR_JVET_K0217
  , bool doPred/* = true*/
  , SizeType DMVRwidth/* = 0*/
  , SizeType DMVRheight/* = 0*/
#endif
                                    )
{
  JVET_J0090_SET_REF_PICTURE( refPic, compID );
#if JEM_TOOLS
  const int       nFilterIdx = nFRUCMode ? pu.cs->slice->getSPS()->getSpsNext().getFRUCRefineFilter() : 0;
  const ChromaFormat  chFmt  = pu.chromaFormat;
  const bool          rndRes = ( !bi || pu.cu->LICFlag );

  int iAddPrecShift = 0;

  if( _mv.highPrec )
  {
    CHECKD( !pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!" );

    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY( compID, chFmt );

  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );

  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;

  CHECKD( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( xFrac & 3 ) != 0 ), "Invalid fraction" );
  CHECKD( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( yFrac & 3 ) != 0 ), "Invalid fraction" );
#elif JVET_K0346 || JVET_K_AFFINE
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  int iAddPrecShift = 0;

  if (_mv.highPrec)
  {
    CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!");

    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX(compID, chFmt);
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY(compID, chFmt);

  int xFrac = _mv.hor & ((1 << shiftHor) - 1);
  int yFrac = _mv.ver & ((1 << shiftVer) - 1);

  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;

  CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv() && ((xFrac & 3) != 0), "Invalid fraction");
  CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv() && ((yFrac & 3) != 0), "Invalid fraction");
#else
  const ChromaFormat  chFmt  = pu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = 2 + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + ::getComponentScaleY( compID, chFmt );
  
  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );
#endif

  PelBuf &dstBuf  = dstPic.bufs[compID];
  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  CPelBuf refBuf;
  {
    Position offset = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
#if DMVR_JVET_K0217
    if (DMVRwidth)
    {
      refBuf = refPic->getRecoBuf(CompArea(compID, chFmt, offset, Size(DMVRwidth, DMVRheight)));
    }
    else
#endif
    refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  }

#if JEM_TOOLS
#if JVET_K0485_BIO
  // backup data
  int backupWidth = width;
  int backupHeight = height;
  Pel *backupDstBufPtr = dstBuf.buf;
  int backupDstBufStride = dstBuf.stride;
  Pel *backupRefBuf = (Pel*)refBuf.buf;
  int backupRefBufStride = refBuf.stride;
#endif

  if( bBIOApplied && compID == COMPONENT_Y && !bDMVRApplied )
  {
#if JVET_K0485_BIO
    width = width + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2;
    height = height + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2;

    // change MC output
    dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID];
    dstBuf.stride = width;

    CPelBuf refBufFMC;
    {
      Position offsetFMC = pu.blocks[compID].pos().offset((_mv.getHor() >> shiftHor) - 3, (_mv.getVer() >> shiftVer) - 3);
      refBufFMC = refPic->getRecoBuf(CompArea(compID, chFmt, offsetFMC, pu.blocks[compID].size()));
    }

    // padding FMC region for BIO
    int padRefPelStride;
    bool isFracMC = (xFrac != 0 || yFrac != 0);
    xPadRefFromFMC(refBufFMC.buf, refBufFMC.stride, backupWidth, backupHeight, m_pBIOPadRef, padRefPelStride, isFracMC);

    refBuf.buf = m_pBIOPadRef + 3 * padRefPelStride + 3;
    refBuf.stride = padRefPelStride;
#else
    Pel*  pGradY    = m_pGradY0;
    Pel*  pGradX    = m_pGradX0;

    int   iWidthG   = width;
    int   iHeightG  = height;

    if( m_iRefListIdx == 0 )
    {
      pGradY  = m_pGradY0;
      pGradX  = m_pGradX0;
    }
    else
    {
      pGradY  = m_pGradY1;
      pGradX  = m_pGradX1;
    }

    const int         refStride   = refBuf.stride;
    const Pel* const  ref         = refBuf.buf;

    xGradFilterY  ( ref, refStride, pGradY, iWidthG, iWidthG, iHeightG, yFrac, xFrac, clpRng.bd );
    xGradFilterX  ( ref, refStride, pGradX, iWidthG, iWidthG, iHeightG, yFrac, xFrac, clpRng.bd );
#endif
  }
#endif
#if DMVR_JVET_K0217
  if (doPred)
  {
    if (DMVRwidth)
    {
      width = DMVRwidth;
      height = DMVRheight;
    }
#endif
  if( yFrac == 0 )
  {
#if JEM_TOOLS
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes, chFmt, clpRng, nFilterIdx);
#else
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes, chFmt, clpRng);
#endif
  }
  else if( xFrac == 0 )
  {
#if JEM_TOOLS
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes, chFmt, clpRng, nFilterIdx);
#else
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes, chFmt, clpRng);
#endif
  }
  else
  {
#if DMVR_JVET_K0217
      PelBuf tmpBuf = DMVRwidth ? PelBuf(m_filteredBlockTmp[0][compID], Size(DMVRwidth, DMVRheight)) : PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
#if JVET_K0485_BIO
      if (DMVRwidth == 0)
        tmpBuf.stride = dstBuf.stride;
#endif
#else
      PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
#if JVET_K0485_BIO
      tmpBuf.stride = dstBuf.stride;
#endif
#endif

    int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
#if JEM_TOOLS
    if( isLuma(compID) && nFilterIdx == 1 )
    {
      vFilterSize = NTAPS_LUMA_FRUC;
    }
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng, nFilterIdx);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, rndRes, chFmt, clpRng, nFilterIdx);
#else
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, rndRes, chFmt, clpRng);
#endif
    JVET_J0090_SET_CACHE_ENABLE( true );
  }
#if DMVR_JVET_K0217
  }
#endif
#if JEM_TOOLS
#if JVET_K0485_BIO
  if (bBIOApplied && compID == COMPONENT_Y && !bDMVRApplied)
  {
    // restore data 
    width = backupWidth;
    height = backupHeight;
    dstBuf.buf = backupDstBufPtr;
    dstBuf.stride = backupDstBufStride;
    refBuf.buf = backupRefBuf;
    refBuf.stride = backupRefBufStride;
  }
#endif
  if( pu.cu->LICFlag && doLic )
  {
    xLocalIlluComp( pu, compID, *refPic, _mv, bi, dstBuf );
  }
#endif
}

#if JEM_TOOLS || JVET_K_AFFINE
#if !JEM_TOOLS
void InterPrediction::xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng )
#else
void InterPrediction::xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, const bool& bBIOApplied /*= false*/ )
#endif
{
#if JVET_K0337_AFFINE_6PARA
  if ( (pu.cu->affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2])
    || (pu.cu->affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1])
    )
#else
  if( _mv[0] == _mv[1] )
#endif
  {
#if JVET_K_AFFINE_BUG_FIXES
    Mv mvTemp = _mv[0];
    clipMv( mvTemp, pu.cu->lumaPos(), *pu.cs->sps );
#if !JEM_TOOLS
    xPredInterBlk( compID, pu, refPic, mvTemp, dstPic, bi, clpRng );
#else
    xPredInterBlk( compID, pu, refPic, mvTemp, dstPic, bi, clpRng, bBIOApplied, false, FRUC_MERGE_OFF, true );
#endif
#else
#if !JEM_TOOLS
    xPredInterBlk( compID, pu, refPic, _mv[0], dstPic, bi, clpRng );
#else
    xPredInterBlk( compID, pu, refPic, _mv[0], dstPic, bi, clpRng, bBIOApplied, false, FRUC_MERGE_OFF, true );
#endif
#endif
    return;
  }

  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat chFmt = pu.chromaFormat;
  int iScaleX = ::getComponentScaleX( compID, chFmt );
  int iScaleY = ::getComponentScaleY( compID, chFmt );

  Mv mvLT =_mv[0];
  Mv mvRT =_mv[1];
  Mv mvLB =_mv[2];

  mvLT.setHighPrec();
  mvRT.setHighPrec();
  mvLB.setHighPrec();

  // get affine sub-block width and height
  const int width  = pu.Y().width;
  const int height = pu.Y().height;
#if JVET_K0184_AFFINE_4X4
  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
#else
  int blockWidth   = width;
  int blockHeight  = height;
  int mvWx = std::max<int>( abs((mvRT - mvLT).getHor()), abs((mvRT - mvLT).getVer()) );
  int mvWy = std::max<int>( abs((mvLB - mvLT).getHor()), abs((mvLB - mvLT).getVer()) );

  int iMvPrecision = 4;
  iMvPrecision -= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;

  if (mvWx)
  {
    blockWidth = std::max<int>( (int)( ( width >> iMvPrecision ) / mvWx ), 1 );
    while (width % blockWidth)
    {
      blockWidth--;
    }
    blockWidth = std::max<int>( AFFINE_MIN_BLOCK_SIZE, blockWidth );
  }
  if (mvWy)
  {
    blockHeight = std::max<int>( (int)( ( height >> iMvPrecision ) / mvWy ), 1 );
    while (height % blockHeight)
    {
      blockHeight--;
    }
    blockHeight = std::max<int>( AFFINE_MIN_BLOCK_SIZE, blockHeight );
  }
#endif

  blockWidth  >>= iScaleX;
  blockHeight >>= iScaleY;
  const int cxWidth  = width  >> iScaleX;
  const int cxHeight = height >> iScaleY;
  const int iHalfBW  = blockWidth  >> 1;
  const int iHalfBH  = blockHeight >> 1;

  const int iBit = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - g_aucLog2[cxWidth]);
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - g_aucLog2[cxWidth]);
#if JVET_K0337_AFFINE_6PARA
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - g_aucLog2[cxHeight]);
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - g_aucLog2[cxHeight]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }
#else
  iDMvVerX = -iDMvHorY;
  iDMvVerY = iDMvHorX;
#endif

  int iMvScaleHor = mvLT.getHor() << iBit;
  int iMvScaleVer = mvLT.getVer() << iBit;
  const SPS &sps    = *pu.cs->sps;
  const int iMvShift = 4;
  const int iOffset  = 8;
  const int iHorMax = ( sps.getPicWidthInLumaSamples()     + iOffset -      pu.Y().x - 1 ) << iMvShift;
  const int iHorMin = (      -(int)pu.cs->pcv->maxCUWidth  - iOffset - (int)pu.Y().x + 1 ) << iMvShift;
  const int iVerMax = ( sps.getPicHeightInLumaSamples()    + iOffset -      pu.Y().y - 1 ) << iMvShift;
  const int iVerMin = (      -(int)pu.cs->pcv->maxCUHeight - iOffset - (int)pu.Y().y + 1 ) << iMvShift;

  PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
  const int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

  const int shift = iBit - 4 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE + 2;

  // get prediction block by block
  for ( int h = 0; h < cxHeight; h += blockHeight )
  {
    for ( int w = 0; w < cxWidth; w += blockWidth )
    {
      int iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
      int iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
#if JVET_K_AFFINE_BUG_FIXES
      roundAffineMv( iMvScaleTmpHor, iMvScaleTmpVer, shift );
#else
      iMvScaleTmpHor >>= shift;
      iMvScaleTmpVer >>= shift;
#endif

      // clip and scale
      iMvScaleTmpHor = std::min<int>( iHorMax, std::max<int>( iHorMin, iMvScaleTmpHor ) );
      iMvScaleTmpVer = std::min<int>( iVerMax, std::max<int>( iVerMin, iMvScaleTmpVer ) );

      // get the MV in high precision
      int xFrac, yFrac, xInt, yInt;

      if (!iScaleX)
      {
        xInt  = iMvScaleTmpHor >> 4;
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt  = iMvScaleTmpHor >> 5;
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt  = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt  = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }

      const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID] ) );
      PelBuf &dstBuf = dstPic.bufs[compID];

      if ( yFrac == 0 )
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, xFrac, !bi, chFmt, clpRng );
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVer( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, clpRng );
      }
      else
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf - ((vFilterSize>>1) -1)*refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( false );
        m_if.filterVer( compID, tmpBuf.buf + ((vFilterSize>>1) -1)*tmpBuf.stride, tmpBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, false, !bi, chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( true );
      }
    }
  }
}
#endif

int getMSB( unsigned x )
{
  int msb = 0, bits = ( sizeof(int) << 3 ), y = 1;
  while( x > 1u )
  {
    bits >>= 1;
    y      = x >> bits;
    if( y )
    {
      x    = y;
      msb += bits;
    }
  }
  msb += y;
  return msb;
}

#if JEM_TOOLS
void InterPrediction::xGetLICParams( const CodingUnit& cu,
                                     const ComponentID compID,
                                     const Picture&    refPic,
                                     const Mv&         mv,
                                           int&        shift,
                                           int&        scale,
                                           int&        offset )
{
  const int       lumaShift     = ( mv.highPrec ? 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 2 );
  const int       horShift      = ( lumaShift + ::getComponentScaleX(compID, cu.chromaFormat) );
  const int       verShift      = ( lumaShift + ::getComponentScaleY(compID, cu.chromaFormat) );
  const int       horIntMv      = ( mv.getHor() + ( ( 1 << horShift ) >> 1 ) ) >> horShift;
  const int       verIntMv      = ( mv.getVer() + ( ( 1 << verShift ) >> 1 ) ) >> verShift;
  const int       cuWidth       = cu.blocks[compID].width;
  const int       cuHeight      = cu.blocks[compID].height;
  const int       bitDepth      = cu.cs->sps->getBitDepth( toChannelType( compID ) );
  const int       precShift     = std::max( 0, bitDepth - 12 );
  const int       maxNumMinus1  = 30 - 2 * std::min( bitDepth, 12 ) - 1;
  const int       minDimBit     = g_aucPrevLog2[std::min( cuHeight, cuWidth )];
  const int       minDim        = 1 << minDimBit;
        int       minStepBit    = ( !cu.cs->pcv->rectCUs || minDim > 8 ? 1 : 0 );
        while   ( minDimBit > minStepBit + maxNumMinus1 )  { minStepBit++; } //make sure log2(2*minDim/tmpStep) + 2*min(bitDepth,12) <= 30
  const int       numSteps      = minDim >> minStepBit;
  const Picture&  currPic       = *cu.cs->picture;
  const int       dimShift      = minDimBit - minStepBit;

  //----- get correlation data -----
  int x = 0, y = 0, xx = 0, xy = 0, cntShift = 0;
  const CodingUnit* const cuAbove = cu.cs->getCU( cu.blocks[compID].pos().offset(  0, -1 ), toChannelType( compID ) );
  const CodingUnit* const cuLeft  = cu.cs->getCU( cu.blocks[compID].pos().offset( -1,  0 ), toChannelType( compID ) );
  const CPelBuf recBuf            = cuAbove || cuLeft ? currPic.getRecoBuf( cu.cs->picture->blocks[compID] ) : CPelBuf();
  const CPelBuf refBuf            = cuAbove || cuLeft ? refPic .getRecoBuf( refPic.blocks[compID]          ) : CPelBuf();

  // above
  if( cuAbove )
  {
    Mv            subPelMv  ( horIntMv << horShift, verIntMv << verShift, mv.highPrec );
                  clipMv    ( subPelMv, cuAbove->lumaPos(), *cu.cs->sps );
    const int     hOff    = ( subPelMv.getHor() >> horShift );
    const int     vOff    = ( subPelMv.getVer() >> verShift ) - 1;
    const Pel*    ref     = refBuf.bufAt( cu.blocks[compID].pos().offset( hOff, vOff ) );
    const Pel*    rec     = recBuf.bufAt( cu.blocks[compID].pos().offset(    0,   -1 ) );

    for( int k = 0; k < numSteps; k++ )
    {
      int refVal  = ref[( ( k * cuWidth ) >> dimShift )] >> precShift;
      int recVal  = rec[( ( k * cuWidth ) >> dimShift )] >> precShift;

      JVET_J0090_CACHE_ACCESS( &ref[( ( k * cuWidth ) >> dimShift )], __FILE__, __LINE__ );
      x          += refVal;
      y          += recVal;
      xx         += refVal * refVal;
      xy         += refVal * recVal;
    }

    cntShift = dimShift;
  }
  // left
  if( cuLeft )
  {
    Mv            subPelMv  ( horIntMv << horShift, verIntMv << verShift, mv.highPrec );
                  clipMv    ( subPelMv, cuLeft->lumaPos(), *cu.cs->sps );
    const int     hOff    = ( subPelMv.getHor() >> horShift ) - 1;
    const int     vOff    = ( subPelMv.getVer() >> verShift );
    const Pel*    ref     = refBuf.bufAt( cu.blocks[compID].pos().offset( hOff, vOff ) );
    const Pel*    rec     = recBuf.bufAt( cu.blocks[compID].pos().offset(   -1,    0 ) );

    for( int k = 0; k < numSteps; k++ )
    {
      int refVal     = ref[refBuf.stride * ( ( k * cuHeight ) >> dimShift )] >> precShift;
      int recVal     = rec[recBuf.stride * ( ( k * cuHeight ) >> dimShift )] >> precShift;
      x             += refVal;
      y             += recVal;
      xx            += refVal * refVal;
      xy            += refVal * recVal;
    }

    cntShift += ( cntShift ? 1 : dimShift );
  }

  //----- determine scale and offset -----
  shift = m_LICShift;
  if( cntShift == 0 )
  {
    scale   = ( 1 << shift );
    offset  = 0;
    return;
  }

  const int cropShift     = std::max( 0, bitDepth - precShift + cntShift - 15 );
  const int xzOffset      = ( xx >> m_LICRegShift );
  const int sumX          = x << precShift;
  const int sumY          = y << precShift;
  const int sumXX         = ( ( xx + xzOffset ) >> ( cropShift << 1 ) ) << cntShift;
  const int sumXY         = ( ( xy + xzOffset ) >> ( cropShift << 1 ) ) << cntShift;
  const int sumXsumX      = ( x >> cropShift ) * ( x >> cropShift );
  const int sumXsumY      = ( x >> cropShift ) * ( y >> cropShift );
        int a1            = sumXY - sumXsumY;
        int a2            = sumXX - sumXsumX;
        int scaleShiftA2  = getMSB( abs( a2 ) ) - 6;
        int scaleShiftA1  = scaleShiftA2 - m_LICShiftDiff;
            scaleShiftA2  = std::max( 0, scaleShiftA2 );
            scaleShiftA1  = std::max( 0, scaleShiftA1 );
  const int scaleShiftA   = scaleShiftA2 + 15 - shift - scaleShiftA1;
            a1            =               a1 >> scaleShiftA1;
            a2            = Clip3( 0, 63, a2 >> scaleShiftA2 );
            scale         = int( ( int64_t( a1 ) * int64_t( m_LICMultApprox[a2] ) ) >> scaleShiftA );
            scale         = Clip3( 0, 1 << ( shift + 2 ), scale );
  const int maxOffset     = ( 1 << ( bitDepth - 1 ) ) - 1;
  const int minOffset     = -1 - maxOffset;
            offset        = ( sumY - ( ( scale * sumX ) >> shift ) + ( ( 1 << ( cntShift ) ) >> 1 ) ) >> cntShift;
            offset        = Clip3( minOffset, maxOffset, offset );
}

void InterPrediction::xLocalIlluComp( const PredictionUnit& pu,
                                      const ComponentID     compID,
                                      const Picture&        refPic,
                                      const Mv&             mv,
                                      const bool            biPred,
                                            PelBuf&         dstBuf )
{
  int shift = 0, scale = 0, offset = 0;

  xGetLICParams( *pu.cu, compID, refPic, mv, shift, scale, offset );

  const int bitDepth   = pu.cs->sps->getBitDepth( toChannelType( compID ) );
  const ClpRng& clpRng = pu.cu->cs->slice->clpRng(compID);

  dstBuf.linearTransform( scale, shift, offset, true, clpRng );

  if( biPred )
  {
    const int biShift   =  IF_INTERNAL_PREC - bitDepth;
    const Pel biOffset  = -IF_INTERNAL_OFFS;
    ClpRng clpRngDummy;
    dstBuf.linearTransform( 1, -biShift, biOffset, false, clpRngDummy );
  }
}



void InterPrediction::applyBiOptFlow( const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, const int &iRefIdx0, const int &iRefIdx1, PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths )
{
  const int     iHeight     = pcYuvDst.Y().height;
  const int     iWidth      = pcYuvDst.Y().width;
#if JVET_K0485_BIO
  int           iHeightG   = iHeight + 2 * JVET_K0485_BIO_EXTEND_SIZE;
  int           iWidthG    = iWidth + 2 * JVET_K0485_BIO_EXTEND_SIZE;
  int           offsetPos  = iWidthG*JVET_K0485_BIO_EXTEND_SIZE + JVET_K0485_BIO_EXTEND_SIZE;
#else
  int           iHeightG    = iHeight;
  int           iWidthG     = iWidth;
#endif
//  int           iStrideTemp = 2 + 2 * iWidthG;
  Pel*          pGradX0     = m_pGradX0;
  Pel*          pGradX1     = m_pGradX1;
  Pel*          pGradY0     = m_pGradY0;
  Pel*          pGradY1     = m_pGradY1;
#if JVET_K0485_BIO
  int           stridePredMC = iWidthG + 2;
  const Pel*    pSrcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + stridePredMC + 1;
  const Pel*    pSrcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + stridePredMC + 1;
  const int     iSrc0Stride = stridePredMC;
  const int     iSrc1Stride = stridePredMC;
#else
  const Pel*    pSrcY0      = pcYuvSrc0.Y().buf;
  const Pel*    pSrcY1      = pcYuvSrc1.Y().buf;
  const int     iSrc0Stride = pcYuvSrc0.Y().stride;
  const int     iSrc1Stride = pcYuvSrc1.Y().stride;
#endif
  Pel*          pDstY       = pcYuvDst.Y().buf;
  const int     iDstStride  = pcYuvDst.Y().stride;
  const Pel*    pSrcY0Temp  = pSrcY0;
  const Pel*    pSrcY1Temp  = pSrcY1;

#if JVET_K0485_BIO
  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    Pel* dstTempPtr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + stridePredMC + 1;
    Pel* pGradY = (refList == 0) ? m_pGradY0 : m_pGradY1;
    Pel* pGradX = (refList == 0) ? m_pGradX0 : m_pGradX1;

    bioGradFilter(dstTempPtr, stridePredMC, iWidthG, iHeightG, iWidthG, pGradX, pGradY);
  }
#else
  int dT0 = pu.cs->slice->getRefPOC( REF_PIC_LIST_0 , iRefIdx0 ) - pu.cs->slice->getPOC();
  int dT1 = pu.cs->slice->getPOC() - pu.cs->slice->getRefPOC( REF_PIC_LIST_1 , iRefIdx1 );
  if( dT0 * dT1 < 0 )
  {
    Pel* tmpGradX0 = m_pGradX0;
    Pel* tmpGradX1 = m_pGradX1;
    Pel* tmpGradY0 = m_pGradY0;
    Pel* tmpGradY1 = m_pGradY1;
    for( int y = 0; y < iHeightG; y++ )
    {
      for( int x = 0; x < iWidthG ; x++ )
      {
        tmpGradX0[x] *= dT0;
        tmpGradX1[x] *= dT1;
        tmpGradY0[x] *= dT0;
        tmpGradY1[x] *= dT1;
      }
      tmpGradX0 += iWidthG;
      tmpGradX1 += iWidthG;
      tmpGradY0 += iWidthG;
      tmpGradY1 += iWidthG;
    }
  }
#endif
  
  const ClpRng& clpRng        = pu.cu->cs->slice->clpRng(COMPONENT_Y);
  const int   bitDepth        = clipBitDepths.recon[ toChannelType(COMPONENT_Y) ];
  const int   shiftNum        = IF_INTERNAL_PREC + 1 - bitDepth;
  const int   offset          = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
  const bool  bShortRefMV     = ( pu.cs->slice->getCheckLDC() && PU::isBIOLDB(pu) );
#if JVET_K0485_BIO
  const int64_t limit         = (16 << (IF_INTERNAL_PREC - bShortRefMV - bitDepth));
#else
  const int64_t limit         = ( 12 << (IF_INTERNAL_PREC - bShortRefMV - bitDepth ) );
#endif
  const int64_t regularizator_1 = 500 * (1<<(bitDepth-8)) * (1<<(bitDepth-8));
  const int64_t regularizator_2 = regularizator_1<<1;
  const int64_t denom_min_1     = 700 * (1<<(bitDepth-8)) * (1<<(bitDepth-8));
  const int64_t denom_min_2     = denom_min_1<<1;

  int64_t* m_piDotProductTemp1 = m_piDotProduct1;
  int64_t* m_piDotProductTemp2 = m_piDotProduct2;
  int64_t* m_piDotProductTemp3 = m_piDotProduct3;
  int64_t* m_piDotProductTemp5 = m_piDotProduct5;
  int64_t* m_piDotProductTemp6 = m_piDotProduct6;

  int64_t temp=0, tempX=0, tempY=0;
  for( int y = 0; y < iHeightG; y++ )
  {
    for( int x = 0; x < iWidthG; x++ )
    {
#if JVET_K0485_BIO
      temp  = (pSrcY0Temp[x] - pSrcY1Temp[x]);
      tempX = (pGradX0[x] + pGradX1[x]);
      tempY = (pGradY0[x] + pGradY1[x]);
#else
      temp  = (int64_t)( pSrcY0Temp[x] - pSrcY1Temp[x] );
      tempX = (int64_t)( pGradX0   [x] + pGradX1   [x] );
      tempY = (int64_t)( pGradY0   [x] + pGradY1   [x] );
#endif
      m_piDotProductTemp1[x] =  tempX * tempX;
      m_piDotProductTemp2[x] =  tempX * tempY;
      m_piDotProductTemp3[x] = -tempX * temp<<5;
      m_piDotProductTemp5[x] =  tempY * tempY<<1;
      m_piDotProductTemp6[x] = -tempY * temp<<6;
    }
    pSrcY0Temp          += iSrc0Stride;
    pSrcY1Temp          += iSrc1Stride;
    pGradX0             += iWidthG;
    pGradX1             += iWidthG;
    pGradY0             += iWidthG;
    pGradY1             += iWidthG;
    m_piDotProductTemp1 += iWidthG;
    m_piDotProductTemp2 += iWidthG;
    m_piDotProductTemp3 += iWidthG;
    m_piDotProductTemp5 += iWidthG;
    m_piDotProductTemp6 += iWidthG;
  }

  int xUnit = (iWidth >> 2);
  int yUnit = (iHeight >> 2);

  Pel *pDstY0 = pDstY;
  pGradX0 = m_pGradX0; pGradX1 = m_pGradX1;
  pGradY0 = m_pGradY0; pGradY1 = m_pGradY1;

  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
#if JVET_K0485_BIO
      if (m_bioPredSubBlkDist[yu*xUnit + xu] < m_bioSubBlkDistThres)
      {
        pSrcY0Temp = pSrcY0 + (stridePredMC + 1) + ((yu*iSrc0Stride + xu) << 2);
        pSrcY1Temp = pSrcY1 + (stridePredMC + 1) + ((yu*iSrc1Stride + xu) << 2);
        pDstY0 = pDstY + ((yu*iDstStride + xu) << 2);

        g_pelBufOP.addAvg4(pSrcY0Temp, iSrc0Stride, pSrcY1Temp, iSrc1Stride, pDstY0, iDstStride, (1 << 2), (1 << 2), shiftNum, offset, clpRng);
        continue;
      }
#endif
      int64_t sGxdI = 0, sGydI = 0, sGxGy = 0, sGx2 = 0, sGy2 = 0;
      int64_t tmpx = 0, tmpy = 0;

#if JVET_K0485_BIO
      m_piDotProductTemp1 = m_piDotProduct1 + offsetPos + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp2 = m_piDotProduct2 + offsetPos + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp3 = m_piDotProduct3 + offsetPos + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp5 = m_piDotProduct5 + offsetPos + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp6 = m_piDotProduct6 + offsetPos + ((yu*iWidthG + xu) << 2);

      calcBlkGradient(xu << 2, yu << 2, m_piDotProductTemp1, m_piDotProductTemp2, m_piDotProductTemp3, m_piDotProductTemp5, m_piDotProductTemp6,
                      sGx2, sGy2, sGxGy, sGxdI, sGydI, iWidthG, iHeightG, (1 << 2));
#else
      m_piDotProductTemp1 = m_piDotProduct1 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp2 = m_piDotProduct2 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp3 = m_piDotProduct3 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp5 = m_piDotProduct5 + ((yu*iWidthG + xu) << 2);
      m_piDotProductTemp6 = m_piDotProduct6 + ((yu*iWidthG + xu) << 2);

      calcBlkGradient(xu << 2, yu << 2, m_piDotProductTemp1, m_piDotProductTemp2, m_piDotProductTemp3, m_piDotProductTemp5, m_piDotProductTemp6,
                      sGx2, sGy2, sGxGy, sGxdI, sGydI, iWidthG, iHeightG);

      sGxdI >>= 4;
      sGydI >>= 4;
      sGxGy >>= 4;
      sGx2 >>= 4;
      sGy2 >>= 4;
#endif

      sGx2 += regularizator_1;
      sGy2 += regularizator_2;

      if (sGx2 > denom_min_1)
      {
        tmpx = divide64(sGxdI, sGx2);
        tmpx = Clip3(-limit, limit, tmpx);
      }
      if (sGy2 > denom_min_2)
      {
        tmpy = divide64((sGydI - tmpx * sGxGy), sGy2);
        tmpy = Clip3(-limit, limit, tmpy);
      }

#if JVET_K0485_BIO
      pSrcY0Temp = pSrcY0 + (stridePredMC + 1) + ((yu*iSrc0Stride + xu) << 2);
      pSrcY1Temp = pSrcY1 + (stridePredMC + 1) + ((yu*iSrc0Stride + xu) << 2);
      pGradX0 = m_pGradX0 + offsetPos + ((yu*iWidthG + xu) << 2);
      pGradX1 = m_pGradX1 + offsetPos + ((yu*iWidthG + xu) << 2);
      pGradY0 = m_pGradY0 + offsetPos + ((yu*iWidthG + xu) << 2);
      pGradY1 = m_pGradY1 + offsetPos + ((yu*iWidthG + xu) << 2);

      pDstY0 = pDstY + ((yu*iDstStride + xu) << 2);
      g_pelBufOP.addBIOAvg4(pSrcY0Temp, iSrc0Stride, pSrcY1Temp, iSrc1Stride, pDstY0, iDstStride, pGradX0, pGradX1, pGradY0, pGradY1, iWidthG, (1 << 2), (1 << 2), (int)tmpx, (int)tmpy, shiftNum, offset, clpRng);
#else
      pSrcY0Temp = pSrcY0 + ((yu*iSrc0Stride + xu) << 2);
      pSrcY1Temp = pSrcY1 + ((yu*iSrc0Stride + xu) << 2);
      pGradX0 = m_pGradX0 + ((yu*iWidthG + xu) << 2);
      pGradX1 = m_pGradX1 + ((yu*iWidthG + xu) << 2);
      pGradY0 = m_pGradY0 + ((yu*iWidthG + xu) << 2);
      pGradY1 = m_pGradY1 + ((yu*iWidthG + xu) << 2);

      pDstY0 = pDstY + ((yu*iDstStride + xu) << 2);

      // apply BIO offset for the sub-block
      for (int y = 0; y < 4; y++)
      {
        for (int x = 0; x < 4; x++)
        {
          int b = (int)tmpx * (pGradX0[x] - pGradX1[x]) + (int)tmpy * (pGradY0[x] - pGradY1[x]);
          b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));

          pDstY0[x] = ( ClipPel( ( int16_t ) ( ( pSrcY0Temp[x] + pSrcY1Temp[x] + b + offset ) >> shiftNum ), clpRng ) );
        }
        pDstY0 += iDstStride; pSrcY0Temp += iSrc0Stride; pSrcY1Temp += iSrc1Stride;
        pGradX0 += iWidthG; pGradX1 += iWidthG; pGradY0 += iWidthG; pGradY1 += iWidthG;
      }
#endif
    }  // xu
  }  // yu
}

#if JVET_K0485_BIO
bool InterPrediction::xCalcBiPredSubBlkDist(const PredictionUnit &pu, const Pel* pYuvSrc0, const int src0Stride, const Pel* pYuvSrc1, const int src1Stride, const BitDepths &clipBitDepths)
{
  const int     width = pu.lwidth();
  const int     height = pu.lheight();
  const int     clipbd = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(clipbd - 8);
  const int     shift = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int     xUnit = (width >> 2);
  const int     yUnit = (height >> 2);

  m_bioDistThres = (shift <= 5) ? (((32 << (clipbd - 8))*width*height) >> (5 - shift)) : (((32 << (clipbd - 8))*width*height) << (shift - 5));
  m_bioSubBlkDistThres = (shift <= 5) ? (((64 << (clipbd - 8)) << 4) >> (5 - shift)) : (((64 << (clipbd - 8)) << 4) << (shift - 5));

  m_bioDistThres >>= distortionShift;
  m_bioSubBlkDistThres >>= distortionShift;

  DistParam cDistParam;
  Distortion dist = 0;
  for (int yu = 0, blkIdx = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++, blkIdx++)
    {
      const Pel* pPred0 = pYuvSrc0 + ((yu*src0Stride + xu) << 2);
      const Pel* pPred1 = pYuvSrc1 + ((yu*src1Stride + xu) << 2);

      m_pcRdCost->setDistParam(cDistParam, pPred0, pPred1, src0Stride, src1Stride, clipbd, COMPONENT_Y, (1 << 2), (1 << 2), 0, 1, false, true);
      m_bioPredSubBlkDist[blkIdx] = cDistParam.distFunc(cDistParam);
      dist += m_bioPredSubBlkDist[blkIdx];
    }
  }

  return (dist >= m_bioDistThres);
}
#endif
#endif

#if JEM_TOOLS
void InterPrediction::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bBIOApplied )
#else
void InterPrediction::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs )
#endif
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
#if JEM_TOOLS
    if( bBIOApplied )
    {
#if JVET_K0485_BIO
      const int  src0Stride = pu.lwidth() + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2;
      const int  src1Stride = pu.lwidth() + 2 * JVET_K0485_BIO_EXTEND_SIZE + 2;
      const Pel* pSrcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + 2 * src0Stride + 2;
      const Pel* pSrcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + 2 * src1Stride + 2;

      bool bioEnabled = xCalcBiPredSubBlkDist(pu, pSrcY0, src0Stride, pSrcY1, src1Stride, clipBitDepths);
      if (bioEnabled)
#endif
      applyBiOptFlow( pu, pcYuvSrc0, pcYuvSrc1, iRefIdx0, iRefIdx1, pcYuvDst, clipBitDepths );
#if JVET_K0485_BIO
      else
        pcYuvDst.bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);
#endif
    }
    pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, bBIOApplied );
#else
    pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs );
#endif
  }
  else if( iRefIdx0 >= 0 && iRefIdx1 < 0 )
  {
    pcYuvDst.copyClip( pcYuvSrc0, clpRngs );
  }
  else if( iRefIdx0 < 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst.copyClip( pcYuvSrc1, clpRngs );
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList 
#if JVET_K0076_CPR_DT
  , const bool luma, const bool chroma
#endif
)
{
#if JVET_K0076_CPR_DT
    // dual tree handling for CPR as the only ref
 if (!luma || !chroma)
 {
    if (!luma && chroma)
    {
      xChromaMC(pu, predBuf);
      return;
    }
    else // (luma && !chroma)
    {
      xPredInterUni(pu, eRefPicList, predBuf, false, false, false, luma, chroma);
      return;
    }
 }
  // else, go with regular MC below
#endif
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  if( eRefPicList != REF_PIC_LIST_X )
  {
#if JEM_TOOLS
    if( !pu.cu->LICFlag && ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
#else
    if( ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
#endif
    {
      xPredInterUni         ( pu,          eRefPicList, predBuf, true );
      xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred );
    }
    else
    {
      xPredInterUni( pu, eRefPicList, predBuf, false );
    }
  }
  else
  {
#if JEM_TOOLS || JVET_K0346
#if JVET_K0076_CPR
    if (pu.mergeType != MRG_TYPE_DEFAULT_N && pu.mergeType != MRG_TYPE_IBC)
#else
    if( pu.mergeType != MRG_TYPE_DEFAULT_N )
#endif
    {
      xSubPuMC( pu, predBuf, eRefPicList );
    }
    else if( xCheckIdenticalMotion( pu ) )
#else
    if( xCheckIdenticalMotion( pu ) )
#endif
    {
      xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false );
    }
    else
    {
      xPredInterBi( pu, predBuf );
    }
  }
  return;
}

void InterPrediction::motionCompensation( CodingUnit &cu, const RefPicList &eRefPicList 
#if JVET_K0076_CPR_DT
  , const bool luma, const bool chroma
#endif
)
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
#if JEM_TOOLS
    pu.mvRefine = true;
    motionCompensation( pu, predBuf, eRefPicList 
#if JVET_K0076_CPR_DT
      , luma, chroma
#endif
    );
    pu.mvRefine = false;
#else
    motionCompensation( pu, predBuf, eRefPicList );
#endif
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ 
#if JVET_K0076_CPR_DT
  , const bool luma, const bool chroma
#endif
)
{
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  motionCompensation( pu, predBuf, eRefPicList 
#if JVET_K0076_CPR_DT
    , luma, chroma
#endif
  );
}

#if JEM_TOOLS
/** Function for sub-block based Overlapped Block Motion Compensation (OBMC).
*
* This function can:
* 1. Perform sub-block OBMC for a CU.
* 2. Before motion estimation, subtract (scaled) predictors generated by applying neighboring motions to current CU/PU from the original signal of current CU/PU,
*    to make the motion estimation biased to OBMC.
*/
void InterPrediction::subBlockOBMC( CodingUnit  &cu )
{
  if( !cu.cs->sps->getSpsNext().getUseOBMC() || !cu.obmcFlag )
  {
    return;
  }

  for( auto &pu : CU::traversePUs( cu ) )
  {
    subBlockOBMC( pu, nullptr, false );
  }
}

void InterPrediction::subBlockOBMC( PredictionUnit  &pu, PelUnitBuf* pDst, bool bOBMC4ME )
{
  if( !pu.cs->sps->getSpsNext().getUseOBMC() || !pu.cu->obmcFlag )
  {
    return;
  }

  CodingStructure &cs        = *pu.cs;

  PelUnitBuf pcYuvPred       = pDst == nullptr ? pu.cs->getPredBuf( pu ) : *pDst;
  PelUnitBuf pcYuvTmpPred1   = m_tmpObmcBuf.subBuf( UnitAreaRelative( *pu.cu, pu ) );

  const PartSize   ePartSize = pu.cu->partSize;
  const UnitArea   orgPuArea = pu;

  const uint32_t uiWidth         = pu.lwidth();
  const uint32_t uiHeight        = pu.lheight();

  const uint32_t uiMinCUW        = pu.cs->pcv->minCUWidth;

  const uint32_t uiOBMCBlkSize   = pu.cs->sps->getSpsNext().getOBMCBlkSize();

  const uint32_t uiHeightInBlock = uiHeight         / uiMinCUW;
  const uint32_t uiWidthInBlock  = uiWidth          / uiMinCUW;
  const uint32_t uiHeightInCU    = pu.cu->lheight() / uiMinCUW;
  const uint32_t uiWidhtInCU     = pu.cu->lwidth()  / uiMinCUW;
  const uint32_t uiStep          = uiOBMCBlkSize    / uiMinCUW;

  const bool bOBMCSimp       = cs.pcv->only2Nx2N ? uiWidth * uiHeight < 64 : ( pu.cu->lwidth() == 8 && ePartSize != SIZE_2Nx2N );

  const bool bATMVP          = ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.mergeType == MRG_TYPE_SUBPU_ATMVP_EXT );
  const bool bFruc           = ( pu.frucMrgMode == FRUC_MERGE_TEMPLATE || pu.frucMrgMode == FRUC_MERGE_BILATERALMV );
  const bool bAffine         = pu.cu->affine;

        int  i1stPUWidth     = -1, i1stPUHeight = -1;
        bool b2ndPU          = false;
  const bool bVerticalPU     = false;
  const bool bHorizontalPU   = false;

  const bool bTwoPUs         = ( bVerticalPU || bHorizontalPU );

  if( bTwoPUs )
  {
    PredictionUnit* fPU = pu.cu->firstPU;
    i1stPUWidth   = fPU->lwidth();
    i1stPUHeight  = fPU->lheight();

    if( pu.cu->firstPU->next && pu.lumaPos() == fPU->next->lumaPos() )
    {
      b2ndPU = true;
    }

    i1stPUWidth  /= uiMinCUW;
    i1stPUHeight /= uiMinCUW;
  }

  const int avgLength      = ( cs.pcv->rectCUs ) ? 1 << ( ( ( ( int ) log2( pu.cu->lumaSize().width ) + ( int ) log2( pu.cu->lumaSize().height ) - 3 ) >> 1 ) + MIN_CU_LOG2 ) : pu.cu->lumaSize().width;
  const int nRefineBlkSize = std::max( avgLength >> pu.cs->slice->getSPS()->getSpsNext().getFRUCSmallBlkRefineDepth(), FRUC_MERGE_REFINE_MINBLKSIZE );

  const bool bNormal2Nx2N  = ePartSize == SIZE_2Nx2N && !bATMVP && !bFruc && !bAffine;
  const bool bSubMotion    = ePartSize == SIZE_2Nx2N && ( bATMVP || bFruc || bAffine );

  MotionInfo currMi  = pu.getMotionInfo();
  MotionInfo NeighMi = MotionInfo();

  int maxDir =  bNormal2Nx2N ? 2 : 4;

  for (int iSubX = 0; iSubX < uiWidthInBlock; iSubX += uiStep)
  {
    for (int iSubY = 0; iSubY < uiHeightInBlock; iSubY += uiStep)
    {
      if (bNormal2Nx2N && iSubX && iSubY)
      {
        continue;
      }

      bool bCURBoundary = bVerticalPU   ? ( iSubX == uiWidhtInCU  - uiStep ) : b2ndPU ? iSubX + i1stPUWidth   == uiWidhtInCU  - uiStep : ( iSubX == uiWidhtInCU  - uiStep ) ;
      bool bCUBBoundary = bHorizontalPU ? ( iSubY == uiHeightInCU - uiStep ) : b2ndPU ? iSubY + i1stPUHeight  == uiHeightInCU - uiStep : ( iSubY == uiHeightInCU - uiStep ) ;

      for (int iDir = 0; iDir < maxDir; iDir++) //iDir: 0 - above, 1 - left, 2 - below, 3 - right
      {
        if ((iDir == 3 && bCURBoundary) || (iDir == 2 && bCUBBoundary))
        {
          continue;
        }

        bool bVerPUBound = false;
        bool bHorPUBound = false;

        if( bNormal2Nx2N ) //skip unnecessary check for CU boundary
        {
          if( ( iDir == 1 && !iSubY && iSubX ) || ( iDir == 0 && !iSubX && iSubY ) )
          {
            continue;
          }
        }
        else
        {
          bool bCheckNeig = bSubMotion || ( iSubX == 0 && iDir == 1 ) || ( iSubY == 0 && iDir == 0 ); //CU boundary or NxN or 2nx2n_ATMVP
          if( !bCheckNeig && bTwoPUs )
          {
            bCheckNeig |= bFruc;
            bCheckNeig |= bATMVP;

            //PU boundary
            if( !b2ndPU )
            {
              bVerPUBound = bVerticalPU   && ( ( iDir == 2 && iSubY == i1stPUHeight - uiStep ) );
              bHorPUBound = bHorizontalPU && ( ( iDir == 3 && iSubX == i1stPUWidth  - uiStep ) );
            }

            bCheckNeig |= ( bVerPUBound || bHorPUBound );
          }
          if( !bCheckNeig )
          {
            continue;
          }
        }

#if JVET_K0346
        bool bSubBlockOBMCSimp = (bOBMCSimp || ((pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.mergeType == MRG_TYPE_SUBPU_ATMVP_EXT) && (1 << pu.cs->slice->getSubPuMvpSubblkLog2Size()) == 4));
#else
        bool bSubBlockOBMCSimp = ( bOBMCSimp || ( (pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.mergeType == MRG_TYPE_SUBPU_ATMVP_EXT ) && ( 1 << pu.cs->sps->getSpsNext().getSubPuMvpLog2Size() ) == 4 ) );
#endif
        bSubBlockOBMCSimp |= ( bFruc && nRefineBlkSize == 4 );
        bSubBlockOBMCSimp |= bAffine;

        if( PU::getNeighborMotion( pu, NeighMi, Position( iSubX * uiMinCUW, iSubY * uiMinCUW ), iDir, ( bATMVP || bFruc || bAffine ) ) )
        {
          //store temporary motion information
          pu              = NeighMi;
          pu.cu->partSize = SIZE_2Nx2N;
          pu.cu->affine   = false;
          pu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( orgPuArea.lumaPos().offset( iSubX * uiMinCUW, iSubY * uiMinCUW ), Size{ uiOBMCBlkSize, uiOBMCBlkSize } ) ) );

          const UnitArea predArea = UnitAreaRelative( orgPuArea, pu );

          PelUnitBuf cPred = pcYuvPred    .subBuf( predArea );
          PelUnitBuf cTmp1 = pcYuvTmpPred1.subBuf( predArea );

          xSubBlockMotionCompensation( pu, cTmp1 );

          if( bOBMC4ME )
          {
            xSubtractOBMC( pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
          }
          else
          {
            xSubblockOBMC( COMPONENT_Y,  pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cb, pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cr, pu, cPred, cTmp1, iDir, bSubBlockOBMCSimp );
          }

          //restore motion information
          pu.cu->partSize  = ePartSize;
          pu               = currMi;
          pu.cu->affine    = bAffine;
          pu.UnitArea::operator=( orgPuArea );
        }
      }
    }
  }
}


// Function for (weighted) averaging predictors of current block and predictors generated by applying neighboring motions to current block.
void InterPrediction::xSubblockOBMC(const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, int iDir, bool bOBMCSimp)
{
  int iWidth  = pu.blocks[eComp].width;
  int iHeight = pu.blocks[eComp].height;

  if( iWidth == 0 || iHeight == 0 )
  {
    return;
  }

  PelBuf *pDst = &pcYuvPredDst.bufs[eComp];//.at(0, 0);
  PelBuf *pSrc = &pcYuvPredSrc.bufs[eComp];//.at(0, 0);

  if( iDir == 0 ) //above
  {
    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, 0 ) = ( 3 * pDst->at( i, 0 ) + pSrc->at( i, 0 ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( i, 1 ) = ( 7 * pDst->at( i, 1 ) + pSrc->at( i, 1 ) + 4 ) >> 3;
      }

      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( i, 2 ) = ( 15 * pDst->at( i, 2 ) + pSrc->at( i, 2 ) +  8 ) >> 4;
        pDst->at( i, 3 ) = ( 31 * pDst->at( i, 3 ) + pSrc->at( i, 3 ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( 0, i ) = ( 3 * pDst->at( 0, i ) + pSrc->at( 0, i ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( 1, i ) = ( 7 * pDst->at( 1, i ) + pSrc->at( 1, i ) + 4 ) >> 3;
      }

      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( 2, i ) = ( 15 * pDst->at( 2, i ) + pSrc->at( 2, i ) +  8 ) >> 4;
        pDst->at( 3, i ) = ( 31 * pDst->at( 3, i ) + pSrc->at( 3, i ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, iHeight - 1 ) = ( 3 * pDst->at( i, iHeight - 1 ) + pSrc->at( i, iHeight - 1 ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( i, iHeight - 2 ) = ( 7 * pDst->at( i, iHeight - 2 ) + pSrc->at( i, iHeight - 2 ) + 4 ) >> 3;
      }
      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( i, iHeight - 3 ) = ( 15 * pDst->at( i, iHeight - 3 ) + pSrc->at( i, iHeight - 3 ) +  8 ) >> 4;
        pDst->at( i, iHeight - 4 ) = ( 31 * pDst->at( i, iHeight - 4 ) + pSrc->at( i, iHeight - 4 ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 3 ) //right
  {
    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( iWidth - 1, i ) = ( 3 * pDst->at( iWidth - 1, i ) + pSrc->at( iWidth - 1, i ) + 2 ) >> 2;

      if( !bOBMCSimp || eComp == COMPONENT_Y )
      {
        pDst->at( iWidth - 2, i ) = ( 7 * pDst->at( iWidth - 2, i ) + pSrc->at( iWidth - 2, i ) + 4 ) >> 3;
      }

      if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
      {
        pDst->at( iWidth - 3, i ) = ( 15 * pDst->at( iWidth - 3, i ) + pSrc->at( iWidth - 3, i ) +  8 ) >> 4;
        pDst->at( iWidth - 4, i ) = ( 31 * pDst->at( iWidth - 4, i ) + pSrc->at( iWidth - 4, i ) + 16 ) >> 5;
      }
    }
  }
}

// Function for subtracting (scaled) predictors generated by applying neighboring motions to current block from the original signal of current block.
void InterPrediction::xSubtractOBMC( PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, int iDir, bool bOBMCSimp )
{
  int iWidth  = pu.lwidth();
  int iHeight = pu.lheight();

  PelBuf *pDst = &pcYuvPredDst.bufs[COMPONENT_Y];
  PelBuf *pSrc = &pcYuvPredSrc.bufs[COMPONENT_Y];

  if( iDir == 0 ) //above
  {
    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, 0 ) += ( pDst->at( i, 0 ) - pSrc->at( i, 0 ) + 2 ) >> 2;
      pDst->at( i, 1 ) += ( pDst->at( i, 1 ) - pSrc->at( i, 1 ) + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      for( int i = 0; i < iWidth; i++ )
      {
        pDst->at( i, 2 ) += ( pDst->at( i, 2 ) - pSrc->at( i, 2 ) +  8 ) >> 4;
        pDst->at( i, 3 ) += ( pDst->at( i, 3 ) - pSrc->at( i, 3 ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( 0, i ) += ( pDst->at( 0, i ) - pSrc->at( 0, i ) + 2 ) >> 2;
      pDst->at( 1, i ) += ( pDst->at( 1, i ) - pSrc->at( 1, i ) + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      for( int i = 0; i < iHeight; i++ )
      {
        pDst->at( 2, i ) += ( pDst->at( 2, i ) - pSrc->at( 2, i ) +  8 ) >> 4;
        pDst->at( 3, i ) += ( pDst->at( 3, i ) - pSrc->at( 3, i ) + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    if( !bOBMCSimp )
    {
      for( int i = 0; i < iWidth; i++ )
      {
        pDst->at( i, iHeight - 4 ) += ( pDst->at( i, iHeight - 4 ) - pSrc->at( i, iHeight - 4 ) + 16 ) >> 5;
        pDst->at( i, iHeight - 3 ) += ( pDst->at( i, iHeight - 3 ) - pSrc->at( i, iHeight - 3 ) +  8 ) >> 4;
      }
    }

    for( int i = 0; i < iWidth; i++ )
    {
      pDst->at( i, iHeight - 2 ) += ( pDst->at( i, iHeight - 2 ) - pSrc->at( i, iHeight - 2 ) + 4 ) >> 3;
      pDst->at( i, iHeight - 1 ) += ( pDst->at( i, iHeight - 1 ) - pSrc->at( i, iHeight - 1 ) + 2 ) >> 2;
    }
  }

  if( iDir == 3 ) //right
  {
    if( !bOBMCSimp )
    {
      for( int i = 0; i < iHeight; i++ )
      {
        pDst->at( iWidth - 4, i ) += ( pDst->at( iWidth - 4, i ) - pSrc->at( iWidth - 4, i ) + 16 ) >> 5;
        pDst->at( iWidth - 3, i ) += ( pDst->at( iWidth - 3, i ) - pSrc->at( iWidth - 3, i ) +  8 ) >> 4;
      }
    }

    for( int i = 0; i < iHeight; i++ )
    {
      pDst->at( iWidth - 2, i ) += ( pDst->at( iWidth - 2, i ) - pSrc->at( iWidth - 2, i ) + 4 ) >> 3;
      pDst->at( iWidth - 1, i ) += ( pDst->at( iWidth - 1, i ) - pSrc->at( iWidth - 1, i ) + 2 ) >> 2;
    }
  }
}
#endif

#if JEM_TOOLS
void InterPrediction::xSubBlockMotionCompensation( PredictionUnit &pu, PelUnitBuf &pcYuvPred )
{
  if( xCheckIdenticalMotion( pu ) )
  {
    xPredInterUni( pu, REF_PIC_LIST_0, pcYuvPred, false );
  }
  else
  {
    xPredInterBi( pu, pcYuvPred, true );
  }
}
#endif

#if JEM_TOOLS
#if JVET_K0485_BIO
void InterPrediction::gradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* pGradX, Pel* pGradY)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      pGradY[x] = (pSrc[x + srcStride] - pSrc[x - srcStride]) >> 4;
      pGradX[x] = (pSrc[x + 1] - pSrc[x - 1]) >> 4;
    }
    pGradX += gradStride;
    pGradY += gradStride;
    pSrc += srcStride;
  }
}
#else
void InterPrediction::xGradFilterX( const Pel* piRefY, int iRefStride, Pel* piDstY, int iDstStride, int iWidth, int iHeight, int iMVyFrac, int iMVxFrac, const int bitDepth )
{
  static const int iBIOGradShift = 4;
  if( iMVyFrac == 0 )
  {
    gradFilter1DHor( piRefY, iRefStride,  iWidth, iHeight, iDstStride,  piDstY, iMVxFrac, iBIOGradShift );
    return;
  }

  int   tmpStride = iWidth + BIO_FILTER_LENGTH_MINUS_1;
  Pel*  tmp       = m_filteredBlockTmp[0][0];
  int   shift0    = bitDepth-8;
  int   shift1    = 6 + iBIOGradShift - shift0;
  fracFilter2DVer ( piRefY - BIO_FILTER_HALF_LENGTH_MINUS_1, iRefStride, iWidth+BIO_FILTER_LENGTH_MINUS_1, iHeight, tmpStride,  tmp,    iMVyFrac, shift0 );
  JVET_J0090_SET_CACHE_ENABLE( false );
  gradFilter2DHor ( tmp    + BIO_FILTER_HALF_LENGTH_MINUS_1, tmpStride,  iWidth,                           iHeight, iDstStride, piDstY, iMVxFrac, shift1 );
  JVET_J0090_SET_CACHE_ENABLE( true );
}

void InterPrediction::xGradFilterY( const Pel* piRefY, int iRefStride, Pel* piDstY, int iDstStride, int iWidth, int iHeight, int iMVyFrac, int iMVxFrac, const int bitDepth )
{
  static const int iBIOGradShift = 4;
  if( iMVxFrac == 0 )
  {
    gradFilter1DVer( piRefY, iRefStride, iWidth, iHeight, iDstStride, piDstY, iMVyFrac, iBIOGradShift );
    return;
  }

  int   tmpStride = iWidth + BIO_FILTER_LENGTH_MINUS_1;
  Pel*  tmp       = m_filteredBlockTmp[0][0];
  int   shift0    = bitDepth-8;
  int   shift1    = 6 + iBIOGradShift - shift0;
  gradFilter2DVer ( piRefY - BIO_FILTER_HALF_LENGTH_MINUS_1, iRefStride, iWidth+BIO_FILTER_LENGTH_MINUS_1, iHeight, tmpStride,  tmp,    iMVyFrac, shift0 );
  JVET_J0090_SET_CACHE_ENABLE( false );
  fracFilter2DHor ( tmp    + BIO_FILTER_HALF_LENGTH_MINUS_1, tmpStride,  iWidth,                           iHeight, iDstStride, piDstY, iMVxFrac, shift1 );
  JVET_J0090_SET_CACHE_ENABLE( true );
}

Pel InterPrediction::optical_flow_averaging( int64_t s1, int64_t s2, int64_t s3, int64_t s5, int64_t s6, Pel pGradX0, Pel pGradX1, Pel pGradY0, Pel pGradY1, Pel pSrcY0Temp, Pel pSrcY1Temp,
                                             const int shiftNum, const int offset, const int64_t limit, const int64_t denom_min_1, const int64_t denom_min_2, const ClpRng& clpRng )
{
  int64_t vx = 0;
  int64_t vy = 0;
  int64_t b=0;

  if( s1 > denom_min_1 )
  {
    vx = s3  / s1;
    vx = ( vx > limit ? limit : vx < -limit ? -limit : vx );
  }
  if( s5 > denom_min_2 )
  {
    vy = ( s6 - vx*s2 ) / s5;
    vy = ( vy > limit ? limit : vy < -limit ? -limit : vy );
  }

  b = vx * ( pGradX0 - pGradX1 ) + vy * ( pGradY0 - pGradY1 );
  b = ( b > 0 ? (b+32)>>6 : -((-b+32)>>6) );
  return ClipPel( (int16_t)((pSrcY0Temp + pSrcY1Temp + b + offset) >> shiftNum), clpRng );
}

const int16_t m_lumaGradientFilter[4<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][BIO_FILTER_LENGTH] =
{
  {       8,     -39,      -3,      46,     -17,       5 },   //0
  {       8,     -32,     -13,      50,     -18,       5 },   //1 -->-->
  {       7,     -27,     -20,      54,     -19,       5 },   //2 -->
  {       6,     -21,     -29,      57,     -18,       5 },   //3 -->-->
  {       4,     -17,     -36,      60,     -15,       4 },   //4
  {       3,      -9,     -44,      61,     -15,       4 },   //5 -->-->
  {       1,      -4,     -48,      61,     -13,       3 },   //6 -->
  {       0,       1,     -54,      60,      -9,       2 },   //7 -->-->
  {      -1,       4,     -57,      57,      -4,       1 },   //8
  {      -2,       9,     -60,      54,      -1,       0 },   //9 -->-->
  {      -3,      13,     -61,      48,       4,      -1 },   //10-->
  {      -4,      15,     -61,      44,       9,      -3 },   //11-->-->
  {      -4,      15,     -60,      36,      17,      -4 },   //12
  {      -5,      18,     -57,      29,      21,      -6 },   //13-->-->
  {      -5,      19,     -54,      20,      27,      -7 },   //14-->
  {      -5,      18,     -50,      13,      32,      -8 }    //15-->-->
};

const int16_t m_lumaInterpolationFilter[4<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][BIO_FILTER_LENGTH] =
{
  {       0,       0,      64,       0,       0,       0 },   //0
  {       1,      -3,      64,       4,      -2,       0 },   //1 -->-->
  {       1,      -6,      62,       9,      -3,       1 },   //2 -->
  {       2,      -8,      60,      14,      -5,       1 },   //3 -->-->
  {       2,      -9,      57,      19,      -7,       2 },   //4
  {       3,     -10,      53,      24,      -8,       2 },   //5 -->-->
  {       3,     -11,      50,      29,      -9,       2 },   //6 -->
  {       3,     -11,      44,      35,     -10,       3 },   //7 -->-->
  {       1,      -7,      38,      38,      -7,       1 },   //8
  {       3,     -10,      35,      44,     -11,       3 },   //9 -->-->
  {       2,      -9,      29,      50,     -11,       3 },   //10-->
  {       2,      -8,      24,      53,     -10,       3 },   //11-->-->
  {       2,      -7,      19,      57,      -9,       2 },   //12
  {       1,      -5,      14,      60,      -8,       2 },   //13-->-->
  {       1,      -3,       9,      62,      -6,       1 },   //14-->
  {       0,      -2,       4,      64,      -3,       1 }    //15-->-->
};

inline void InterPrediction::gradFilter2DVer( const Pel* piSrc, int iSrcStride,  int width, int height, int iDstStride, Pel*& rpiDst, int iMV, const int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1 * iSrcStride;
  const Pel*  piSrcTmp1     = piSrcTmp  + iSrcStride;
  const Pel*  piSrcTmp2     = piSrcTmp1 + iSrcStride;
  const Pel*  piSrcTmp3     = piSrcTmp2 + iSrcStride;
  const Pel*  piSrcTmp4     = piSrcTmp3 + iSrcStride;
  const Pel*  piSrcTmp5     = piSrcTmp4 + iSrcStride;
  const int16_t *const coeffs = m_lumaGradientFilter[iMV];

  const int   iOffSet       = ( iShift > 0 ? ( 1 << ( iShift - 1 ) ) : 0 );

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      JVET_J0090_CACHE_ACCESS( piSrcTmp,  __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp1, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp2, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp3, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp4, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp5, __FILE__, __LINE__ );
      iSum     =  (  coeffs[0] * *piSrcTmp++
                   + coeffs[1] * *piSrcTmp1++
                   + coeffs[2] * *piSrcTmp2++
                   + coeffs[3] * *piSrcTmp3++
                   + coeffs[4] * *piSrcTmp4++
                   + coeffs[5] * *piSrcTmp5++ );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
    }

    piSrcTmp  += iSrcStride;
    piSrcTmp1 += iSrcStride;
    piSrcTmp2 += iSrcStride;
    piSrcTmp3 += iSrcStride;
    piSrcTmp4 += iSrcStride;
    piSrcTmp5 += iSrcStride;
    piDst     += iDstStride;
  }
}

inline void InterPrediction::gradFilter1DVer( const Pel* piSrc, int iSrcStride, int width, int height, int iDstStride, Pel*& rpiDst, int iMV, const int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1 * iSrcStride;
  const Pel*  piSrcTmp1     = piSrcTmp  + iSrcStride;
  const Pel*  piSrcTmp2     = piSrcTmp1 + iSrcStride;
  const Pel*  piSrcTmp3     = piSrcTmp2 + iSrcStride;
  const Pel*  piSrcTmp4     = piSrcTmp3 + iSrcStride;
  const Pel*  piSrcTmp5     = piSrcTmp4 + iSrcStride;
  const int16_t *const coeffs = m_lumaGradientFilter[iMV];

  const int   iOffSet       = 1 << ( iShift - 1 );

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      JVET_J0090_CACHE_ACCESS( piSrcTmp,  __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp1, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp2, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp3, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp4, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp5, __FILE__, __LINE__ );
      iSum     =   (  coeffs[0] * *piSrcTmp++
                    + coeffs[1] * *piSrcTmp1++
                    + coeffs[2] * *piSrcTmp2++
                    + coeffs[3] * *piSrcTmp3++
                    + coeffs[4] * *piSrcTmp4++
                    + coeffs[5] * *piSrcTmp5++ );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
    }

    piSrcTmp  += iSrcStride;
    piSrcTmp1 += iSrcStride;
    piSrcTmp2 += iSrcStride;
    piSrcTmp3 += iSrcStride;
    piSrcTmp4 += iSrcStride;
    piSrcTmp5 += iSrcStride;
    piDst     += iDstStride;
  }
}

inline void InterPrediction::gradFilter1DHor( const Pel* piSrc, int iSrcStride, int width, int height, int iDstStride, Pel*& rpiDst, int iMV, const int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1;
  const int   iOffSet       = 1 << ( iShift - 1 );
  const int16_t *const coeffs = m_lumaGradientFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      for ( size_t i = 0 ; i < 6 ; i++ )
      {
        JVET_J0090_CACHE_ACCESS( piSrcTmp + i, __FILE__, __LINE__ );
      }
      iSum     =  (  coeffs[0] * piSrcTmp[0]
                   + coeffs[1] * piSrcTmp[1]
                   + coeffs[2] * piSrcTmp[2]
                   + coeffs[3] * piSrcTmp[3]
                   + coeffs[4] * piSrcTmp[4]
                   + coeffs[5] * piSrcTmp[5] );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;

      piSrcTmp++;
    }

    piSrcTmp += iSrcStride;
    piDst    += iDstStride;
  }
}

inline void InterPrediction::gradFilter2DHor( const Pel* piSrc, int iSrcStride, int width, int height, int iDstStride, Pel*& rpiDst, int iMV, const int iShift)
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1;
  const int   iOffSet       = ( iShift > 0 ? 1 << ( iShift - 1 ) : 0 );
  const int16_t *const coeffs = m_lumaGradientFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      for ( size_t i = 0 ; i < 6 ; i++ )
      {
        JVET_J0090_CACHE_ACCESS( piSrcTmp + i, __FILE__, __LINE__ );
      }
      iSum     =   (  coeffs[0] * piSrcTmp[0]
                    + coeffs[1] * piSrcTmp[1]
                    + coeffs[2] * piSrcTmp[2]
                    + coeffs[3] * piSrcTmp[3]
                    + coeffs[4] * piSrcTmp[4]
                    + coeffs[5] * piSrcTmp[5] );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;

      piSrcTmp++;
    }

    piSrcTmp += iSrcStride;
    piDst    += iDstStride;
  }
}

inline void InterPrediction::fracFilter2DVer( const Pel* piSrc, int iSrcStride, int width, int height, int iDstStride, Pel*& rpiDst, int iMV, const int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1 * iSrcStride;
  const Pel*  piSrcTmp1     = piSrcTmp  + iSrcStride;
  const Pel*  piSrcTmp2     = piSrcTmp1 + iSrcStride;
  const Pel*  piSrcTmp3     = piSrcTmp2 + iSrcStride;
  const Pel*  piSrcTmp4     = piSrcTmp3 + iSrcStride;
  const Pel*  piSrcTmp5     = piSrcTmp4 + iSrcStride;
  const int   iOffSet       = ( iShift > 0 ? ( 1 << ( iShift - 1 ) ) - ( 8192 << iShift ) : -8192 );
  const int16_t *const coeffs = m_lumaInterpolationFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      JVET_J0090_CACHE_ACCESS( piSrcTmp,  __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp1, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp2, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp3, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp4, __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( piSrcTmp5, __FILE__, __LINE__ );
      iSum     =  (  coeffs[0] * *piSrcTmp ++
                   + coeffs[1] * *piSrcTmp1++
                   + coeffs[2] * *piSrcTmp2++
                   + coeffs[3] * *piSrcTmp3++
                   + coeffs[4] * *piSrcTmp4++
                   + coeffs[5] * *piSrcTmp5++ );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
    }

    piSrcTmp  += iSrcStride;
    piSrcTmp1 += iSrcStride;
    piSrcTmp2 += iSrcStride;
    piSrcTmp3 += iSrcStride;
    piSrcTmp4 += iSrcStride;
    piSrcTmp5 += iSrcStride;
    piDst     += iDstStride;
  }
}

inline void InterPrediction::fracFilter2DHor( const Pel* piSrc, int iSrcStride, int width, int height, int iDstStride, Pel*& rpiDst, int iMV, const int iShift )
{
  Pel*        piDst         = rpiDst;
  int         iSum          = 0;
  const Pel*  piSrcTmp      = piSrc - BIO_FILTER_HALF_LENGTH_MINUS_1;
  const int   iOffSet       = ( iShift > 0 ? 1 << ( iShift - 1 ) : 0 );
  const int16_t *const coeffs = m_lumaInterpolationFilter[iMV];

  iSrcStride               -= width;
  iDstStride               -= width;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      for ( size_t i = 0 ; i < 6 ; i++ )
      {
        JVET_J0090_CACHE_ACCESS( piSrcTmp + i, __FILE__, __LINE__ );
      }
      iSum     = (  coeffs[0] * piSrcTmp[0]
                  + coeffs[1] * piSrcTmp[1]
                  + coeffs[2] * piSrcTmp[2]
                  + coeffs[3] * piSrcTmp[3]
                  + coeffs[4] * piSrcTmp[4]
                  + coeffs[5] * piSrcTmp[5] );
      iSum     = ( iSum >= 0 ? ( iSum + iOffSet ) >> iShift : -( ( -iSum + iOffSet ) >> iShift ) );
      *piDst++ = ( Pel ) iSum;
      piSrcTmp++;
    }

    piSrcTmp += iSrcStride;
    piDst    += iDstStride;
  }
}

inline int GetMSB64( uint64_t x )
{
  int iMSB = 0, bits = (sizeof(int64_t) << 3);
  uint64_t y = 1;

  while (x > 1)
  {
    bits >>= 1;
    y = x >> bits;

    if (y)
    {
      x = y;
      iMSB += bits;
    }
  }

  iMSB += (int)y;

  return iMSB;
}
#endif

inline int64_t InterPrediction::divide64( int64_t numer, int64_t denom )
{
  int64_t d;
#if JVET_K0485_BIO
  int msbIdx = 0;
  for (msbIdx = 0; msbIdx<64; msbIdx++)
  {
    if (denom < ((int64_t)1 << msbIdx))
    {
      break;
    }
  }

  int shiftIdx = msbIdx - 1;
  d = (numer >> shiftIdx);
#else
  const int64_t iShiftA2 = 6;
  const int64_t iAccuracyShift = 15;
  const int64_t iMaxVal = 63;
  int64_t iScaleShiftA2 = 0;
  int64_t iScaleShiftA1 = 0;

  uint8_t signA1 = numer < 0;
  uint8_t signA2 = denom < 0;

  numer = (signA1) ? -numer : numer;
  denom = (signA2) ? -denom : denom;

  iScaleShiftA2 = GetMSB64(denom) - iShiftA2;
  iScaleShiftA1 = iScaleShiftA2 - 12;

  if (iScaleShiftA1 < 0)
  {
    iScaleShiftA1 = 0;
  }

  if (iScaleShiftA2 < 0)
  {
    iScaleShiftA2 = 0;
  }

  int64_t iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iScaleShiftA1;

  int64_t a2s = (denom >> iScaleShiftA2) > iMaxVal ? iMaxVal : (denom >> iScaleShiftA2);
  int64_t a1s = (numer >> iScaleShiftA1);

  int64_t aI64 = (a1s * (int64_t)m_uiaBIOShift[a2s]) >> iScaleShiftA;

  d = (signA1 + signA2 == 1) ? -aI64 : aI64;
#endif

  return d;
}

#if JVET_K0485_BIO
void InterPrediction::calcBlkGradient(int sx, int sy, int64_t *arraysGx2, int64_t *arraysGxGy, int64_t *arraysGxdI, int64_t *arraysGy2, int64_t *arraysGydI, int64_t &sGx2, int64_t &sGy2, int64_t &sGxGy, int64_t &sGxdI, int64_t &sGydI, int width, int height, int unitSize)
#else
void InterPrediction::calcBlkGradient( int sx, int sy, int64_t *arraysGx2, int64_t *arraysGxGy, int64_t *arraysGxdI, int64_t *arraysGy2, int64_t *arraysGydI, int64_t &sGx2, int64_t &sGy2, int64_t &sGxGy, int64_t &sGxdI, int64_t &sGydI, int iWidth, int iHeight )
#endif
{
#if !JVET_K0485_BIO
  static const uint32_t weightTbl[8][8] = { {1, 2, 3, 4, 4, 3, 2, 1},
      {2, 4, 6, 8, 8, 6, 4, 2},
      {3, 6, 9, 12, 12, 9, 6, 3},
      {4, 8, 12, 16, 16, 12, 8, 4},
      {4, 8, 12, 16, 16, 12, 8, 4},
      {3, 6, 9, 12, 12, 9, 6, 3},
      {2, 4, 6, 8, 8, 6, 4, 2},
      {1, 2, 3, 4, 4, 3, 2, 1 } };
#endif

  int64_t *pGx2 = arraysGx2;
  int64_t *pGy2 = arraysGy2;
  int64_t *pGxGy = arraysGxGy;
  int64_t *pGxdI = arraysGxdI;
  int64_t *pGydI = arraysGydI;

#if JVET_K0485_BIO
  // set to the above row due to JVET_K0485_BIO_EXTEND_SIZE
  pGx2  -= (JVET_K0485_BIO_EXTEND_SIZE*width);
  pGy2  -= (JVET_K0485_BIO_EXTEND_SIZE*width);
  pGxGy -= (JVET_K0485_BIO_EXTEND_SIZE*width);
  pGxdI -= (JVET_K0485_BIO_EXTEND_SIZE*width);
  pGydI -= (JVET_K0485_BIO_EXTEND_SIZE*width);

  for (int y = -JVET_K0485_BIO_EXTEND_SIZE; y < unitSize + JVET_K0485_BIO_EXTEND_SIZE; y++)
  {
    for (int x = -JVET_K0485_BIO_EXTEND_SIZE; x < unitSize + JVET_K0485_BIO_EXTEND_SIZE; x++)
    {
      sGx2 += pGx2[x];
      sGy2 += pGy2[x];
      sGxGy += pGxGy[x];
      sGxdI += pGxdI[x];
      sGydI += pGydI[x];
    }
    pGx2 += width;
    pGy2 += width;
    pGxGy += width;
    pGxdI += width;
    pGydI += width;
#else
  int x0;

  if (sy > 0 && iHeight > 4)
  {
    pGx2 -= (iWidth << 1);
    pGy2 -= (iWidth << 1);
    pGxGy -= (iWidth << 1);
    pGxdI -= (iWidth << 1);
    pGydI -= (iWidth << 1);
  }

  for (int y = -2; y < 6; y++)
  {
    for (int x = -2; x < 6; x++)
    {
      uint32_t weight = weightTbl[y + 2][x + 2];
      x0 = x;
      if (sx + x < 0)       x0 = 0;
      if (sx + x >= iWidth) x0 = 3;

      sGx2 += weight*pGx2[x0];
      sGy2 += weight*pGy2[x0];
      sGxGy += weight*pGxGy[x0];
      sGxdI += weight*pGxdI[x0];
      sGydI += weight*pGydI[x0];
    }

    if (sy + y < 0 || sy + y >= iHeight - 1)
    {
      continue;
    }
    pGx2  += iWidth;
    pGy2  += iWidth;
    pGxGy += iWidth;
    pGxdI += iWidth;
    pGydI += iWidth;
#endif
  }
}

static const int FRUC_MERGE_MV_SEARCHPATTERN_CROSS    = 0;
static const int FRUC_MERGE_MV_SEARCHPATTERN_SQUARE   = 1;
static const int FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND  = 2;
static const int FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON  = 3;

#if DISTORTION_TYPE_BUGFIX
Distortion InterPrediction::xFrucGetTempMatchCost(PredictionUnit &pu, int nWidth, int nHeight,
                                                  RefPicList eCurRefPicList, const MvField &rCurMvField,
                                                  Distortion uiMVCost)
#else
uint32_t InterPrediction::xFrucGetTempMatchCost( PredictionUnit& pu, int nWidth, int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, uint32_t uiMVCost )
#endif
{
  const int nMVUnit = 2;

#if DISTORTION_TYPE_BUGFIX
  Distortion uiCost = uiMVCost;
#else
  uint32_t uiCost = uiMVCost;
#endif

  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR = pu.cu->LICFlag;

  const SPS &sps = *pu.cs->sps;

  if( m_bFrucTemplateAvailabe[0] )
  {
    Mv mvTop( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvTop.setHighPrec();
    }
    mvTop += rCurMvField.mv;

    clipMv(mvTop, pu.cu->lumaPos(), sps);

    PelUnitBuf pcBufPredRefTop = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );
    PelUnitBuf pcBufPredCurTop = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );

    CHECK( rCurMvField.refIdx < 0, "invalid ref idx" );

    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, rCurMvField.refIdx), mvTop, pcBufPredRefTop, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);

    m_pcRdCost->setDistParam( cDistParam, pcBufPredCurTop.Y(), pcBufPredRefTop.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false );

    uiCost += cDistParam.distFunc( cDistParam ); //TODO: check distFunc
  }

  if( m_bFrucTemplateAvailabe[1] )
  {
    Mv mvLeft( - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvLeft.setHighPrec();
    }
    mvLeft += rCurMvField.mv;

    clipMv(mvLeft, pu.cu->lumaPos(), sps);

    PelUnitBuf pcBufPredRefLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );
    PelUnitBuf pcBufPredCurLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );

    CHECK( rCurMvField.refIdx < 0, "invalid ref idx" );

    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, rCurMvField.refIdx), mvLeft, pcBufPredRefLeft, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);

    m_pcRdCost->setDistParam( cDistParam, pcBufPredCurLeft.Y(), pcBufPredRefLeft.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false );

    uiCost += cDistParam.distFunc( cDistParam ); //TODO: check distFunc
  }

  return uiCost;
}

#if DISTORTION_TYPE_BUGFIX
Distortion InterPrediction::xFrucGetBilaMatchCost(PredictionUnit &pu, int nWidth, int nHeight,
                                                  RefPicList eCurRefPicList, const MvField &rCurMvField,
                                                  MvField &rPairMVField, Distortion uiMVCost)
#else
uint32_t InterPrediction::xFrucGetBilaMatchCost(PredictionUnit &pu, int nWidth, int nHeight, RefPicList eCurRefPicList,
                                            const MvField &rCurMvField, MvField &rPairMVField, uint32_t uiMVCost)
#endif
{
#if DISTORTION_TYPE_BUGFIX
  Distortion uiCost = std::numeric_limits<Distortion>::max();
#else
  uint32_t uiCost = MAX_UINT;
#endif

  if( PU::getMvPair( pu, eCurRefPicList , rCurMvField , rPairMVField ) )
  {
    RefPicList eTarRefPicList = ( RefPicList )( 1 - ( int )eCurRefPicList );
    PelUnitBuf pcBufPredA = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], nWidth, nHeight) ) );
    PelUnitBuf pcBufPredB = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], nWidth, nHeight) ) );

    CHECK( rCurMvField.refIdx < 0 || rPairMVField.refIdx < 0, "invalid ref idx" );

    const SPS &sps = *pu.cs->sps;

    Mv mvAp = rCurMvField.mv;
    clipMv(mvAp, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, rCurMvField.refIdx), mvAp, pcBufPredA, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_BILATERALMV, false);

    Mv mvBp = rPairMVField.mv;
    clipMv(mvBp, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eTarRefPicList, rPairMVField.refIdx), mvBp, pcBufPredB, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_BILATERALMV, false);

    DistParam cDistParam;
    cDistParam.applyWeight = false;
    cDistParam.useMR = pu.cu->LICFlag;
    m_pcRdCost->setDistParam( cDistParam, pcBufPredA.Y(), pcBufPredB.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false );
    uiCost = cDistParam.distFunc( cDistParam )  + uiMVCost; //TODO: check distFunc
  }

  return uiCost;
}

bool InterPrediction::xFrucIsInList( const MvField & rMvField , std::list<MvField> & rList )
{
  std::list<MvField>::iterator pos = rList.begin();
  while( pos != rList.end() )
  {
    if( rMvField == *pos )
      return( true );
    pos++;
  }
  return( false );
}

void InterPrediction::xFrucInsertMv2StartList( const MvField & rMvField , std::list<MvField> & rList, bool setHighPrec )
{
  CHECK( rMvField.refIdx < 0, "invalid ref idx" );

  MvField mf = rMvField;
  if( setHighPrec )
  {
    mf.mv.setHighPrec();
  }

  // do not use zoom in FRUC for now
  if( xFrucIsInList( mf , rList ) == false )
    rList.push_back( mf );
}

void InterPrediction::xFrucCollectBlkStartMv( PredictionUnit& pu, const MergeCtx& mergeCtx, RefPicList eTargetRefList, int nTargetRefIdx, AMVPInfo* pInfo )
{
  // add merge candidates to the list
  m_listMVFieldCand[0].clear();
  m_listMVFieldCand[1].clear();

  if ((nTargetRefIdx >= 0) && pInfo)   //Here we are in AMVP mode
  {
    // add AMVP candidates to the list
    for (int nAMVPIndex = 0; nAMVPIndex < pInfo->numCand; nAMVPIndex++)
    {
      MvField mvCnd;
      mvCnd.setMvField( pInfo->mvCand[nAMVPIndex], nTargetRefIdx );
      xFrucInsertMv2StartList( mvCnd, m_listMVFieldCand[eTargetRefList], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
    }
  }

  for( int nMergeIndex = 0; nMergeIndex < mergeCtx.numValidMergeCand << 1; nMergeIndex++ )
  {
    bool mrgTpDflt = ( pu.cs->sps->getSpsNext().getUseSubPuMvp() ) ? mergeCtx.mrgTypeNeighbours[nMergeIndex>>1] == MRG_TYPE_DEFAULT_N : true;
#if JVET_K0076_CPR
    if ((mergeCtx.interDirNeighbours[nMergeIndex >> 1] == 1 || mergeCtx.interDirNeighbours[nMergeIndex >> 1] == 3) && pu.cs->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[nMergeIndex].refIdx)->getPOC() == pu.cs->slice->getPOC())
      continue;
#endif
    if( mergeCtx.mvFieldNeighbours[nMergeIndex].refIdx >= 0 && mrgTpDflt )
    {
      if( nTargetRefIdx >= 0 && ( mergeCtx.mvFieldNeighbours[nMergeIndex].refIdx != nTargetRefIdx || ( nMergeIndex & 0x01 ) != ( int )eTargetRefList ) )
        continue;
      xFrucInsertMv2StartList( mergeCtx.mvFieldNeighbours[nMergeIndex] , m_listMVFieldCand[nMergeIndex&0x01], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
    }
  }

  // add uni-lateral candidates to the list
  if( pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
  {
    Position puPos    = pu.lumaPos();
    Size puSize       = pu.lumaSize();

    int halfPuHeight = ( ( puSize.height + 4 ) / 2 ) & ( ~3 );
    int halfPuWidth  = ( ( puSize.width  + 4 ) / 2 ) & ( ~3 );

    MvField mvCand;

    for( int y = puPos.y; y < puPos.y + puSize.height; y += halfPuHeight )
    {
      for( int x = puPos.x; x < puPos.x + puSize.width; x += halfPuWidth )
      {
        const MotionInfo &frucMi = pu.getMotionInfoFRUC( Position{ x, y } );

        for( int nList = 0 ; nList < 2 ; nList++ )
        {
          RefPicList eCurList = ( RefPicList )nList;
          if( frucMi.interDir & ( 1 << eCurList ) )
          {
            if( nTargetRefIdx >= 0 && ( frucMi.refIdx[eCurList] != nTargetRefIdx || eCurList != eTargetRefList ) )
              continue;
            mvCand.setMvField( frucMi.mv[eCurList], frucMi.refIdx[eCurList] );
            xFrucInsertMv2StartList( mvCand , m_listMVFieldCand[nList&0x01], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
          }
        }
      }
    }
  }

  // add some neighbors if not already present
  int nbSpatialCandTested = NB_FRUC_CAND_ADDED;

  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  for (int neighbor = 0; neighbor < nbSpatialCandTested; neighbor++)
  {
    if (neighbor == 0)       // top neighbor
    {
      neibPos = pu.lumaPos().offset( 0, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 1)  // left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, 0 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 2)  // top-left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 3)  // top-right neighbor
    {
      neibPos = pu.Y().topRight().offset( 1, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 4)  // below-left neighbor
    {
      neibPos = pu.Y().bottomLeft().offset( -1, 1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else
    {
      THROW("Invalid neighbor id!");
    }

    if (neibPU)
    {
      for (int nList = 0; nList < 2; nList++)
      {
        RefPicList eCurList = (RefPicList)nList;
        if( neibPU->getMotionInfo( neibPos ).interDir & ( 1 << eCurList ) )
        {
          if ( (nTargetRefIdx >= 0) && ((neibPU->getMotionInfo( neibPos ).refIdx[eCurList] != nTargetRefIdx) || (eCurList != eTargetRefList)) )
            continue;
          MvField mvCand;
          mvCand.setMvField( neibPU->getMotionInfo( neibPos ).mv[eCurList], neibPU->getMotionInfo( neibPos ).refIdx[eCurList] );
          xFrucInsertMv2StartList( mvCand , m_listMVFieldCand[nList&0x01], pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
      }
    }
  }
}

void InterPrediction::xFrucCollectSubBlkStartMv( PredictionUnit& pu, const MergeCtx& mergeCtx, RefPicList eRefPicList , const MvField& rMvStart , int nSubBlkWidth , int nSubBlkHeight, Position basePuPos )
{
  std::list<MvField> & rStartMvList = m_listMVFieldCand[eRefPicList];
  rStartMvList.clear();

  // start Mv
  xFrucInsertMv2StartList( rMvStart , rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );

  // add some neighbors if not already present

  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  for (int neighbor = 0; neighbor < std::max(2,NB_FRUC_CAND_ADDED_SUB); neighbor++)
  {
    if (neighbor == 0)       // top neighbor
    {
      neibPos = pu.lumaPos().offset( 0, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 1)  // left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, 0 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 2)  // top-left neighbor
    {
      neibPos = pu.lumaPos().offset( -1, -1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 3)  // top-right neighbor
    {
      neibPos = pu.Y().topRight().offset( 1, -1 );
      neibPU = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else if (neighbor == 4)  // below-left neighbor
    {
      neibPos = pu.Y().bottomLeft().offset( -1, 1 );
      neibPU  = pu.cs->getPURestricted( neibPos, pu, pu.chType );
    }
    else
    {
      THROW("Invalid neighbor id!");
    }

    if( neibPU && neibPU->getMotionInfo( neibPos ).interDir & ( 1 << eRefPicList ) && neibPU->getMotionInfo( neibPos ).refIdx[eRefPicList] == rMvStart.refIdx )
    {
      MvField mvCand;
      mvCand.setMvField( neibPU->getMotionInfo( neibPos ).mv[eRefPicList], neibPU->getMotionInfo( neibPos ).refIdx[eRefPicList] );
      xFrucInsertMv2StartList( mvCand , rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
    }
  }

  int nCurPOC    = pu.cs->slice->getPOC();
  int nCurRefPOC = pu.cs->slice->getRefPOC( eRefPicList, rMvStart.refIdx );

  // scaled TMVP, collocated positions and bottom right positions
  int nMaxPositions = 1;

  for( int n = 0; n < nMaxPositions; n++ )
  {
    for( int nRefIdx = pu.cs->slice->getNumRefIdx( eRefPicList ) - 1; nRefIdx >= 0; nRefIdx-- )
    {
      MvField mvCand;
      const Picture* pColPic  = pu.cs->slice->getRefPic( eRefPicList, nRefIdx );

      const unsigned scale = ( pu.cs->pcv->noMotComp ? 1 : 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4) );

      const unsigned mask  = ~( scale - 1 );

      int x_off = n == 0 ? 0 : nSubBlkWidth;
      int y_off = n == 0 ? 0 : nSubBlkHeight;

      Position _pos = Position{ pu.lumaPos().x + x_off, pu.lumaPos().y + y_off };

      const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

      const MotionInfo &colMi = pColPic->cs->getMotionInfo( pos );

      for( int nRefListColPic = 0; nRefListColPic < 2; nRefListColPic++ )
      {
        if( colMi.interDir & ( 1 << nRefListColPic ) ) // TODO: check if refIdx is always NOT_VALID, not 0 as set
        {
          CHECK( colMi.isInter == false, "invalid motion info" );
          Mv rColMv = colMi.mv[nRefListColPic];

          if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
          {
            rColMv.setHighPrec();
          }

          mvCand.refIdx = rMvStart.refIdx;
          mvCand.mv     = PU::scaleMv( rColMv , nCurPOC , nCurRefPOC , pColPic->getPOC(), pColPic->cs->slice->getRefPOC( ( RefPicList )nRefListColPic , colMi.refIdx[nRefListColPic] ), pu.cs->slice );
          if( mvCand.refIdx < 0 )
          {
            printf( "base" );
          }
          xFrucInsertMv2StartList( mvCand , rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
      }
    }
  }

  if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
  {
    //add supPu merge candidates
    Position subPos   = pu.lumaPos() - basePuPos;
#if JVET_K0346
    const Slice& slice = *pu.cs->slice;
    int numPartLine   = std::max(pu.lumaSize().width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    int numPartCol    = std::max(pu.lumaSize().height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    int puHeight      = numPartCol == 1 ? pu.lumaSize().height : 1 << slice.getSubPuMvpSubblkLog2Size();
    int puWidth       = numPartLine == 1 ? pu.lumaSize().width : 1 << slice.getSubPuMvpSubblkLog2Size();
#else
    int iNumPartLine  = std::max( pu.lumaSize().width  >> pu.cs->sps->getSpsNext().getSubPuMvpLog2Size(), 1u );
    int iNumPartCol   = std::max( pu.lumaSize().height >> pu.cs->sps->getSpsNext().getSubPuMvpLog2Size(), 1u );
    int iPUHeight     = iNumPartCol  == 1 ? pu.lumaSize().height : 1 << pu.cs->sps->getSpsNext().getSubPuMvpLog2Size();
    int iPUWidth      = iNumPartLine == 1 ? pu.lumaSize().width  : 1 << pu.cs->sps->getSpsNext().getSubPuMvpLog2Size();
#endif

#if JVET_K0346
    for (int y = subPos.y; y < subPos.y + pu.lumaSize().height; y += puHeight)
    {
      for (int x = subPos.x; x < subPos.x + pu.lumaSize().width; x += puWidth)
#else
    for( int y = subPos.y; y < subPos.y + pu.lumaSize().height; y += iPUHeight )
    {
      for( int x = subPos.x; x < subPos.x + pu.lumaSize().width; x += iPUWidth )
#endif
      {
        const MotionInfo subPuMi = mergeCtx.subPuMvpMiBuf.at( g_miScaling.scale( Position( x, y ) ) );
        if( rMvStart.refIdx == subPuMi.refIdx[eRefPicList] && subPuMi.interDir & ( 1 << eRefPicList ) )
        {
          MvField mvCand = MvField( subPuMi.mv[eRefPicList], subPuMi.refIdx[eRefPicList] );
          xFrucInsertMv2StartList( mvCand, rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
        const MotionInfo subPuExtMi = mergeCtx.subPuMvpExtMiBuf.at( g_miScaling.scale( Position( x, y ) ) );
        if( rMvStart.refIdx == subPuExtMi.refIdx[eRefPicList] && subPuExtMi.interDir & ( 1 << eRefPicList ) )
        {
          MvField mvCand = MvField( subPuExtMi.mv[eRefPicList], subPuExtMi.refIdx[eRefPicList] );
          xFrucInsertMv2StartList( mvCand, rStartMvList, pu.cs->sps->getSpsNext().getUseHighPrecMv() );
        }
      }
    }
  }
}

#if DISTORTION_TYPE_BUGFIX
Distortion InterPrediction::xFrucFindBestMvFromList(MvField *pBestMvField, RefPicList &rBestRefPicList,
                                                    PredictionUnit &pu, const MvField &rMvStart, int nBlkWidth,
                                                    int nBlkHeight, bool bTM, bool bMvCost)
#else
uint32_t InterPrediction::xFrucFindBestMvFromList( MvField* pBestMvField, RefPicList& rBestRefPicList, PredictionUnit& pu, const MvField& rMvStart, int nBlkWidth, int nBlkHeight, bool bTM, bool bMvCost )
#endif
{
#if DISTORTION_TYPE_BUGFIX
  Distortion uiMinCost = std::numeric_limits<Distortion>::max();
#else
  uint32_t uiMinCost = MAX_UINT;
#endif

  int nRefPicListStart = 0;
  int nRefPicListEnd = 1;
  if( bTM || bMvCost )  // Limit search to bestList in Template mode and for all sub-blocks (Template and Bilateral modes)
  {
    nRefPicListStart = nRefPicListEnd = rBestRefPicList;
  }
  for( int nRefPicList = nRefPicListStart ; nRefPicList <= nRefPicListEnd ; nRefPicList++ )
  {
    RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
    for( std::list<MvField>::iterator pos = m_listMVFieldCand[eCurRefPicList].begin() ; pos != m_listMVFieldCand[eCurRefPicList].end() ; pos++ )
    {
      MvField mvPair;

      if( !bTM && eCurRefPicList == REF_PIC_LIST_1 && !pu.cs->slice->getCheckLDC() )
      {
        // for normal B picture
        if( !PU::getMvPair( pu, REF_PIC_LIST_1 , *pos , mvPair ) || xFrucIsInList( mvPair , m_listMVFieldCand[0] ) )
          // no paired MV or the pair has been checked in list0
          continue;
      }

#if DISTORTION_TYPE_BUGFIX
      Distortion uiCost = 0;
#else
      uint32_t uiCost = 0;
#endif
      if( bMvCost )
      {
        uiCost = xFrucGetMvCost( rMvStart.mv , pos->mv, MAX_INT, FRUC_MERGE_REFINE_MVWEIGHT, pu.cs->sps->getSpsNext().getUseHighPrecMv() ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0 );
        if( uiCost > uiMinCost )
          continue;
      }

      if( bTM )
      {
        uiCost = xFrucGetTempMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, *pos, uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, *pos, mvPair, uiCost );
      }

      if( uiCost < uiMinCost )
      {
        uiMinCost = uiCost;
        pBestMvField[eCurRefPicList] = *pos;
        if( !bTM )
        {
          rBestRefPicList = eCurRefPicList;
          pBestMvField[!eCurRefPicList] = mvPair;
        }
      }
    }
  }

  return uiMinCost;
}

bool InterPrediction::deriveFRUCMV( PredictionUnit &pu )
{
  CHECK( !pu.mergeFlag, "merge mode must be used here" );

  bool bAvailable = false;

  MergeCtx mrgCtx;
  if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( pu.lumaSize() );
    mrgCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
    mrgCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
  }

  PU::getInterMergeCandidates( pu, mrgCtx );

  bAvailable = xFrucFindBlkMv( pu, mrgCtx );
  if( bAvailable )
  {
    xFrucRefineSubBlkMv( pu, mrgCtx, pu.frucMrgMode == FRUC_MERGE_TEMPLATE );
  }

  return bAvailable;
}

#if DISTORTION_TYPE_BUGFIX
Distortion InterPrediction::xFrucGetMvCost(const Mv &rMvStart, const Mv &rMvCur, int nSearchRange, int nWeighting,
                                           uint32_t precShift)
#else
uint32_t InterPrediction::xFrucGetMvCost( const Mv& rMvStart, const Mv& rMvCur, int nSearchRange, int nWeighting, uint32_t precShift )
#endif
{
  Mv mvDist = rMvStart - rMvCur;
#if DISTORTION_TYPE_BUGFIX
  Distortion uiCost = std::numeric_limits<Distortion>::max();
#else
  uint32_t uiCost = MAX_UINT;
#endif
  if( mvDist.getAbsHor() <= nSearchRange && mvDist.getAbsVer() <= nSearchRange )
  {
    uiCost = ( mvDist.getAbsHor() + mvDist.getAbsVer() ) * nWeighting;
    uiCost >>= precShift;
  }

  return uiCost;
}

#if DISTORTION_TYPE_BUGFIX
Distortion InterPrediction::xFrucRefineMv(MvField *pBestMvField, RefPicList eCurRefPicList, Distortion uiMinCost,
                                          int nSearchMethod, PredictionUnit &pu, const MvField &rMvStart, int nBlkWidth,
                                          int nBlkHeight, bool bTM, bool bMvCostZero)
#else
uint32_t InterPrediction::xFrucRefineMv( MvField* pBestMvField, RefPicList eCurRefPicList, uint32_t uiMinCost, int nSearchMethod, PredictionUnit& pu, const MvField& rMvStart, int nBlkWidth, int nBlkHeight, bool bTM, bool bMvCostZero )
#endif
{
  int nSearchStepShift = 0;
  if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
  {
    nSearchStepShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  switch( nSearchMethod )
  {
    case 0:
      // cross
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1 );
      }
      break;
    case 1:
      // square
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_SQUARE>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
      }
      break;
    case 2:
      // diamond
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, 1, bMvCostZero );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
      }
      break;
    case 3:
      // no refinement
      break;
    case 4:
      // hexagon
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, 1, bMvCostZero );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
      }
      break;
    case 5:
      // adaptive cross
      if( rMvStart.refIdx != pBestMvField[eCurRefPicList].refIdx || rMvStart.mv != pBestMvField[eCurRefPicList].mv )
      {
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift, MAX_UINT, bMvCostZero );
        if( nSearchStepShift > 0 )
        {
          uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField, eCurRefPicList, pu, rMvStart, nBlkWidth, nBlkHeight, uiMinCost, bTM, nSearchStepShift-1, 1, bMvCostZero );
        }
      }
      break;
    default:
      THROW( "Invalid search method" );
  }

  return uiMinCost;
}

#if DISTORTION_TYPE_BUGFIX
template<int SearchPattern>
Distortion InterPrediction::xFrucRefineMvSearch(MvField *pBestMvField, RefPicList eCurRefPicList, PredictionUnit &pu,
                                                const MvField &rMvStart, int nBlkWidth, int nBlkHeight,
                                                Distortion uiMinDist, bool bTM, int nSearchStepShift,
                                                uint32_t uiMaxSearchRounds, bool bMvCostZero)
#else
template<int SearchPattern>
uint32_t InterPrediction::xFrucRefineMvSearch ( MvField* pBestMvField, RefPicList eCurRefPicList, PredictionUnit& pu, const MvField& rMvStart, int nBlkWidth, int nBlkHeight, uint32_t uiMinDist, bool bTM, int nSearchStepShift, uint32_t uiMaxSearchRounds, bool bMvCostZero )
#endif
{
  const Mv mvSearchOffsetCross  [4] = { Mv(  0 , 1 ) , Mv( 1 , 0 ) , Mv(  0 , -1 ) , Mv( -1 ,  0 ) };
  const Mv mvSearchOffsetSquare [8] = { Mv( -1 , 1 ) , Mv( 0 , 1 ) , Mv(  1 ,  1 ) , Mv(  1 ,  0 ) , Mv(  1 , -1 ) , Mv(  0 , -1 ) , Mv( -1 , -1 ) , Mv( -1 , 0 )  };
  const Mv mvSearchOffsetDiamond[8] = { Mv(  0 , 2 ) , Mv( 1 , 1 ) , Mv(  2 ,  0 ) , Mv(  1 , -1 ) , Mv(  0 , -2 ) , Mv( -1 , -1 ) , Mv( -2 ,  0 ) , Mv( -1 , 1 ) };
  const Mv mvSearchOffsetHexagon[6] = { Mv(  2 , 0 ) , Mv( 1 , 2 ) , Mv( -1 ,  2 ) , Mv( -2 ,  0 ) , Mv( -1 , -2 ) , Mv(  1 , -2 ) };

  int nDirectStart = 0 , nDirectEnd = 0 , nDirectRounding = 0 , nDirectMask = 0;
  const Mv * pSearchOffset;
  if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_CROSS )
  {
    nDirectEnd = 3;
    nDirectRounding = 4;
    nDirectMask = 0x03;
    pSearchOffset = mvSearchOffsetCross;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_SQUARE )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetSquare;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetDiamond;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
  {
    nDirectEnd = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    THROW( "Invalid search pattern" );
  }

  int nBestDirect;
  int rSearchRange = pu.cs->sps->getSpsNext().getFRUCRefineRange();
  if( !pu.cs->sps->getSpsNext().getUseHighPrecMv() )
  {
    rSearchRange >>= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }
  for( uint32_t uiRound = 0 ; uiRound < uiMaxSearchRounds ; uiRound++ )
  {
    nBestDirect = -1;
    MvField mvCurCenter = pBestMvField[eCurRefPicList];
    for( int nIdx = nDirectStart ; nIdx <= nDirectEnd ; nIdx++ )
    {
      int nDirect;
      if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        nDirect = ( nIdx + nDirectRounding ) & nDirectMask;
      }

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      MvField mvCand = mvCurCenter;
      if( mvCand.mv.highPrec && !mvOffset.highPrec )
      {
        mvOffset.highPrec = true;
      }
      mvCand.mv += mvOffset;
#if DISTORTION_TYPE_BUGFIX
      Distortion uiCost = (Distortion) xFrucGetMvCost(
        rMvStart.mv, mvCand.mv, rSearchRange, FRUC_MERGE_REFINE_MVWEIGHT,
        pu.cs->sps->getSpsNext().getUseHighPrecMv() ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0);
      if (bMvCostZero && uiCost != std::numeric_limits<Distortion>::max())
#else
      uint32_t uiCost = xFrucGetMvCost( rMvStart.mv, mvCand.mv, rSearchRange, FRUC_MERGE_REFINE_MVWEIGHT, pu.cs->sps->getSpsNext().getUseHighPrecMv() ? VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE : 0 );
      if (bMvCostZero && uiCost != MAX_UINT)
#endif
      {
        uiCost = 0;
      }
      if( uiCost > uiMinDist )
        continue;
      MvField mvPair;

      if( bTM )
      {
        if( PU::isSameMVField( pu, eCurRefPicList, mvCand, ( RefPicList )( !eCurRefPicList ), pBestMvField[!eCurRefPicList] ) )
          continue;
        uiCost = xFrucGetTempMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, mvCand, uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pu, nBlkWidth, nBlkHeight, eCurRefPicList, mvCand, mvPair, uiCost );
      }
      if( uiCost < uiMinDist )
      {
        uiMinDist = uiCost;
        pBestMvField[eCurRefPicList] = mvCand;
        if( !bTM )
          pBestMvField[!eCurRefPicList] = mvPair;
        nBestDirect = nDirect;
      }
    }

    if( nBestDirect == -1 )
      break;
    int nStep = 1;
    if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_SQUARE || SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
    {
      nStep = 2 - ( nBestDirect & 0x01 );
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  return uiMinDist;
}

bool InterPrediction::frucFindBlkMv4Pred( PredictionUnit& pu, RefPicList eTargetRefPicList, const int nTargetRefIdx, AMVPInfo* pInfo )
{
  MergeCtx mrgCtx;
  if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( pu.lumaSize() );
    mrgCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
    mrgCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
  }

  PU::getInterMergeCandidates( pu, mrgCtx );

  bool bAvailable = false;
  if( pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
  {
    int nWidth  = pu.lumaSize().width;
    int nHeight = pu.lumaSize().height;
    if( xFrucGetCurBlkTemplate( pu, nWidth, nHeight ) )
    {
      // find best start
      xFrucCollectBlkStartMv( pu, mrgCtx, eTargetRefPicList, nTargetRefIdx, pInfo );
      MvField mvStart[2] , mvFinal[2];
#if DISTORTION_TYPE_BUGFIX
      Distortion uiMinCost = xFrucFindBestMvFromList(mvStart, eTargetRefPicList, pu, mvStart[eTargetRefPicList], nWidth,
                                                     nHeight, true, false);
#else
      uint32_t uiMinCost = xFrucFindBestMvFromList( mvStart, eTargetRefPicList, pu, mvStart[eTargetRefPicList], nWidth, nHeight, true, false );
#endif
      if( mvStart[eTargetRefPicList].refIdx >= 0 )
      {
        // refine Mv
        mvFinal[eTargetRefPicList] = mvStart[eTargetRefPicList];
        xFrucRefineMv( mvFinal, eTargetRefPicList, uiMinCost, 2, pu, mvStart[eTargetRefPicList], nWidth, nHeight, true );
        bAvailable = true;
        // save Mv
        pu.mv[eTargetRefPicList] = mvFinal[eTargetRefPicList].mv;
      }
    }
  }

  return bAvailable;
}

bool InterPrediction::xFrucFindBlkMv( PredictionUnit& pu, const MergeCtx& mergeCtx )
{
  bool bAvailable = false;
  int nWidth  = pu.lumaSize().width;
  int nHeight = pu.lumaSize().height;
  MvField mvStart[2] , mvFinal[2];

  const int nSearchMethod = 2;
  if( pu.frucMrgMode == FRUC_MERGE_BILATERALMV )
  {
    if( !pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() || pu.cs->slice->isInterP() )
      return( false );

    xFrucCollectBlkStartMv( pu, mergeCtx );

    RefPicList eBestRefPicList = REF_PIC_LIST_0;
#if DISTORTION_TYPE_BUGFIX
    Distortion uiMinCost =
      xFrucFindBestMvFromList(mvStart, eBestRefPicList, pu, mvStart[eBestRefPicList], nWidth, nHeight, false, false);
#else
    uint32_t uiMinCost = xFrucFindBestMvFromList( mvStart, eBestRefPicList, pu, mvStart[eBestRefPicList], nWidth, nHeight, false, false );
#endif

    if( mvStart[eBestRefPicList].refIdx >= 0 )
    {
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];
      uiMinCost = xFrucRefineMv( mvFinal, eBestRefPicList, uiMinCost, nSearchMethod, pu, mvStart[eBestRefPicList], nWidth, nHeight, false );
      //Save best list for sub-blocks
      m_bilatBestRefPicList = eBestRefPicList;
      bAvailable = true;
    }
  }
  else if( pu.frucMrgMode == FRUC_MERGE_TEMPLATE )
  {
    if( !pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
      return( false );
    if( !xFrucGetCurBlkTemplate( pu, nWidth, nHeight ) )
      return( false );

    xFrucCollectBlkStartMv( pu, mergeCtx );

#if DISTORTION_TYPE_BUGFIX
    Distortion uiMinCost[2];
#else
    uint32_t uiMinCost[2];
#endif
    // find the best Mvs from the two lists first and then refine Mvs: try to avoid duplicated Mvs
    for( int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
      uiMinCost[nRefPicList] = xFrucFindBestMvFromList( mvStart, eCurRefPicList, pu, mvStart[nRefPicList], nWidth, nHeight, true, false );
    }
    mvFinal[0] = mvStart[0];
    mvFinal[1] = mvStart[1];
    for( int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      if( mvStart[nRefPicList].refIdx >= 0 )
      {
        uiMinCost[nRefPicList] = xFrucRefineMv( mvFinal, ( RefPicList )nRefPicList, uiMinCost[nRefPicList], nSearchMethod, pu, mvStart[nRefPicList], nWidth, nHeight, true, true );
        bAvailable = true;
      }
    }

    if (mvFinal[0].refIdx >= 0 && mvFinal[1].refIdx >= 0)
    {
      //calculate cost for bi-refinement
      xFrucUpdateTemplate( pu, nWidth, nHeight, REF_PIC_LIST_0, mvFinal[REF_PIC_LIST_0] );
#if DISTORTION_TYPE_BUGFIX
      Distortion uiCostBi = xFrucGetTempMatchCost(pu, nWidth, nHeight, REF_PIC_LIST_1, mvFinal[REF_PIC_LIST_1], 0);
#else
      uint32_t uiCostBi = xFrucGetTempMatchCost( pu, nWidth, nHeight, REF_PIC_LIST_1, mvFinal[REF_PIC_LIST_1], 0 );
#endif

      if (2 * uiCostBi <= 5 * std::min(uiMinCost[0], uiMinCost[1]) )  // if (uiMinCostBi <= 5/4*2*min(uiMinCost[0], uiMinCost[1]) )
      {
        //do nothing
      }
      else if (uiMinCost[0] <= uiMinCost[1])
      {
        mvFinal[1].refIdx = -1;
        mvFinal[1].mv.set( 0, 0 );
      }
      else
      {
        mvFinal[0].refIdx = -1;
        mvFinal[0].mv.set( 0, 0 );
      }
    }
  }
  else
  {
    THROW( "Invalid mode" );
  }

  if( bAvailable )
  {
    // save Mv
    pu.mv    [REF_PIC_LIST_0] = mvFinal[0].mv;
    pu.refIdx[REF_PIC_LIST_0] = mvFinal[0].refIdx;
    pu.mv    [REF_PIC_LIST_1] = mvFinal[1].mv;
    pu.refIdx[REF_PIC_LIST_1] = mvFinal[1].refIdx;
    pu.interDir = ( mvFinal[0].refIdx >= 0 ) + ( ( mvFinal[1].refIdx >=0 ) << 1 );

    CHECK( pu.interDir < 1 || pu.interDir > 3, "invalid inter dir " );

    PU::spanMotionInfo( pu );
  }

  return bAvailable;
}

bool InterPrediction::xFrucRefineSubBlkMv( PredictionUnit& pu, const MergeCtx &mergeCtx, bool bTM )
{
  int nRefineBlockSize = xFrucGetSubBlkSize( pu, pu.lumaSize().width, pu.lumaSize().height );

  const int nSearchMethod = 5;

  Position puPos  = pu.lumaPos();
  PredictionUnit subPu;
  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeFlag = pu.mergeFlag;
  subPu.mergeType = pu.mergeType;
  subPu.idx       = pu.idx;

  for( int y = puPos.y; y < puPos.y + pu.lumaSize().height; y += nRefineBlockSize )
  {
    for( int x = puPos.x; x < puPos.x + pu.lumaSize().width; x += nRefineBlockSize )
    {
      MotionInfo mi = pu.getMotionInfo( Position{ x, y } );

      // start from the best Mv of the full block
      MvField mvStart[2] , mvFinal[2];
      mvStart[0].setMvField( mi.mv[REF_PIC_LIST_0] , mi.refIdx[REF_PIC_LIST_0] );
      mvStart[1].setMvField( mi.mv[REF_PIC_LIST_1] , mi.refIdx[REF_PIC_LIST_1] );
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];

      int dx = nRefineBlockSize;
      int dy = nRefineBlockSize;

      subPu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( x, y, dx, dy ) ) );
      subPu = mi;

      // refinement
      if( bTM )
      {
        if( !xFrucGetCurBlkTemplate( subPu, nRefineBlockSize, nRefineBlockSize ) ) // TODO: sub pu position!!!
          continue;
        for( int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
        {
          if( mvStart[nRefPicList].refIdx >= 0 )
          {
            RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
            xFrucCollectSubBlkStartMv( subPu, mergeCtx, eCurRefPicList, mvStart[eCurRefPicList], nRefineBlockSize, nRefineBlockSize, puPos );
#if DISTORTION_TYPE_BUGFIX
            Distortion uiMinCost = xFrucFindBestMvFromList(mvFinal, eCurRefPicList, subPu, mvStart[eCurRefPicList],
                                                           nRefineBlockSize, nRefineBlockSize, bTM, true);
#else
            uint32_t uiMinCost = xFrucFindBestMvFromList( mvFinal, eCurRefPicList, subPu, mvStart[eCurRefPicList], nRefineBlockSize, nRefineBlockSize, bTM, true );
#endif
            uiMinCost = xFrucRefineMv( mvFinal, eCurRefPicList, uiMinCost, nSearchMethod, subPu, mvStart[eCurRefPicList], nRefineBlockSize, nRefineBlockSize, bTM );
          }
        }
      }
      else
      {
        //Use same reference frame as for entire block, i.e. same refidx and same list
        RefPicList eBestRefPicList = m_bilatBestRefPicList;
        xFrucCollectSubBlkStartMv( subPu, mergeCtx, eBestRefPicList, mvStart[eBestRefPicList], nRefineBlockSize, nRefineBlockSize, puPos );

#if DISTORTION_TYPE_BUGFIX
        Distortion uiMinCost = xFrucFindBestMvFromList(mvFinal, eBestRefPicList, subPu, mvStart[eBestRefPicList],
                                                       nRefineBlockSize, nRefineBlockSize, bTM, true);
#else
        uint32_t uiMinCost = xFrucFindBestMvFromList( mvFinal, eBestRefPicList, subPu, mvStart[eBestRefPicList], nRefineBlockSize, nRefineBlockSize, bTM, true );
#endif
        uiMinCost = xFrucRefineMv( mvFinal, eBestRefPicList, uiMinCost, nSearchMethod, subPu, mvStart[eBestRefPicList], nRefineBlockSize, nRefineBlockSize, bTM );
      }

      // save Mv
      if( !( mvFinal[0] == mvStart[0] && mvFinal[1] == mvStart[1] ) )
      {
        subPu.mv    [REF_PIC_LIST_0] = mvFinal[0].mv;
        subPu.refIdx[REF_PIC_LIST_0] = mvFinal[0].refIdx;
        subPu.mv    [REF_PIC_LIST_1] = mvFinal[1].mv;
        subPu.refIdx[REF_PIC_LIST_1] = mvFinal[1].refIdx;
        subPu.interDir = ( mvFinal[0].refIdx >= 0 ) + ( ( mvFinal[1].refIdx >=0 ) << 1 );
        subPu.mergeType = MRG_TYPE_DEFAULT_N; // only a hack, TODO: do it better
        CHECK( subPu.interDir < 1 || subPu.interDir > 3, "invalid inter dir" );
        PU::spanMotionInfo( subPu );
      }
    }
  }

  return true;
}

int InterPrediction::xFrucGetSubBlkSize( PredictionUnit& pu, int nBlkWidth, int nBlkHeight )
{
  int avgLength = pu.cs->pcv->rectCUs ? 1 << ( ( ( ( int ) log2( pu.cu->lumaSize().width ) + ( int ) log2( pu.cu->lumaSize().height ) - 3 ) >> 1 ) + MIN_CU_LOG2 ) : pu.cu->lumaSize().width;
  int nRefineBlkSize = std::max( avgLength >> pu.cs->slice->getSPS()->getSpsNext().getFRUCSmallBlkRefineDepth(), FRUC_MERGE_REFINE_MINBLKSIZE );

  while( true )
  {
    int nMask = nRefineBlkSize - 1;
    if( nRefineBlkSize > std::min( nBlkWidth , nBlkHeight ) || ( nBlkWidth & nMask ) || ( nBlkHeight & nMask ) )
      nRefineBlkSize >>= 1;
    else
      break;
  }
  CHECK( nRefineBlkSize < FRUC_MERGE_REFINE_MINBLKSIZE, "invalid refine block size" );
  return( nRefineBlkSize );
}

bool InterPrediction::xFrucGetCurBlkTemplate( PredictionUnit& pu, int nCurBlkWidth, int nCurBlkHeight )
{
  m_bFrucTemplateAvailabe[0] = xFrucIsTopTempAvailable( pu );
  m_bFrucTemplateAvailabe[1] = xFrucIsLeftTempAvailable( pu );

  if( !m_bFrucTemplateAvailabe[0] && !m_bFrucTemplateAvailabe[1] )
    return false;

  const int nMVUnit = 2;

  if( m_bFrucTemplateAvailabe[0] )
  {
    Mv mvTop( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    PelUnitBuf pcMbBuf = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[0][0], nCurBlkWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvTop.setHighPrec();
    }
    xPredInterBlk(COMPONENT_Y, pu, pu.cs->slice->getPic(), mvTop, pcMbBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
  }

  if( m_bFrucTemplateAvailabe[1] )
  {
    Mv mvLeft( - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    PelUnitBuf pcMbBuf = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[1][0], FRUC_MERGE_TEMPLATE_SIZE, nCurBlkHeight) ) );

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvLeft.setHighPrec();
    }
    xPredInterBlk(COMPONENT_Y, pu, pu.cs->slice->getPic(), mvLeft, pcMbBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
  }

  return true;
}

bool InterPrediction::xFrucIsTopTempAvailable( PredictionUnit& pu )
{
  const CodingStructure &cs     = *pu.cs;
        Position posRT          = pu.Y().topRight();
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );

  return ( puAbove && pu.cu != puAbove->cu );
}

bool InterPrediction::xFrucIsLeftTempAvailable( PredictionUnit& pu )
{
  const CodingStructure &cs    = *pu.cs;
        Position posLB         = pu.Y().bottomLeft();
  const PredictionUnit *puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );

  return ( puLeft && pu.cu !=puLeft->cu );
}

void InterPrediction::xFrucUpdateTemplate( PredictionUnit& pu, int nWidth, int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField )
{
  const int nMVUnit = 2;

  const MvField *pMvFieldOther = &rCurMvField;
  PelUnitBuf pcBufPredRefTop  = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );
  PelUnitBuf pcBufPredRefLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );

  PelUnitBuf pcBufPredCurTop  = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[0][0], nWidth, FRUC_MERGE_TEMPLATE_SIZE) ) );
  PelUnitBuf pcBufPredCurLeft = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPredFrucTemplate[1][0], FRUC_MERGE_TEMPLATE_SIZE, nHeight) ) );

  const SPS &sps = *pu.cs->sps;

  if( m_bFrucTemplateAvailabe[0] )
  {
    Mv mvOther( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    mvOther += pMvFieldOther->mv;

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvOther.setHighPrec();
    }

    clipMv(mvOther, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, pMvFieldOther->refIdx), mvOther, pcBufPredRefTop, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
    pcBufPredCurTop.removeHighFreq( pcBufPredRefTop, false, pu.cs->slice->clpRngs() );
  }

  if (m_bFrucTemplateAvailabe[1])
  {
    Mv mvOther( -( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ), 0 );
    mvOther += pMvFieldOther->mv;

    if( pu.cs->sps->getSpsNext().getUseHighPrecMv() )
    {
      mvOther.setHighPrec();
    }

    clipMv(mvOther, pu.cu->lumaPos(), sps);
    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(eCurRefPicList, pMvFieldOther->refIdx), mvOther, pcBufPredRefLeft, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false, FRUC_MERGE_TEMPLATE, false);
    pcBufPredCurLeft.removeHighFreq( pcBufPredRefLeft, false, pu.cs->slice->clpRngs() );
  }
}
#if !DMVR_JVET_K0217
void InterPrediction::xPredInterLines( const PredictionUnit& pu, const Picture* refPic, Mv &_mv, PelUnitBuf &dstPic, const bool &bi, const ClpRng& clpRng )
{
  clipMv( _mv, pu.lumaPos(), *pu.cs->sps );

  const ChromaFormat chFmt  = pu.chromaFormat;
  const ComponentID compID  = COMPONENT_Y;

  int iAddPrecShift = 0;

  if( _mv.highPrec )
  {
    CHECK( !pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!" );

    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX( compID, chFmt );
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY( compID, chFmt );

  Position offset      = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
  const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  PelBuf &dstBuf = dstPic.bufs[compID];

  int xFrac = _mv.hor & ( ( 1 << shiftHor ) - 1 );
  int yFrac = _mv.ver & ( ( 1 << shiftVer ) - 1 );

  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;

  CHECK( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( xFrac & 3 ) != 0 ), "Invalid fraction" );
  CHECK( !pu.cs->sps->getSpsNext().getUseHighPrecMv() && ( ( yFrac & 3 ) != 0 ), "Invalid fraction" );

  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  if( yFrac == 0 )
  {
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, !bi, chFmt, clpRng, 0);
  }
  else if( xFrac == 0 )
  {
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, !bi, chFmt, clpRng, 0);
  }
  else
  {
    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], Size(width,height));

    int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,      chFmt, clpRng, 0);
    JVET_J0090_SET_CACHE_ENABLE( false );
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, !bi, chFmt, clpRng, 0);
    JVET_J0090_SET_CACHE_ENABLE( true );
  }
}

void InterPrediction::xFillPredBlckAndBorder( const PredictionUnit& pu, RefPicList eRefPicList, int iWidth, int iHeight, PelBuf &cTmpY )
{
  int iRefIdx = pu.refIdx[eRefPicList];
  Mv  mvOrg   = pu.mv[eRefPicList];
  Mv  mv;

  const Picture* refPic = pu.cu->slice->getRefPic(eRefPicList, iRefIdx);

  const int nMVUnit = 2;

  int dstStride = MAX_CU_SIZE + DMVR_INTME_RANGE*2;

  JVET_J0090_SET_REF_PICTURE( refPic, COMPONENT_Y );
  PelUnitBuf cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0], dstStride, iWidth + 2 * DMVR_INTME_RANGE, iHeight + 2 * DMVR_INTME_RANGE ) ) );
  cPred.Y().subBuf( Position{ DMVR_INTME_RANGE, DMVR_INTME_RANGE }, pu.lumaSize() ).copyFrom( cTmpY.subBuf( Position{ 0, 0 }, pu.lumaSize() ) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0], dstStride, iWidth + 2 * DMVR_INTME_RANGE, DMVR_INTME_RANGE ) ) );
  //top row
  mv = mvOrg + Mv( -(DMVR_INTME_RANGE << nMVUnit), -(DMVR_INTME_RANGE << nMVUnit) );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0] + dstStride*DMVR_INTME_RANGE, MAX_CU_SIZE + DMVR_INTME_RANGE*2, DMVR_INTME_RANGE, iHeight ) ) );
  //left column
  mv = mvOrg + Mv( -(DMVR_INTME_RANGE << nMVUnit), 0 );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0] + dstStride*DMVR_INTME_RANGE + iWidth + DMVR_INTME_RANGE, MAX_CU_SIZE + DMVR_INTME_RANGE*2, DMVR_INTME_RANGE, iHeight ) ) );
  //right column
  mv = mvOrg + Mv( (iWidth << nMVUnit), 0 );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );

  cPred = ( PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVR[0] + dstStride*(iHeight + DMVR_INTME_RANGE), MAX_CU_SIZE + DMVR_INTME_RANGE*2, iWidth + 2 * DMVR_INTME_RANGE, DMVR_INTME_RANGE ) ) );
  //bottom row
  mv = mvOrg + Mv( -(DMVR_INTME_RANGE << nMVUnit), (iHeight << nMVUnit) );
  xPredInterLines( pu, refPic, mv, cPred, false, pu.cs->slice->clpRng(COMPONENT_Y) );
}

#if DISTORTION_TYPE_BUGFIX
Distortion InterPrediction::xDirectMCCost(int iBitDepth, Pel *pRef, uint32_t uiRefStride, const Pel *pOrg, uint32_t uiOrgStride,
                                          int iWidth, int iHeight)
#else
uint32_t InterPrediction::xDirectMCCost( int iBitDepth, Pel* pRef, uint32_t uiRefStride, const Pel* pOrg, uint32_t uiOrgStride, int iWidth, int iHeight )
#endif
{
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR = false;

  m_pcRdCost->setDistParam( cDistParam, pOrg, pRef, uiOrgStride, uiRefStride, iBitDepth, COMPONENT_Y, iWidth, iHeight );

#if DISTORTION_TYPE_BUGFIX
  Distortion uiCost = cDistParam.distFunc(cDistParam);
#else
  uint32_t uiCost = cDistParam.distFunc( cDistParam );
#endif

  return uiCost;
}

#if DISTORTION_TYPE_BUGFIX
void InterPrediction::xBIPMVRefine(PredictionUnit &pu, RefPicList eRefPicList, int iWidth, int iHeight,
                                   const CPelUnitBuf &pcYuvOrg, uint32_t uiMaxSearchRounds, uint32_t nSearchStepShift,
                                   Distortion &uiMinCost, bool fullPel /*= true*/)
#else
void InterPrediction::xBIPMVRefine( PredictionUnit& pu, RefPicList eRefPicList, int iWidth, int iHeight, const CPelUnitBuf &pcYuvOrg, uint32_t uiMaxSearchRounds, uint32_t nSearchStepShift, uint32_t& uiMinCost, bool fullPel /*= true*/ )
#endif
{
  const Mv mvSearchOffsetSquare[8] = { Mv(-1 , 1) , Mv(0 , 1) , Mv(1 , 1) , Mv(1 , 0) , Mv(1 , -1) , Mv(0 , -1) , Mv(-1 , -1) , Mv(-1 , 0) };

  int nDirectStart = 0, nDirectEnd = 0, nDirectRounding = 0, nDirectMask = 0;
  const Mv * pSearchOffset;

  nDirectEnd = 7;
  nDirectRounding = 8;
  nDirectMask = 0x07;
  pSearchOffset = mvSearchOffsetSquare;

  Mv cMvOrg = pu.mv[eRefPicList];
  Mv cBestMv = cMvOrg;

  int nBestDirect;

  for (uint32_t uiRound = 0; uiRound < uiMaxSearchRounds; uiRound++)
  {
    nBestDirect = -1;
    Mv cMvCtr = cBestMv;

    for (int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      int nDirect = (nIdx + nDirectRounding) & nDirectMask;

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;

      if( pu.cu->slice->getSPS()->getSpsNext().getUseHighPrecMv() )
      {
        CHECK( !cMvOrg.highPrec, "wrong" );
        mvOffset.highPrec = true;
      }

      Mv cMvTemp = cMvCtr;
      cMvTemp += mvOffset;

#if DISTORTION_TYPE_BUGFIX
      Distortion uiCost;
#else
      uint32_t uiCost;
#endif

      if ( fullPel )
      {
        Mv cMvD = cMvTemp;
        cMvD -= cMvOrg;
        cMvD >>= nSearchStepShift;

        CHECK( cMvD.getAbsHor() > DMVR_INTME_RANGE || cMvD.getAbsVer() > DMVR_INTME_RANGE, "wrong");

        int iRefStride = MAX_CU_SIZE + DMVR_INTME_RANGE*2;

        Pel* pRef = m_cYuvPredTempDMVR[0] + (DMVR_INTME_RANGE + cMvD.getVer()) * iRefStride + DMVR_INTME_RANGE + cMvD.getHor();
        uiCost = xDirectMCCost( pu.cs->sps->getBitDepth(toChannelType(COMPONENT_Y)), pRef, iRefStride, (Pel*) pcYuvOrg.Y().buf, pcYuvOrg.bufs[0].stride, iWidth, iHeight );
      }
      else
      {
        int iRefStride = MAX_CU_SIZE + DMVR_INTME_RANGE*2 + 16;
        Pel* pRef = m_filteredBlock[pSearchOffset[nDirect].getAbsHor()][pSearchOffset[nDirect].getAbsVer()][0]; //TODO: to be checked if correct

        if (pSearchOffset[nDirect].getHor() == 1)
        {
          pRef++;
        }
        if (pSearchOffset[nDirect].getVer() == 1)
        {
          pRef += iRefStride;
        }
        uiCost = xDirectMCCost( pu.cs->sps->getBitDepth(toChannelType(COMPONENT_Y)), pRef, iRefStride, (Pel*) pcYuvOrg.Y().buf, pcYuvOrg.bufs[0].stride, iWidth, iHeight );
      }

      if (uiCost < uiMinCost)
      {
        uiMinCost = uiCost;
        nBestDirect = nDirect;
        cBestMv = cMvTemp;
      }
    }

    if (nBestDirect == -1)
      break;
    int nStep = 2 - (nBestDirect & 0x01);

    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  pu.mv[eRefPicList] = cBestMv;
}
#else
MRSADtype InterPrediction::xDirectMCCostDMVR(const Pel* pSrcL0, const Pel* pSrcL1, uint32_t stride, SizeType width, SizeType height, const DistParam &cDistParam)
{
  MRSADtype sum = 0;
  const int16_t deltaC = (int16_t)round((double)(cDistParam.meanL0 - cDistParam.meanL1) / (width * height));

  for (SizeType j = 0; j < height; j++)
  {
    for (SizeType i = 0; i < width; i++)
    {
      sum += abs(pSrcL0[i] - pSrcL1[i] - deltaC);
    }
    pSrcL0 += stride;
    pSrcL1 += stride;
  }
  return sum;
}

void InterPrediction::sumUpSamples(const Pel * pDst, uint32_t  refStride, SizeType cuWidth, SizeType cuHeight, int32_t& Avg)
{
  Avg = 0;
  if (cuWidth == 1)
  {
    for (int j = 0; j < cuHeight; j++)
    {
      Avg += *pDst;
      pDst += refStride;
    }
  }
  else
  {
    for (int j = 0; j < cuHeight; j++)
    {
      for (int i = 0; i < cuWidth; i += 4)
      {
        Avg += pDst[i + 0];
        Avg += pDst[i + 1];
        Avg += pDst[i + 2];
        Avg += pDst[i + 3];
      }
      pDst += refStride;
    }
  }
}
void InterPrediction::xBIPMVRefine(PredictionUnit& pu, uint32_t nSearchStepShift, MRSADtype& minCost, DistParam &cDistParam, Mv *refineMv /* = nullptr*/)
{
  const SizeType cuWidth = pu.lumaSize().width;
  const SizeType cuHeight = pu.lumaSize().height;
  const bool bFracPel = (nSearchStepShift % 2);

  const Mv cMvOrgL0 = pu.mv[REF_PIC_LIST_0];
  Mv cBestMvL0;
  const Mv cMvOrgL1 = pu.mv[REF_PIC_LIST_1];
  int32_t bestL0Mean = std::numeric_limits<int32_t>::max();
  int32_t bestL1Mean = std::numeric_limits<int32_t>::max();;
  if (!bFracPel)
  {
    m_currentSADsArray = { { NotDefinedSAD, NotDefinedSAD, NotDefinedSAD, NotDefinedSAD, NotDefinedSAD, NotDefinedSAD, NotDefinedSAD, NotDefinedSAD, minCost } };
    SAD_POINT_INDEX cornerIndex(SAD_POINT_INDEX::TOP_LEFT);
    const std::array<SAD_POINT_INDEX, SAD_POINT_INDEX::COUNT> &lastDirectionIndexesArray = currentSADIndexesSet[m_lastDirection];
    int32_t LineMeanL0[4] = { 0, 0, 0, 0 };
    int32_t LineMeanL1[4] = { 0, 0, 0, 0 };
    const int32_t refStride = m_cYuvPredTempL0.Y().stride;
    for (SAD_POINT_INDEX nIdx = SAD_POINT_INDEX::BOTTOM; nIdx <= SAD_POINT_INDEX::TOP_LEFT; ++nIdx)
    {
      Mv cMvDL0(m_pSearchOffset[nIdx], cMvOrgL0.highPrec);
      const Pel *pRefL0 = m_cYuvPredTempL0.Y().bufAt(cMvDL0.getHor() + (cDistParam.MVDL0.getHor() >> nSearchStepShift), cMvDL0.getVer() + (cDistParam.MVDL0.getVer() >> nSearchStepShift));
      const Mv cMvDL1(-cMvDL0);
      const Pel* pRefL1 = m_cYuvPredTempL1.Y().bufAt(cMvDL1.getHor() + (cDistParam.MVDL1.getHor() >> nSearchStepShift), cMvDL1.getVer() + (cDistParam.MVDL1.getVer() >> nSearchStepShift));
      int32_t AvgDest = 0;
      switch (nIdx)
      {
      case SAD_POINT_INDEX::BOTTOM:
        sumUpSamples(pRefL0 + (cuHeight - 1)*refStride, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL0 = AvgDest;
        sumUpSamples(pRefL0 - refStride, refStride, cuWidth, 1, AvgDest);
        LineMeanL0[3] = cDistParam.meanL0 - AvgDest;
        cDistParam.meanL0 += cDistParam.partOfMeanL0 - AvgDest;

        sumUpSamples(pRefL1, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL1 = AvgDest;
        sumUpSamples(pRefL1 + cuHeight*refStride, refStride, cuWidth, 1, AvgDest);
        LineMeanL1[2] = cDistParam.meanL1 - AvgDest;
        cDistParam.meanL1 += cDistParam.partOfMeanL1 - AvgDest;
        break;
      case SAD_POINT_INDEX::TOP:
        sumUpSamples(pRefL0, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL0 = AvgDest;
        sumUpSamples(pRefL0 + cuHeight*refStride, refStride, cuWidth, 1, AvgDest);
        LineMeanL0[2] = cDistParam.meanL0 - AvgDest;
        cDistParam.meanL0 += cDistParam.partOfMeanL0 - AvgDest;

        sumUpSamples(pRefL1 + (cuHeight - 1)*refStride, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL1 = AvgDest;
        sumUpSamples(pRefL1 - refStride, refStride, cuWidth, 1, AvgDest);
        LineMeanL1[3] = cDistParam.meanL1 - AvgDest;
        cDistParam.meanL1 += cDistParam.partOfMeanL1 - AvgDest;
        break;
      case SAD_POINT_INDEX::RIGHT:
        sumUpSamples(pRefL0 + cuWidth - 1, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL0 = AvgDest;
        sumUpSamples(pRefL0 - 1, refStride, 1, cuHeight, AvgDest);
        LineMeanL0[1] = cDistParam.meanL0 - AvgDest;
        cDistParam.meanL0 += cDistParam.partOfMeanL0 - AvgDest;

        sumUpSamples(pRefL1, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL1 = AvgDest;
        sumUpSamples(pRefL1 + cuWidth, refStride, 1, cuHeight, AvgDest);
        LineMeanL1[0] = cDistParam.meanL1 - AvgDest;
        cDistParam.meanL1 += cDistParam.partOfMeanL1 - AvgDest;
        break;
      case SAD_POINT_INDEX::LEFT:
        sumUpSamples(pRefL0, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL0 = AvgDest;
        sumUpSamples(pRefL0 + cuWidth, refStride, 1, cuHeight, AvgDest);
        LineMeanL0[0] = cDistParam.meanL0 - AvgDest;
        cDistParam.meanL0 += cDistParam.partOfMeanL0 - AvgDest;

        sumUpSamples(pRefL1 + cuWidth - 1, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL1 = AvgDest;
        sumUpSamples(pRefL1 - 1, refStride, 1, cuHeight, AvgDest);
        LineMeanL1[1] = cDistParam.meanL1 - AvgDest;
        cDistParam.meanL1 += cDistParam.partOfMeanL1 - AvgDest;
        break;
      default:
        CHECK(nIdx != SAD_POINT_INDEX::TOP_LEFT, "WRONG INDEX");
        const int32_t x = (cMvDL0.getHor() == -1) ? 0 : -1;
        const int32_t y = (cMvDL0.getVer() == -1) ? 0 : -refStride;
        const int32_t yIndex = (cMvDL0.getVer() == -1) ? 0 : -1;

        const int32_t xL1 = (x) ? 0 : -1;
        const int32_t yL1 = (y) ? 0 : -refStride;
        const int32_t yIndexL1 = (yIndex) ? 0 : -1;

        const int32_t sign = cMvDL0.getHor() * cMvDL0.getVer();

        cDistParam.meanL0 = cDistParam.partOfMeanL0 + LineMeanL0[-x] + LineMeanL0[2 - yIndex] + sign * pRefL0[x + y] - sign * pRefL0[y + (int32_t)cuWidth + x] - sign * pRefL0[x + cuHeight*refStride + y] + sign * pRefL0[cuHeight*refStride + (int32_t)cuWidth + y + x];
        cDistParam.meanL1 = cDistParam.partOfMeanL1 + LineMeanL1[-xL1] + LineMeanL1[2 - yIndexL1] + sign * pRefL1[xL1 + yL1] - sign * pRefL1[yL1 + (int32_t)cuWidth + xL1] - sign * pRefL1[xL1 + cuHeight*refStride + yL1] + sign * pRefL1[cuHeight*refStride + (int32_t)cuWidth + yL1 + xL1];
      }
      const SAD_POINT_INDEX currentIndex = (nIdx != SAD_POINT_INDEX::TOP_LEFT) ? (SAD_POINT_INDEX)nIdx : cornerIndex;
      const SAD_POINT_INDEX currentPoint = lastDirectionIndexesArray[currentIndex];

      const MRSADtype cost = (currentPoint != SAD_POINT_INDEX::NOT_AVAILABLE && m_previousSADsArray[currentPoint] != NotDefinedSAD) ?
        m_previousSADsArray[currentPoint] 
        : xDirectMCCostDMVR(pRefL0, pRefL1, refStride, cuWidth, cuHeight, cDistParam);

      m_currentSADsArray[currentIndex] = cost;
      if (nIdx == SAD_POINT_INDEX::LEFT)
      {
        int32_t down = -1, right = -1;
        if (m_currentSADsArray[SAD_POINT_INDEX::BOTTOM] < m_currentSADsArray[SAD_POINT_INDEX::TOP])
        {
          down = 1;
          cornerIndex += 2;
        }
        if (m_currentSADsArray[SAD_POINT_INDEX::RIGHT] < m_currentSADsArray[SAD_POINT_INDEX::LEFT])
        {
          right = 1;
          cornerIndex += 1;
        }
        m_pSearchOffset[SAD_POINT_INDEX::TOP_LEFT].set(right, down);
      }
      if (cost < minCost)
      {
        minCost = cost;
        cBestMvL0 = cMvDL0;
        m_lastDirection = currentIndex;
        bestL0Mean = cDistParam.meanL0;
        bestL1Mean = cDistParam.meanL1;
      }
    }
  }
  else
  {
    const int32_t refStride = m_HalfPelFilteredBuffL0[0][1].Y().stride;
    for (SAD_POINT_INDEX nIdx = SAD_POINT_INDEX::BOTTOM; nIdx <= SAD_POINT_INDEX::LEFT; ++nIdx)
    {
		  Mv cMvDL0(m_pSearchOffset[nIdx], cMvOrgL0.highPrec);
      Mv cMvDL1(-cMvDL0);
      const Pel *pRefL0 = m_filteredBlock[cMvDL0.getAbsHor()][cMvDL0.getAbsVer()][COMPONENT_Y];
      const Pel *pRefL1 = m_filteredBlockL1[cMvDL1.getAbsHor()][cMvDL1.getAbsVer()];
      int32_t AvgDest = 0;
      switch (nIdx)
      {
      case SAD_POINT_INDEX::BOTTOM:
        pRefL0 += refStride;
        sumUpSamples(pRefL0, refStride, cuWidth, cuHeight, AvgDest);
        cDistParam.meanL0 = AvgDest;
        cDistParam.partOfMeanL0 = cDistParam.meanL0;
        sumUpSamples(pRefL1, refStride, cuWidth, cuHeight, AvgDest);
        cDistParam.meanL1 = AvgDest;
        cDistParam.partOfMeanL1 = cDistParam.meanL1;
        break;
      case SAD_POINT_INDEX::TOP:
        sumUpSamples(pRefL0, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL0 = AvgDest;
        sumUpSamples(pRefL0 + cuHeight * refStride, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL0 += cDistParam.partOfMeanL0 - AvgDest;
        sumUpSamples(pRefL1, refStride, cuWidth, 1, AvgDest);
        pRefL1 += refStride;
        cDistParam.meanL1 = cDistParam.partOfMeanL1 - AvgDest;
        sumUpSamples(pRefL1 + (cuHeight - 1) * refStride, refStride, cuWidth, 1, AvgDest);
        cDistParam.meanL1 += AvgDest;
        break;
      case SAD_POINT_INDEX::RIGHT:
        pRefL0++;
        sumUpSamples(pRefL0, refStride, cuWidth, cuHeight, AvgDest);
        cDistParam.meanL0 = AvgDest;
        cDistParam.partOfMeanL0 = cDistParam.meanL0;
        sumUpSamples(pRefL1, refStride, cuWidth, cuHeight, AvgDest);
        cDistParam.meanL1 = AvgDest;
        cDistParam.partOfMeanL1 = cDistParam.meanL1;
        break;
      case SAD_POINT_INDEX::LEFT:
        sumUpSamples(pRefL0, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL0 = AvgDest;
        sumUpSamples(pRefL0 + cuWidth, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL0 += cDistParam.partOfMeanL0 - AvgDest;
        sumUpSamples(pRefL1, refStride, 1, cuHeight, AvgDest);
        pRefL1++;
        cDistParam.meanL1 = cDistParam.partOfMeanL1 - AvgDest;
        sumUpSamples(pRefL1 + cuWidth - 1, refStride, 1, cuHeight, AvgDest);
        cDistParam.meanL1 += AvgDest;
        break;
      default:
        CHECK(1, "WRONG INDEX");
      }
      const MRSADtype cost = xDirectMCCostDMVR(pRefL0, pRefL1, refStride, cuWidth, cuHeight, cDistParam);
      if (cost < minCost)
      {
        minCost = cost;
        cBestMvL0 = cMvDL0;
        if (refineMv)
        {
          *refineMv = cMvDL0;
        }
      }
    }
  }
  if (!cBestMvL0.IsZero())
  {
    pu.mv[REF_PIC_LIST_0] = (cBestMvL0 << nSearchStepShift) + cMvOrgL0;
    pu.mv[REF_PIC_LIST_1] = ((-cBestMvL0) << nSearchStepShift) + cMvOrgL1;
    if (!bFracPel)
    {
      m_previousSADsArray = m_currentSADsArray;
      cDistParam.meanL0 = bestL0Mean;
      cDistParam.meanL1 = bestL1Mean;
    }
  }
}

void InterPrediction::xGenerateFracPixel(PredictionUnit& pu, uint32_t nSearchStepShift, const ClpRngs &clpRngs)
{
  const Mv cMvOrgL0 = pu.mv[REF_PIC_LIST_0];
  const Mv cMvOrgL1 = pu.mv[REF_PIC_LIST_1];
  const SizeType cuHeight(pu.lumaSize().height);
  const SizeType notAlignedWidth(pu.lumaSize().width);
  const SizeType alignSize(8);
  const SizeType restAlign = notAlignedWidth % alignSize;
  const SizeType cuWidth = restAlign ? notAlignedWidth + (alignSize - restAlign) : notAlignedWidth;
  const  int32_t bufferStride = MAX_CU_SIZE + m_bufferWidthExtSize + 16;
  const uint32_t bufferWidth = restAlign ? cuWidth : cuWidth + alignSize;
  const uint32_t bufferHeight = cuHeight + 1;
  //(0,-/+1)
  Mv cMvL0(cMvOrgL0 + Mv(0, -1 << nSearchStepShift, cMvOrgL0.highPrec));
  m_HalfPelFilteredBuffL0[0][1] = PelUnitBuf(pu.chromaFormat, PelBuf(m_filteredBlock[0][1][COMPONENT_Y], bufferStride, bufferWidth, bufferHeight));
  clipMv(cMvL0, pu.cu->lumaPos(), *pu.cu->slice->getSPS());
  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[REF_PIC_LIST_0]), cMvL0
    , m_HalfPelFilteredBuffL0[0][1]
    , true, clpRngs.comp[COMPONENT_Y], false, true, FRUC_MERGE_OFF, false
    , true
    , cuWidth
    , cuHeight + 1
  );
  Mv cMvL1(cMvOrgL1 + Mv(0, -1 << nSearchStepShift, cMvOrgL1.highPrec));
  m_HalfPelFilteredBuffL1[0][1] = PelUnitBuf(pu.chromaFormat, PelBuf(m_filteredBlockL1[0][1], bufferStride, bufferWidth, bufferHeight));
  clipMv(cMvL1, pu.cu->lumaPos(), *pu.cu->slice->getSPS());
  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[REF_PIC_LIST_1]), cMvL1
    , m_HalfPelFilteredBuffL1[0][1]
    , true, clpRngs.comp[COMPONENT_Y], false, true, FRUC_MERGE_OFF, false
    , true
    , cuWidth
    , cuHeight + 1
  );
  //(-/+1,0)
  cMvL0 = cMvOrgL0 + Mv(-1 << nSearchStepShift, 0, cMvOrgL0.highPrec);
  m_HalfPelFilteredBuffL0[1][0] = PelUnitBuf(pu.chromaFormat, PelBuf(m_filteredBlock[1][0][COMPONENT_Y], bufferStride, bufferWidth, bufferHeight));
  clipMv(cMvL0, pu.cu->lumaPos(), *pu.cu->slice->getSPS());
  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[REF_PIC_LIST_0]), cMvL0
    , m_HalfPelFilteredBuffL0[1][0]
    , true, clpRngs.comp[COMPONENT_Y], false, true, FRUC_MERGE_OFF, false
    , true
    , bufferWidth
    , cuHeight
  );
  cMvL1 = cMvOrgL1 + Mv(-1 << nSearchStepShift, 0, cMvOrgL1.highPrec);
  m_HalfPelFilteredBuffL1[1][0] = PelUnitBuf(pu.chromaFormat, PelBuf(m_filteredBlockL1[1][0], bufferStride, bufferWidth, bufferHeight));
  clipMv(cMvL1, pu.cu->lumaPos(), *pu.cu->slice->getSPS());
  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[REF_PIC_LIST_1]), cMvL1
    , m_HalfPelFilteredBuffL1[1][0]
    , true, clpRngs.comp[COMPONENT_Y], false, true, FRUC_MERGE_OFF, false
    , true
    , bufferWidth
    , cuHeight
  );
}
#endif  // DMVR_JVET_K0217
void InterPrediction::xProcessDMVR( PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bBIOApplied )
{
  const int iRefIdx0  = pu.refIdx[0];
  const int iRefIdx1  = pu.refIdx[1];

  PelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y())) :
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[0][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvDst.Cr())) );
  PelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y())) :
                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[1][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvDst.Cr())) );


  //mv refinement
  uint32_t searchStepShift = 2;
  if( pu.cu->slice->getSPS()->getSpsNext().getUseHighPrecMv() )
  {
    searchStepShift += 2;
    pu.mv[0].setHighPrec();
    pu.mv[1].setHighPrec();
  }
#if !DMVR_JVET_K0217
  pcYuvDst.addAvg( srcPred0, srcPred1, clpRngs, false, true );

  //list 0
  //get init cost
  srcPred0.Y().toLast( clpRngs.comp[COMPONENT_Y] );
#if DISTORTION_TYPE_BUGFIX
  Distortion uiMinCost =
    xDirectMCCost(clpRngs.comp[COMPONENT_Y].bd, pcYuvDst.Y().buf, pcYuvDst.Y().stride, srcPred0.Y().buf,
                  srcPred0.Y().stride, pu.lumaSize().width, pu.lumaSize().height);
#else
  uint32_t uiMinCost = xDirectMCCost( clpRngs.comp[COMPONENT_Y].bd, pcYuvDst.Y().buf, pcYuvDst.Y().stride, srcPred0.Y().buf, srcPred0.Y().stride, pu.lumaSize().width, pu.lumaSize().height );
#endif

  xFillPredBlckAndBorder( pu, REF_PIC_LIST_0, pu.lumaSize().width, pu.lumaSize().height, srcPred0.Y() );

  xBIPMVRefine( pu, REF_PIC_LIST_0, pu.lumaSize().width, pu.lumaSize().height, pcYuvDst, DMVR_INTME_RANGE, searchStepShift, uiMinCost );

  Mv mv = pu.mv[0];
  m_iRefListIdx = 0;

  clipMv( mv, pu.lumaPos(), *pu.cs->sps );


  for( uint32_t comp = COMPONENT_Y; comp < srcPred0.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );

    xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_0, iRefIdx0 ), mv, srcPred0, true, clpRngs.comp[compID], bBIOApplied, false, FRUC_MERGE_OFF, true );
  }

  //list 1
  //get init cost
  srcPred1.Y().toLast( clpRngs.comp[COMPONENT_Y] );
  uiMinCost = xDirectMCCost( clpRngs.comp[COMPONENT_Y].bd, pcYuvDst.Y().buf, pcYuvDst.Y().stride, srcPred1.Y().buf, srcPred1.Y().stride, pu.lumaSize().width, pu.lumaSize().height );

  xFillPredBlckAndBorder( pu, REF_PIC_LIST_1, pu.lumaSize().width, pu.lumaSize().height, srcPred1.Y() );

  xBIPMVRefine( pu, REF_PIC_LIST_1, pu.lumaSize().width, pu.lumaSize().height, pcYuvDst, DMVR_INTME_RANGE, searchStepShift, uiMinCost );

  mv = pu.mv[1];
  m_iRefListIdx = 1;

  clipMv( mv, pu.lumaPos(), *pu.cs->sps );


  for( uint32_t comp = COMPONENT_Y; comp < srcPred1.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );

    xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_1, iRefIdx1 ), mv, srcPred1, true, clpRngs.comp[compID], bBIOApplied, false, FRUC_MERGE_OFF, true );
  }
#else
  const Size cuSize(pu.lumaSize());
  Mv L0StartPoint;
  Mv L1StartPoint;
  Mv StartingMVL0(pu.mv[REF_PIC_LIST_0]);
  Mv StartingMVL1(pu.mv[REF_PIC_LIST_1]);

  Mv initialMV0(StartingMVL0);
  Mv initialMV1(StartingMVL1);
#if DMVR_JVET_SEARCH_RANGE_K0217 > 2
  m_checkedMVsList.clear();
  m_checkedMVsList.push_back(StartingMVL0);
#endif
  DistParam cDistParam;
  MRSADtype minCost = std::numeric_limits<MRSADtype>::max();

  const Mv mvBlkExt(m_searchRange << searchStepShift, pu.mv[REF_PIC_LIST_0].highPrec);
  const SizeType MC_extension = m_searchRange << 1;
  const Mv searchOffsetMv(-(int32_t)(m_searchRange << searchStepShift), pu.mv[REF_PIC_LIST_0].highPrec);
  const uint32_t dstStride = MAX_CU_SIZE + m_bufferWidthExtSize;

  m_cYuvPredTempL0 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL0, dstStride, cuSize.width + MC_extension, cuSize.height + MC_extension));
  Mv mvL0(pu.mv[REF_PIC_LIST_0] + searchOffsetMv);
  m_iRefListIdx = 0;
  clipMv(mvL0, pu.lumaPos(), *pu.cs->sps);
  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, iRefIdx0), mvL0, m_cYuvPredTempL0, true, clpRngs.comp[COMPONENT_Y], false, false
    , FRUC_MERGE_OFF
    , false, true, cuSize.width + MC_extension, cuSize.height + MC_extension
  );

  m_cYuvPredTempL1 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL1, dstStride, srcPred1.Y().width + MC_extension, srcPred1.Y().height + MC_extension));
  Mv mvL1(pu.mv[REF_PIC_LIST_1] + searchOffsetMv);
  m_iRefListIdx = 1;
  clipMv(mvL1, pu.lumaPos(), *pu.cs->sps);
  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, iRefIdx1), mvL1, m_cYuvPredTempL1, true, clpRngs.comp[COMPONENT_Y], false, false
    , FRUC_MERGE_OFF
    , false, true, cuSize.width + MC_extension, cuSize.height + MC_extension
  );

  bool notZeroCost(true);
  for (int i = 0; i < m_searchRange; i++)
  {
    initialMV0 = pu.mv[REF_PIC_LIST_0];
    initialMV1 = pu.mv[REF_PIC_LIST_1];

    L0StartPoint = initialMV0 - StartingMVL0 + mvBlkExt;
    Pel *AddrL0 = m_cYuvPredTempL0.Y().bufAt(L0StartPoint.getHor() >> searchStepShift, L0StartPoint.getVer() >> searchStepShift);
    L1StartPoint = initialMV1 - StartingMVL1 + mvBlkExt;
    Pel *AddrL1 = m_cYuvPredTempL1.Y().bufAt(L1StartPoint.getHor() >> searchStepShift, L1StartPoint.getVer() >> searchStepShift);
    int32_t AvgL0 = 0;
    int32_t AvgL1 = 0;
    if (i == 0)
    {
      sumUpSamples(AddrL0, dstStride, cuSize.width, cuSize.height, AvgL0);
      sumUpSamples(AddrL1, dstStride, cuSize.width, cuSize.height, AvgL1);

      cDistParam.meanL0 = AvgL0;
      cDistParam.meanL1 = AvgL1;
      m_lastDirection = SAD_POINT_INDEX::CENTER;
    }
    else
    {
      AvgL0 = cDistParam.meanL0;
      AvgL1 = cDistParam.meanL1;
    }
    cDistParam.partOfMeanL0 = AvgL0;
    cDistParam.partOfMeanL1 = AvgL1;
    if (i == 0)
    {
        minCost = xDirectMCCostDMVR(AddrL0, AddrL1, dstStride, cuSize.width, cuSize.height, cDistParam);
    }
    if (!minCost)
    {
	    notZeroCost = false;
	    break;
    }
    cDistParam.MVDL0 = L0StartPoint;
    cDistParam.MVDL1 = L1StartPoint;
    xBIPMVRefine(pu, searchStepShift, minCost, cDistParam);
#if DMVR_JVET_SEARCH_RANGE_K0217 > 2
    auto found = std::find(m_checkedMVsList.cbegin(), m_checkedMVsList.cend(), pu.mv[REF_PIC_LIST_0]);
    if (found != m_checkedMVsList.end())
    {
      break;
    }
    else
    {
      m_checkedMVsList.push_back(pu.mv[REF_PIC_LIST_0]);
    }
#else
    if (i == 0 && StartingMVL0 == pu.mv[REF_PIC_LIST_0])
    {
      break;
    }

#endif
  }

  // Half Pel refinement
  bool bApplyHalfPel = (notZeroCost
    && ((StartingMVL0 - pu.mv[REF_PIC_LIST_0]).getAbsHor() >> searchStepShift) < m_searchRange
    && ((StartingMVL0 - pu.mv[REF_PIC_LIST_0]).getAbsVer() >> searchStepShift) < m_searchRange);
  MRSADtype uiIntPelCostL0 = minCost;
  Mv cRefineMv;
  if (bApplyHalfPel)
  {
    xGenerateFracPixel(pu, searchStepShift - 1, clpRngs);
    xBIPMVRefine(pu, searchStepShift - 1, minCost, cDistParam, &cRefineMv);
  }
  if (bApplyHalfPel && minCost < uiIntPelCostL0)
  {
    srcPred0.Y().copyFrom(m_HalfPelFilteredBuffL0[cRefineMv.getAbsHor()][cRefineMv.getAbsVer()].Y().subBuf(Position{ cRefineMv.getHor() == 1 ? 1 : 0, cRefineMv.getVer() == 1 ? 1 : 0 }, cuSize));
    srcPred1.Y().copyFrom(m_HalfPelFilteredBuffL1[cRefineMv.getAbsHor()][cRefineMv.getAbsVer()].Y().subBuf(Position{ -cRefineMv.getHor() == 1 ? 1 : 0, -cRefineMv.getVer() == 1 ? 1 : 0 }, cuSize));
  }
  else
  {
    L0StartPoint = pu.mv[REF_PIC_LIST_0] - StartingMVL0 + mvBlkExt;
    L1StartPoint = pu.mv[REF_PIC_LIST_1] - StartingMVL1 + mvBlkExt;

    srcPred0.Y().copyFrom(m_cYuvPredTempL0.Y().subBuf(Position{ L0StartPoint.getHor() >> searchStepShift, L0StartPoint.getVer() >> searchStepShift }, cuSize));
    srcPred1.Y().copyFrom(m_cYuvPredTempL1.Y().subBuf(Position{ L1StartPoint.getHor() >> searchStepShift, L1StartPoint.getVer() >> searchStepShift }, cuSize));
  }
  m_iRefListIdx = 0;
  mvL0 = pu.mv[REF_PIC_LIST_0];
  clipMv(mvL0, pu.lumaPos(), *pu.cs->sps);

  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, iRefIdx0), mvL0, srcPred0, true, clpRngs.comp[COMPONENT_Y], bBIOApplied, false, FRUC_MERGE_OFF, true, bBIOApplied);
  xPredInterBlk(COMPONENT_Cb, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, iRefIdx0), mvL0, srcPred0, true, clpRngs.comp[COMPONENT_Cb], bBIOApplied, false, FRUC_MERGE_OFF, true);
  xPredInterBlk(COMPONENT_Cr, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, iRefIdx0), mvL0, srcPred0, true, clpRngs.comp[COMPONENT_Cr], bBIOApplied, false, FRUC_MERGE_OFF, true);
  
  m_iRefListIdx = 1;
  mvL1 = pu.mv[REF_PIC_LIST_1];
  clipMv(mvL1, pu.lumaPos(), *pu.cs->sps);

  xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, iRefIdx1), mvL1, srcPred1, true, clpRngs.comp[COMPONENT_Y], bBIOApplied, false, FRUC_MERGE_OFF, true, bBIOApplied);
  xPredInterBlk(COMPONENT_Cb, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, iRefIdx1), mvL1, srcPred1, true, clpRngs.comp[COMPONENT_Cb], bBIOApplied, false, FRUC_MERGE_OFF, true);
  xPredInterBlk(COMPONENT_Cr, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, iRefIdx1), mvL1, srcPred1, true, clpRngs.comp[COMPONENT_Cr], bBIOApplied, false, FRUC_MERGE_OFF, true);

#if DMVR_JVET_LOW_LATENCY_K0217
  pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - StartingMVL0;
  pu.mv[REF_PIC_LIST_0] = StartingMVL0;
  pu.mv[REF_PIC_LIST_1] = StartingMVL1;
#endif
#endif // #if !DMVR_JVET_K0217
  PU::spanMotionInfo( pu );
}
#endif

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign( CacheModel *cache )
{
  m_cacheModel = cache;
  m_if.cacheAssign( cache );
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif

//! \}
