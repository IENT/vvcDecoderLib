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

/** \file     CrossCompPrediction.cpp
    \brief    cross component prediction class
*/

#include "CrossCompPrediction.h"

#include "Unit.h"
#include "CommonDef.h"
#include "CodingStructure.h"
#include "Slice.h"

//! \ingroup CommonLib
//! \{

SChar CrossComponentPrediction::xCalcCrossComponentPredictionAlpha( TransformUnit &tu, const ComponentID &compID, Bool useRecoResidual )
{
  const CPelBuf pResiL = useRecoResidual ? tu.cs->getResiBuf( tu.Y() ) : tu.cs->getOrgResiBuf( tu.Y() );
  const CPelBuf pResiC = tu.cs->getResiBuf( tu.blocks[compID] );

  const Int diffBitDepth = tu.cs->sps->getDifferentialLumaChromaBitDepth();

  SChar alpha = 0;
  Int SSxy = 0;
  Int SSxx = 0;

  for( UInt uiY = 0; uiY < pResiL.height; uiY++ )
  {
    for( UInt uiX = 0; uiX < pResiL.width; uiX++ )
    {
      const Pel scaledResiL = rightShift( pResiL.at( uiX, uiY ), diffBitDepth );
      SSxy += ( scaledResiL * pResiC.at( uiX, uiY ) );
      SSxx += ( scaledResiL * scaledResiL );
    }
  }

  if( SSxx != 0 )
  {
    Double dAlpha = SSxy / Double( SSxx );
    alpha = SChar( Clip3<Int>( -16, 16, ( Int ) ( dAlpha * 16 ) ) );

    static const SChar alphaQuant[17] = { 0, 1, 1, 2, 2, 2, 4, 4, 4, 4, 4, 4, 8, 8, 8, 8, 8 };

    alpha = ( alpha < 0 ) ? -alphaQuant[Int( -alpha )] : alphaQuant[Int( alpha )];
  }

  tu.compAlpha[compID] = alpha;

  return alpha;
}


Void CrossComponentPrediction::crossComponentPrediction(        TransformUnit &tu,
                                                          const ComponentID   &compID,
                                                          const CPelBuf       &piResiL,
                                                          const CPelBuf       &piResiC,
                                                                PelBuf        &piResiT,
                                                          const Bool          &reverse )
{
  const Int alpha = tu.compAlpha[compID];
  const Int diffBitDepth = tu.cs->sps->getDifferentialLumaChromaBitDepth();

#if !RExt__HIGH_BIT_DEPTH_SUPPORT
  ClpRng clpRng; //not limited by adaptive clipping
  clpRng.min = std::numeric_limits<Pel>::min();
  clpRng.max = std::numeric_limits<Pel>::max();
#endif

  for( Int y = 0; y < piResiT.height; y++ )
  {
    if( reverse )
    {
      // A constraint is to be added to the HEVC Standard to limit the size of pResiL and pResiC at this point.
      // The likely form of the constraint is to either restrict the values to CoeffMin to CoeffMax,
      // or to be representable in a bitDepthY+4 or bitDepthC+4 signed integer.
      //  The result of the constraint is that for 8/10/12bit profiles, the input values
      //  can be represented within a 16-bit Pel-type.
#if RExt__HIGH_BIT_DEPTH_SUPPORT
      for( Int x = 0; x < piResiT.width; x++ )
      {
        piResiT.at( x, y ) = piResiC.at( x, y ) + ( ( alpha * rightShift( piResiL.at( x, y ), diffBitDepth ) ) >> 3 );
      }
#else
      for( Int x = 0; x < piResiT.width; x++ )
      {
        piResiT.at( x, y ) = ClipPel<Int>( piResiC.at( x, y ) + ( ( alpha * rightShift<Int>( Int( piResiL.at( x, y ) ), diffBitDepth ) ) >> 3 ), clpRng );
      }
#endif
    }
    else
    {
      // Forward does not need clipping. Pel type should always be big enough.
      for( Int x = 0; x < piResiT.width; x++ )
      {
        piResiT.at( x, y ) = piResiC.at( x, y ) - ( ( alpha * rightShift<Int>( Int( piResiL.at( x, y ) ), diffBitDepth ) ) >> 3 );
      }
    }
  }
}
