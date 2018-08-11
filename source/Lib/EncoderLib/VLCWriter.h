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

/** \file     VLCWriter.h
 *  \brief    Writer for high level syntax
 */

#ifndef __VLCWRITER__
#define __VLCWRITER__

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Slice.h"
#include "CABACWriter.h"

//! \ingroup EncoderLib
//! \{

#if ENABLE_TRACING

#define WRITE_CODE( value, length, name)    xWriteCodeTr ( value, length, name )
#define WRITE_UVLC( value,         name)    xWriteUvlcTr ( value,         name )
#define WRITE_SVLC( value,         name)    xWriteSvlcTr ( value,         name )
#define WRITE_FLAG( value,         name)    xWriteFlagTr ( value,         name )

extern bool g_HLSTraceEnable;
#else

#define WRITE_CODE( value, length, name)     xWriteCode ( value, length )
#define WRITE_UVLC( value,         name)     xWriteUvlc ( value )
#define WRITE_SVLC( value,         name)     xWriteSvlc ( value )
#define WRITE_FLAG( value,         name)     xWriteFlag ( value )

#endif



class VLCWriter
{
protected:

  OutputBitstream*    m_pcBitIf;

  VLCWriter() : m_pcBitIf(NULL) {}
  virtual ~VLCWriter() {}

  Void  setBitstream          ( OutputBitstream* p )  { m_pcBitIf = p;  }

  Void  xWriteCode            ( UInt uiCode, UInt uiLength );
  Void  xWriteUvlc            ( UInt uiCode );
  Void  xWriteSvlc            ( Int  iCode   );
  Void  xWriteFlag            ( UInt uiCode );
#if ENABLE_TRACING
  Void  xWriteCodeTr          ( UInt value, UInt  length, const TChar *pSymbolName);
  Void  xWriteUvlcTr          ( UInt value,               const TChar *pSymbolName);
  Void  xWriteSvlcTr          ( Int  value,               const TChar *pSymbolName);
  Void  xWriteFlagTr          ( UInt value,               const TChar *pSymbolName);
#endif
  Void  xWriteRbspTrailingBits();
};



class AUDWriter : public VLCWriter
{
public:
  AUDWriter() {};
  virtual ~AUDWriter() {};

  Void  codeAUD(OutputBitstream& bs, const Int pictureType);
};



class HLSWriter : public VLCWriter
{
public:
  HLSWriter() {}
  virtual ~HLSWriter() {}

#if JEM_TOOLS
  void  init( CABACDataStore& cabacDataStore ) { m_CABACDataStore = &cabacDataStore; }
#endif
private:
  Void xCodeShortTermRefPicSet  ( const ReferencePictureSet* pcRPS, Bool calledFromSliceHeader, Int idx );
  Bool xFindMatchingLTRP        ( Slice* pcSlice, UInt *ltrpsIndex, Int ltrpPOC, Bool usedFlag );
  Void xCodePredWeightTable     ( Slice* pcSlice );
#if HEVC_USE_SCALING_LISTS
  Void xCodeScalingList         ( const ScalingList* scalingList, UInt sizeId, UInt listId);
#endif
#if JEM_TOOLS
  void xCodeCABACWSizes         ( Slice* pcSlice );
#endif
public:
  Void  setBitstream            ( OutputBitstream* p )  { m_pcBitIf = p;  }
  UInt  getNumberOfWrittenBits  ()                      { return m_pcBitIf->getNumberOfWrittenBits();  }
  Void  codeVUI                 ( const VUI *pcVUI, const SPS* pcSPS );
  Void  codeSPSNext             ( const SPSNext& spsNext, const bool usePCM );
  Void  codeSPS                 ( const SPS* pcSPS );
  Void  codePPS                 ( const PPS* pcPPS );
#if HEVC_VPS
  Void  codeVPS                 ( const VPS* pcVPS );
#endif
  Void  codeSliceHeader         ( Slice* pcSlice );
  Void  codePTL                 ( const PTL* pcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1);
  Void  codeProfileTier         ( const ProfileTierLevel* ptl, const Bool bIsSubLayer );
  Void  codeHrdParameters       ( const HRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1 );
#if HEVC_TILES_WPP
  Void  codeTilesWPPEntryPoint  ( Slice* pSlice );
#endif
#if HEVC_USE_SCALING_LISTS
  Void  codeScalingList         ( const ScalingList &scalingList );
#endif

#if JVET_K0371_ALF
  void alf( const AlfSliceParam& alfSliceParam );
  void alfFilter( const AlfSliceParam& alfSliceParam, const bool isChroma );

private:
  void xWriteTruncBinCode( UInt uiSymbol, const int uiMaxSymbol );
  void alfGolombEncode( const int coeff, const int k );
  void truncatedUnaryEqProb( int symbol, int maxSymbol );
#endif

#if JEM_TOOLS
private:
  CABACDataStore* m_CABACDataStore;
#endif
};

//! \}

#endif
