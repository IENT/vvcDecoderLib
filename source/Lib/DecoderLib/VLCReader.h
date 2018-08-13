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

/** \file     VLCWReader.h
 *  \brief    Reader for high level syntax
 */

#ifndef __VLCREADER__
#define __VLCREADER__

#include "CommonLib/Rom.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Slice.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CABACReader.h"

#if ENABLE_TRACING

#define READ_CODE(length, code, name)     xReadCodeTr ( length, code, name )
#define READ_UVLC(        code, name)     xReadUvlcTr (         code, name )
#define READ_SVLC(        code, name)     xReadSvlcTr (         code, name )
#define READ_FLAG(        code, name)     xReadFlagTr (         code, name )

#else

#if RExt__DECODER_DEBUG_BIT_STATISTICS

#define READ_CODE(length, code, name)     xReadCode ( length, code, name )
#define READ_UVLC(        code, name)     xReadUvlc (         code, name )
#define READ_SVLC(        code, name)     xReadSvlc (         code, name )
#define READ_FLAG(        code, name)     xReadFlag (         code, name )

#else

#define READ_CODE(length, code, name)     xReadCode ( length, code )
#define READ_UVLC(        code, name)     xReadUvlc (         code )
#define READ_SVLC(        code, name)     xReadSvlc (         code )
#define READ_FLAG(        code, name)     xReadFlag (         code )

#endif

#endif

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class VLCReader
{
protected:
  InputBitstream*   m_pcBitstream;

  VLCReader() : m_pcBitstream (NULL) {};
  virtual ~VLCReader() {};

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  void  xReadCode    ( UInt   length, UInt& val, const TChar *pSymbolName );
  void  xReadUvlc    (                UInt& val, const TChar *pSymbolName );
  void  xReadSvlc    (                 Int& val, const TChar *pSymbolName );
  void  xReadFlag    (                UInt& val, const TChar *pSymbolName );
#else
  void  xReadCode    ( UInt   length, UInt& val );
  void  xReadUvlc    (                UInt& val );
  void  xReadSvlc    (                 Int& val );
  void  xReadFlag    (                UInt& val );
#endif
#if ENABLE_TRACING
  void  xReadCodeTr  ( UInt  length, UInt& rValue, const TChar *pSymbolName );
  void  xReadUvlcTr  (               UInt& rValue, const TChar *pSymbolName );
  void  xReadSvlcTr  (                Int& rValue, const TChar *pSymbolName );
  void  xReadFlagTr  (               UInt& rValue, const TChar *pSymbolName );
#endif
public:
  void  setBitstream ( InputBitstream* p )   { m_pcBitstream = p; }
  InputBitstream* getBitstream() { return m_pcBitstream; }

protected:
  void xReadRbspTrailingBits();
};



class AUDReader: public VLCReader
{
public:
  AUDReader() {};
  virtual ~AUDReader() {};
  void parseAccessUnitDelimiter(InputBitstream* bs, UInt &picType);
};



class FDReader: public VLCReader
{
public:
  FDReader() {};
  virtual ~FDReader() {};
  void parseFillerData(InputBitstream* bs, UInt &fdSize);
};



class HLSyntaxReader : public VLCReader
{
public:
  HLSyntaxReader();
  virtual ~HLSyntaxReader();

#if JEM_TOOLS
  void  init( CABACDataStore& cabacDataStore ) { m_CABACDataStore = &cabacDataStore; }
#endif
protected:
  void  parseShortTermRefPicSet            (SPS* pcSPS, ReferencePictureSet* pcRPS, Int idx);

public:
  void  setBitstream        ( InputBitstream* p )   { m_pcBitstream = p; }
#if HEVC_VPS
  void  parseVPS            ( VPS* pcVPS );
#endif
  void  parseSPSNext        ( SPSNext& spsNext, const bool usePCM );
  void  parseSPS            ( SPS* pcSPS );
  void  parsePPS            ( PPS* pcPPS );
  void  parseVUI            ( VUI* pcVUI, SPS* pcSPS );
  void  parsePTL            ( PTL *rpcPTL, bool profilePresentFlag, Int maxNumSubLayersMinus1 );
  void  parseProfileTier    ( ProfileTierLevel *ptl, const bool bIsSubLayer );
  void  parseHrdParameters  ( HRD *hrd, bool cprms_present_flag, UInt tempLevelHigh );
  void  parseSliceHeader    ( Slice* pcSlice, ParameterSetManager *parameterSetManager, const Int prevTid0POC );
  void  parseTerminatingBit ( UInt& ruiBit );
  void  parseRemainingBytes ( bool noTrailingBytesExpected );

  void  parsePredWeightTable( Slice* pcSlice, const SPS *sps );
#if HEVC_USE_SCALING_LISTS
  void  parseScalingList    ( ScalingList* scalingList );
  void  decodeScalingList   ( ScalingList *scalingList, UInt sizeId, UInt listId);
#endif

#if JVET_K0371_ALF
  void alf( AlfSliceParam& alfSliceParam );
  void alfFilter( AlfSliceParam& alfSliceParam, const bool isChroma );

private:
  int truncatedUnaryEqProb( const int maxSymbol );
  void xReadTruncBinCode( UInt& ruiSymbol, const int uiMaxSymbol );
  int  alfGolombDecode( const int k );
#endif

protected:
  bool  xMoreRbspData();
#if JEM_TOOLS
  void  xParseCABACWSizes   ( Slice* pcSlice, const SPS* pcSPS );
#endif
#if JEM_TOOLS
private:
  CABACDataStore* m_CABACDataStore;
#endif
};


//! \}

#endif
