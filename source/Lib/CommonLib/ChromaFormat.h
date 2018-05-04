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

#ifndef __CHROMAFORMAT__
#define __CHROMAFORMAT__

#include "Common.h"
#include "CommonDef.h"
#include "Rom.h"

#include <iostream>
#include <vector>

//======================================================================================================================
//Chroma format utility functions  =====================================================================================
//======================================================================================================================


static inline ChannelType toChannelType             (const ComponentID id)                         { return (id==COMPONENT_Y)? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA; }
static inline Bool        isLuma                    (const ComponentID id)                         { return (id==COMPONENT_Y);                                          }
static inline Bool        isLuma                    (const ChannelType id)                         { return (id==CHANNEL_TYPE_LUMA);                                    }
static inline Bool        isChroma                  (const ComponentID id)                         { return (id!=COMPONENT_Y);                                          }
static inline Bool        isChroma                  (const ChannelType id)                         { return (id!=CHANNEL_TYPE_LUMA);                                    }
static inline UInt        getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt==CHROMA_444)) ? 0 : 1;                  }
static inline UInt        getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt!=CHROMA_420)) ? 0 : 1;                  }
static inline UInt        getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);               }
static inline UInt        getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);               }
static inline UInt        getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMPONENT;                  }
static inline UInt        getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CHANNEL_TYPE;               }
static inline Bool        isChromaEnabled           (const ChromaFormat fmt)                       { return !(fmt==CHROMA_400);                                         }
static inline ComponentID getFirstComponentOfChannel(const ChannelType id)                         { return (isLuma(id) ? COMPONENT_Y : COMPONENT_Cb);                  }

InputColourSpaceConversion stringToInputColourSpaceConvert(const std::string &value, const Bool bIsForward);
std::string getListOfColourSpaceConverts(const Bool bIsForward);

//------------------------------------------------

static inline UInt getTotalSamples(const UInt width, const UInt height, const ChromaFormat format)
{
  const UInt samplesPerChannel = width * height;

  switch (format)
  {
    case CHROMA_400: return  samplesPerChannel;           break;
    case CHROMA_420: return (samplesPerChannel * 3) >> 1; break;
    case CHROMA_422: return  samplesPerChannel * 2;       break;
    case CHROMA_444: return  samplesPerChannel * 3;       break;
    default:
      EXIT( "ERROR: Unrecognised chroma format in getTotalSamples() " );
      break;
  }

  return MAX_UINT;
}

//------------------------------------------------

static inline uint64_t getTotalFracBits(const UInt width, const UInt height, const ChromaFormat format, const Int bitDepths[MAX_NUM_CHANNEL_TYPE])
{
  unsigned bitsPerSampleTimes2 = MAX_UINT;
  switch (format)
  {
  case CHROMA_400: bitsPerSampleTimes2 =   2 *  bitDepths[CHANNEL_TYPE_LUMA];                                              break;
  case CHROMA_420: bitsPerSampleTimes2 = ( 2 * (bitDepths[CHANNEL_TYPE_LUMA]*2 +   bitDepths[CHANNEL_TYPE_CHROMA]) ) >> 1; break;
  case CHROMA_422: bitsPerSampleTimes2 =   2 * (bitDepths[CHANNEL_TYPE_LUMA]   +   bitDepths[CHANNEL_TYPE_CHROMA]);        break;
  case CHROMA_444: bitsPerSampleTimes2 =   2 * (bitDepths[CHANNEL_TYPE_LUMA]   + 2*bitDepths[CHANNEL_TYPE_CHROMA]);        break;
  default:
      EXIT( "ERROR: Unrecognised chroma format in getTotalFracBits() " );
    break;
  }
  return uint64_t( width * height * bitsPerSampleTimes2 ) << ( SCALE_BITS - 1 );
}

#if HEVC_USE_PART_SIZE
//------------------------------------------------

// In HM, a CU only has one chroma intra prediction direction, that corresponds to the top left luma intra prediction
// even if the NxN PU split occurs when 4 sub-TUs exist for chroma.
// Use this function to allow NxN PU splitting for chroma.

static inline Bool enable4ChromaPUsInIntraNxNCU(const ChromaFormat chFmt)
{
  return (chFmt == CHROMA_444);
}

//------------------------------------------------

//returns the part index of the luma region that is co-located with the specified chroma region

static inline UInt
getChromasCorrespondingPULumaIdx(const UInt lumaZOrderIdxInCtu,
                                 const ChromaFormat chFmt,
                                 const Int partsPerMinCU  // 1<<(2*(sps->getMaxCodingDepth() - sps->getLog2DiffMaxMinCodingBlockSize()))
                                 )
{
  return enable4ChromaPUsInIntraNxNCU(chFmt) ? lumaZOrderIdxInCtu : lumaZOrderIdxInCtu & (~(partsPerMinCU-1));
}

#endif
#if HEVC_USE_RQT && HEVC_422
//------------------------------------------------

// If chroma format is 4:2:2 and a chroma-square-sub-tu is possible for the smallest TU, then increase the depth by 1 to allow for more parts.

static inline UInt getMaxCUDepthOffset(const ChromaFormat chFmt, const UInt quadtreeTULog2MinSize)
{
  return (chFmt==CHROMA_422 && quadtreeTULog2MinSize>2) ? 1 : 0;
}

#endif

//======================================================================================================================
//Intra prediction  ====================================================================================================
//======================================================================================================================

static inline Bool filterIntraReferenceSamples (const ChannelType chType, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled)
{
  return (!intraReferenceSmoothingDisabled) && (isLuma(chType) || (chFmt == CHROMA_444));
}


//------------------------------------------------

static inline Int getTransformShift(const Int channelBitDepth, const Size size, const Int maxLog2TrDynamicRange)
{
  return maxLog2TrDynamicRange - channelBitDepth - ( ( g_aucLog2[size.width] + g_aucLog2[size.height] ) >> 1 );
}


//------------------------------------------------

static inline Int getScaledChromaQP(Int unscaledChromaQP, const ChromaFormat chFmt)
{
  return g_aucChromaScale[chFmt][Clip3(0, (chromaQPMappingTableSize - 1), unscaledChromaQP)];
}


#if HEVC_USE_SCALING_LISTS
//======================================================================================================================
//Scaling lists  =======================================================================================================
//======================================================================================================================

static inline Int getScalingListType(const PredMode predMode, const ComponentID compID)
{
  return ((predMode != MODE_INTER) ? 0 : MAX_NUM_COMPONENT) + MAP_CHROMA(compID);
}
#endif

#endif
