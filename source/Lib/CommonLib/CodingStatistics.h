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

#ifndef __CODINGSTATISTICS__
#define __CODINGSTATISTICS__

#include "CommonDef.h"
#include <stdio.h>
#include <string>
#include <map>
#include <math.h>
#include "ChromaFormat.h"

static const Int64 CODINGSTATISTICS_ENTROPYSCALE = 32768;


enum CodingStatisticsType
{
  STATS__NAL_UNIT_TOTAL_BODY,// This is a special case and is not included in the total sums.
  STATS__NAL_UNIT_PACKING,
  STATS__EMULATION_PREVENTION_3_BYTES,
  STATS__NAL_UNIT_HEADER_BITS,
  STATS__CABAC_INITIALISATION,
  STATS__CABAC_BITS__TQ_BYPASS_FLAG,
  STATS__CABAC_BITS__SKIP_FLAG,
  STATS__CABAC_BITS__MERGE_FLAG,
  STATS__CABAC_BITS__MERGE_INDEX,
  STATS__CABAC_BITS__MVP_IDX,
  STATS__CABAC_BITS__SPLIT_FLAG,
  STATS__CABAC_BITS__PART_SIZE,
  STATS__CABAC_BITS__PRED_MODE,
  STATS__CABAC_BITS__INTRA_DIR_ANG,
  STATS__CABAC_BITS__INTRA_PDPC_FLAG,
  STATS__CABAC_BITS__INTER_DIR,
  STATS__CABAC_BITS__REF_FRM_IDX,
  STATS__CABAC_BITS__MVD,
  STATS__CABAC_BITS__MVD_EP,
#if JEM_TOOLS || JVET_K_AFFINE
  STATS__CABAC_BITS__AFFINE_FLAG,
#if JVET_K0337_AFFINE_6PARA
  STATS__CABAC_BITS__AFFINE_TYPE,
#endif
#endif
  STATS__CABAC_BITS__TRANSFORM_SUBDIV_FLAG,
  STATS__CABAC_BITS__QT_ROOT_CBF,
  STATS__CABAC_BITS__DELTA_QP_EP,
  STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT,
#if JEM_TOOLS
  STATS__CABAC_BITS__LIC_FLAG,
#endif
  STATS__CABAC_BITS__QT_CBF,
  STATS__CABAC_BITS__CROSS_COMPONENT_PREDICTION,
  STATS__CABAC_BITS__TRANSFORM_SKIP_FLAGS,

  STATS__CABAC_BITS__LAST_SIG_X_Y,
  STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,
  STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,
#if JVET_K0072
  STATS__CABAC_BITS__PAR_FLAG,
#endif
  STATS__CABAC_BITS__GT1_FLAG,
  STATS__CABAC_BITS__GT2_FLAG,
  STATS__CABAC_BITS__SIGN_BIT,
  STATS__CABAC_BITS__ESCAPE_BITS,
  STATS__CABAC_BITS__SAO,
#if JEM_TOOLS
  STATS__CABAC_BITS__ALF,
  STATS__CABAC_BITS__NSST,
#endif
  STATS__CABAC_TRM_BITS,
  STATS__CABAC_FIXED_BITS,
  STATS__CABAC_PCM_ALIGN_BITS,
  STATS__CABAC_PCM_CODE_BITS,
  STATS__BYTE_ALIGNMENT_BITS,
  STATS__TRAILING_BITS,
  STATS__EXPLICIT_RDPCM_BITS,
  STATS__CABAC_EP_BIT_ALIGNMENT,
  STATS__CABAC_BITS__ALIGNED_SIGN_BIT,
  STATS__CABAC_BITS__ALIGNED_ESCAPE_BITS,
#if JEM_TOOLS
  STATS__CABAC_BITS__OBMC_FLAG,
#endif
#if JVET_K0357_AMVR
  STATS__CABAC_BITS__IMV_FLAG,
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  STATS__CABAC_BITS__EMT_CU_FLAG,
  STATS__CABAC_BITS__EMT_TU_INDEX,
#endif
  STATS__CABAC_BITS__OTHER,
  STATS__CABAC_BITS__INVALID,
  STATS__NUM_STATS
};

static inline const TChar* getName(CodingStatisticsType name)
{
  static const TChar *statNames[]=
  {
    "NAL_UNIT_TOTAL_BODY", // This is a special case and is not included in the total sums.
    "NAL_UNIT_PACKING",
    "EMULATION_PREVENTION_3_BYTES",
    "NAL_UNIT_HEADER_BITS",
    "CABAC_INITIALISATION-and-rounding",
    "CABAC_BITS__TQ_BYPASS_FLAG",
    "CABAC_BITS__SKIP_FLAG",
    "CABAC_BITS__MERGE_FLAG",
    "CABAC_BITS__MERGE_INDEX",
    "CABAC_BITS__MVP_IDX",
    "CABAC_BITS__SPLIT_FLAG",
    "CABAC_BITS__PART_SIZE",
    "CABAC_BITS__PRED_MODE",
    "CABAC_BITS__INTRA_DIR_ANG",
    "CABAC_BITS__INTRA_PDPC_FLAG",
    "CABAC_BITS__INTER_DIR",
    "CABAC_BITS__REF_FRM_IDX",
    "CABAC_BITS__MVD",
    "CABAC_BITS__MVD_EP",
#if JEM_TOOLS || JVET_K_AFFINE
    "CABAC_BITS__AFFINE_FLAG",
#if JVET_K0337_AFFINE_6PARA
    "CABAC_BITS__AFFINE_TYPE",
#endif
#endif
    "CABAC_BITS__TRANSFORM_SUBDIV_FLAG",
    "CABAC_BITS__QT_ROOT_CBF",
    "CABAC_BITS__DELTA_QP_EP",
    "CABAC_BITS__CHROMA_QP_ADJUSTMENT",
#if JEM_TOOLS
    "CABAC_BITS__LIC_FLAG",
#endif
    "CABAC_BITS__QT_CBF",
    "CABAC_BITS__CROSS_COMPONENT_PREDICTION",
    "CABAC_BITS__TRANSFORM_SKIP_FLAGS",
    "CABAC_BITS__LAST_SIG_X_Y",
    "CABAC_BITS__SIG_COEFF_GROUP_FLAG",
    "CABAC_BITS__SIG_COEFF_MAP_FLAG",
#if JVET_K0072
    "CABAC_BITS__PAR_FLAG",
#endif
    "CABAC_BITS__GT1_FLAG",
    "CABAC_BITS__GT2_FLAG",
    "CABAC_BITS__SIGN_BIT",
    "CABAC_BITS__ESCAPE_BITS",
    "CABAC_BITS__SAO",
#if JEM_TOOLS
    "CABAC_BITS__ALF",
    "CABAC_BITS__NSST",

#endif
    "CABAC_TRM_BITS",
    "CABAC_FIXED_BITS",
    "CABAC_PCM_ALIGN_BITS",
    "CABAC_PCM_CODE_BITS",
    "BYTE_ALIGNMENT_BITS",
    "TRAILING_BITS",
    "EXPLICIT_RDPCM_BITS",
    "CABAC_EP_BIT_ALIGNMENT",
    "CABAC_BITS__ALIGNED_SIGN_BIT",
    "CABAC_BITS__ALIGNED_ESCAPE_BITS",
#if JEM_TOOLS
    "CABAC_BITS__OBMC_FLAG",
#endif
#if JVET_K0357_AMVR
    "CABAC_BITS__IMV_FLAG",
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
    "CABAC_BITS__EMT_CU_FLAG",
    "CABAC_BITS__EMT_TU_INDX",
#endif
    "CABAC_BITS__OTHER",
    "CABAC_BITS__INVALID"
  };
  CHECK( STATS__NUM_STATS != sizeof( statNames ) / sizeof( TChar* ) || name >= STATS__NUM_STATS, "stats out of range" );
  return statNames[name];
}

static inline bool isAlignedBins( CodingStatisticsType statT ) { return statT == STATS__CABAC_BITS__ALIGNED_SIGN_BIT || statT == STATS__CABAC_BITS__ALIGNED_ESCAPE_BITS; }

static const UInt CODING_STATS_NUM_WIDTHS     = 20; // just define the number of widths and heigts as 15
static const UInt CODING_STATS_NUM_HEIGHTS    = 20;
static const UInt CODING_STATS_NUM_SIZES      = CODING_STATS_NUM_HEIGHTS * CODING_STATS_NUM_WIDTHS;
static const UInt CODING_STATS_NUM_SUBCLASSES = CODING_STATS_NUM_SIZES * (1 + MAX_NUM_COMPONENT + MAX_NUM_CHANNEL_TYPE);

class CodingStatisticsClassType
{
public:

  CodingStatisticsClassType( const CodingStatisticsType t ) : type( t ), subClass( 0 )
  {
  }

  CodingStatisticsClassType( const CodingStatisticsType t, const UInt width, const UInt height ) : type( t ), subClass( gp_sizeIdxInfo->idxFrom( height ) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom( width ) )
  {
  }

  CodingStatisticsClassType( const CodingStatisticsType t, const Int width, const Int height ) : type( t ), subClass( gp_sizeIdxInfo->idxFrom( height ) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom( width ) )
  {
  }

  CodingStatisticsClassType( const CodingStatisticsType t, const ComponentID cid ) : type( t ), subClass( ( cid + 1 ) * CODING_STATS_NUM_SIZES )
  {
  }

  CodingStatisticsClassType( const CodingStatisticsType t, const ChannelType chid ) : type( t ), subClass( ( chid + MAX_NUM_COMPONENT + 1 ) * CODING_STATS_NUM_SIZES )
  {
  }

  CodingStatisticsClassType( const CodingStatisticsType t, const UInt width, const UInt height, const ComponentID cid ) : type( t ), subClass( ( cid + 1 ) * CODING_STATS_NUM_SIZES + gp_sizeIdxInfo->idxFrom( height ) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom( width ) )
  {
  }

  CodingStatisticsClassType( const CodingStatisticsType t, const UInt width, const UInt height, const ChannelType chid ) : type( t ), subClass( ( chid + MAX_NUM_COMPONENT + 1 ) * CODING_STATS_NUM_SIZES + gp_sizeIdxInfo->idxFrom( height ) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom( width ) )
  {
  }

  ~CodingStatisticsClassType()
  {
    // should not be relevant, but it looks like some instances are used after they are destroyed
    type = STATS__CABAC_BITS__INVALID;
    subClass = 0;
  }

  static UInt GetSubClassWidth( const UInt subClass )
  {
    return subClass % CODING_STATS_NUM_WIDTHS;
  }

  static UInt GetSubClassHeight( const UInt subClass )
  {
    return ( subClass % CODING_STATS_NUM_SIZES ) / CODING_STATS_NUM_WIDTHS;
  }

  static const TChar *GetSubClassString( const UInt subClass )
  {
    CHECK( subClass >= CODING_STATS_NUM_SUBCLASSES, "Subclass does not exist" );
    static const TChar *strings[1 + MAX_NUM_COMPONENT + MAX_NUM_CHANNEL_TYPE] = { "-", "Y", "Cb", "Cr", "Luma", "Chroma" };
    return strings[subClass / CODING_STATS_NUM_SIZES];
  }

  CodingStatisticsType type;
  UInt subClass;
};



class CodingStatistics
{
public:

  struct StatLogValue
  {
    UInt values[512 + 1];
    StatLogValue()
    {
      const Double es = Double( CODINGSTATISTICS_ENTROPYSCALE );

      values[0] = 0;

      for( UInt i = 1; i < sizeof( values ) / sizeof( UInt ); i++ )
      {
        values[i] = UInt( log( Double( i ) )*es / log( 2.0 ) );
      }
    }
  };

  struct SStat
  {
    SStat() : bits( 0 ), count( 0 ), sum( 0 ), classCount( 0 ) { }

    Int64 bits;
    Int64 count;
    Int64 sum;
    Int64 classCount;

    void clear() { bits = 0; count = 0; sum = 0; classCount = 0; }

    SStat &operator+=( const SStat &src )
    {
      bits += src.bits; count += src.count; sum += src.sum; classCount += src.classCount; return *this;
    }
  };

  class CodingStatisticsData
  {
  private:
    SStat statistics    [STATS__NUM_STATS + 1][CODING_STATS_NUM_SUBCLASSES];
    SStat statistics_ep [STATS__NUM_STATS + 1][CODING_STATS_NUM_SUBCLASSES];
    std::map<std::string, SStat> mappings_ep;
    friend class CodingStatistics;
  };

private:

  CodingStatisticsData data;

  CodingStatistics() : data()
  {
  }

  static void OutputLine( const TChar *pName, const TChar sep, UInt wIdx, UInt hIdx, const TChar *pSubClassStr, const SStat &sCABAC, const SStat &sEP )
  {
    if( wIdx == 0 && hIdx == 0 )
    {
      OutputLine( pName, sep, "-", "-", pSubClassStr, sCABAC, sEP );
    }
    else
    {
      printf( "%c%-45s%c  %6d %6d %6s ", sep == '~' ? '[' : ' ', pName, sep, gp_sizeIdxInfo->sizeFrom( wIdx ), gp_sizeIdxInfo->sizeFrom( hIdx ), pSubClassStr );
      if( sCABAC.count > 0 )
      {
        const Double quote = 100.0 * sCABAC.count / ( Double ) sCABAC.classCount;
        const Double ratio = 100.0 * sCABAC.bits / ( Double ) sCABAC.count;
        printf( "%11.2f%% %12lld %12lld %12lld %11.2f%%", quote, sCABAC.count, sCABAC.sum, sCABAC.bits, ratio );
      }
      else
      {
        printf( "         -/- %12lld %12lld %12lld          -/-", sCABAC.count, sCABAC.sum, sCABAC.bits );
      }
      printf( " %12lld %12lld %12lld %12lld (%12lld)%c\n", sEP.count, sEP.sum, sEP.bits, sCABAC.bits + sEP.bits, ( sCABAC.bits + sEP.bits ) / 8, sep == '~' ? ']' : ' ' );
    }
  }
  static void OutputLine( const TChar *pName, const TChar sep, const TChar *pWidthString, const TChar *pHeightString, const TChar *pSubClassStr, const SStat &sCABAC, const SStat &sEP )
  {
    printf( "%c%-45s%c  %6s %6s %6s ", sep == '~' ? '[' : ' ', pName, sep, pWidthString, pHeightString, pSubClassStr );
    if( sCABAC.count > 0 )
    {
      const Double quote = 100.0 * sCABAC.count / ( Double ) sCABAC.classCount;
      const Double ratio = 100.0 * sCABAC.bits / ( Double ) sCABAC.count;
      printf( "%11.2f%% %12lld %12lld %12lld %11.2f%%", quote, sCABAC.count, sCABAC.sum, sCABAC.bits, ratio );
    }
    else
    {
      printf( "         -/- %12lld %12lld %12lld          -/-", sCABAC.count, sCABAC.sum, sCABAC.bits );
    }
    printf( " %12lld %12lld %12lld %12lld (%12lld)%c\n", sEP.count, sEP.sum, sEP.bits, sCABAC.bits + sEP.bits, ( sCABAC.bits + sEP.bits ) / 8, sep == '~' ? ']' : ' ' );
  }
  static void OutputLine( const TChar *pName, const TChar sep, const TChar *pWidthString, const TChar *pHeightString, const TChar *pSubClassStr, const SStat &sEP )
  {
    printf( "%c%-45s%c  %6s %6s %6s          -/- %12s %12s %12s %9s-/- %12lld %12lld %12lld %12lld (%12lld)%c\n",
            sep == '~' ? '[' : ' ', pName, sep, pWidthString, pHeightString, pSubClassStr,
            "", "", "", "", sEP.count, sEP.sum, sEP.bits, sEP.bits, ( sEP.bits ) / 8, sep == '~' ? ']' : ' ' );
  }

  static void OutputDashedLine( const TChar *pText )
  {
    printf( "--%s", pText );
    UInt tot = 0;
    for( ; pText[tot] != 0; tot++ );

    tot += 2;
    for( ; tot < 202; tot++ )
    {
      printf( "-" );
    }
    printf( "\n" );
  }

public:

  ~CodingStatistics()
  {
    const Int64 es = CODINGSTATISTICS_ENTROPYSCALE;

    Int64 countTotal = 0;
    Int64 classCounts[STATS__NUM_STATS];
    std::fill_n( classCounts, ( size_t ) STATS__NUM_STATS, 0 );

    Int64 cr = 0; // CABAC remainder, which is added to "STATS__CABAC_INITIALISATION"
    {
      Int64 totalCABACbits = 0, roundedCABACbits = 0;
      for( Int i = STATS__NAL_UNIT_PACKING; i < STATS__NUM_STATS; i++ )
      {
        Int64 classCount = 0;

        for( UInt c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++ )
        {
          totalCABACbits    += data.statistics[i][c].bits;
          roundedCABACbits  += data.statistics[i][c].bits / es;
          classCount        += data.statistics[i][c].count;
        }

        for( UInt c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++ )
        {
          data.statistics[i][c].classCount = classCount;
        }

        classCounts[i] = classCount;
        countTotal    += classCount;
      }
      Int64 remainder = totalCABACbits - roundedCABACbits * es;
      cr = ( remainder + es / 2 ) / es;
    }

    classCounts[0] = countTotal;

    printf( "Note %s will be excluded from the total as it should be the sum of all the other entries (except for %s)\n", getName( STATS__NAL_UNIT_TOTAL_BODY ), getName( STATS__NAL_UNIT_PACKING ) );
    printf( " %-45s-   Width Height   Type  CABAC quote  CABAC count    CABAC Sum   CABAC bits  CABAC ratio     EP Count       EP Sum      EP bits   Total bits ( Total bytes)\n", "Decoder statistics" );

    OutputDashedLine( "" );
    SStat cabacTotalBits, epTotalBits;

    cabacTotalBits.classCount = countTotal;
    epTotalBits   .classCount = countTotal;

    SStat statTotals_cabac[CODING_STATS_NUM_SUBCLASSES];
    SStat statTotals_ep   [CODING_STATS_NUM_SUBCLASSES];

    for( Int i = 0; i < STATS__NUM_STATS; i++ )
    {
      SStat cabacSubTotal, epSubTotal;
      cabacSubTotal.classCount = classCounts[i];
      epSubTotal   .classCount = classCounts[i];
      bool bHadClassifiedEntry = false;

      const TChar *pName = getName( CodingStatisticsType( i ) );

      for( UInt c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++ )
      {
        SStat &sCABACorig = data.statistics[i][c];
        SStat &sEP        = data.statistics_ep[i][c];

        if( sCABACorig.bits == 0 && sEP.bits == 0 )
        {
          continue;
        }

        SStat sCABAC;
        {
          Int64 thisCABACbits = sCABACorig.bits / es;
          if( i == STATS__CABAC_INITIALISATION && sCABACorig.bits != 0 )
          {
            thisCABACbits += cr;
            cr = 0;
          }
          sCABAC.bits       = thisCABACbits;
          sCABAC.count      = sCABACorig.count;
          sCABAC.sum        = sCABACorig.sum;
          sCABAC.classCount = classCounts[i];
        }
        UInt wIdx = CodingStatisticsClassType::GetSubClassWidth( c );
        UInt hIdx = CodingStatisticsClassType::GetSubClassHeight( c );
        OutputLine( pName, ':', wIdx, hIdx, CodingStatisticsClassType::GetSubClassString( c ), sCABAC, sEP );
        cabacSubTotal += sCABAC;
        epSubTotal    += sEP;

        if( i != STATS__NAL_UNIT_TOTAL_BODY )
        {
          cabacTotalBits      += sCABAC;
          epTotalBits         += sEP;
          statTotals_cabac[c] += sCABAC;
          statTotals_ep[c]    += sEP;

        }
        bHadClassifiedEntry = bHadClassifiedEntry || ( c != 0 );
      }

      if( bHadClassifiedEntry )
      {
        cabacSubTotal.classCount = classCounts[i];
        OutputLine( pName, '~', "~~ST~~", "~~ST~~", "~~ST~~", cabacSubTotal, epSubTotal );
      }
      if( i == STATS__NAL_UNIT_TOTAL_BODY )
      {
        OutputDashedLine( "" );
      }
    }
    OutputDashedLine( "" );
    OutputLine( "CABAC Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cabacTotalBits, epTotalBits );

    OutputDashedLine( "CAVLC HEADER BITS" );
    SStat cavlcTotalBits;
    for( std::map<std::string, SStat>::iterator it = data.mappings_ep.begin(); it != data.mappings_ep.end(); it++ )
    {
      SStat s = it->second;
      cavlcTotalBits += s;
      OutputLine( it->first.c_str(), ':', "-", "-", "-", s );
    }

    OutputDashedLine( "" );
    OutputLine( "CAVLC Header Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cavlcTotalBits );

    // Now output the breakdowns
    OutputDashedLine( "CABAC Break down by size" );
    for( UInt w = 0; w < CODING_STATS_NUM_WIDTHS; w++ )
    {
      for( UInt h = 0; h < CODING_STATS_NUM_HEIGHTS; h++ )
      {
        SStat subTotalCabac, subTotalEP;
        for( UInt c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES )
        {
          subTotalCabac += statTotals_cabac[c + h * CODING_STATS_NUM_WIDTHS + w];
          subTotalEP    += statTotals_ep   [c + h * CODING_STATS_NUM_WIDTHS + w];
        }
        if( subTotalCabac.bits != 0 || subTotalEP.bits != 0 )
        {
          OutputLine( "CABAC by size Sub-total", '=', w, h, "All", subTotalCabac, subTotalEP );
        }
      }
    }
    OutputDashedLine( "Break down by component/Channel type" );
    for( UInt c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES )
    {
      SStat subTotalCabac, subTotalEP;
      for( UInt w = 0; w < CODING_STATS_NUM_WIDTHS; w++ )
      {
        for( UInt h = 0; h < CODING_STATS_NUM_HEIGHTS; h++ )
        {
          subTotalCabac += statTotals_cabac[c + h * CODING_STATS_NUM_WIDTHS + w];
          subTotalEP    += statTotals_ep   [c + h * CODING_STATS_NUM_WIDTHS + w];
        }
      }
      if( subTotalCabac.bits != 0 || subTotalEP.bits != 0 )
      {
        OutputLine( "CABAC by type Sub-total", '=', "-", "-", CodingStatisticsClassType::GetSubClassString( c ), subTotalCabac, subTotalEP );
      }
    }
    OutputDashedLine( "Break down by size and component/Channel type" );
    for( UInt c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES )
    {
      for( UInt w = 0; w < CODING_STATS_NUM_WIDTHS; w++ )
      {
        for( UInt h = 0; h < CODING_STATS_NUM_HEIGHTS; h++ )
        {
          SStat subTotalCabac, subTotalEP;
          subTotalCabac += statTotals_cabac[c + h * CODING_STATS_NUM_WIDTHS + w];
          subTotalEP    += statTotals_ep   [c + h * CODING_STATS_NUM_WIDTHS + w];

          if( subTotalCabac.bits != 0 || subTotalEP.bits != 0 )
          {
            OutputLine( "CABAC by size and type Sub-total", '=', w, h, CodingStatisticsClassType::GetSubClassString( c ), subTotalCabac, subTotalEP );
          }
        }
      }
    }

    OutputDashedLine( "" );
    cabacTotalBits.classCount = countTotal;
    OutputLine      ( "CABAC Sub-total",        '~', "~~ST~~", "~~ST~~", "~~ST~~", cabacTotalBits, epTotalBits );
    OutputLine      ( "CAVLC Header Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cavlcTotalBits );
    OutputDashedLine( "GRAND TOTAL" );
    epTotalBits += cavlcTotalBits;
    OutputLine      ( "TOTAL",                  '~', "~~GT~~", "~~GT~~", "~~GT~~", cabacTotalBits, epTotalBits );
  }

  static CodingStatistics& GetSingletonInstance()
  {
    static CodingStatistics* inst = nullptr;
    if( !inst )
    {
      inst = new CodingStatistics;
    }

    return *inst;
  }

  static void DestroyInstance()
  {
    CodingStatistics* cs = &GetSingletonInstance();
    delete cs;
  }

  static const CodingStatisticsData &GetStatistics()                        { return GetSingletonInstance().data; }

  static void SetStatistics       ( const CodingStatisticsData &src )       { GetSingletonInstance().data = src; }

  static SStat &GetStatisticEP    ( const CodingStatisticsClassType &stat ) { return GetSingletonInstance().data.statistics_ep[stat.type][stat.subClass]; }

  static SStat &GetStatisticEP    ( const std::string &str )                { return GetSingletonInstance().data.mappings_ep[str]; }

  static SStat &GetStatisticEP    ( const TChar *pKey )                     { return GetStatisticEP( std::string( pKey ) ); }

  static int getNumOnes( Int bins )
  {
    CHECK( bins < 0, "Bins should not be nagative" );

    int count = 0;
    while( bins )
    {
      count += bins & 1;
      bins >>= 1;
    }
    return count;
  }

  static void IncrementStatisticEP( const CodingStatisticsClassType &stat, const Int numBits, const Int value )
  {
    CHECK( stat.type == STATS__CABAC_BITS__INVALID, "Should never be used." );
    SStat &s = GetStatisticEP( stat );
    s.bits  += numBits;
    s.count++;
    s.sum   += getNumOnes( value );
  }

  static void IncrementStatisticEP( const std::string &str, const Int numBits, const Int value )
  {
    SStat &s = GetStatisticEP( str );
    s.bits  += numBits;
    s.count++;
    s.sum   += getNumOnes( value );
  }

  static void IncrementStatisticEP( const TChar *pKey, const Int numBits, const Int value )
  {
    SStat &s = GetStatisticEP( pKey );
    s.bits  += numBits;
    s.count++;
    s.sum   += getNumOnes( value );
  }

  StatLogValue values;

  static void UpdateCABACStat( const CodingStatisticsClassType &stat, UInt uiRangeBefore, UInt uiRangeAfter, Int val )
  {
    CHECK( stat.type == STATS__CABAC_BITS__INVALID, "Should never be used." );
    CodingStatistics &inst = GetSingletonInstance();
    // doing rangeBefore*p(x)=rangeAfter
    // p(x)=rangeAfter/rangeBefore
    // entropy = -log2(p(x))=-log(p(x))/log(2) = -(log rangeAfter - log rangeBefore) / log(2) = (log rangeBefore / log 2 - log rangeAfter / log 2)
    SStat &s = inst.data.statistics[stat.type][stat.subClass];
    s.bits  += inst.values.values[uiRangeBefore] - inst.values.values[uiRangeAfter];
    s.count++;
    s.sum   += val;
  }
};

#endif
