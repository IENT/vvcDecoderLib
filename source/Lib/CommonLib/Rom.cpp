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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"
#if JEM_TOOLS
#include "CommonLib/BilateralFilter.h"
#endif

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

#if ENABLE_TRACING
CDTrace *g_trace_ctx = NULL;
#endif


//! \ingroup CommonLib
//! \{

MsgLevel g_verbosity = VERBOSE;

const TChar* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
#if HEVC_VPS
  case NAL_UNIT_VPS:                    return "VPS";
#endif
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
  }
}

class ScanGenerator
{
private:
  UInt m_line, m_column;
  const UInt m_blockWidth, m_blockHeight;
  const UInt m_stride;
  const CoeffScanType m_scanType;

public:
  ScanGenerator(UInt blockWidth, UInt blockHeight, UInt stride, CoeffScanType scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  UInt GetCurrentX() const { return m_column; }
  UInt GetCurrentY() const { return m_line; }

  UInt GetNextIndex(UInt blockOffsetX, UInt blockOffsetY)
  {
    const UInt rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:

        if ((m_column == m_blockWidth - 1) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
        {
          m_line += m_column + 1;
          m_column = 0;

          if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
          {
            m_column += m_line - (m_blockHeight - 1);
            m_line = m_blockHeight - 1;
          }
        }
        else
        {
          m_column++;
          m_line--;
        }
        break;

#if HEVC_USE_MDCS
      //------------------------------------------------
      case SCAN_HOR:

        if (m_column == m_blockWidth - 1)
        {
          m_line++;
          m_column = 0;
        }
        else
        {
          m_column++;
        }
        break;

      //------------------------------------------------

      case SCAN_VER:

        if (m_line == m_blockHeight - 1)
        {
          m_column++;
          m_line = 0;
        }
        else
        {
          m_line++;
        }
        break;

#endif
      //------------------------------------------------

      default:

        THROW("ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex");
        break;
    }

    return rtn;
  }
};

#if JEM_TOOLS
Int g_aiLMDivTableLow[] = {
  0, 0, 21845, 0, 13107, 43690, 18724, 0, 50972, 39321, 53620, 21845, 15123, 9362, 4369, 0, 3855, 58254, 17246, 52428, 49932, 59578, 25644, 43690, 28835, 40329, 16990, 37449, 56496, 34952, 4228, 0, 61564, 34695, 29959, 29127, 15941, 41391, 26886, 26214, 28771, 24966, 6096, 29789, 23301, 45590, 25098, 21845, 30761, 47185, 1285, 20164, 34622, 41263, 36938, 18724, 49439, 61016, 51095, 17476, 23635, 2114, 16644, 0, 16131, 63550, 9781, 50115, 52238, 14979, 2769, 14563, 49376, 40738, 53302, 20695, 7660, 13443, 37330, 13107, 5663, 14385, 38689, 12483, 771, 3048, 18832, 47662, 23563, 11650, 11522, 22795, 45100, 12549, 55878, 43690, 41213, 48148, 64212, 23592, 57100, 33410, 17815, 10082, 9986, 17311, 31849, 53399, 16233, 51237, 27159, 9362, 63216, 57487, 57557, 63276, 8962, 25547, 47362, 8738, 40621, 11817, 53281, 33825, 18874, 8322, 2064, 0, 2032, 8065, 18009, 31775, 49275, 4890, 29612, 57825, 23918, 58887, 31589, 7489, 52056, 34152, 19248, 7281, 63728, 57456, 53944, 53137, 54979, 59419, 868, 10347, 22273, 36598, 53274, 6721, 27967, 51433, 11540, 39321, 3663, 35599, 4020, 39960, 12312, 52112, 28255, 6241, 51575, 33153, 16479, 1524, 53792, 42184, 32206, 23831, 17031, 11781, 8054, 5825, 5069, 5761, 7878, 11397, 16295, 22550, 30139, 39042, 49238, 60707, 7891, 21845, 37012, 53374, 5377, 24074, 43912, 64874, 21406, 44564, 3260, 28550, 54882, 16705, 45075, 8907, 39258, 5041, 37314, 4993, 39135, 8655, 44613, 15924, 53648, 26699, 604, 40884, 16458, 58386, 35585, 13579, 57895, 37449, 17767, 64376, 46192, 28743, 12019, 61546, 46244, 31638, 17720, 4481, 57448, 45541, 34288, 23681, 13710, 4369, 61185, 53078, 45578, 38676, 32366, 26640, 21491, 16912, 12896, 9437, 6527, 4161, 2331, 1032, 257, 0, 255, 1016, 2277, 4032, 6277, 9004, 12210, 15887, 20031, 24637, 29699, 35213, 41173, 47574, 54411, 61680, 3840, 11959, 20494, 29443, 38801, 48562, 58724, 3744, 14693, 26028, 37746, 49844, 62316, 9624, 22834, 36408, 50342, 64632, 13737, 28728, 44063, 59740, 10219, 26568, 43249, 60257, 12055, 29709, 47682, 434, 19033, 37941, 57155, 11136, 30953, 51067, 5938, 26637, 47624, 3360, 24916, 46751, 3328, 25716, 48376, 5770, 28967, 52428, 10616, 34599, 58840, 17799, 42547, 2010, 27256, 52748, 12947, 38924, 65140, 26056, 52743, 14127, 41277, 3120, 30726, 58555, 21072, 49344, 12300, 41007, 4394, 33530, 62876, 26896, 56659, 21092, 51264, 16103, 46678, 11915, 42886, 8515, 39875, 5890, 37632, 4027, 36145, 2912, 35400, 2534, 35385, 2880, 36089, 3939, 37500, 5698, 39605, 8147, 42395, 11275, 45857, 15069, 49982, 19521, 54758, 24619, 60175, 30353, 688, 36713, 7357, 43690, 14639, 51274, 22522, 59455, 30999, 2688, 40059, 12037, 49693, 21956, 59894, 32437, 5117, 43471, 16425, 55050, 28273, 1630, 40655, 14275, 53561, 27441, 1449, 41120, 15382, 55305, 29818, 4453, 44748, 19629, 60166, 35288, 10529, 51425, 26902, 2496, 43742, 19567, 61042, 37095, 13261, 55074, 31463, 7962, 50106, 26824, 3649, 46117, 23157, 302, 43088, 20442, 63436, 40997, 18660, 61961, 39826, 17792, 61393, 39557, 17819, 61715, 40171, 18724, 62908, 41651, 20489, 64956, 43980, 23096, 2304, 47139, 26529, 6009, 51115, 30773, 10519, 55890, 35811, 15819, 61448, 41628, 21892, 2240, 48208, 28724, 9322, 55538, 36301, 17144, 63604, 44608, 25692, 6855, 53632, 34952, 16349, 63360, 44911, 26539, 8242, 55557, 37410, 19338, 1340, 48951, 31099, 13320, 61149, 43513, 25949, 8456, 56569, 39216, 21932, 4718, 53109, 36031, 19022, 2080, 50741, 33933, 17191, 516, 49441, 32896, 16416, 0,
};
Int g_aiLMDivTableHigh[] = {
  65536, 32768, 21845, 16384, 13107, 10922, 9362, 8192, 7281, 6553, 5957, 5461, 5041, 4681, 4369, 4096, 3855, 3640, 3449, 3276, 3120, 2978, 2849, 2730, 2621, 2520, 2427, 2340, 2259, 2184, 2114, 2048, 1985, 1927, 1872, 1820, 1771, 1724, 1680, 1638, 1598, 1560, 1524, 1489, 1456, 1424, 1394, 1365, 1337, 1310, 1285, 1260, 1236, 1213, 1191, 1170, 1149, 1129, 1110, 1092, 1074, 1057, 1040, 1024, 1008, 992, 978, 963, 949, 936, 923, 910, 897, 885, 873, 862, 851, 840, 829, 819, 809, 799, 789, 780, 771, 762, 753, 744, 736, 728, 720, 712, 704, 697, 689, 682, 675, 668, 661, 655, 648, 642, 636, 630, 624, 618, 612, 606, 601, 595, 590, 585, 579, 574, 569, 564, 560, 555, 550, 546, 541, 537, 532, 528, 524, 520, 516, 512, 508, 504, 500, 496, 492, 489, 485, 481, 478, 474, 471, 468, 464, 461, 458, 455, 451, 448, 445, 442, 439, 436, 434, 431, 428, 425, 422, 420, 417, 414, 412, 409, 407, 404, 402, 399, 397, 394, 392, 390, 387, 385, 383, 381, 378, 376, 374, 372, 370, 368, 366, 364, 362, 360, 358, 356, 354, 352, 350, 348, 346, 344, 343, 341, 339, 337, 336, 334, 332, 330, 329, 327, 326, 324, 322, 321, 319, 318, 316, 315, 313, 312, 310, 309, 307, 306, 304, 303, 302, 300, 299, 297, 296, 295, 293, 292, 291, 289, 288, 287, 286, 284, 283, 282, 281, 280, 278, 277, 276, 275, 274, 273, 271, 270, 269, 268, 267, 266, 265, 264, 263, 262, 261, 260, 259, 258, 257, 256, 255, 254, 253, 252, 251, 250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 240, 240, 239, 238, 237, 236, 235, 234, 234, 233, 232, 231, 230, 229, 229, 228, 227, 226, 225, 225, 224, 223, 222, 222, 221, 220, 219, 219, 218, 217, 217, 216, 215, 214, 214, 213, 212, 212, 211, 210, 210, 209, 208, 208, 207, 206, 206, 205, 204, 204, 203, 202, 202, 201, 201, 200, 199, 199, 198, 197, 197, 196, 196, 195, 195, 194, 193, 193, 192, 192, 191, 191, 190, 189, 189, 188, 188, 187, 187, 186, 186, 185, 185, 184, 184, 183, 183, 182, 182, 181, 181, 180, 180, 179, 179, 178, 178, 177, 177, 176, 176, 175, 175, 174, 174, 173, 173, 172, 172, 172, 171, 171, 170, 170, 169, 169, 168, 168, 168, 167, 167, 166, 166, 165, 165, 165, 164, 164, 163, 163, 163, 162, 162, 161, 161, 161, 160, 160, 159, 159, 159, 158, 158, 157, 157, 157, 156, 156, 156, 155, 155, 154, 154, 154, 153, 153, 153, 152, 152, 152, 151, 151, 151, 150, 150, 149, 149, 149, 148, 148, 148, 147, 147, 147, 146, 146, 146, 145, 145, 145, 144, 144, 144, 144, 143, 143, 143, 142, 142, 142, 141, 141, 141, 140, 140, 140, 140, 139, 139, 139, 138, 138, 138, 137, 137, 137, 137, 136, 136, 136, 135, 135, 135, 135, 134, 134, 134, 134, 133, 133, 133, 132, 132, 132, 132, 131, 131, 131, 131, 130, 130, 130, 130, 129, 129, 129, 129, 128, 128, 128, 128,
};

const Int g_aiMFLM_MinSize[] = {  0,  0 };
const Int g_aiMMLM_MinSize[] = {  0,  0 };
const Int g_aiNonLMPosThrs[] = {  3,  1,  0 };
Int g_aiLMCodeWord[LM_SYMBOL_NUM][16];

#if JVET_B0051_NON_MPM_MODE
const Int g_ipred_mode_table[] = { 1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,10,10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15 };
#endif

#endif

// initialize ROM variables
Void initROM()
{
  Int i, c;

#if RExt__HIGH_BIT_DEPTH_SUPPORT
  {
    c = 64;
    const Double s = sqrt((Double)c) * (64 << COM16_C806_TRANS_PREC);


    for (Int k = 0; k < c; k++)
    {
      for (Int n = 0; n < c; n++)
      {
        Double w0, v;
        const Double PI = 3.14159265358979323846;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        short sv = (short)(s * v + (v > 0 ? 0.5 : -0.5));
        if (g_aiT64[0][0][c*c + k*c + n] != sv)
        {
          msg(WARNING, "trap");
        }
      }
    }
  }
#endif





  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset(g_aucLog2, 0, sizeof(g_aucLog2));
  c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    g_aucNextLog2[i] = i <= 1 ? 0 : c + 1;

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[i] = c;
    g_aucLog2    [i] = c;
  }

  c = 2; //for the 2x2 transforms if QTBT is on

  const Double PI = 3.14159265358979323846;

  for (i = 0; i < 7; i++)
  {
    TMatrixCoeff *iT = NULL;
    const Double s = sqrt((Double)c) * (64 << COM16_C806_TRANS_PREC);

    switch (i)
    {
    case 0: iT = g_aiTr2[0][0]; break;
    case 1: iT = g_aiTr4[0][0]; break;
    case 2: iT = g_aiTr8[0][0]; break;
    case 3: iT = g_aiTr16[0][0]; break;
    case 4: iT = g_aiTr32[0][0]; break;
    case 5: iT = g_aiTr64[0][0]; break;
    case 6: iT = g_aiTr128[0][0]; break;
    case 7: exit(0); break;
    }

    for (Int k = 0; k < c; k++)
    {
      for (Int n = 0; n < c; n++)
      {
        Double w0, w1, v;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        iT[DCT2*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DCT-V
        w0 = (k == 0) ? sqrt(0.5) : 1.0;
        w1 = (n == 0) ? sqrt(0.5) : 1.0;
        v = cos(PI*n*k / (c - 0.5)) * w0 * w1 * sqrt(2.0 / (c - 0.5));
        iT[DCT5*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DCT-VIII
        v = cos(PI*(k + 0.5)*(n + 0.5) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
        iT[DCT8*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DST-I
        v = sin(PI*(n + 1)*(k + 1) / (c + 1)) * sqrt(2.0 / (c + 1));
        iT[DST1*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DST-VII
        v = sin(PI*(k + 0.5)*(n + 1) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
        iT[DST7*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));
      }
    }
    c <<= 1;
  }
  gp_sizeIdxInfo = new SizeIndexInfoLog2();
  gp_sizeIdxInfo->init(MAX_CU_SIZE);


  generateTrafoBlockSizeScaling(*gp_sizeIdxInfo);

#if JEM_TOOLS
  const Double scPi = 2.0 * 3.14159265358979323846 / (Double)NSST_HYGT_PTS;
  for (i = 0; i < NSST_HYGT_PTS; i++)
  {
    g_tabSinCos[i].c = Int(floor(cos(i * scPi) * 1024.0 + 0.5));
    g_tabSinCos[i].s = Int(floor(sin(i * scPi) * 1024.0 + 0.5));
  }

#endif
  SizeIndexInfoLog2 sizeInfo;
  sizeInfo.init(MAX_CU_SIZE);

  // initialize scan orders
  for (UInt blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  {
    for (UInt blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
    {
      const UInt blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
      const UInt blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
      const UInt totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        g_scanOrder     [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]    = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][0] = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][1] = new UInt[totalValues];

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          const int posY      = rasterPos / blockWidth;
          const int posX      = rasterPos - ( posY * blockWidth );
          g_scanOrder     [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]   [scanPosition] = rasterPos;
          g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][0][scanPosition] = posX;
          g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][1][scanPosition] = posY;
        }
      }

      if( blockWidthIdx >= sizeInfo.numWidths() || blockHeightIdx >= sizeInfo.numHeights() )
      {
        // size indizes greater than numIdxs are sizes than are only used when grouping - they will never come up as a block size - thus they can be skipped at this point
        for( UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++ )
        {
          g_scanOrder     [SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx]    = nullptr;
          g_scanOrderPosXY[SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx][0] = nullptr;
          g_scanOrderPosXY[SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx][1] = nullptr;

        }

        continue;
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
      const UInt  log2CGWidth    = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
      const UInt  log2CGHeight   = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;

      const UInt  groupWidth     = 1 << log2CGWidth;
      const UInt  groupHeight    = 1 << log2CGHeight;
      const UInt  widthInGroups  = blockWidth >> log2CGWidth;
      const UInt  heightInGroups = blockHeight >> log2CGHeight;

      const UInt  groupSize      = groupWidth    * groupHeight;
      const UInt  totalGroups    = widthInGroups * heightInGroups;

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]    = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0] = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1] = new UInt[totalValues];


        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (UInt groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const UInt groupPositionY  = fullBlockScan.GetCurrentY();
          const UInt groupPositionX  = fullBlockScan.GetCurrentX();
          const UInt groupOffsetX    = groupPositionX * groupWidth;
          const UInt groupOffsetY    = groupPositionY * groupHeight;
          const UInt groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (UInt scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            const int posY      = rasterPos / blockWidth;
            const int posX      = rasterPos - ( posY * blockWidth );

            g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]   [groupOffsetScan + scanPosition] = rasterPos;
            g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0][groupOffsetScan + scanPosition] = posX;
            g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1][groupOffsetScan + scanPosition] = posY;
          }

          fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
#if JEM_TOOLS && !ENABLE_BMS

  // initialize CoefTopLeftDiagScan8x8 for NSST
  for (UInt blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
  {
    const UInt blockWidth = sizeInfo.sizeFrom(blockWidthIdx);

    const static UChar g_auiXYDiagScan8x8[64][2] =
    {
      { 0, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 1 }, { 2, 0 }, { 0, 3 }, { 1, 2 },
      { 2, 1 }, { 3, 0 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 2, 3 }, { 3, 2 }, { 3, 3 },
      { 0, 4 }, { 0, 5 }, { 1, 4 }, { 0, 6 }, { 1, 5 }, { 2, 4 }, { 0, 7 }, { 1, 6 },
      { 2, 5 }, { 3, 4 }, { 1, 7 }, { 2, 6 }, { 3, 5 }, { 2, 7 }, { 3, 6 }, { 3, 7 },
      { 4, 0 }, { 4, 1 }, { 5, 0 }, { 4, 2 }, { 5, 1 }, { 6, 0 }, { 4, 3 }, { 5, 2 },
      { 6, 1 }, { 7, 0 }, { 5, 3 }, { 6, 2 }, { 7, 1 }, { 6, 3 }, { 7, 2 }, { 7, 3 },
      { 4, 4 }, { 4, 5 }, { 5, 4 }, { 4, 6 }, { 5, 5 }, { 6, 4 }, { 4, 7 }, { 5, 6 },
      { 6, 5 }, { 7, 4 }, { 5, 7 }, { 6, 6 }, { 7, 5 }, { 6, 7 }, { 7, 6 }, { 7, 7 }
    };
    for (int i = 0; i < 64; i++)
    {
      g_auiCoefTopLeftDiagScan8x8[blockWidthIdx][i] = g_auiXYDiagScan8x8[i][0] + g_auiXYDiagScan8x8[i][1] * blockWidth;
    }
  }
#endif
}

Void destroyROM()
{
  unsigned numWidths = gp_sizeIdxInfo->numAllWidths();
  unsigned numHeights = gp_sizeIdxInfo->numAllHeights();

  for (UInt groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (UInt scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (UInt blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++)
      {
        for (UInt blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++)
        {
          delete[] g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
          g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;

          delete[] g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][0];
          g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][0] = nullptr;

          delete[] g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][1];
          g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][1] = nullptr;

        }
      }
    }
  }

  delete gp_sizeIdxInfo;
  gp_sizeIdxInfo = nullptr;
}



void generateTrafoBlockSizeScaling(SizeIndexInfo& sizeIdxInfo)
{
  for (SizeType y = 0; y < sizeIdxInfo.numHeights(); y++)
  {
    for (SizeType x = 0; x < sizeIdxInfo.numWidths(); x++)
    {
      SizeType h = sizeIdxInfo.sizeFrom(y);
      SizeType w = sizeIdxInfo.sizeFrom(x);
      double factor = sqrt(h) * sqrt(w) / (double)(1 << ((g_aucLog2[h] + g_aucLog2[w]) / 2));

      g_BlockSizeTrafoScale[h][w][0] = ((int)(factor + 0.9) != 1) ? (int)(factor * (double)(1 << ADJ_QUANT_SHIFT)) : 1;
      g_BlockSizeTrafoScale[h][w][1] = ((int)(factor + 0.9) != 1) ? (int)((double)(1 << ADJ_DEQUANT_SHIFT) / factor + 0.5) : 1;
    }
  }
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

const Int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const Int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};

//--------------------------------------------------------------------------------------------------
//structures
#if JEM_TOOLS
//EMT transform sets
const Int g_aiTrSubsetIntra[3][2] = { { DST7, DCT8 }, { DST7, DST1 }, { DST7, DCT5 } };
const Int g_aiTrSubsetInter[4] = { DCT8, DST7 };

const UChar g_aucTrSetVert[NUM_INTRA_MODE - 1] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66
   2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0
};
const UChar g_aucTrSetVert35[35] =
{//0  1  2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26    27    28    29    30    31    32    33    34
   2, 1, 0,    1,    0,    1,    0,    1,    0,    0,    0,    0,    0,    1,    0,    1,    0,    1,    0,    1,    0,    1,    0,    1,    2,    2,    2,    2,    2,    1,    0,    1,    0,    1,    0
};
const UChar g_aucTrSetHorz[NUM_INTRA_MODE - 1] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66
   2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0
};
const UChar g_aucTrSetHorz35[35] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34
   2, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0
};

//EMT threshold
const UInt g_EmtSigNumThr = 2;

#endif
//EMT transform coeficient variable
TMatrixCoeff g_aiTr2  [NUM_TRANS_TYPE][  2][  2];
TMatrixCoeff g_aiTr4  [NUM_TRANS_TYPE][  4][  4];
TMatrixCoeff g_aiTr8  [NUM_TRANS_TYPE][  8][  8];
TMatrixCoeff g_aiTr16 [NUM_TRANS_TYPE][ 16][ 16];
TMatrixCoeff g_aiTr32 [NUM_TRANS_TYPE][ 32][ 32];
TMatrixCoeff g_aiTr64 [NUM_TRANS_TYPE][ 64][ 64];
TMatrixCoeff g_aiTr128[NUM_TRANS_TYPE][128][128];


//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------

const UChar g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize] =
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const UChar g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1] =
{
  {3, 3, 3, 3, 2, 2},  //   4x4,   4x8,   4x16,   4x32,   4x64,   4x128,
  {3, 3, 3, 3, 3, 2},  //   8x4,   8x8,   8x16,   8x32,   8x64,   8x128,
  {3, 3, 3, 3, 3, 2},  //  16x4,  16x8,  16x16,  16x32,  16x64,  16x128,
  {3, 3, 3, 3, 3, 2},  //  32x4,  32x8,  32x16,  32x32,  32x64,  32x128,
  {2, 3, 3, 3, 3, 2},  //  64x4,  64x8,  64x16,  64x32,  64x64,  64x128,
  {2, 2, 2, 2, 2, 3},  // 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,
};

const UChar g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3,  //  64x64
  3   // 128x128
};
const UChar g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5,  //  64x64   33
  5   // 128x128
};

const UChar g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 2, 2, 2, 2, 2, 3,  4,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 44, 45, 46, 46, 46, 47, 48, 48, 48, 49, 50, 51, 52, 52, 52, 53, 54, 54, 54, 55, 56, 56, 56, 57, 58, 59, 60, DM_CHROMA_IDX };

extern const UChar  g_intraMode65to33AngMapping[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 3, 3, 4, 4, 5, 5,  6,  6,  7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, DM_CHROMA_IDX };

extern const UChar g_intraMode33to65AngMapping[36] =
//                                   H                               D                               V
//0, 1, 2, 3, 4, 5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, DM
{ 0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, DM_CHROMA_IDX };

#if JEM_TOOLS
const Int g_intraCubicFilter[32][4] = {
  {   0, 256,   0,   0 }, //  0 Integer-Pel
  {  -3, 252,   8,  -1 }, //  1
  {  -5, 247,  17,  -3 }, //  2
  {  -7, 242,  25,  -4 }, //  3
  {  -9, 236,  34,  -5 }, //  4
  { -10, 230,  43,  -7 }, //  5
  { -12, 224,  52,  -8 }, //  6
  { -13, 217,  61,  -9 }, //  7
  { -14, 210,  70, -10 }, //  8
  { -15, 203,  79, -11 }, //  9
  { -16, 195,  89, -12 }, // 10
  { -16, 187,  98, -13 }, // 11
  { -16, 179, 107, -14 }, // 12
  { -16, 170, 116, -14 }, // 13
  { -17, 162, 126, -15 }, // 14
  { -16, 153, 135, -16 }, // 15
  { -16, 144, 144, -16 }, // 16 Half-Pel
  { -16, 135, 153, -16 }, // 17
  { -15, 126, 162, -17 }, // 18
  { -14, 116, 170, -16 }, // 19
  { -14, 107, 179, -16 }, // 20
  { -13,  98, 187, -16 }, // 21
  { -12,  89, 195, -16 }, // 22
  { -11,  79, 203, -15 }, // 23
  { -10,  70, 210, -14 }, // 24
  {  -9,  61, 217, -13 }, // 25
  {  -8,  52, 224, -12 }, // 26
  {  -7,  43, 230, -10 }, // 27
  {  -5,  34, 236,  -9 }, // 28
  {  -4,  25, 242,  -7 }, // 29
  {  -3,  17, 247,  -5 }, // 30
  {  -1,   8, 252,  -3 }, // 31
};
const Int g_intraGaussFilter[32][4] = {
#if HM_4TAPIF_AS_IN_JEM
  {  47, 161,  47,   1 }, //  0 Integer-Pel
#else
  {  47, 162,  47,   0 }, //  0 Integer-Pel
#endif
  {  43, 161,  51,   1 }, //  1
  {  40, 160,  54,   2 }, //  2
  {  37, 159,  58,   2 }, //  3
  {  34, 158,  62,   2 }, //  4
  {  31, 156,  67,   2 }, //  5
  {  28, 154,  71,   3 }, //  6
  {  26, 151,  76,   3 }, //  7
  {  23, 149,  80,   4 }, //  8
  {  21, 146,  85,   4 }, //  9
  {  19, 142,  90,   5 }, // 10
  {  17, 139,  94,   6 }, // 11
  {  16, 135,  99,   6 }, // 12
  {  14, 131, 104,   7 }, // 13
  {  13, 127, 108,   8 }, // 14
  {  11, 123, 113,   9 }, // 15
  {  10, 118, 118,  10 }, // 16 Half-Pel
  {   9, 113, 123,  11 }, // 17
  {   8, 108, 127,  13 }, // 18
  {   7, 104, 131,  14 }, // 19
  {   6,  99, 135,  16 }, // 20
  {   6,  94, 139,  17 }, // 21
  {   5,  90, 142,  19 }, // 22
  {   4,  85, 146,  21 }, // 23
  {   4,  80, 149,  23 }, // 24
  {   3,  76, 151,  26 }, // 25
  {   3,  71, 154,  28 }, // 26
  {   2,  67, 156,  31 }, // 27
  {   2,  62, 158,  34 }, // 28
  {   2,  58, 159,  37 }, // 29
  {   2,  54, 160,  40 }, // 30
  {   1,  51, 161,  43 }, // 31
};

#endif

// ====================================================================================================================
// Decision tree templates
// ====================================================================================================================

const DecisionTreeTemplate g_mtSplitDTT = compile(
  decision( DTT_SPLIT_DO_SPLIT_DECISION,
  /*0*/ DTT_SPLIT_NO_SPLIT,
  /*1*/ decision( DTT_SPLIT_HV_DECISION,
        /*0*/ decision( DTT_SPLIT_H_IS_BT_12_DECISION,
              /*0*/ DTT_SPLIT_TT_HORZ,
              /*1*/ DTT_SPLIT_BT_HORZ ),
        /*1*/ decision( DTT_SPLIT_V_IS_BT_12_DECISION,
              /*0*/ DTT_SPLIT_TT_VERT,
              /*1*/ DTT_SPLIT_BT_VERT ) ) ) );


#if JEM_TOOLS
const DecisionTreeTemplate g_intraLumaMpmDTT = compile(
  decision( DTT_INTRA_MPM_ISGT_0,
  /*0*/ DTT_INTRA_MPM_0,
  /*1*/ decision( DTT_INTRA_MPM_ISGT_1,
        /*0*/ DTT_INTRA_MPM_1,
        /*1*/ decision( DTT_INTRA_MPM_ISGT_2,
              /*0*/ DTT_INTRA_MPM_2,
              /*1*/ decision( DTT_INTRA_MPM_ISGT_3,
                    /*0*/ DTT_INTRA_MPM_3,
                    /*1*/ decision( DTT_INTRA_MPM_4,
                          /*0*/ DTT_INTRA_MPM_4,
                          /*1*/ DTT_INTRA_MPM_5 ) ) ) ) ) );

const int g_pdpcParam[5][6] = {
  { 33,   7,  33,   7,  30,  3 },
  { 40,   8,  40,   8, -19,  1 },
  { 32,   2,  32,   2, -37,  1 },
  { 31,  -2,  31,  -2,  -4,  3 },
  { 20,  -2,  20,  -2,   5,  5 },
};

const int g_pdpc_pred_param[5][35][6] = {
{ {  33,   7,  33,   7,  30,    3 },
  {  25,   5,  25,   5,   0,    0 },
  {  10,   8,  29,   4,  11,    1 },
  {  10,   8,  29,   4,  11,    1 },
  {  17,   5,  20,   5,  52,    1 },
  {  17,   5,  20,   5,  52,    1 },
  {  21,   3,  18,   7,  70,    2 },
  {  21,   3,  18,   7,  70,    2 },
  {  20,   1,  18,  11,  63,    2 },
  {  20,   1,  18,  11,  63,    2 },
  {  16,   1,  30,  24,  56,    1 },
  {  16,   1,  30,  24,  56,    1 },
  {  15,   0,  15,  14,  67,    3 },
  {  15,   0,  15,  14,  67,    3 },
  {  15,   2,   9,   2,  62,    1 },
  {  15,   2,   9,   2,  62,    1 },
  {  11,   4,  10,   2,  40,    1 },
  {  11,   4,  10,   2,  40,    1 },
  {   4,   3,   4,   3,  22,    1 },
  {  10,   2,  11,   4,  40,    1 },
  {  10,   2,  11,   4,  40,    1 },
  {   9,   2,  15,   2,  62,    1 },
  {   9,   2,  15,   2,  62,    1 },
  {  15,  14,  15,   0,  67,    3 },
  {  15,  14,  15,   0,  67,    3 },
  {  30,  24,  16,   1,  56,    1 },
  {  30,  24,  16,   1,  56,    1 },
  {  18,  11,  20,   1,  63,    2 },
  {  18,  11,  20,   1,  63,    2 },
  {  18,   7,  21,   3,  70,    2 },
  {  18,   7,  21,   3,  70,    2 },
  {  20,   5,  17,   5,  52,    1 },
  {  20,   5,  17,   5,  52,    1 },
  {  29,   4,  10,   8,  11,    1 },
  {  29,   4,  10,   8,  11,    1 } },
{ {  36,   7,  36,   7,  26,    3 },
  {  33,   8,  33,   8,   0,    0 },
  {  22,   7,  32,   6,  24,    3 },
  {  22,   7,  32,   6,  24,    3 },
  {  35,   4,  29,   8,  45,    2 },
  {  35,   4,  29,   8,  45,    2 },
  {  41,   3,  27,  12,  65,    3 },
  {  41,   3,  27,  12,  65,    3 },
  {  54,   1,  26,  16,  63,    2 },
  {  54,   1,  26,  16,  63,    2 },
  {  54,  -1,  34,  25,  52,    1 },
  {  54,  -1,  34,  25,  52,    1 },
  {  24,  -1,  21,  20,  62,    1 },
  {  24,  -1,  21,  20,  62,    1 },
  {  21,   3,  19,   3,  35,    1 },
  {  21,   3,  19,   3,  35,    1 },
  {  19,   4,  21,   3,  36,    2 },
  {  19,   4,  21,   3,  36,    2 },
  {  15,   6,  15,   6,  23,    2 },
  {  21,   3,  19,   4,  36,    2 },
  {  21,   3,  19,   4,  36,    2 },
  {  19,   3,  21,   3,  35,    1 },
  {  19,   3,  21,   3,  35,    1 },
  {  21,  20,  24,  -1,  62,    1 },
  {  21,  20,  24,  -1,  62,    1 },
  {  34,  25,  54,  -1,  52,    1 },
  {  34,  25,  54,  -1,  52,    1 },
  {  26,  16,  54,   1,  63,    2 },
  {  26,  16,  54,   1,  63,    2 },
  {  27,  12,  41,   3,  65,    3 },
  {  27,  12,  41,   3,  65,    3 },
  {  29,   8,  35,   4,  45,    2 },
  {  29,   8,  35,   4,  45,    2 },
  {  32,   6,  22,   7,  24,    3 },
  {  32,   6,  22,   7,  24,    3 } },
{ {  45,   5,  45,   5,  -5,    3 },
  {  36,   8,  36,   8,   0,    0 },
  {  30,   6,  46,   6, -15,    3 },
  {  30,   6,  46,   6, -15,    3 },
  {  31,   5,  39,   8,  15,    3 },
  {  31,   5,  39,   8,  15,    3 },
  {  35,   3,  35,  11,  42,    3 },
  {  35,   3,  35,  11,  42,    3 },
  {  45,   1,  35,  19,  46,    3 },
  {  45,   1,  35,  19,  46,    3 },
  {  32,   0,  40,  32,  47,    3 },
  {  32,   0,  40,  32,  47,    3 },
  {  38,   0,  23,  13,  38,    2 },
  {  38,   0,  23,  13,  38,    2 },
  {  26,   2,  24,   0,  28,    3 },
  {  26,   2,  24,   0,  28,    3 },
  {  25,   2,  23,   0,  19,    3 },
  {  25,   2,  23,   0,  19,    3 },
  {  29,   1,  29,   1,  -7,    3 },
  {  24,   0,  25,   2,  19,    3 },
  {  24,   0,  25,   2,  19,    3 },
  {  24,   0,  26,   2,  28,    3 },
  {  24,   0,  26,   2,  28,    3 },
  {  23,  13,  38,   0,  38,    2 },
  {  23,  13,  38,   0,  38,    2 },
  {  40,  32,  32,   0,  47,    3 },
  {  40,  32,  32,   0,  47,    3 },
  {  35,  19,  45,   1,  46,    3 },
  {  35,  19,  45,   1,  46,    3 },
  {  35,  11,  35,   3,  42,    3 },
  {  35,  11,  35,   3,  42,    3 },
  {  39,   8,  31,   5,  15,    3 },
  {  39,   8,  31,   5,  15,    3 },
  {  46,   6,  30,   6, -15,    3 },
  {  46,   6,  30,   6, -15,    3 } },
{ {  46,   6,  46,   6,  -3,    5 },
  {  44,   4,  44,   4,   0,    0 },
  {  33,   3,  52,   4, -18,    5 },
  {  33,   3,  52,   4, -18,    5 },
  {  38,   3,  50,   5,  -5,    5 },
  {  38,   3,  50,   5,  -5,    5 },
  {  40,   2,  47,   9,  16,    5 },
  {  40,   2,  47,   9,  16,    5 },
  {  48,   1,  45,  17,  22,    5 },
  {  48,   1,  45,  17,  22,    5 },
  {  45,  -1,  46,  30,  36,    5 },
  {  45,  -1,  46,  30,  36,    5 },
  {  41,   1,  37,  -1,  14,    5 },
  {  41,   1,  37,  -1,  14,    5 },
  {  35,   1,  39,  -2,   3,    5 },
  {  35,   1,  39,  -2,   3,    5 },
  {  41,  -1,  43,  -1,  -7,    5 },
  {  41,  -1,  43,  -1,  -7,    5 },
  {  32,   0,  32,   0,  -6,    5 },
  {  43,  -1,  41,  -1,  -7,    5 },
  {  43,  -1,  41,  -1,  -7,    5 },
  {  39,  -2,  35,   1,   3,    5 },
  {  39,  -2,  35,   1,   3,    5 },
  {  37,  -1,  41,   1,  14,    5 },
  {  37,  -1,  41,   1,  14,    5 },
  {  46,  30,  45,  -1,  36,    5 },
  {  46,  30,  45,  -1,  36,    5 },
  {  45,  17,  48,   1,  22,    5 },
  {  45,  17,  48,   1,  22,    5 },
  {  47,   9,  40,   2,  16,    5 },
  {  47,   9,  40,   2,  16,    5 },
  {  50,   5,  38,   3,  -5,    5 },
  {  50,   5,  38,   3,  -5,    5 },
  {  52,   4,  33,   3, -18,    5 },
  {  52,   4,  33,   3, -18,    5 } },
{ {  42,   5,  42,   5,  -5,    7 },
  {  40,   3,  40,   3,   0,    0 },
  {  28,   2,  49,   3, -22,    7 },
  {  28,   2,  49,   3, -22,    7 },
  {  27,   2,  48,   3, -16,    7 },
  {  27,   2,  48,   3, -16,    7 },
  {  27,   1,  44,   5,   8,    7 },
  {  27,   1,  44,   5,   8,    7 },
  {  41,   2,  39,  12,  16,    7 },
  {  41,   2,  39,  12,  16,    7 },
  {  42,   0,  38,  21,  24,    7 },
  {  42,   0,  38,  21,  24,    7 },
  {  38,   0,  34,  -4,   5,    7 },
  {  38,   0,  34,  -4,   5,    7 },
  {  37,   1,  43,  -1,  -5,    7 },
  {  37,   1,  43,  -1,  -5,    7 },
  {  25,   0,  42,  -1, -13,    7 },
  {  25,   0,  42,  -1, -13,    7 },
  {  27,  -1,  27,  -1, -11,    7 },
  {  42,  -1,  25,   0, -13,    7 },
  {  42,  -1,  25,   0, -13,    7 },
  {  43,  -1,  37,   1,  -5,    7 },
  {  43,  -1,  37,   1,  -5,    7 },
  {  34,  -4,  38,   0,   5,    7 },
  {  34,  -4,  38,   0,   5,    7 },
  {  38,  21,  42,   0,  24,    7 },
  {  38,  21,  42,   0,  24,    7 },
  {  39,  12,  41,   2,  16,    7 },
  {  39,  12,  41,   2,  16,    7 },
  {  44,   5,  27,   1,   8,    7 },
  {  44,   5,  27,   1,   8,    7 },
  {  48,   3,  27,   2, -16,    7 },
  {  48,   3,  27,   2, -16,    7 },
  {  49,   3,  28,   2, -22,    7 },
  {  49,   3,  28,   2, -22,    7 } } };
#endif

// ====================================================================================================================
// Misc.
// ====================================================================================================================
SizeIndexInfo*           gp_sizeIdxInfo = NULL;
int                      g_BlockSizeTrafoScale[MAX_CU_SIZE + 1][MAX_CU_SIZE + 1][2];
SChar                    g_aucLog2    [MAX_CU_SIZE + 1];
SChar                    g_aucNextLog2[MAX_CU_SIZE + 1];
SChar                    g_aucPrevLog2[MAX_CU_SIZE + 1];

UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
UInt* g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
UInt* g_scanOrderPosXY[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1][2];
#if JEM_TOOLS && !ENABLE_BMS
UInt  g_auiCoefTopLeftDiagScan8x8[MAX_CU_SIZE / 2 + 1][64];

#endif

const UInt ctxIndMap4x4[4 * 4] =
{
  0, 1, 4, 5,
  2, 3, 4, 5,
  6, 6, 8, 8,
  7, 7, 8, 8
};


const UInt g_uiMinInGroup[LAST_SIGNIFICANT_GROUPS] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
const UInt g_uiGroupIdx[MAX_TU_SIZE] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11
,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12
,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13 };
#if JVET_K0072
const UInt g_auiGoRicePars[ 32 ] =
{
  0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 2
};
#endif
const UInt g_auiGoRiceRange[MAX_GR_ORDER_RESIDUAL] =
{
  6, 5, 6, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION
};

#if HEVC_USE_SCALING_LISTS
const TChar *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA2X2_LUMA",
    "INTRA2X2_CHROMAU",
    "INTRA2X2_CHROMAV",
    "INTER2X2_LUMA",
    "INTER2X2_CHROMAU",
    "INTER2X2_CHROMAV"
  },
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const TChar *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const Int g_quantTSDefault4x4[4 * 4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const Int g_quantIntraDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const Int g_quantInterDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

const UInt g_scalingListSize [SCALING_LIST_SIZE_NUM] = { 4, 16, 64, 256, 1024, 4096, 16384 };
const UInt g_scalingListSizeX[SCALING_LIST_SIZE_NUM] = { 2,  4,  8,  16,   32,   64,   128 };
#endif

const UChar g_NonMPM[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };


//! \}
