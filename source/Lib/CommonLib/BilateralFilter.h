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
#ifndef BILATERALFILTER_H
#define BILATERALFILTER_H

#include "CommonDef.h"

#if JEM_TOOLS

#include "Unit.h"
#include "Buffer.h"

#define BILATERAL_FILTER_MAX_DENOMINATOR_PLUS_ONE  325
#define BITS_PER_DIV_LUT_ENTRY 14
#define SIGN_IF_NEG(x) (-((x)<0))                             // Returns -1 if x is negative, otherwise 0

class BilateralFilter
{
private:
  static const int SpatialSigmaValue;
  static const int spatialSigmaBlockLengthOffsets[5];
  unsigned short** m_bilateralFilterTable;
  int m_bilateralCenterWeightTable[5];
  short tempblock[ MAX_CU_SIZE*MAX_CU_SIZE ];
  unsigned divToMulOneOverN[BILATERAL_FILTER_MAX_DENOMINATOR_PLUS_ONE];
  uint8_t divToMulShift[BILATERAL_FILTER_MAX_DENOMINATOR_PLUS_ONE];

  void smoothBlockBilateralFilter( unsigned uiWidth, unsigned uiHeight, short block[], int isInterBlock, int qp);

public:
  BilateralFilter();
  ~BilateralFilter();

  void create();
  void destroy();

  void createdivToMulLUTs();
  void createBilateralFilterTable(int qp);
  void bilateralFilterInter(PelBuf& resiBuf, const CPelBuf& predBuf, int qp, const ClpRng& clpRng);
  void bilateralFilterIntra(PelBuf& recoBuf, int qp);
};

#endif

#endif /* BILATERALFILTER_H */
