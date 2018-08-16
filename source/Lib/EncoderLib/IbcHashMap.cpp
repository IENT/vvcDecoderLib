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

/** \file     IbcHashMap.cpp
    \brief    IBC hash map encoder class
*/

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "IbcHashMap.h"

#include <nmmintrin.h>

using namespace std;

//! \ingroup IbcHashMap
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

IbcHashMap::IbcHashMap()
{
  m_picWidth = 0;
  m_picHeight = 0;
  m_pos2Hash = NULL;
}

IbcHashMap::~IbcHashMap()
{
  destroy();
}

void IbcHashMap::init(const int picWidth, const int picHeight)
{
  if (picWidth != m_picWidth || picHeight != m_picHeight)
  {
    destroy();
  }

  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_pos2Hash = new unsigned int*[m_picHeight];
  m_pos2Hash[0] = new unsigned int[m_picWidth * m_picHeight];
  for (int n = 1; n < m_picHeight; n++)
  {
    m_pos2Hash[n] = m_pos2Hash[n - 1] + m_picWidth;
  }
}

void IbcHashMap::destroy()
{
  if (m_pos2Hash != NULL)
  {
    if (m_pos2Hash[0] != NULL)
    {
      delete[] m_pos2Hash[0];
    }
    delete[] m_pos2Hash;
  }
  m_pos2Hash = NULL;
}

template<int maskBit>
unsigned int IbcHashMap::xxCalcBlockHash(const Pel* pel, const int stride, const int width, const int height, unsigned int crc)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      crc = _mm_crc32_u16(crc, pel[x] >> maskBit);
    }
    pel += stride;
  }
  return crc;
}

template<ChromaFormat chromaFormat, int maskBit>
void IbcHashMap::xxBuildPicHashMap(const PelUnitBuf& pic)
{
  const int chromaScalingX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, chromaFormat);
  const int chromaScalingY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, chromaFormat);
  const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
  const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
  const Pel* pelY = NULL;
  const Pel* pelCb = NULL;
  const Pel* pelCr = NULL;

  Position pos;
  for (pos.y = 0; pos.y + MIN_PU_SIZE <= pic.Y().height; pos.y++)
  {
    // row pointer
    pelY = pic.Y().bufAt(0, pos.y);
    if (chromaFormat != CHROMA_400)
    {
      int chromaY = pos.y >> chromaScalingY;
      pelCb = pic.Cb().bufAt(0, chromaY);
      pelCr = pic.Cr().bufAt(0, chromaY);
    }

    for (pos.x = 0; pos.x + MIN_PU_SIZE <= pic.Y().width; pos.x++)
    {
      // 0x1FF is just an initial value
      unsigned int hashValue = 0x1FF;

      // luma part
      hashValue = xxCalcBlockHash<maskBit>(&pelY[pos.x], pic.Y().stride, MIN_PU_SIZE, MIN_PU_SIZE, hashValue);

      // chroma part
      if (chromaFormat != CHROMA_400)
      {
        int chromaX = pos.x >> chromaScalingX;
        hashValue = xxCalcBlockHash<maskBit>(&pelCb[chromaX], pic.Cb().stride, chromaMinBlkWidth, chromaMinBlkHeight, hashValue);
        hashValue = xxCalcBlockHash<maskBit>(&pelCr[chromaX], pic.Cr().stride, chromaMinBlkWidth, chromaMinBlkHeight, hashValue);
      }

      // hash table
      m_hash2Pos[hashValue].push_back(pos);
      m_pos2Hash[pos.y][pos.x] = hashValue;
    }
  }
}

void IbcHashMap::rebuildPicHashMap(const PelUnitBuf& pic)
{
  m_hash2Pos.clear();

  switch (pic.chromaFormat)
  {
  case CHROMA_400:
    xxBuildPicHashMap<CHROMA_400, 0>(pic);
    break;
  case CHROMA_420:
    xxBuildPicHashMap<CHROMA_420, 0>(pic);
    break;
  case CHROMA_422:
    xxBuildPicHashMap<CHROMA_422, 0>(pic);
    break;
  case CHROMA_444:
    xxBuildPicHashMap<CHROMA_444, 0>(pic);
    break;
  default:
    THROW("invalid chroma fomat");
    break;
  }
}

bool IbcHashMap::ibcHashMatch(const Area& lumaArea, std::vector<Position>& cand, const CodingStructure& cs, const int maxCand, const int searchRange4SmallBlk)
{
  cand.clear();

  // find the block with least candidates
  size_t minSize = MAX_UINT;
  unsigned int targetHashOneBlock = 0;
  for (SizeType y = 0; y < lumaArea.height && minSize > 1; y += MIN_PU_SIZE)
  {
    for (SizeType x = 0; x < lumaArea.width && minSize > 1; x += MIN_PU_SIZE)
    {
      unsigned int hash = m_pos2Hash[lumaArea.pos().y + y][lumaArea.pos().x + x];
      if (m_hash2Pos[hash].size() < minSize)
      {
        minSize = m_hash2Pos[hash].size();
        targetHashOneBlock = hash;
      }
    }
  }

  if (m_hash2Pos[targetHashOneBlock].size() > 1)
  {
    std::vector<Position>& candOneBlock = m_hash2Pos[targetHashOneBlock];

    // check whether whole block match
    for (std::vector<Position>::iterator refBlockPos = candOneBlock.begin(); refBlockPos != candOneBlock.end(); refBlockPos++)
    {
      Position bottomRight = refBlockPos->offset(lumaArea.width - 1, lumaArea.height - 1);
      bool wholeBlockMatch = true;
      if (lumaArea.width > MIN_PU_SIZE || lumaArea.height > MIN_PU_SIZE)
      {
#if JVET_K0076_CPR
        if (!cs.isDecomp(bottomRight, cs.chType) || bottomRight.x >= m_picWidth || bottomRight.y >= m_picHeight)
#else
        if (!cs.isDecomp(bottomRight, CHANNEL_TYPE_LUMA) || bottomRight.x >= m_picWidth || bottomRight.y >= m_picHeight)
#endif
        {
          continue;
        }
        for (SizeType y = 0; y < lumaArea.height && wholeBlockMatch; y += MIN_PU_SIZE)
        {
          for (SizeType x = 0; x < lumaArea.width && wholeBlockMatch; x += MIN_PU_SIZE)
          {
            // whether the reference block and current block has the same hash
            wholeBlockMatch &= (m_pos2Hash[lumaArea.pos().y + y][lumaArea.pos().x + x] == m_pos2Hash[refBlockPos->y + y][refBlockPos->x + x]);
          }
        }
      }
      else
      {
#if JVET_K0076_CPR
        if (abs(refBlockPos->x - lumaArea.x) > searchRange4SmallBlk || abs(refBlockPos->y - lumaArea.y) > searchRange4SmallBlk || !cs.isDecomp(bottomRight, cs.chType))
#else
        if (abs(refBlockPos->x - lumaArea.x) > searchRange4SmallBlk || abs(refBlockPos->y - lumaArea.y) > searchRange4SmallBlk || !cs.isDecomp(bottomRight, CHANNEL_TYPE_LUMA))
#endif
        {
          continue;
        }
      }
      if (wholeBlockMatch)
      {
        cand.push_back(*refBlockPos);
        if (cand.size() > maxCand)
        {
          break;
        }
      }
    }
  }

  return cand.size() > 0;
}

int IbcHashMap::getHashHitRatio(const Area& lumaArea)
{
  int maxX = std::min((int)(lumaArea.x + lumaArea.width), m_picWidth);
  int maxY = std::min((int)(lumaArea.y + lumaArea.height), m_picHeight);
  int hit = 0, total = 0;
  for (int y = lumaArea.y; y < maxY; y += MIN_PU_SIZE)
  {
    for (int x = lumaArea.x; x < maxX; x += MIN_PU_SIZE)
    {
      const unsigned int hash = m_pos2Hash[y][x];
      hit += (m_hash2Pos[hash].size() > 1);
      total++;
    }
  }
  return 100 * hit / total;
}


//! \}
