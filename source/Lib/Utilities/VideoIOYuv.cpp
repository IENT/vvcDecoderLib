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

/** \file     VideoIOYuv.cpp
    \brief    YUV file I/O class
*/

#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <memory.h>

#include "CommonLib/Rom.h"
#include "VideoIOYuv.h"
#include "CommonLib/Unit.h"

using namespace std;

#define FLIP_PIC 0

// ====================================================================================================================
// Local Functions
// ====================================================================================================================

/**
 * Scale all pixels in img depending upon sign of shiftbits by a factor of
 * 2<sup>shiftbits</sup>.
 *
 * @param areabuf buffer to be scaled
 * @param shiftbits if zero, no operation performed
 *                  if > 0, multiply by 2<sup>shiftbits</sup>, see scalePlane()
 *                  if < 0, divide and round by 2<sup>shiftbits</sup> and clip,
 *                          see invScalePlane().
 * @param minval  minimum clipping value when dividing.
 * @param maxval  maximum clipping value when dividing.
 */
static Void scalePlane( PelBuf& areaBuf, const Int shiftbits, const Pel minval, const Pel maxval)
{
  const unsigned width  = areaBuf.width;
  const unsigned height = areaBuf.height;
  const unsigned stride = areaBuf.stride;
        Pel*        img = areaBuf.bufAt(0,0);

  if( 0 == shiftbits )
  {
    return;
  }

  if( shiftbits > 0)
  {
    for( unsigned y = 0; y < height; y++, img+=stride)
    {
      for( unsigned x = 0; x < width; x++)
      {
        img[x] <<= shiftbits;
      }
    }
  }
  else if (shiftbits < 0)
  {
    const int shiftbitsr =- shiftbits;
    const Pel rounding = 1 << (shiftbitsr-1);

    for( unsigned y = 0; y < height; y++, img+=stride)
    {
      for( unsigned x = 0; x < width; x++)
      {
        img[x] = Clip3(minval, maxval, Pel((img[x] + rounding) >> shiftbitsr));
      }
    }
  }
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * Open file for reading/writing Y'CbCr frames.
 *
 * Frames read/written have bitdepth fileBitDepth, and are automatically
 * formatted as 8 or 16 bit word values (see VideoIOYuv::write()).
 *
 * Image data read or written is converted to/from internalBitDepth
 * (See scalePlane(), VideoIOYuv::read() and VideoIOYuv::write() for
 * further details).
 *
 * \param pchFile          file name string
 * \param bWriteMode       file open mode: true=write, false=read
 * \param fileBitDepth     bit-depth array of input/output file data.
 * \param MSBExtendedBitDepth
 * \param internalBitDepth bit-depth array to scale image data to/from when reading/writing.
 */
Void VideoIOYuv::open( const std::string &fileName, Bool bWriteMode, const Int fileBitDepth[MAX_NUM_CHANNEL_TYPE], const Int MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE], const Int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  //NOTE: files cannot have bit depth greater than 16
  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_fileBitdepth       [ch] = std::min<UInt>(fileBitDepth[ch], 16);
    m_MSBExtendedBitDepth[ch] = MSBExtendedBitDepth[ch];
    m_bitdepthShift      [ch] = internalBitDepth[ch] - m_MSBExtendedBitDepth[ch];

    if (m_fileBitdepth[ch] > 16)
    {
      if (bWriteMode)
      {
        std::cerr << "\nWARNING: Cannot write a yuv file of bit depth greater than 16 - output will be right-shifted down to 16-bit precision\n" << std::endl;
      }
      else
      {
        EXIT( "ERROR: Cannot read a yuv file of bit depth greater than 16" );
      }
    }
  }

  if ( bWriteMode )
  {
    m_cHandle.open( fileName.c_str(), ios::binary | ios::out );

    if( m_cHandle.fail() )
    {
      EXIT( "failed to write reconstructed YUV file" );
    }
  }
  else
  {
    m_cHandle.open( fileName.c_str(), ios::binary | ios::in );

    if( m_cHandle.fail() )
    {
      EXIT( "failed to open Input YUV file");
    }
  }

  return;
}

Void VideoIOYuv::close()
{
  m_cHandle.close();
}

Bool VideoIOYuv::isEof()
{
  return m_cHandle.eof();
}

Bool VideoIOYuv::isFail()
{
  return m_cHandle.fail();
}

/**
 * Skip numFrames in input.
 *
 * This function correctly handles cases where the input file is not
 * seekable, by consuming bytes.
 */
Void VideoIOYuv::skipFrames(UInt numFrames, UInt width, UInt height, ChromaFormat format)
{
  if (!numFrames)
  {
    return;
  }

  //------------------
  //set the frame size according to the chroma format
  streamoff frameSize = 0;
  UInt wordsize=1; // default to 8-bit, unless a channel with more than 8-bits is detected.
  for (UInt component = 0; component < getNumberValidComponents(format); component++)
  {
    ComponentID compID=ComponentID(component);
    frameSize += (width >> getComponentScaleX(compID, format)) * (height >> getComponentScaleY(compID, format));
    if (m_fileBitdepth[toChannelType(compID)] > 8)
    {
      wordsize=2;
    }
  }
  frameSize *= wordsize;
  //------------------

  const streamoff offset = frameSize * numFrames;

  /* attempt to seek */
  if (!!m_cHandle.seekg(offset, ios::cur))
  {
    return; /* success */
  }
  m_cHandle.clear();

  /* fall back to consuming the input */
  TChar buf[512];
  const streamoff offset_mod_bufsize = offset % sizeof(buf);
  for (streamoff i = 0; i < offset - offset_mod_bufsize; i += sizeof(buf))
  {
    m_cHandle.read(buf, sizeof(buf));
  }
  m_cHandle.read(buf, offset_mod_bufsize);
}

/**
 * Read width*height pixels from fd into dst, optionally
 * padding the left and right edges by edge-extension.  Input may be
 * either 8bit or 16bit little-endian lsb-aligned words.
 *
 * @param dst          destination image plane
 * @param fd           input file stream
 * @param is16bit      true if input file carries > 8bit data, false otherwise.
 * @param stride444    distance between vertically adjacent pixels of dst.
 * @param width444     width of active area in dst.
 * @param height444    height of active area in dst.
 * @param pad_x444     length of horizontal padding.
 * @param pad_y444     length of vertical padding.
 * @param compID       chroma component
 * @param destFormat   chroma format of image
 * @param fileFormat   chroma format of file
 * @param fileBitDepth component bit depth in file
 * @return true for success, false in case of error
 */
static Bool readPlane(Pel* dst,
                      istream& fd,
                      Bool is16bit,
                      UInt stride444,
                      UInt width444,
                      UInt height444,
                      UInt pad_x444,
                      UInt pad_y444,
                      const ComponentID compID,
                      const ChromaFormat destFormat,
                      const ChromaFormat fileFormat,
                      const UInt fileBitDepth)
{
  const UInt csx_file =getComponentScaleX(compID, fileFormat);
  const UInt csy_file =getComponentScaleY(compID, fileFormat);
  const UInt csx_dest =getComponentScaleX(compID, destFormat);
  const UInt csy_dest =getComponentScaleY(compID, destFormat);

  const UInt width_dest       = width444 >>csx_dest;
  const UInt height_dest      = height444>>csy_dest;
  const UInt pad_x_dest       = pad_x444>>csx_dest;
  const UInt pad_y_dest       = pad_y444>>csy_dest;
  const UInt stride_dest      = stride444>>csx_dest;

  const UInt full_width_dest  = width_dest+pad_x_dest;
  const UInt full_height_dest = height_dest+pad_y_dest;

  const UInt stride_file      = (width444 * (is16bit ? 2 : 1)) >> csx_file;
  std::vector<UChar> bufVec(stride_file);
  UChar *buf=&(bufVec[0]);

  Pel  *pDstPad              = dst + stride_dest * height_dest;
  Pel  *pDstBuf              = dst;
  const Int dstbuf_stride    = stride_dest;

  if (compID!=COMPONENT_Y && (fileFormat==CHROMA_400 || destFormat==CHROMA_400))
  {
    if (destFormat!=CHROMA_400)
    {
      // set chrominance data to mid-range: (1<<(fileBitDepth-1))
      const Pel value=Pel(1<<(fileBitDepth-1));
      for (UInt y = 0; y < full_height_dest; y++, pDstBuf+= dstbuf_stride)
      {
        for (UInt x = 0; x < full_width_dest; x++)
        {
          pDstBuf[x] = value;
        }
      }
    }

    if (fileFormat!=CHROMA_400)
    {
      const UInt height_file      = height444>>csy_file;
      fd.seekg(height_file*stride_file, ios::cur);
      if (fd.eof() || fd.fail() )
      {
        return false;
      }
    }
  }
  else
  {
    const UInt mask_y_file=(1<<csy_file)-1;
    const UInt mask_y_dest=(1<<csy_dest)-1;
    for(UInt y444=0; y444<height444; y444++)
    {
      if ((y444&mask_y_file)==0)
      {
        // read a new line
        fd.read(reinterpret_cast<TChar*>(buf), stride_file);
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ((y444&mask_y_dest)==0)
      {
        // process current destination line
        if (csx_file < csx_dest)
        {
          // eg file is 444, dest is 422.
          const UInt sx=csx_dest-csx_file;
          if (!is16bit)
          {
            for (UInt x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = buf[x<<sx];
            }
          }
          else
          {
            for (UInt x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = Pel(buf[(x<<sx)*2+0]) | (Pel(buf[(x<<sx)*2+1])<<8);
            }
          }
        }
        else
        {
          // eg file is 422, dest is 444.
          const UInt sx=csx_file-csx_dest;
          if (!is16bit)
          {
            for (UInt x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = buf[x>>sx];
            }
          }
          else
          {
            for (UInt x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = Pel(buf[(x>>sx)*2+0]) | (Pel(buf[(x>>sx)*2+1])<<8);
            }
          }
        }

        // process right hand side padding
        const Pel val=dst[width_dest-1];
        for (UInt x = width_dest; x < full_width_dest; x++)
        {
          pDstBuf[x] = val;
        }

        pDstBuf+= dstbuf_stride;
      }
    }

    // process lower padding
    for (UInt y = height_dest; y < full_height_dest; y++, pDstPad+=stride_dest)
    {
      for (UInt x = 0; x < full_width_dest; x++)
      {
        pDstPad[x] = (pDstPad - stride_dest)[x];
      }
    }
  }
  return true;
}

/**
 * Write an image plane (width444*height444 pixels) from src into output stream fd.
 *
 * @param fd         output file stream
 * @param src        source image
 * @param is16bit    true if input file carries > 8bit data, false otherwise.
 * @param stride444  distance between vertically adjacent pixels of src.
 * @param width444   width of active area in src.
 * @param height444  height of active area in src.
 * @param compID       chroma component
 * @param srcFormat    chroma format of image
 * @param fileFormat   chroma format of file
 * @param fileBitDepth component bit depth in file
 * @return true for success, false in case of error
 */
static Bool writePlane(ostream& fd, const Pel* src, Bool is16bit,
                       const UInt stride_src,
                       UInt width444, UInt height444,
                       const ComponentID compID,
                       const ChromaFormat srcFormat,
                       const ChromaFormat fileFormat,
                       const Int fileBitDepth)
{
  const UInt csx_file =getComponentScaleX(compID, fileFormat);
  const UInt csy_file =getComponentScaleY(compID, fileFormat);
  const UInt csx_src  =getComponentScaleX(compID, srcFormat);
  const UInt csy_src  =getComponentScaleY(compID, srcFormat);

/*  const UInt stride_src      = stride444>>csx_src;*/

  const UInt stride_file      = (width444 * (is16bit ? 2 : 1)) >> csx_file;
  const UInt width_file       = width444 >>csx_file;
  const UInt height_file      = height444>>csy_file;

  std::vector<UChar> bufVec(stride_file);
  UChar *buf=&(bufVec[0]);

  const Pel *pSrcBuf         = src;
  const Int srcbuf_stride    = stride_src;



  if (compID!=COMPONENT_Y && (fileFormat==CHROMA_400 || srcFormat==CHROMA_400))
  {
    if (fileFormat!=CHROMA_400)
    {
      const UInt value = 1u << (fileBitDepth - 1);

      for(UInt y=0; y< height_file; y++)
      {
        if (!is16bit)
        {
          UChar val(value);
          for (UInt x = 0; x < width_file; x++)
          {
            buf[x]=val;
          }
        }
        else
        {
          UShort val(value);
          for (UInt x = 0; x < width_file; x++)
          {
            buf[2*x+0]= (val>>0) & 0xff;
            buf[2*x+1]= (val>>8) & 0xff;
          }
        }

        fd.write(reinterpret_cast<const TChar*>(buf), stride_file);
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }
    }
  }
  else
  {
    const UInt mask_y_file = (1 << csy_file) - 1;
    const UInt mask_y_src  = (1 << csy_src ) - 1;

    for (UInt y444 = 0; y444 < height444; y444++)
    {
      if ((y444 & mask_y_file) == 0)
      {
        // write a new line
        if (csx_file < csx_src)
        {
          // eg file is 444, source is 422.
          const UInt sx = csx_src - csx_file;
          if (!is16bit)
          {
            for (UInt x = 0; x < width_file; x++)
            {
              buf[x] = (UChar)(pSrcBuf[x>>sx]);
            }
          }
          else
          {
            for (UInt x = 0; x < width_file; x++)
            {
              buf[2*x  ] = (pSrcBuf[x>>sx]>>0) & 0xff;
              buf[2*x+1] = (pSrcBuf[x>>sx]>>8) & 0xff;
            }
          }
        }
        else
        {
          // eg file is 422, source is 444.
          const UInt sx = csx_file - csx_src;
          if (!is16bit)
          {
            for (UInt x = 0; x < width_file; x++)
            {
              buf[x] = (UChar)(pSrcBuf[x<<sx]);
            }
          }
          else
          {
            for (UInt x = 0; x < width_file; x++)
            {
              buf[2*x  ] = (pSrcBuf[x<<sx]>>0) & 0xff;
              buf[2*x+1] = (pSrcBuf[x<<sx]>>8) & 0xff;
            }
          }
        }

        fd.write (reinterpret_cast<const TChar*>(buf), stride_file);
        if (fd.eof() || fd.fail())
        {
          return false;
        }
      }

      if ((y444 & mask_y_src) == 0)
      {
        pSrcBuf += srcbuf_stride;
      }
    }
  }
  return true;
}

static Bool writeField(ostream& fd, const Pel* top, const Pel* bottom, Bool is16bit,
                       const UInt stride_src,
                       UInt width444, UInt height444,
                       const ComponentID compID,
                       const ChromaFormat srcFormat,
                       const ChromaFormat fileFormat,
                       const UInt fileBitDepth, const Bool isTff)
{
  const UInt csx_file =getComponentScaleX(compID, fileFormat);
  const UInt csy_file =getComponentScaleY(compID, fileFormat);
  const UInt csx_src  =getComponentScaleX(compID, srcFormat);
  const UInt csy_src  =getComponentScaleY(compID, srcFormat);

  /*const UInt stride_src      = stride444>>csx_src;*/

  const UInt stride_file      = (width444 * (is16bit ? 2 : 1)) >> csx_file;
  const UInt width_file       = width444 >>csx_file;
  const UInt height_file      = height444>>csy_file;

  std::vector<UChar> bufVec(stride_file * 2);
  UChar *buf=&(bufVec[0]);

  if (compID!=COMPONENT_Y && (fileFormat==CHROMA_400 || srcFormat==CHROMA_400))
  {
    if (fileFormat!=CHROMA_400)
    {
      const UInt value=1<<(fileBitDepth-1);

      for(UInt y=0; y< height_file; y++)
      {
        for (UInt field = 0; field < 2; field++)
        {
          UChar *fieldBuffer = buf + (field * stride_file);

          if (!is16bit)
          {
            UChar val(value);
            for (UInt x = 0; x < width_file; x++)
            {
              fieldBuffer[x]=val;
            }
          }
          else
          {
            UShort val(value);
            for (UInt x = 0; x < width_file; x++)
            {
              fieldBuffer[2*x+0]= (val>>0) & 0xff;
              fieldBuffer[2*x+1]= (val>>8) & 0xff;
            }
          }
        }

        fd.write(reinterpret_cast<const TChar*>(buf), (stride_file * 2));
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }
    }
  }
  else
  {
    const UInt mask_y_file=(1<<csy_file)-1;
    const UInt mask_y_src =(1<<csy_src )-1;
    for(UInt y444=0; y444<height444; y444++)
    {
      if ((y444&mask_y_file)==0)
      {
        for (UInt field = 0; field < 2; field++)
        {
          UChar *fieldBuffer = buf + (field * stride_file);
          const Pel *src     = (((field == 0) && isTff) || ((field == 1) && (!isTff))) ? top : bottom;

          // write a new line
          if (csx_file < csx_src)
          {
            // eg file is 444, source is 422.
            const UInt sx=csx_src-csx_file;
            if (!is16bit)
            {
              for (UInt x = 0; x < width_file; x++)
              {
                fieldBuffer[x] = (UChar)(src[x>>sx]);
              }
            }
            else
            {
              for (UInt x = 0; x < width_file; x++)
              {
                fieldBuffer[2*x  ] = (src[x>>sx]>>0) & 0xff;
                fieldBuffer[2*x+1] = (src[x>>sx]>>8) & 0xff;
              }
            }
          }
          else
          {
            // eg file is 422, src is 444.
            const UInt sx=csx_file-csx_src;
            if (!is16bit)
            {
              for (UInt x = 0; x < width_file; x++)
              {
                fieldBuffer[x] = (UChar)(src[x<<sx]);
              }
            }
            else
            {
              for (UInt x = 0; x < width_file; x++)
              {
                fieldBuffer[2*x  ] = (src[x<<sx]>>0) & 0xff;
                fieldBuffer[2*x+1] = (src[x<<sx]>>8) & 0xff;
              }
            }
          }
        }

        fd.write(reinterpret_cast<const TChar*>(buf), (stride_file * 2));
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ((y444&mask_y_src)==0)
      {
        top    += stride_src;
        bottom += stride_src;
      }

    }
  }
  return true;
}

/**
 * Read one Y'CbCr frame, performing any required input scaling to change
 * from the bitdepth of the input file to the internal bit-depth.
 *
 * If a bit-depth reduction is required, and internalBitdepth >= 8, then
 * the input file is assumed to be ITU-R BT.601/709 compliant, and the
 * resulting data is clipped to the appropriate legal range, as if the
 * file had been provided at the lower-bitdepth compliant to Rec601/709.
 *
 * @param pPicYuvUser      input picture YUV buffer class pointer
 * @param pPicYuvTrueOrg
 * @param ipcsc
 * @param aiPad            source padding size, aiPad[0] = horizontal, aiPad[1] = vertical
 * @param format           chroma format
 * @return true for success, false in case of error
 */
Bool VideoIOYuv::read ( PelUnitBuf& pic, PelUnitBuf& picOrg, const InputColourSpaceConversion ipcsc, Int aiPad[2], ChromaFormat format, const Bool bClipToRec709 )
{
  // check end-of-file
  if ( isEof() )
  {
    return false;
  }

  if (format >= NUM_CHROMA_FORMAT)
  {
    format = picOrg.chromaFormat;
  }

  Bool is16bit = false;

  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_fileBitdepth[ch] > 8)
    {
      is16bit=true;
    }
  }

  const PelBuf areaBufY = picOrg.get(COMPONENT_Y);
  const UInt stride444      = areaBufY.stride;

  // compute actual YUV width & height excluding padding size
  const UInt pad_h444       = aiPad[0];
  const UInt pad_v444       = aiPad[1];

  const UInt width_full444  = areaBufY.width;
  const UInt height_full444 = areaBufY.height;

  const UInt width444       = width_full444 - pad_h444;
  const UInt height444      = height_full444 - pad_v444;

  for( UInt comp=0; comp < ::getNumberValidComponents(format); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const ChannelType chType=toChannelType(compID);

    const Int desired_bitdepth = m_MSBExtendedBitDepth[chType] + m_bitdepthShift[chType];

    const Bool b709Compliance=(bClipToRec709) && (m_bitdepthShift[chType] < 0 && desired_bitdepth >= 8);     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
    const Pel minval = b709Compliance? ((   1 << (desired_bitdepth - 8))   ) : 0;
    const Pel maxval = b709Compliance? ((0xff << (desired_bitdepth - 8)) -1) : (1 << desired_bitdepth) - 1;
    Pel* const dst = picOrg.get(compID).bufAt(0,0);
    if ( ! readPlane( dst, m_cHandle, is16bit, stride444, width444, height444, pad_h444, pad_v444, compID, picOrg.chromaFormat, format, m_fileBitdepth[chType]))
    {
      return false;
    }

    if( (size_t)compID < picOrg.bufs.size() )
    {
      scalePlane( picOrg.get(compID), m_bitdepthShift[chType], minval, maxval);
    }
  }

  ColourSpaceConvert( picOrg, pic, ipcsc, true);

  return true;
}

/**
 * Write one Y'CbCr frame. No bit-depth conversion is performed, pcPicYuv is
 * assumed to be at TVideoIO::m_fileBitdepth depth.
 *
 * @param pPicYuvUser      input picture YUV buffer class pointer
 * @param ipCSC
 * @param confLeft         conformance window left border
 * @param confRight        conformance window right border
 * @param confTop          conformance window top border
 * @param confBottom       conformance window bottom border
 * @param format           chroma format
 * @return true for success, false in case of error
 */
Bool VideoIOYuv::write( const CPelUnitBuf& pic,
                        const InputColourSpaceConversion ipCSC, Int confLeft, Int confRight, Int confTop, Int confBottom, ChromaFormat format, const Bool bClipToRec709 )
{
  PelStorage interm;

  if (ipCSC!=IPCOLOURSPACE_UNCHANGED)
  {
    interm.create( pic.chromaFormat, Area( Position(), pic.Y()) );
    ColourSpaceConvert(pic, interm, ipCSC, false);
  }

  const CPelUnitBuf& picC = (ipCSC==IPCOLOURSPACE_UNCHANGED) ? pic : interm;

  // compute actual YUV frame size excluding padding size
  Bool is16bit = false;
  Bool nonZeroBitDepthShift=false;

  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_fileBitdepth[ch] > 8)
    {
      is16bit=true;
    }
    if (m_bitdepthShift[ch] != 0)
    {
      nonZeroBitDepthShift=true;
    }
  }

  Bool retval = true;
  if (format>=NUM_CHROMA_FORMAT)
  {
    format= picC.chromaFormat;
  }

  PelStorage picZ;
  if (nonZeroBitDepthShift)
  {
    picZ.create( picC.chromaFormat, Area( Position(), picC.Y() ) );
    picZ.copyFrom( picC );

    for(UInt comp=0; comp < ::getNumberValidComponents( picZ.chromaFormat ); comp++)
    {
      const ComponentID compID=ComponentID(comp);
      const ChannelType ch=toChannelType(compID);
      const Bool b709Compliance = bClipToRec709 && (-m_bitdepthShift[ch] < 0 && m_MSBExtendedBitDepth[ch] >= 8);     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
      const Pel minval = b709Compliance? ((   1 << (m_MSBExtendedBitDepth[ch] - 8))   ) : 0;
      const Pel maxval = b709Compliance? ((0xff << (m_MSBExtendedBitDepth[ch] - 8)) -1) : (1 << m_MSBExtendedBitDepth[ch]) - 1;

      scalePlane( picZ.get(compID), -m_bitdepthShift[ch], minval, maxval);
    }
  }

  const CPelUnitBuf& picO = nonZeroBitDepthShift ? picZ : picC;

  const CPelBuf areaY     = picO.get(COMPONENT_Y);
  const UInt    width444  = areaY.width - confLeft - confRight;
  const UInt    height444 = areaY.height -  confTop  - confBottom;

  if ((width444 == 0) || (height444 == 0))
  {
    msg( WARNING, "\nWarning: writing %d x %d luma sample output picture!", width444, height444);
  }

  for(UInt comp=0; retval && comp < ::getNumberValidComponents(format); comp++)
  {
    const ComponentID compID      = ComponentID(comp);
    const ChannelType ch          = toChannelType(compID);
    const UInt        csx         = ::getComponentScaleX(compID, format);
    const UInt        csy         = ::getComponentScaleY(compID, format);
    const CPelBuf     area        = picO.get(compID);
    const Int         planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;
    if (!writePlane (m_cHandle, area.bufAt (0, 0) + planeOffset, is16bit, area.stride,
                     width444, height444, compID, picO.chromaFormat, format, m_fileBitdepth[ch]))
    {
      retval = false;
    }
  }

  return retval;
}

Bool VideoIOYuv::write( const CPelUnitBuf& picTop, const CPelUnitBuf& picBottom, const InputColourSpaceConversion ipCSC, Int confLeft, Int confRight, Int confTop, Int confBottom, ChromaFormat format, const Bool isTff, const Bool bClipToRec709 )
{
  PelStorage intermTop;
  PelStorage intermBottom;

  if( ipCSC != IPCOLOURSPACE_UNCHANGED )
  {
    intermTop   .create( picTop.   chromaFormat, Area( Position(), picTop.   Y()) );
    intermBottom.create( picBottom.chromaFormat, Area( Position(), picBottom.Y()) );
    ColourSpaceConvert( picTop,    intermTop, ipCSC, false);
    ColourSpaceConvert( picBottom, intermBottom, ipCSC, false);
  }
  const CPelUnitBuf& picTopC    = (ipCSC==IPCOLOURSPACE_UNCHANGED) ? picTop    : intermTop;
  const CPelUnitBuf& picBottomC = (ipCSC==IPCOLOURSPACE_UNCHANGED) ? picBottom : intermBottom;

  Bool is16bit = false;
  Bool nonZeroBitDepthShift=false;

  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_fileBitdepth[ch] > 8)
    {
      is16bit=true;
    }
    if (m_bitdepthShift[ch] != 0)
    {
      nonZeroBitDepthShift=true;
    }
  }

  PelStorage picTopZ;
  PelStorage picBottomZ;

  for (UInt field = 0; field < 2; field++)
  {
    const CPelUnitBuf& picC    = (field == 0) ? picTopC : picBottomC;

    if (format>=NUM_CHROMA_FORMAT)
    {
      format = picC.chromaFormat;
    }

    PelStorage& picZ    = (field == 0) ? picTopZ : picBottomZ;

    if (nonZeroBitDepthShift)
    {
      picZ.create( picC.chromaFormat, Area( Position(), picC.Y() ) );
      picZ.copyFrom( picC );

      for(UInt comp=0; comp < ::getNumberValidComponents( picZ.chromaFormat ); comp++)
      {
        const ComponentID compID=ComponentID(comp);
        const ChannelType ch=toChannelType(compID);
        const Bool b709Compliance=bClipToRec709 && (-m_bitdepthShift[ch] < 0 && m_MSBExtendedBitDepth[ch] >= 8);     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
        const Pel minval = b709Compliance? ((   1 << (m_MSBExtendedBitDepth[ch] - 8))   ) : 0;
        const Pel maxval = b709Compliance? ((0xff << (m_MSBExtendedBitDepth[ch] - 8)) -1) : (1 << m_MSBExtendedBitDepth[ch]) - 1;

        scalePlane( picZ.get(compID), -m_bitdepthShift[ch], minval, maxval);
      }
    }
  }

  const CPelUnitBuf& picTopO     = nonZeroBitDepthShift ? picTopZ    : picTopC;
  const CPelUnitBuf& picBottomO  = nonZeroBitDepthShift ? picBottomZ : picBottomC;

  Bool retval = true;
  CHECK( picTopO.chromaFormat != picBottomO.chromaFormat, "Incompatible formats of bottom and top fields" );

  const ChromaFormat dstChrFormat = picTopO.chromaFormat;
  for (UInt comp = 0; retval && comp < ::getNumberValidComponents(dstChrFormat); comp++)
  {
    const ComponentID compID     = ComponentID(comp);
    const ChannelType ch         = toChannelType(compID);
    const CPelBuf     areaTop    = picTopO.   get( compID );
    const CPelBuf     areaBottom = picBottomO.get( compID );
    const CPelBuf     areaTopY   = picTopO.Y();
    const UInt        width444   = areaTopY.width  - (confLeft + confRight);
    const UInt        height444  = areaTopY.height - (confTop + confBottom);

    CHECK(areaTop.width  == areaBottom.width , "Incompatible formats");
    CHECK(areaTop.height == areaBottom.height, "Incompatible formats");
    CHECK(areaTop.stride == areaBottom.stride, "Incompatible formats");

    if ((width444 == 0) || (height444 == 0))
    {
      msg( WARNING, "\nWarning: writing %d x %d luma sample output picture!", width444, height444);
    }

    const UInt csx = ::getComponentScaleX(compID, dstChrFormat );
    const UInt csy = ::getComponentScaleY(compID, dstChrFormat );
    const Int planeOffset  = (confLeft>>csx) + ( confTop>>csy) * areaTop.stride; //offset is for entire frame - round up for top field and down for bottom field

    if (! writeField(m_cHandle,
                     (areaTop.   bufAt(0,0) + planeOffset),
                     (areaBottom.bufAt(0,0) + planeOffset),
                     is16bit,
                     areaTop.stride,
                     width444, height444, compID, dstChrFormat, format, m_fileBitdepth[ch], isTff))
    {
      retval=false;
    }
  }

  return retval;
}


// static member
Void VideoIOYuv::ColourSpaceConvert(const CPelUnitBuf &src, PelUnitBuf &dest, const InputColourSpaceConversion conversion, Bool bIsForwards)
{
  const ChromaFormat  format       = src.chromaFormat;
  const UInt          numValidComp = ::getNumberValidComponents(format);

  switch (conversion)
  {
    case IPCOLOURSPACE_YCbCrtoYYY:
      if (format!=CHROMA_444)
      {
        // only 444 is handled.
        CHECK( format != CHROMA_444, "Chroma format other than 444 not supported" );
      }

      {
        for(UInt comp=0; comp<numValidComp; comp++)
        {
          dest.get( ComponentID(comp) ).copyFrom( src.get( ComponentID(bIsForwards?0:comp) ) );
        }
      }
      break;
    case IPCOLOURSPACE_YCbCrtoYCrCb:
      {
        for(UInt comp=0; comp<numValidComp; comp++)
        {
          dest.get( ComponentID((numValidComp-comp)%numValidComp) ).copyFrom( src.get( ComponentID(comp) ) );
        }
      }
      break;

    case IPCOLOURSPACE_RGBtoGBR:
      {
        if (format!=CHROMA_444)
        {
          // only 444 is handled.
          CHECK(format!=CHROMA_444, "Chroma format other than 444 not supported");
        }

        // channel re-mapping
        for(UInt comp=0; comp<numValidComp; comp++)
        {
          const ComponentID compIDsrc=ComponentID((comp+1)%numValidComp);
          const ComponentID compIDdst=ComponentID(comp);

          dest.get( bIsForwards?compIDdst:compIDsrc ).copyFrom( src.get( bIsForwards?compIDsrc:compIDdst ) );
        }
      }
      break;

    case IPCOLOURSPACE_UNCHANGED:
    default:
      {
        for(UInt comp=0; comp<numValidComp; comp++)
        {
          dest.get( ComponentID(comp) ).copyFrom( src.get( ComponentID(comp) ) );
        }
      }
      break;
  }
}


