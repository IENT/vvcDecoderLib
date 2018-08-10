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

/** \file     Contexts.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTS__
#define __CONTEXTS__

#include "CommonDef.h"
#include "Slice.h"

#include <vector>

struct BinFracBits
{
  uint32_t intBits[2];
};


enum BPMType
{
  BPM_Undefined = 0,
  BPM_Std,
#if JEM_TOOLS
  BPM_JMP,
  BPM_JAW,
  BPM_JMPAW,
#endif
  BPM_NUM
};

class ProbModelTables
{
protected:
  static const uint8_t      m_NextState       [128][2];       // Std
  static const uint32_t     m_EstFracBits     [128];          // Std
  static const BinFracBits  m_BinFracBits_128 [128];          // Std
#if JEM_TOOLS
  static const BinFracBits  m_BinFracBits_256 [256];          //             MP   MPI
#endif
  static const uint32_t     m_EstFracProb     [128];          // Std
  static const uint8_t      m_LPSTable_64_4   [ 64][4];       // Std
#if JEM_TOOLS
  static const uint16_t     m_LPSTable_512_64 [512][64];      //       JMP
#endif
  static const uint8_t      m_RenormTable_32  [ 32];          // Std         MP   MPI
#if JEM_TOOLS
  static const uint8_t      m_RenormTable_128 [128];          //       JMP
#endif
#if JEM_TOOLS
  static const uint16_t     m_InistateToCount [128];          //       JMP   MP   MPI
#endif
};



class BinProbModelBase : public ProbModelTables
{
public:
  BinProbModelBase () {}
  ~BinProbModelBase() {}
  static uint32_t estFracBitsEP ()                    { return  (       1 << SCALE_BITS ); }
  static uint32_t estFracBitsEP ( unsigned numBins )  { return  ( numBins << SCALE_BITS ); }
};




class BinProbModel_Std : public BinProbModelBase
{
public:
  BinProbModel_Std  () : m_State( 0 ) {}
  ~BinProbModel_Std ()                {}
public:
  void            init              ( int qp, int initId );
  void            update            ( unsigned bin )                    { m_State = m_NextState       [m_State][bin]; }
  static uint8_t  getDefaultWinSize ()                                  { return uint8_t(0); }
  void            setLog2WindowSize ( uint8_t log2WindowSize )          {}
  void            estFracBitsUpdate ( unsigned bin, uint64_t& b )       {      b += m_EstFracBits     [m_State ^bin];
                                                                          m_State = m_NextState       [m_State][bin]; }
  uint32_t        estFracBits       ( unsigned bin )              const { return    m_EstFracBits     [m_State ^bin]; }
  static uint32_t estFracBitsTrm    ( unsigned bin )                    { return  ( bin ? 0x3bfbb : 0x0010c ); }
  BinFracBits     getFracBitsArray  ()                            const { return    m_BinFracBits_128 [m_State]; }
public:
  uint8_t         state             ()                            const { return  ( m_State >> 1 ); }
  uint8_t         mps               ()                            const { return  ( m_State  & 1 ); }
  uint8_t         getLPS            ( unsigned range )            const { return    m_LPSTable_64_4   [m_State>>1][(range>>6)&3]; }
  static uint8_t  getRenormBitsLPS  ( unsigned LPS )                    { return    m_RenormTable_32  [LPS>>3]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return    1; }
  uint16_t        getState          ()                            const { return    uint16_t(m_State); }
  void            setState          ( uint16_t pState )                 { m_State = uint8_t ( pState); }
public:
  uint64_t        estFracExcessBits ( const BinProbModel_Std& r ) const
  {
    return ( ((uint64_t)m_EstFracProb[m_State^0]) * m_EstFracBits[r.m_State^0]
        +    ((uint64_t)m_EstFracProb[m_State^1]) * m_EstFracBits[r.m_State^1] + ( 1 << ( SCALE_BITS - 1 ) ) ) >> SCALE_BITS;
  }
private:
  uint8_t   m_State;
};


#if JEM_TOOLS
// JEM multi parameter CABAC
class BinProbModel_JMP : public BinProbModelBase
{
public:
  BinProbModel_JMP  () : m_P0( 0 ), m_P1( 0 ) {}
  ~BinProbModel_JMP ()                        {}
public:
  void  init( int qp, int initId );
  void  update( unsigned bin )
  {
    if( bin )
    {
      m_P0 += ( ( 32768 - m_P0 ) >> m_Log2WindowSize0 );
      m_P1 += ( ( 32768 - m_P1 ) >> m_Log2WindowSize1 );
    }
    else
    {
      m_P0 -= ( m_P0 >> m_Log2WindowSize0 );
      m_P1 -= ( m_P1 >> m_Log2WindowSize1 );
    }
  }
  static uint8_t  getDefaultWinSize ()                                  { return uint8_t(0); }
  void            setLog2WindowSize ( uint8_t log2WindowSize )          {}
  void            estFracBitsUpdate ( unsigned bin, uint64_t& b )       { b += estFracBits( bin ); update( bin ); }
  uint32_t        estFracBits       ( unsigned bin )              const { return m_BinFracBits_256[ ( m_P0 + m_P1 ) >> 8 ].intBits[bin]; }
  static uint32_t estFracBitsTrm    ( unsigned bin )                    { return m_BinFracBits_256[0].intBits[ bin ]; }
  BinFracBits     getFracBitsArray  ()                            const { return m_BinFracBits_256[ ( m_P0 + m_P1 ) >> 8 ]; }
public:
  uint16_t        state             ()                            const { return ( (m_P0 + m_P1) >> 1 ); }
  uint16_t        mps               ()                            const { return 1; }
  uint16_t        getLPS            ( unsigned range )            const { return m_LPSTable_512_64[ state() >> 6 ][ (range>>2) - 64 ]; }
  static uint16_t getRenormBitsLPS  ( unsigned LPS )                    { return m_RenormTable_128[ LPS   >> 2 ]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return m_RenormTable_128[ range >> 2 ]; }
  uint16_t        getState          ()                            const { return state(); }
  void            setState          ( uint16_t pState )                 { m_P1 = m_P0 = pState; }
public:
  uint64_t        estFracExcessBits ( const BinProbModel_JMP& r ) const
  {
    const  int32_t prob = (m_P0 + m_P1) >> 1;
    return uint32_t( ( int64_t(32768 - prob) * r.estFracBits(0) + int64_t(prob) * r.estFracBits(1) + ( 1 << ( SCALE_BITS - 1 ) ) ) >> SCALE_BITS);
  }
private:
  static const uint8_t m_Log2WindowSize0 = 4;
  static const uint8_t m_Log2WindowSize1 = 8;
protected:
  uint16_t  m_P0;
  uint16_t  m_P1;
};


// JEM adaptive window CABAC
class BinProbModel_JAW : public BinProbModelBase
{
public:
  BinProbModel_JAW  () : m_P0( 0 ), m_Log2WindowSize0( 4 ) {}
  ~BinProbModel_JAW ()                                     {}
public:
  void  init( int qp, int initId );
  void  update( unsigned bin )
  {
    if( bin )
    {
      m_P0 += ( ( 32768 - m_P0 ) >> m_Log2WindowSize0 );
    }
    else
    {
      m_P0 -= ( m_P0 >> m_Log2WindowSize0 );
    }
  }
  static uint8_t  getDefaultWinSize ()                                  { return m_DefaultLog2WindowSize0; }
  void            setLog2WindowSize ( uint8_t log2WindowSize )          { if( log2WindowSize ) { m_Log2WindowSize0 = log2WindowSize; } }
  void            estFracBitsUpdate ( unsigned bin, uint64_t& b )       { b += estFracBits( bin ); update( bin ); }
  uint32_t        estFracBits       ( unsigned bin )              const { return m_BinFracBits_256[ m_P0 >> 7 ].intBits[bin]; }
  static uint32_t estFracBitsTrm    ( unsigned bin )                    { return m_BinFracBits_256[0].intBits[ bin ]; }
  BinFracBits     getFracBitsArray  ()                            const { return m_BinFracBits_256[ m_P0 >> 7 ]; }
public:
  uint16_t        state             ()                            const { return m_P0; }
  uint16_t        mps               ()                            const { return 1; }
  uint16_t        getLPS            ( unsigned range )            const { return m_LPSTable_512_64[ state() >> 6 ][ (range>>2) - 64 ]; }
  static uint16_t getRenormBitsLPS  ( unsigned LPS )                    { return m_RenormTable_128[ LPS   >> 2 ]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return m_RenormTable_128[ range >> 2 ]; }
  uint16_t        getState          ()                            const { return state(); }
  void            setState          ( uint16_t pState )                 { m_P0 = pState; }
public:
  uint64_t        estFracExcessBits ( const BinProbModel_JAW& r ) const
  {
    const  int32_t prob = m_P0;
    return uint32_t( ( int64_t(32768 - prob) * r.estFracBits(0) + int64_t(prob) * r.estFracBits(1) + ( 1 << ( SCALE_BITS - 1 ) ) ) >> SCALE_BITS);
  }
private:
  static const uint8_t m_DefaultLog2WindowSize0 = 6;
protected:
  uint16_t  m_P0;
  uint8_t   m_Log2WindowSize0;
};


// JEM multi parameter adaptive window CABAC
class BinProbModel_JMPAW : public BinProbModelBase
{
public:
  BinProbModel_JMPAW  () : m_P0( 0 ), m_P1( 0 ), m_Log2WindowSize0( 4 ) {}
  ~BinProbModel_JMPAW ()                                                {}
public:
  void  init( int qp, int initId );
  void  update( unsigned bin )
  {
    if( bin )
    {
      m_P0 += ( ( 32768 - m_P0 ) >> m_Log2WindowSize0 );
      m_P1 += ( ( 32768 - m_P1 ) >> m_Log2WindowSize1 );
    }
    else
    {
      m_P0 -= ( m_P0 >> m_Log2WindowSize0 );
      m_P1 -= ( m_P1 >> m_Log2WindowSize1 );
    }
  }
  static uint8_t  getDefaultWinSize ()                                  { return m_DefaultLog2WindowSize0; }
  void            setLog2WindowSize ( uint8_t log2WindowSize )          { if( log2WindowSize ) { m_Log2WindowSize0 = log2WindowSize; } }
  void            estFracBitsUpdate ( unsigned bin, uint64_t& b )       { b += estFracBits( bin ); update( bin ); }
  uint32_t        estFracBits       ( unsigned bin )              const { return m_BinFracBits_256[ ( m_P0 + m_P1 ) >> 8 ].intBits[bin]; }
  static uint32_t estFracBitsTrm    ( unsigned bin )                    { return m_BinFracBits_256[0].intBits[ bin ]; }
  BinFracBits     getFracBitsArray  ()                            const { return m_BinFracBits_256[ ( m_P0 + m_P1 ) >> 8 ]; }
public:
  uint16_t        state             ()                            const { return ( (m_P0 + m_P1) >> 1 ); }
  uint16_t        mps               ()                            const { return 1; }
  uint16_t        getLPS            ( unsigned range )            const { return m_LPSTable_512_64[ state() >> 6 ][ (range>>2) - 64 ]; }
  static uint16_t getRenormBitsLPS  ( unsigned LPS )                    { return m_RenormTable_128[ LPS   >> 2 ]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return m_RenormTable_128[ range >> 2 ]; }
  uint16_t        getState          ()                            const { return state(); }
  void            setState          ( uint16_t pState )                 { m_P0 = m_P1 = pState; }
public:
  uint64_t        estFracExcessBits ( const BinProbModel_JMPAW& r ) const
  {
    const  int32_t prob = (m_P0 + m_P1) >> 1;
    return uint32_t( ( int64_t(32768 - prob) * r.estFracBits(0) + int64_t(prob) * r.estFracBits(1) + ( 1 << ( SCALE_BITS - 1 ) ) ) >> SCALE_BITS);
  }
private:
  static const uint8_t m_DefaultLog2WindowSize0 = 4;
  static const uint8_t m_Log2WindowSize1        = 8;
protected:
  uint16_t  m_P0;
  uint16_t  m_P1;
  uint8_t   m_Log2WindowSize0;
};

#endif




class CtxSet
{
public:
  CtxSet( uint16_t offset, uint16_t size ) : Offset( offset ), Size( size ) {}
  CtxSet( const CtxSet& ctxSet ) : Offset( ctxSet.Offset ), Size( ctxSet.Size ) {}
  CtxSet( std::initializer_list<CtxSet> ctxSets );
public:
  uint16_t  operator()  ()  const
  {
    return Offset;
  }
  uint16_t  operator()  ( uint16_t inc )  const
  {
    CHECKD( inc >= Size, "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "]." );
    return Offset + inc;
  }
  bool operator== ( const CtxSet& ctxSet ) const
  {
    return ( Offset == ctxSet.Offset && Size == ctxSet.Size );
  }
  bool operator!= ( const CtxSet& ctxSet ) const
  {
    return ( Offset != ctxSet.Offset || Size != ctxSet.Size );
  }
public:
  uint16_t  Offset;
  uint16_t  Size;
};



class ContextSetCfg
{
public:
  // context sets: specify offset and size
  static const CtxSet   SplitFlag;
  static const CtxSet   BTSplitFlag;
  static const CtxSet   SkipFlag;
  static const CtxSet   MergeFlag;
  static const CtxSet   MergeIdx;
  static const CtxSet   PartSize;
  static const CtxSet   PredMode;
  static const CtxSet   IPredMode       [2];    // [ ChannelType ]
  static const CtxSet   PdpcFlag;
  static const CtxSet   DeltaQP;
  static const CtxSet   InterDir;
  static const CtxSet   RefPic;
#if JEM_TOOLS
  static const CtxSet   AffineFlag;
#endif
  static const CtxSet   Mvd;
  static const CtxSet   TransSubdivFlag;
  static const CtxSet   QtRootCbf;
#if JVET_K0072
  static const CtxSet   QtCbf           [3];    // [ channel ]
#else
  static const CtxSet   QtCbf           [2];    // [ ChannelType ]
#endif
  static const CtxSet   SigCoeffGroup   [4];    // [ ChannelType ]
#if JVET_K0072
#else
  static const CtxSet   SigFlag         [4];    // [ ChannelType ]
#endif
  static const CtxSet   LastX           [2];    // [ ChannelType ]
  static const CtxSet   LastY           [2];    // [ ChannelType ]
#if JVET_K0072
  static const CtxSet   SigFlag         [6];    // [ ChannelType + State ]
  static const CtxSet   ParFlag         [2];    // [ ChannelType ]
  static const CtxSet   GtxFlag         [4];    // [ ChannelType + x ]
#else
  static const CtxSet   GreaterOneFlag  [8];    // [ ContextSet  ]
  static const CtxSet   GreaterTwoFlag;
#endif
  static const CtxSet   MVPIdx;
  static const CtxSet   SaoMergeFlag;
  static const CtxSet   SaoTypeIdx;
#if JEM_TOOLS && !JVET_K0371_ALF
  static const CtxSet   AlfCUCtrlFlags;
  static const CtxSet   AlfUvlcSCModel;
#endif
  static const CtxSet   TransformSkipFlag;
  static const CtxSet   TransquantBypassFlag;
#if JEM_TOOLS
  static const CtxSet   NSSTIdx;
#endif
  static const CtxSet   RdpcmFlag;
  static const CtxSet   RdpcmDir;
#if JEM_TOOLS
  static const CtxSet   EMTTuIndex;
  static const CtxSet   EMTCuFlag;
#endif
  static const CtxSet   CrossCompPred;
  static const CtxSet   ChromaQpAdjFlag;
  static const CtxSet   ChromaQpAdjIdc;
#if JEM_TOOLS
  static const CtxSet   ImvFlag;
  static const CtxSet   LICFlag;
  static const CtxSet   ObmcFlag;
  static const CtxSet   FrucFlag;
  static const CtxSet   FrucMode;
#endif
#if JVET_K0371_ALF
  static const CtxSet   ctbAlfFlag;
#endif
  static const unsigned NumberOfContexts;

  // combined sets for less complex copying
  // NOTE: The contained CtxSet's should directly follow each other in the initalization list;
  //       otherwise, you will copy more elements than you want !!!
  static const CtxSet   Sao;

public:
  static const std::vector<uint8_t>&  getInitTable( unsigned initId );
private:
  static std::vector<std::vector<uint8_t> > sm_InitTables;
  static CtxSet addCtxSet( std::initializer_list<std::initializer_list<uint8_t> > initSet2d );
};



class FracBitsAccess
{
public:
  virtual BinFracBits getFracBitsArray( unsigned ctxId ) const = 0;
};



template <class BinProbModel>
class CtxStore : public FracBitsAccess
{
public:
  CtxStore();
  CtxStore( bool dummy );
  CtxStore( const CtxStore<BinProbModel>& ctxStore );
public:
  void copyFrom   ( const CtxStore<BinProbModel>& src )                        { checkInit(); ::memcpy( m_Ctx,               src.m_Ctx,               sizeof( BinProbModel ) * ContextSetCfg::NumberOfContexts ); }
  void copyFrom   ( const CtxStore<BinProbModel>& src, const CtxSet& ctxSet )  { checkInit(); ::memcpy( m_Ctx+ctxSet.Offset, src.m_Ctx+ctxSet.Offset, sizeof( BinProbModel ) * ctxSet.Size ); }
  void init       ( int qp, int initId );
  void setWinSizes( const std::vector<uint8_t>&   log2WindowSizes );
  void loadPStates( const std::vector<uint16_t>&  probStates );
  void savePStates( std::vector<uint16_t>&        probStates )  const;

  const BinProbModel& operator[]      ( unsigned  ctxId  )  const { return m_Ctx[ctxId]; }
  BinProbModel&       operator[]      ( unsigned  ctxId  )        { return m_Ctx[ctxId]; }
  uint32_t            estFracBits     ( unsigned  bin,
                                        unsigned  ctxId  )  const { return m_Ctx[ctxId].estFracBits(bin); }

  BinFracBits         getFracBitsArray( unsigned  ctxId  )  const { return m_Ctx[ctxId].getFracBitsArray(); }

private:
  inline void checkInit() { if( m_Ctx ) return; m_CtxBuffer.resize( ContextSetCfg::NumberOfContexts ); m_Ctx = m_CtxBuffer.data(); }
private:
  std::vector<BinProbModel> m_CtxBuffer;
  BinProbModel*             m_Ctx;
};



class Ctx;
class SubCtx
{
  friend class Ctx;
public:
  SubCtx( const CtxSet& ctxSet, const Ctx& ctx ) : m_CtxSet( ctxSet          ), m_Ctx( ctx          ) {}
  SubCtx( const SubCtx& subCtx )                 : m_CtxSet( subCtx.m_CtxSet ), m_Ctx( subCtx.m_Ctx ) {}
  const SubCtx& operator= ( const SubCtx& ) = delete;
private:
  const CtxSet  m_CtxSet;
  const Ctx&    m_Ctx;
};



class Ctx : public ContextSetCfg
{
public:
  Ctx();
  Ctx( const BinProbModel_Std*    dummy );
#if JEM_TOOLS
  Ctx( const BinProbModel_JMP*    dummy );
  Ctx( const BinProbModel_JAW*    dummy );
  Ctx( const BinProbModel_JMPAW*  dummy );
#endif
  Ctx( const Ctx&                 ctx   );

public:
#if JEM_TOOLS
  static uint8_t getDefaultWindowSize( unsigned cabacEngineId )
  {
    uint8_t defWinSize  = 0;
    BPMType bpmType     = BPMType( cabacEngineId + BPM_Std );
    switch( bpmType )
    {
    case BPM_JAW:   defWinSize = BinProbModel_JAW  ::getDefaultWinSize(); break;
    case BPM_JMPAW: defWinSize = BinProbModel_JMPAW::getDefaultWinSize(); break;
    default:        break;
    }
    return defWinSize;
  }
  uint8_t getDefaultWindowSize()
  {
    uint8_t defWinSize  = 0;
    switch( m_BPMType )
    {
    case BPM_JAW:   defWinSize = BinProbModel_JAW  ::getDefaultWinSize(); break;
    case BPM_JMPAW: defWinSize = BinProbModel_JMPAW::getDefaultWinSize(); break;
    default:        break;
    }
    return defWinSize;
  }
#endif
  const Ctx& operator= ( const Ctx& ctx )
  {
    m_BPMType = ctx.m_BPMType;
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .copyFrom( ctx.m_CtxStore_Std   );  break;
#if JEM_TOOLS
    case BPM_JMP:   m_CtxStore_JMP  .copyFrom( ctx.m_CtxStore_JMP   );  break;
    case BPM_JAW:   m_CtxStore_JAW  .copyFrom( ctx.m_CtxStore_JAW   );  break;
    case BPM_JMPAW: m_CtxStore_JMPAW.copyFrom( ctx.m_CtxStore_JMPAW );  break;
#endif
    default:        break;
    }
    ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
    return *this;
  }

  SubCtx operator= ( SubCtx&& subCtx )
  {
    m_BPMType = subCtx.m_Ctx.m_BPMType;
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .copyFrom( subCtx.m_Ctx.m_CtxStore_Std,   subCtx.m_CtxSet );  break;
#if JEM_TOOLS
    case BPM_JMP:   m_CtxStore_JMP  .copyFrom( subCtx.m_Ctx.m_CtxStore_JMP,   subCtx.m_CtxSet );  break;
    case BPM_JAW:   m_CtxStore_JAW  .copyFrom( subCtx.m_Ctx.m_CtxStore_JAW,   subCtx.m_CtxSet );  break;
    case BPM_JMPAW: m_CtxStore_JMPAW.copyFrom( subCtx.m_Ctx.m_CtxStore_JMPAW, subCtx.m_CtxSet );  break;
#endif
    default:        break;
    }
    return std::move(subCtx);
  }

  void  init ( int qp, int initId )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .init( qp, initId );  break;
#if JEM_TOOLS
    case BPM_JMP:   m_CtxStore_JMP  .init( qp, initId );  break;
    case BPM_JAW:   m_CtxStore_JAW  .init( qp, initId );  break;
    case BPM_JMPAW: m_CtxStore_JMPAW.init( qp, initId );  break;
#endif
    default:        break;
    }
    for( std::size_t k = 0; k < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS; k++ )
    {
      m_GRAdaptStats[k] = 0;
    }
  }

  void  loadPStates( const std::vector<uint16_t>& probStates )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .loadPStates( probStates );  break;
#if JEM_TOOLS
    case BPM_JMP:   m_CtxStore_JMP  .loadPStates( probStates );  break;
    case BPM_JAW:   m_CtxStore_JAW  .loadPStates( probStates );  break;
    case BPM_JMPAW: m_CtxStore_JMPAW.loadPStates( probStates );  break;
#endif
    default:        break;
    }
  }

  void  savePStates( std::vector<uint16_t>& probStates ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_CtxStore_Std  .savePStates( probStates );  break;
#if JEM_TOOLS
    case BPM_JMP:   m_CtxStore_JMP  .savePStates( probStates );  break;
    case BPM_JAW:   m_CtxStore_JAW  .savePStates( probStates );  break;
    case BPM_JMPAW: m_CtxStore_JMPAW.savePStates( probStates );  break;
#endif
    default:        break;
    }
  }

#if JEM_TOOLS
  void  setWinSizes( const std::vector<uint8_t>* log2WindowSizes )
  {
    if( log2WindowSizes )
    {
      switch( m_BPMType )
      {
      case BPM_JAW:   m_CtxStore_JAW  .setWinSizes( *log2WindowSizes );  break;
      case BPM_JMPAW: m_CtxStore_JMPAW.setWinSizes( *log2WindowSizes );  break;
      default:        break;
      }
    }
  }
#endif
  void  initCtxAndWinSize( unsigned ctxId, const Ctx& ctx, const uint8_t winSize )
  {
    switch( m_BPMType )
    {
    case BPM_Std:
      m_CtxStore_Std  [ctxId] = ctx.m_CtxStore_Std  [ctxId];
      m_CtxStore_Std  [ctxId] . setLog2WindowSize   (winSize);
      break;
#if JEM_TOOLS
    case BPM_JMP:
      m_CtxStore_JMP  [ctxId] = ctx.m_CtxStore_JMP  [ctxId];
      m_CtxStore_JMP  [ctxId] . setLog2WindowSize   (winSize);
      break;
    case BPM_JAW:
      m_CtxStore_JAW  [ctxId] = ctx.m_CtxStore_JAW  [ctxId];
      m_CtxStore_JAW  [ctxId] . setLog2WindowSize   (winSize);
      break;
    case BPM_JMPAW:
      m_CtxStore_JMPAW[ctxId] = ctx.m_CtxStore_JMPAW[ctxId];
      m_CtxStore_JMPAW[ctxId] . setLog2WindowSize   (winSize);
      break;
#endif
    default:
      break;
    }
  }

  const unsigned&     getGRAdaptStats ( unsigned      id )      const { return m_GRAdaptStats[id]; }
  unsigned&           getGRAdaptStats ( unsigned      id )            { return m_GRAdaptStats[id]; }

public:
  unsigned            getBPMType      ()                        const { return m_BPMType; }
  const Ctx&          getCtx          ()                        const { return *this; }
  Ctx&                getCtx          ()                              { return *this; }

  explicit operator   const CtxStore<BinProbModel_Std>  &()     const { return m_CtxStore_Std; }
  explicit operator         CtxStore<BinProbModel_Std>  &()           { return m_CtxStore_Std; }
#if JEM_TOOLS
  explicit operator   const CtxStore<BinProbModel_JMP>  &()     const { return m_CtxStore_JMP;  }
  explicit operator         CtxStore<BinProbModel_JMP>  &()           { return m_CtxStore_JMP;  }
  explicit operator   const CtxStore<BinProbModel_JAW>  &()     const { return m_CtxStore_JAW;  }
  explicit operator         CtxStore<BinProbModel_JAW>  &()           { return m_CtxStore_JAW;  }
  explicit operator   const CtxStore<BinProbModel_JMPAW>&()     const { return m_CtxStore_JMPAW;  }
  explicit operator         CtxStore<BinProbModel_JMPAW>&()           { return m_CtxStore_JMPAW;  }
#endif

  const FracBitsAccess&   getFracBitsAcess()  const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   return m_CtxStore_Std;
#if JEM_TOOLS
    case BPM_JMP:   return m_CtxStore_JMP;
    case BPM_JAW:   return m_CtxStore_JAW;
    case BPM_JMPAW: return m_CtxStore_JMPAW;
#endif
    default:        THROW("BPMType out of range");
    }
  }

private:
  BPMType                       m_BPMType;
  CtxStore<BinProbModel_Std>    m_CtxStore_Std;
#if JEM_TOOLS
  CtxStore<BinProbModel_JMP>    m_CtxStore_JMP;
  CtxStore<BinProbModel_JAW>    m_CtxStore_JAW;
  CtxStore<BinProbModel_JMPAW>  m_CtxStore_JMPAW;
#endif
protected:
  unsigned                      m_GRAdaptStats[RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS];
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

public:
  int64_t cacheId;
  bool    cacheUsed;
#endif
};



typedef dynamic_cache<Ctx> CtxCache;

class TempCtx
{
  TempCtx( const TempCtx& ) = delete;
  const TempCtx& operator=( const TempCtx& ) = delete;
public:
  TempCtx ( CtxCache* cache )                     : m_ctx( *cache->get() ), m_cache( cache ) {}
  TempCtx ( CtxCache* cache, const Ctx& ctx    )  : m_ctx( *cache->get() ), m_cache( cache ) { m_ctx = ctx; }
  TempCtx ( CtxCache* cache, SubCtx&&   subCtx )  : m_ctx( *cache->get() ), m_cache( cache ) { m_ctx = std::forward<SubCtx>(subCtx); }
  ~TempCtx()                                      { m_cache->cache( &m_ctx ); }
  const Ctx& operator=( const Ctx& ctx )          { return ( m_ctx = ctx ); }
  SubCtx     operator=( SubCtx&&   subCtx )       { return m_ctx = std::forward<SubCtx>( subCtx ); }
  operator const Ctx& ()           const          { return m_ctx; }
  operator       Ctx& ()                          { return m_ctx; }
private:
  Ctx&      m_ctx;
  CtxCache* m_cache;
};



class CtxStateBuf
{
public:
  CtxStateBuf () : m_valid(false)                 {}
  ~CtxStateBuf()                                  {}
  inline void reset() { m_valid = false; }
  inline bool getIfValid(Ctx &ctx) const
  {
    if (m_valid)
    {
      ctx.loadPStates(m_states);
      return true;
    }
    return false;
  }
  inline void store(const Ctx &ctx)
  {
    ctx.savePStates(m_states);
    m_valid = true;
  }

private:
  std::vector<uint16_t> m_states;
  bool                  m_valid;
};

class CtxStateArray
{
public:
  CtxStateArray () {}
  ~CtxStateArray() {}
  inline void resetAll()
  {
    for (std::size_t k = 0; k < m_data.size(); k++)
    {
      m_data[k].reset();
    }
  }
  inline void resize(std::size_t reqSize)
  {
    if (m_data.size() < reqSize)
    {
      m_data.resize(reqSize);
    }
  }
  inline bool getIfValid(Ctx &ctx, unsigned id) const
  {
    if (id < m_data.size())
    {
      return m_data[id].getIfValid(ctx);
    }
    return false;
  }
  inline void store(const Ctx &ctx, unsigned id)
  {
    if (id >= m_data.size())
    {
      resize(id + 1);
    }
    m_data[id].store(ctx);
  }

private:
  std::vector<CtxStateBuf> m_data;
};

#if JEM_TOOLS
class CtxStateStore
{
public:
  CtxStateStore () { static_assert( ( B_SLICE < NUMBER_OF_SLICE_TYPES - 1 ) && ( P_SLICE < NUMBER_OF_SLICE_TYPES - 1 ), "index out of bound" ); }
  ~CtxStateStore() {}

  void storeCtx( const Slice* slice, const Ctx& ctx, int id )
  {
    SliceType t = slice->getSliceType();
    if( t != I_SLICE )
    {
      CtxStateArray& ctxStateArray = m_stateBuf[ t ][ slice->getSliceQpBase() ];
      ctxStateArray.resize( slice->getPPS()->pcv->heightInCtus + 1 );
      ctxStateArray.store ( ctx, id );
    }
  }
  bool loadCtx( const Slice* slice, Ctx& ctx, int id ) const
  {
    SliceType t = slice->getSliceType();
    if( t != I_SLICE )
    {
      const CtxStateArray& ctxStateArray = m_stateBuf[ t ][ slice->getSliceQpBase() ];
      return ctxStateArray.getIfValid( ctx, id );
    }
    return false;
  }
  void clearValid()
  {
    for( int t = 0; t < NUMBER_OF_SLICE_TYPES - 1; t++ )
    {
      for( int q = 0; q <= MAX_QP; q++ )
      {
        m_stateBuf[t][q].resetAll();
      }
    }
  }

private:
  CtxStateArray m_stateBuf[ NUMBER_OF_SLICE_TYPES - 1 ][ MAX_QP + 1 ];
};
#endif



class CtxWSizeSet
{
public:
  CtxWSizeSet() : m_valid(false), m_changes(false), m_coded(false), m_log2WinSizes() {}
  bool                          isValid()                   const { return m_valid; }
  const std::vector<uint8_t>&   getWinSizeBuffer()          const { return m_log2WinSizes; }
  std::vector<uint8_t>&         getWinSizeBuffer()                { return m_log2WinSizes; }
  int                           getMode()                   const { return ( !m_valid || !m_changes ? 0 : ( m_coded ? 2 : 1 ) ); }
  void                          setInvalid()                      { m_valid = m_changes = m_coded = false; }
  void                          setCoded()                        { m_coded = true; }
  void                          setValidOnly()                    { m_valid = true; }
  void                          setValid( uint8_t defSize )
  {
    m_valid   = true;
    m_changes = false;
    for( std::size_t n = 0; n < m_log2WinSizes.size(); n++ )
    {
      if( m_log2WinSizes[n] && m_log2WinSizes[n] != defSize )
      {
        m_changes = true;
        return;
      }
    }
  }
private:
  bool                  m_valid;
  bool                  m_changes;
  bool                  m_coded;
  std::vector<uint8_t>  m_log2WinSizes;
};

#if JEM_TOOLS 
class CtxWSizeStore
{
public:
  CtxWSizeStore ();
  ~CtxWSizeStore() {}

  void                        checkInit             ( const SPS*   sps    );
  void                        updateState           ( const Slice* slice,
                                                      const bool   enc    );
  std::vector<uint8_t>&       getReadBuffer         ()                                { return m_readWriteBuffer; }
  const std::vector<uint8_t>& getWriteBuffer        ( const Slice* slice  )           { xSetReadWriteBuffer(slice); return m_readWriteBuffer; }
  bool                        validWinSizes         ( const Slice* slice  )   const;
  const std::vector<uint8_t>* getWinSizes           ( const Slice* slice  )   const;
  CtxWSizeSet&                getWSizeSet           ( const Slice* slice  )           { return m_winSizes[slice->getSliceType()][slice->getSliceQpBase()]; }
  std::streamsize             getNumCodeIds         ()                        const   { return m_readWriteBuffer.size(); }
  int                         getCtxId              ( const std::size_t id)   const   { return m_codeId2ctxId[id]; }

  void                        setSliceWinUpdateMode ( Slice*       slice  )   const
  {
    slice->setCabacWinUpdateMode( m_winSizes[ slice->getSliceType() ][ slice->getSliceQpBase() ].getMode() );
  }

private:
  void                        xSetAllInvalid        ();
  void                        xApplyReadWriteBuffer ( const Slice* slice );
  void                        xSetReadWriteBuffer   ( const Slice* slice );
  void                        xInitMappingTable     ( const SPS* sps );

private:
  bool                        m_isInitialized;
  std::vector<uint8_t>        m_readWriteBuffer;
  std::vector<int>            m_codeId2ctxId;
  CtxWSizeSet                 m_winSizes[ NUMBER_OF_SLICE_TYPES ][ MAX_QP + 1 ];
};
#endif


#if JEM_TOOLS
class CABACDataStore
{
public:
#if JEM_TOOLS 
  void                        checkInit               ( const SPS*    sps  )        {        m_CtxWSizeStore.checkInit(sps); }
  bool                        validWinSizes           ( const Slice* slice )  const { return m_CtxWSizeStore.validWinSizes(slice); }
  void                        setSliceWinUpdateMode   (       Slice* slice )  const {        m_CtxWSizeStore.setSliceWinUpdateMode(slice); }
  void                        setWSizeSetValid        ( const Slice* slice )        {        m_CtxWSizeStore.getWSizeSet(slice).setValid( Ctx::getDefaultWindowSize( slice->getSPS()->getSpsNext().getCABACEngineMode() ) ); }
  void                        setWSizeSetCoded        ( const Slice* slice )        {        m_CtxWSizeStore.getWSizeSet(slice).setCoded(); }
  const std::vector<uint8_t>& getWSizeWriteBuffer     ( const Slice* slice )        { return m_CtxWSizeStore.getWriteBuffer(slice); }
  std::vector<uint8_t>&       getWSizeReadBuffer      ()                            { return m_CtxWSizeStore.getReadBuffer(); }
  const std::vector<uint8_t>* getWinSizes             ( const Slice* slice )  const { return m_CtxWSizeStore.getWinSizes(slice); }
  std::vector<uint8_t>&       getWinSizeBuffer        ( const Slice* slice )        { return m_CtxWSizeStore.getWSizeSet(slice).getWinSizeBuffer(); }
  std::size_t                 getNumWSizeCodeIds      ()                      const { return m_CtxWSizeStore.getNumCodeIds(); }
  int                         getCtxIdFromWSizeCodeId ( std::size_t  id    )  const { return m_CtxWSizeStore.getCtxId(id); }
#endif
  bool loadCtxStates     ( const Slice* slice,       Ctx& ctx, int id ) const { return m_CtxStateStore.loadCtx ( slice, ctx, id ); }
  void storeCtxStates    ( const Slice* slice, const Ctx& ctx, int id )       {        m_CtxStateStore.storeCtx( slice, ctx, id ); }
#if JEM_TOOLS 
  void updateBufferState ( const Slice* slice, bool enc )
#else
  void updateBufferState(const Slice* slice)
#endif
  {
    if ( slice->getPendingRasInit() )
    {
      m_CtxStateStore.clearValid();
    }
#if JEM_TOOLS 
    m_CtxWSizeStore.updateState( slice, enc );
#endif
  }
private:
  CtxStateStore       m_CtxStateStore;
#if JEM_TOOLS 
  CtxWSizeStore       m_CtxWSizeStore;
#endif
};
#endif
#endif
