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

/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#ifndef __CABACWRITER__
#define __CABACWRITER__

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"


//! \ingroup EncoderLib
//! \{


#if JEM_TOOLS
class CABACDataStore;
#endif
#if JVET_K0346
class EncCu;
#endif
class CABACWriter
{
public:
#if JVET_K0346
  CABACWriter(BinEncIf& binEncoder)   : m_BinEncoder(binEncoder), m_Bitstream(0) { m_TestCtx = m_BinEncoder.getCtx(); m_EncCu = NULL; }
#else
  CABACWriter( BinEncIf& binEncoder ) : m_BinEncoder( binEncoder ), m_Bitstream( 0 ) { m_TestCtx = m_BinEncoder.getCtx(); }
#endif
  virtual ~CABACWriter() {}

public:
#if JEM_TOOLS
  void        initCtxModels             ( const Slice&                  slice,
                                          const CABACDataStore*         cabacDataStore );
#else
  void        initCtxModels             ( const Slice&                  slice );
#endif
#if JVET_K0346
  void        setEncCu(EncCu* pcEncCu) { m_EncCu = pcEncCu; }
#endif
  SliceType   getCtxInitId              ( const Slice&                  slice );
  void        initBitstream             ( OutputBitstream*              bitstream )           { m_Bitstream = bitstream; m_BinEncoder.init( m_Bitstream ); }

  const Ctx&  getCtx                    ()                                            const   { return m_BinEncoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinEncoder.getCtx();  }

  void        start                     ()                                                    { m_BinEncoder.start(); }
  void        resetBits                 ()                                                    { m_BinEncoder.resetBits(); }
  uint64_t    getEstFracBits            ()                                            const   { return m_BinEncoder.getEstFracBits(); }
  uint32_t    getNumBins                ()                                                    { return m_BinEncoder.getNumBins(); }
  bool        isEncoding                ()                                                    { return m_BinEncoder.isEncoding(); }

#if JEM_TOOLS
  void        enableBinStore            ( const Slice&                  slice,
                                          CABACDataStore&               cabacDataStore );
  void        estWinSizes               ( const Slice&                  slice,
                                          CABACDataStore&               cabacDataStore ) const;
#endif
public:
  // slice segment data (clause 7.3.8.1)
  void        end_of_slice              ();

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          (       CodingStructure&        cs,       const UnitArea&   area,       int (&qps)[2],  unsigned ctuRsAddr,  bool skipSao = false );

  // sao (clause 7.3.8.3)
  void        sao                       ( const Slice&                  slice,    unsigned          ctuRsAddr );
  void        sao_block_pars            ( const SAOBlkParam&            saoPars,  const BitDepths&  bitDepths,  bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo );
  void        sao_offset_pars           ( const SAOOffset&              ctbPars,  ComponentID       compID,     bool sliceEnabled,  int bitDepth );
#if JEM_TOOLS
  void        alf                       ( const Slice&                  slice,    const ALFParam& alfParam );
  void        alf                       ( const ALFParam&               alfParam, SliceType sliceType, bool isGALF );

#endif
  // coding (quad)tree (clause 7.3.8.4)
#if JVET_K0230_DUAL_CODING_TREE_UNDER_64x64_BLOCK
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
#else
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx );
#endif
  void        split_cu_flag             ( bool                          split,    const CodingStructure& cs,    Partitioner& pm );
  void        split_cu_mode_mt          ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );

  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( const CodingUnit&             cu );
  void        cu_skip_flag              ( const CodingUnit&             cu );
  void        pred_mode                 ( const CodingUnit&             cu );
#if JEM_TOOLS
  void        pdpc_flag                 ( const CodingUnit&             cu );
#endif
  void        pcm_data                  ( const CodingUnit&             cu );
  void        pcm_flag                  ( const CodingUnit&             cu );
  void        cu_pred_data              ( const CodingUnit&             cu );
#if JEM_TOOLS
  void        cu_lic_flag               ( const CodingUnit&             cu );
  void        obmc_flag                 ( const CodingUnit&             cu );
#endif
  void        intra_luma_pred_modes     ( const CodingUnit&             cu );
  void        intra_luma_pred_mode      ( const PredictionUnit&         pu );
  void        intra_chroma_pred_modes   ( const CodingUnit&             cu );
#if JEM_TOOLS||JVET_K0190
  void        intra_chroma_lmc_mode     ( const PredictionUnit&         pu );
#endif
  void        intra_chroma_pred_mode    ( const PredictionUnit&         pu );
  void        cu_residual               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        rqt_root_cbf              ( const CodingUnit&             cu );
  void        end_of_ctu                ( const CodingUnit&             cu,       CUCtx&            cuCtx );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( const PredictionUnit&         pu );
  void        merge_flag                ( const PredictionUnit&         pu );
#if JEM_TOOLS || JVET_K_AFFINE
  void        affine_flag               ( const CodingUnit&             cu );
#endif
  void        merge_idx                 ( const PredictionUnit&         pu );
#if JVET_K0357_AMVR
  void        imv_mode                  ( const CodingUnit&             cu );
#endif
  void        inter_pred_idc            ( const PredictionUnit&         pu );
  void        ref_idx                   ( const PredictionUnit&         pu,       RefPicList        eRefList );
  void        mvp_flag                  ( const PredictionUnit&         pu,       RefPicList        eRefList );

#if JEM_TOOLS
  void        fruc_mrg_mode             ( const PredictionUnit&         pu );
#endif

  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( const TransformUnit&          tu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx,   ChromaCbfs& chromaCbfs );
#if ENABLE_BMS
#if JVET_K0072
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth, const bool prevCbCbf = false );
#else
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth );
#endif
#else
#if JVET_K0072
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, const bool prevCbCbf = false );
#else
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area );
#endif
#endif

#if JVET_K0357_AMVR
  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( const Mv &rMvd, UChar imv );
#else
  void        mvd_coding                ( const Mv &rMvd );
#endif

  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( const TransformUnit&          tu,       CUCtx&            cuCtx,  ChromaCbfs& chromaCbfs );
#if HM_QTBT_AS_IN_JEM_SYNTAX
  void        transform_unit_qtbt       ( const TransformUnit&          tu,       CUCtx&            cuCtx,  ChromaCbfs& chromaCbfs );
#endif
  void        cu_qp_delta               ( const CodingUnit&             cu,       int               predQP, const SChar qp );
  void        cu_chroma_qp_offset       ( const CodingUnit&             cu );
#if (JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT) && !HM_EMT_NSST_AS_IN_JEM
  void        cu_emt_pertu_idx          ( const CodingUnit&             cu );
#endif

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID );
  void        transform_skip_flag       ( const TransformUnit&          tu,       ComponentID       compID );
#if JEM_TOOLS
  void        residual_nsst_mode        ( const CodingUnit&             cu,       CUCtx&            cuCtx  );
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  void        emt_tu_index              ( const TransformUnit&          tu );
  void        emt_cu_flag               ( const CodingUnit&             cu );
#endif
  void        explicit_rdpcm_mode       ( const TransformUnit&          tu,       ComponentID       compID );
  void        last_sig_coeff            ( CoeffCodingContext&           cctx );
#if JVET_K0072
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff, const int stateTransTable, int& state   );
#else
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff  );
#endif

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( const TransformUnit&          tu,       ComponentID       compID );

private:
  void        unary_max_symbol          ( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  void        unary_max_eqprob          ( unsigned symbol,                                   unsigned maxSymbol );
  void        exp_golomb_eqprob         ( unsigned symbol, unsigned count );
  void        encode_sparse_dt          ( DecisionTree& dt, unsigned toCodeId );

#if JEM_TOOLS
  // alf
  void        alf_aux                   ( const ALFParam&               alfParam, bool isGALF );
  void        alf_filter                ( const ALFParam&               alfParam, bool isGALF, bool bChroma = false );
  void        alf_cu_ctrl               ( const ALFParam&               alfParam );
  void        alf_chroma                ( const ALFParam&               alfParam );
  Int         alf_lengthGolomb          ( int coeffVal, int k );
  void        codeAlfUvlc               ( UInt uiCode );
  void        codeAlfSvlc               ( Int iCode );
  void        alfGolombEncode           ( int coeff, int k );

#endif
  // statistic
  unsigned    get_num_written_bits()    { return m_BinEncoder.getNumWrittenBits(); }

#if JEM_TOOLS
  Void  xWriteTruncBinCode(UInt uiSymbol, UInt uiMaxSymbol);
#if JVET_C0038_NO_PREV_FILTERS
  Void  xWriteEpExGolomb(UInt uiSymbol, UInt uiCount);
#endif

#endif
private:
  BinEncIf&         m_BinEncoder;
  OutputBitstream*  m_Bitstream;
  Ctx               m_TestCtx;
#if JVET_K0346
  EncCu*            m_EncCu;
#endif
};



class CABACEncoder
{
public:
  CABACEncoder()
    : m_CABACWriterStd      ( m_BinEncoderStd )
#if JEM_TOOLS
    , m_CABACWriterJMP      ( m_BinEncoderJMP )
    , m_CABACWriterJAW      ( m_BinEncoderJAW )
    , m_CABACWriterJMPAW    ( m_BinEncoderJMPAW )
#endif
    , m_CABACEstimatorStd   ( m_BitEstimatorStd )
#if JEM_TOOLS
    , m_CABACEstimatorJMP   ( m_BitEstimatorJMP )
    , m_CABACEstimatorJAW   ( m_BitEstimatorJAW )
    , m_CABACEstimatorJMPAW ( m_BitEstimatorJMPAW )
#endif
#if JEM_TOOLS
    , m_CABACWriter         { &m_CABACWriterStd,    &m_CABACWriterJMP,    &m_CABACWriterJAW,    &m_CABACWriterJMPAW    }
    , m_CABACEstimator      { &m_CABACEstimatorStd, &m_CABACEstimatorJMP, &m_CABACEstimatorJAW, &m_CABACEstimatorJMPAW }
#else
    , m_CABACWriter         { &m_CABACWriterStd,   }
    , m_CABACEstimator      { &m_CABACEstimatorStd }
#endif
  {}

#if JEM_TOOLS
  CABACWriter*                getCABACWriter          ( const SPS*   sps   )        { return m_CABACWriter   [sps->getSpsNext().getCABACEngineMode()]; }
  CABACWriter*                getCABACEstimator       ( const SPS*   sps   )        { return m_CABACEstimator[sps->getSpsNext().getCABACEngineMode()]; }
#else
  CABACWriter*                getCABACWriter          ( const SPS*   sps   )        { return m_CABACWriter   [0]; }
  CABACWriter*                getCABACEstimator       ( const SPS*   sps   )        { return m_CABACEstimator[0]; }
#endif
private:
  BinEncoder_Std      m_BinEncoderStd;
#if JEM_TOOLS
  BinEncoder_JMP      m_BinEncoderJMP;
  BinEncoder_JAW      m_BinEncoderJAW;
  BinEncoder_JMPAW    m_BinEncoderJMPAW;
#endif
  BitEstimator_Std    m_BitEstimatorStd;
#if JEM_TOOLS
  BitEstimator_JMP    m_BitEstimatorJMP;
  BitEstimator_JAW    m_BitEstimatorJAW;
  BitEstimator_JMPAW  m_BitEstimatorJMPAW;
#endif
  CABACWriter         m_CABACWriterStd;
#if JEM_TOOLS
  CABACWriter         m_CABACWriterJMP;
  CABACWriter         m_CABACWriterJAW;
  CABACWriter         m_CABACWriterJMPAW;
#endif
  CABACWriter         m_CABACEstimatorStd;
#if JEM_TOOLS
  CABACWriter         m_CABACEstimatorJMP;
  CABACWriter         m_CABACEstimatorJAW;
  CABACWriter         m_CABACEstimatorJMPAW;
#endif
  CABACWriter*        m_CABACWriter   [BPM_NUM-1];
  CABACWriter*        m_CABACEstimator[BPM_NUM-1];
};

//! \}

#endif //__CABACWRITER__
