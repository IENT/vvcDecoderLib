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

/** \file     EncLib.h
    \brief    encoder class (header)
*/

#ifndef __ENCTOP__
#define __ENCTOP__

// Include files
#include "CommonLib/TrQuant.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/NAL.h"
#if JEM_TOOLS
#include "CommonLib/BilateralFilter.h"
#endif

#include "Utilities/VideoIOYuv.h"

#include "EncCfg.h"
#include "EncGOP.h"
#include "EncSlice.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "InterSearch.h"
#include "IntraSearch.h"
#include "EncSampleAdaptiveOffset.h"
#if JEM_TOOLS
#include "EncAdaptiveLoopFilter.h"
#endif
#include "RateCtrl.h"


//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class EncLib : public EncCfg
{
private:
  // picture
  Int                       m_iPOCLast;                           ///< time index (POC)
  Int                       m_iNumPicRcvd;                        ///< number of received pictures
  UInt                      m_uiNumAllPicCoded;                   ///< number of coded pictures
  PicList                   m_cListPic;                           ///< dynamic list of pictures

  // encoder search
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  InterSearch              *m_cInterSearch;                       ///< encoder search class
  IntraSearch              *m_cIntraSearch;                       ///< encoder search class
#else
  InterSearch               m_cInterSearch;                       ///< encoder search class
  IntraSearch               m_cIntraSearch;                       ///< encoder search class
#endif
  // coding tool
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  TrQuant                  *m_cTrQuant;                           ///< transform & quantization class
#else
  TrQuant                   m_cTrQuant;                           ///< transform & quantization class
#endif
  LoopFilter                m_cLoopFilter;                        ///< deblocking filter class
  EncSampleAdaptiveOffset   m_cEncSAO;                            ///< sample adaptive offset class
#if JEM_TOOLS
  EncAdaptiveLoopFilter     m_cEncALF;
#endif
  HLSWriter                 m_HLSWriter;                          ///< CAVLC encoder
#if JEM_TOOLS
  CABACDataStore            m_CABACDataStore;
#endif
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  CABACEncoder             *m_CABACEncoder;
#if JEM_TOOLS
  BilateralFilter          *m_bilateralFilter;
#endif
#else
  CABACEncoder              m_CABACEncoder;
#if JEM_TOOLS
  BilateralFilter           m_bilateralFilter;
#endif
#endif

  // processing unit
  EncGOP                    m_cGOPEncoder;                        ///< GOP encoder
  EncSlice                  m_cSliceEncoder;                      ///< slice encoder
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  EncCu                    *m_cCuEncoder;                         ///< CU encoder
#else
  EncCu                     m_cCuEncoder;                         ///< CU encoder
#endif
  // SPS
  ParameterSetMap<SPS>      m_spsMap;                             ///< SPS. This is the base value. This is copied to PicSym
  ParameterSetMap<PPS>      m_ppsMap;                             ///< PPS. This is the base value. This is copied to PicSym
  // RD cost computation
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  RdCost                   *m_cRdCost;                            ///< RD cost computation class
  CtxCache                 *m_CtxCache;                           ///< buffer for temporarily stored context models
#else
  RdCost                    m_cRdCost;                            ///< RD cost computation class
  CtxCache                  m_CtxCache;                           ///< buffer for temporarily stored context models
#endif
  // quality control
  RateCtrl                  m_cRateCtrl;                          ///< Rate control class

  AUWriterIf*               m_AUWriterIf;

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  int                       m_numCuEncStacks;
#endif


public:
  Ctx                       m_entropyCodingSyncContextState;      ///< leave in addition to vector for compatibility
#if ENABLE_WPP_PARALLELISM
  std::vector<Ctx>          m_entropyCodingSyncContextStateVec;   ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row
#endif

protected:
  Void  xGetNewPicBuffer  ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, Int ppsId ); ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
#if HEVC_VPS
  Void  xInitVPS          (VPS &vps, const SPS &sps); ///< initialize VPS from encoder options
#endif
  Void  xInitSPS          (SPS &sps);                 ///< initialize SPS from encoder options
  Void  xInitPPS          (PPS &pps, const SPS &sps); ///< initialize PPS from encoder options
#if HEVC_USE_SCALING_LISTS
  Void  xInitScalingLists (SPS &sps, PPS &pps);   ///< initialize scaling lists
#endif
  Void  xInitHrdParameters(SPS &sps);                 ///< initialize HRD parameters

#if HEVC_TILES_WPP
  Void  xInitPPSforTiles  (PPS &pps);
#endif
  Void  xInitRPS          (SPS &sps, Bool isFieldCoding);           ///< initialize PPS from encoder options

public:
  EncLib();
  virtual ~EncLib();

  Void      create          ();
  Void      destroy         ();
  Void      init            ( Bool isFieldCoding, AUWriterIf* auWriterIf );
  Void      deletePicBuffer ();

  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------

  AUWriterIf*             getAUWriterIf         ()              { return   m_AUWriterIf;           }
  PicList*                getListPic            ()              { return  &m_cListPic;             }
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  InterSearch*            getInterSearch        ( int jId = 0 ) { return  &m_cInterSearch[jId];    }
  IntraSearch*            getIntraSearch        ( int jId = 0 ) { return  &m_cIntraSearch[jId];    }

  TrQuant*                getTrQuant            ( int jId = 0 ) { return  &m_cTrQuant[jId];        }
#else
  InterSearch*            getInterSearch        ()              { return  &m_cInterSearch;         }
  IntraSearch*            getIntraSearch        ()              { return  &m_cIntraSearch;         }

  TrQuant*                getTrQuant            ()              { return  &m_cTrQuant;             }
#endif
  LoopFilter*             getLoopFilter         ()              { return  &m_cLoopFilter;          }
  EncSampleAdaptiveOffset* getSAO               ()              { return  &m_cEncSAO;              }
#if JEM_TOOLS
  EncAdaptiveLoopFilter*  getALF                ()              { return  &m_cEncALF;              }
#endif
  EncGOP*                 getGOPEncoder         ()              { return  &m_cGOPEncoder;          }
  EncSlice*               getSliceEncoder       ()              { return  &m_cSliceEncoder;        }
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  EncCu*                  getCuEncoder          ( int jId = 0 ) { return  &m_cCuEncoder[jId];      }
#else
  EncCu*                  getCuEncoder          ()              { return  &m_cCuEncoder;           }
#endif
  HLSWriter*              getHLSWriter          ()              { return  &m_HLSWriter;            }
#if JEM_TOOLS
  CABACDataStore*         getCABACDataStore     ()              { return  &m_CABACDataStore;       }
#endif
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  CABACEncoder*           getCABACEncoder       ( int jId = 0 ) { return  &m_CABACEncoder[jId];    }
#if JEM_TOOLS
  BilateralFilter*        getBilateralFilter    ( int jId = 0 ) { return  &m_bilateralFilter[jId]; }
#endif

  RdCost*                 getRdCost             ( int jId = 0 ) { return  &m_cRdCost[jId];         }
  CtxCache*               getCtxCache           ( int jId = 0 ) { return  &m_CtxCache[jId];        }
#else
  CABACEncoder*           getCABACEncoder       ()              { return  &m_CABACEncoder;         }
#if JEM_TOOLS
  BilateralFilter*        getBilateralFilter    ()              { return  &m_bilateralFilter;      }
#endif

  RdCost*                 getRdCost             ()              { return  &m_cRdCost;              }
  CtxCache*               getCtxCache           ()              { return  &m_CtxCache;             }
#endif
  RateCtrl*               getRateCtrl           ()              { return  &m_cRateCtrl;            }

  Void selectReferencePictureSet(Slice* slice, Int POCCurr, Int GOPid );
  Int getReferencePictureSetIdxForSOP(Int POCCurr, Int GOPid );

  Bool                   PPSNeedsWriting(Int ppsId);
  Bool                   SPSNeedsWriting(Int spsId);
  const PPS* getPPS( int Id ) { return m_ppsMap.getPS( Id); }

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  void                   setNumCuEncStacks( int n )             { m_numCuEncStacks = n; }
  int                    getNumCuEncStacks()              const { return m_numCuEncStacks; }
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               Int& iNumEncoded );

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               Int& iNumEncoded, Bool isTff );


  Void printSummary(Bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE, m_spsMap.getFirstPS()->getBitDepths()); }

};

//! \}

#endif // __ENCTOP__

