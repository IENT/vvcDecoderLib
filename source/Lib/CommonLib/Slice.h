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

/** \file     Slice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __SLICE__
#define __SLICE__

#include <cstring>
#include <list>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "Rom.h"
#include "ChromaFormat.h"
#include "Common.h"

//! \ingroup CommonLib
//! \{



struct Picture;
class Pic;
class TrQuant;
// ====================================================================================================================
// Constants
// ====================================================================================================================
class PreCalcValues;
static const UInt REF_PIC_LIST_NUM_IDX=32;

typedef std::list<Picture*> PicList;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Reference Picture Set class
class ReferencePictureSet
{
private:
  Int  m_numberOfPictures;
  Int  m_numberOfNegativePictures;
  Int  m_numberOfPositivePictures;
  Int  m_numberOfLongtermPictures;
  Int  m_deltaPOC[MAX_NUM_REF_PICS];
  Int  m_POC[MAX_NUM_REF_PICS];
  Bool m_used[MAX_NUM_REF_PICS];
  Bool m_interRPSPrediction;
  Int  m_deltaRIdxMinus1;
  Int  m_deltaRPS;
  Int  m_numRefIdc;
  Int  m_refIdc[MAX_NUM_REF_PICS+1];
  Bool m_bCheckLTMSB[MAX_NUM_REF_PICS];
  Int  m_pocLSBLT[MAX_NUM_REF_PICS];
  Int  m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  Bool m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];

public:
          ReferencePictureSet();
  virtual ~ReferencePictureSet();
  Int     getPocLSBLT(Int i) const                     { return m_pocLSBLT[i];               }
  Void    setPocLSBLT(Int i, Int x)                    { m_pocLSBLT[i] = x;                  }
  Int     getDeltaPocMSBCycleLT(Int i) const           { return m_deltaPOCMSBCycleLT[i];     }
  Void    setDeltaPocMSBCycleLT(Int i, Int x)          { m_deltaPOCMSBCycleLT[i] = x;        }
  Bool    getDeltaPocMSBPresentFlag(Int i) const       { return m_deltaPocMSBPresentFlag[i]; }
  Void    setDeltaPocMSBPresentFlag(Int i, Bool x)     { m_deltaPocMSBPresentFlag[i] = x;    }
  Void    setUsed(Int bufferNum, Bool used);
  Void    setDeltaPOC(Int bufferNum, Int deltaPOC);
  Void    setPOC(Int bufferNum, Int deltaPOC);
  Void    setNumberOfPictures(Int numberOfPictures);
  Void    setCheckLTMSBPresent(Int bufferNum, Bool b );
  Bool    getCheckLTMSBPresent(Int bufferNum) const;

  Int     getUsed(Int bufferNum) const;
  Int     getDeltaPOC(Int bufferNum) const;
  Int     getPOC(Int bufferNum) const;
  Int     getNumberOfPictures() const;

  Void    setNumberOfNegativePictures(Int number)      { m_numberOfNegativePictures = number; }
  Int     getNumberOfNegativePictures() const          { return m_numberOfNegativePictures;   }
  Void    setNumberOfPositivePictures(Int number)      { m_numberOfPositivePictures = number; }
  Int     getNumberOfPositivePictures() const          { return m_numberOfPositivePictures;   }
  Void    setNumberOfLongtermPictures(Int number)      { m_numberOfLongtermPictures = number; }
  Int     getNumberOfLongtermPictures() const          { return m_numberOfLongtermPictures;   }

  Void    setInterRPSPrediction(Bool flag)             { m_interRPSPrediction = flag;         }
  Bool    getInterRPSPrediction() const                { return m_interRPSPrediction;         }
  Void    setDeltaRIdxMinus1(Int x)                    { m_deltaRIdxMinus1 = x;               }
  Int     getDeltaRIdxMinus1() const                   { return m_deltaRIdxMinus1;            }
  Void    setDeltaRPS(Int x)                           { m_deltaRPS = x;                      }
  Int     getDeltaRPS() const                          { return m_deltaRPS;                   }
  Void    setNumRefIdc(Int x)                          { m_numRefIdc = x;                     }
  Int     getNumRefIdc() const                         { return m_numRefIdc;                  }

  Void    setRefIdc(Int bufferNum, Int refIdc);
  Int     getRefIdc(Int bufferNum) const ;

  Void    sortDeltaPOC();
  Void    printDeltaPOC() const;
};

/// Reference Picture Set set class
class RPSList
{
private:
  std::vector<ReferencePictureSet> m_referencePictureSets;

public:
                                 RPSList()                                            { }
  virtual                        ~RPSList()                                           { }

  Void                           create  (Int numberOfEntries)                            { m_referencePictureSets.resize(numberOfEntries);         }
  Void                           destroy ()                                               { }


  ReferencePictureSet*       getReferencePictureSet(Int referencePictureSetNum)       { return &m_referencePictureSets[referencePictureSetNum]; }
  const ReferencePictureSet* getReferencePictureSet(Int referencePictureSetNum) const { return &m_referencePictureSets[referencePictureSetNum]; }

  Int                            getNumberOfReferencePictureSets() const                  { return Int(m_referencePictureSets.size());              }
};

#if HEVC_USE_SCALING_LISTS
/// SCALING_LIST class
class ScalingList
{
public:
             ScalingList();
  virtual    ~ScalingList()                                                 { }
  Int*       getScalingListAddress(UInt sizeId, UInt listId)                    { return &(m_scalingListCoef[sizeId][listId][0]);            } //!< get matrix coefficient
  const Int* getScalingListAddress(UInt sizeId, UInt listId) const              { return &(m_scalingListCoef[sizeId][listId][0]);            } //!< get matrix coefficient
  Void       checkPredMode(UInt sizeId, UInt listId);

  Void       setRefMatrixId(UInt sizeId, UInt listId, UInt u)                   { m_refMatrixId[sizeId][listId] = u;                         } //!< set reference matrix ID
  UInt       getRefMatrixId(UInt sizeId, UInt listId) const                     { return m_refMatrixId[sizeId][listId];                      } //!< get reference matrix ID

  const Int* getScalingListDefaultAddress(UInt sizeId, UInt listId);                                                                           //!< get default matrix coefficient
  Void       processDefaultMatrix(UInt sizeId, UInt listId);

  Void       setScalingListDC(UInt sizeId, UInt listId, UInt u)                 { m_scalingListDC[sizeId][listId] = u;                       } //!< set DC value
  Int        getScalingListDC(UInt sizeId, UInt listId) const                   { return m_scalingListDC[sizeId][listId];                    } //!< get DC value

  Void       setScalingListPredModeFlag(UInt sizeId, UInt listId, Bool bIsDPCM) { m_scalingListPredModeFlagIsDPCM[sizeId][listId] = bIsDPCM; }
  Bool       getScalingListPredModeFlag(UInt sizeId, UInt listId) const         { return m_scalingListPredModeFlagIsDPCM[sizeId][listId];    }

  Void       checkDcOfMatrix();
  Void       processRefMatrix(UInt sizeId, UInt listId , UInt refListId );
  Bool       xParseScalingList(const std::string &fileName);
  Void       setDefaultScalingList();
  Bool       checkDefaultScalingList();

private:
  Void       outputScalingLists(std::ostream &os) const;
  Bool             m_scalingListPredModeFlagIsDPCM [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< reference list index
  Int              m_scalingListDC                 [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< the DC value of the matrix coefficient for 16x16
  UInt             m_refMatrixId                   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< RefMatrixID
  std::vector<Int> m_scalingListCoef               [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< quantization matrix
};
#endif

class ProfileTierLevel
{
  Int               m_profileSpace;
  Level::Tier       m_tierFlag;
  Profile::Name     m_profileIdc;
  Bool              m_profileCompatibilityFlag[32];
  Level::Name       m_levelIdc;

  Bool              m_progressiveSourceFlag;
  Bool              m_interlacedSourceFlag;
  Bool              m_nonPackedConstraintFlag;
  Bool              m_frameOnlyConstraintFlag;
  UInt              m_bitDepthConstraintValue;
  ChromaFormat      m_chromaFormatConstraintValue;
  Bool              m_intraConstraintFlag;
  Bool              m_onePictureOnlyConstraintFlag;
  Bool              m_lowerBitRateConstraintFlag;

public:
                ProfileTierLevel();

  Int           getProfileSpace() const                     { return m_profileSpace;                }
  Void          setProfileSpace(Int x)                      { m_profileSpace = x;                   }

  Level::Tier   getTierFlag() const                         { return m_tierFlag;                    }
  Void          setTierFlag(Level::Tier x)                  { m_tierFlag = x;                       }

  Profile::Name getProfileIdc() const                       { return m_profileIdc;                  }
  Void          setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  Bool          getProfileCompatibilityFlag(Int i) const    { return m_profileCompatibilityFlag[i]; }
  Void          setProfileCompatibilityFlag(Int i, Bool x)  { m_profileCompatibilityFlag[i] = x;    }

  Level::Name   getLevelIdc() const                         { return m_levelIdc;                    }
  Void          setLevelIdc(Level::Name x)                  { m_levelIdc = x;                       }

  Bool          getProgressiveSourceFlag() const            { return m_progressiveSourceFlag;       }
  Void          setProgressiveSourceFlag(Bool b)            { m_progressiveSourceFlag = b;          }

  Bool          getInterlacedSourceFlag() const             { return m_interlacedSourceFlag;        }
  Void          setInterlacedSourceFlag(Bool b)             { m_interlacedSourceFlag = b;           }

  Bool          getNonPackedConstraintFlag() const          { return m_nonPackedConstraintFlag;     }
  Void          setNonPackedConstraintFlag(Bool b)          { m_nonPackedConstraintFlag = b;        }

  Bool          getFrameOnlyConstraintFlag() const          { return m_frameOnlyConstraintFlag;     }
  Void          setFrameOnlyConstraintFlag(Bool b)          { m_frameOnlyConstraintFlag = b;        }

  UInt          getBitDepthConstraint() const               { return m_bitDepthConstraintValue;     }
  Void          setBitDepthConstraint(UInt bitDepth)        { m_bitDepthConstraintValue=bitDepth;   }

  ChromaFormat  getChromaFormatConstraint() const           { return m_chromaFormatConstraintValue; }
  Void          setChromaFormatConstraint(ChromaFormat fmt) { m_chromaFormatConstraintValue=fmt;    }

  Bool          getIntraConstraintFlag() const              { return m_intraConstraintFlag;         }
  Void          setIntraConstraintFlag(Bool b)              { m_intraConstraintFlag = b;            }

  Bool          getOnePictureOnlyConstraintFlag() const     { return m_onePictureOnlyConstraintFlag;}
  Void          setOnePictureOnlyConstraintFlag(Bool b)     { m_onePictureOnlyConstraintFlag = b;   }

  Bool          getLowerBitRateConstraintFlag() const       { return m_lowerBitRateConstraintFlag;  }
  Void          setLowerBitRateConstraintFlag(Bool b)       { m_lowerBitRateConstraintFlag = b;     }
};


class PTL
{
  ProfileTierLevel m_generalPTL;
  ProfileTierLevel m_subLayerPTL    [MAX_TLAYER-1];      // max. value of max_sub_layers_minus1 is MAX_TLAYER-1 (= 6)
  Bool m_subLayerProfilePresentFlag [MAX_TLAYER-1];
  Bool m_subLayerLevelPresentFlag   [MAX_TLAYER-1];

public:
                          PTL();
  Bool                    getSubLayerProfilePresentFlag(Int i) const   { return m_subLayerProfilePresentFlag[i]; }
  Void                    setSubLayerProfilePresentFlag(Int i, Bool x) { m_subLayerProfilePresentFlag[i] = x;    }

  Bool                    getSubLayerLevelPresentFlag(Int i) const     { return m_subLayerLevelPresentFlag[i];   }
  Void                    setSubLayerLevelPresentFlag(Int i, Bool x)   { m_subLayerLevelPresentFlag[i] = x;      }

  ProfileTierLevel*       getGeneralPTL()                              { return &m_generalPTL;                   }
  const ProfileTierLevel* getGeneralPTL() const                        { return &m_generalPTL;                   }
  ProfileTierLevel*       getSubLayerPTL(Int i)                        { return &m_subLayerPTL[i];               }
  const ProfileTierLevel* getSubLayerPTL(Int i) const                  { return &m_subLayerPTL[i];               }
};

struct HrdSubLayerInfo
{
  Bool fixedPicRateFlag;
  Bool fixedPicRateWithinCvsFlag;
  UInt picDurationInTcMinus1;
  Bool lowDelayHrdFlag;
  UInt cpbCntMinus1;
  UInt bitRateValueMinus1[MAX_CPB_CNT][2];
  UInt cpbSizeValue      [MAX_CPB_CNT][2];
  UInt ducpbSizeValue    [MAX_CPB_CNT][2];
  Bool cbrFlag           [MAX_CPB_CNT][2];
  UInt duBitRateValue    [MAX_CPB_CNT][2];
};

class HRD
{
private:
  Bool m_nalHrdParametersPresentFlag;
  Bool m_vclHrdParametersPresentFlag;
  Bool m_subPicCpbParamsPresentFlag;
  UInt m_tickDivisorMinus2;
  UInt m_duCpbRemovalDelayLengthMinus1;
  Bool m_subPicCpbParamsInPicTimingSEIFlag;
  UInt m_dpbOutputDelayDuLengthMinus1;
  UInt m_bitRateScale;
  UInt m_cpbSizeScale;
  UInt m_ducpbSizeScale;
  UInt m_initialCpbRemovalDelayLengthMinus1;
  UInt m_cpbRemovalDelayLengthMinus1;
  UInt m_dpbOutputDelayLengthMinus1;
  HrdSubLayerInfo m_HRD[MAX_TLAYER];

public:
  HRD()
  :m_nalHrdParametersPresentFlag       (0)
  ,m_vclHrdParametersPresentFlag       (0)
  ,m_subPicCpbParamsPresentFlag        (false)
  ,m_tickDivisorMinus2                 (0)
  ,m_duCpbRemovalDelayLengthMinus1     (0)
  ,m_subPicCpbParamsInPicTimingSEIFlag (false)
  ,m_dpbOutputDelayDuLengthMinus1      (0)
  ,m_bitRateScale                      (0)
  ,m_cpbSizeScale                      (0)
  ,m_initialCpbRemovalDelayLengthMinus1(23)
  ,m_cpbRemovalDelayLengthMinus1       (23)
  ,m_dpbOutputDelayLengthMinus1        (23)
  {}

  virtual ~HRD() {}

  Void    setNalHrdParametersPresentFlag( Bool flag )                                { m_nalHrdParametersPresentFlag = flag;                      }
  Bool    getNalHrdParametersPresentFlag( ) const                                    { return m_nalHrdParametersPresentFlag;                      }

  Void    setVclHrdParametersPresentFlag( Bool flag )                                { m_vclHrdParametersPresentFlag = flag;                      }
  Bool    getVclHrdParametersPresentFlag( ) const                                    { return m_vclHrdParametersPresentFlag;                      }

  Void    setSubPicCpbParamsPresentFlag( Bool flag )                                 { m_subPicCpbParamsPresentFlag = flag;                       }
  Bool    getSubPicCpbParamsPresentFlag( ) const                                     { return m_subPicCpbParamsPresentFlag;                       }

  Void    setTickDivisorMinus2( UInt value )                                         { m_tickDivisorMinus2 = value;                               }
  UInt    getTickDivisorMinus2( ) const                                              { return m_tickDivisorMinus2;                                }

  Void    setDuCpbRemovalDelayLengthMinus1( UInt value )                             { m_duCpbRemovalDelayLengthMinus1 = value;                   }
  UInt    getDuCpbRemovalDelayLengthMinus1( ) const                                  { return m_duCpbRemovalDelayLengthMinus1;                    }

  Void    setSubPicCpbParamsInPicTimingSEIFlag( Bool flag)                           { m_subPicCpbParamsInPicTimingSEIFlag = flag;                }
  Bool    getSubPicCpbParamsInPicTimingSEIFlag( ) const                              { return m_subPicCpbParamsInPicTimingSEIFlag;                }

  Void    setDpbOutputDelayDuLengthMinus1(UInt value )                               { m_dpbOutputDelayDuLengthMinus1 = value;                    }
  UInt    getDpbOutputDelayDuLengthMinus1( ) const                                   { return m_dpbOutputDelayDuLengthMinus1;                     }

  Void    setBitRateScale( UInt value )                                              { m_bitRateScale = value;                                    }
  UInt    getBitRateScale( ) const                                                   { return m_bitRateScale;                                     }

  Void    setCpbSizeScale( UInt value )                                              { m_cpbSizeScale = value;                                    }
  UInt    getCpbSizeScale( ) const                                                   { return m_cpbSizeScale;                                     }
  Void    setDuCpbSizeScale( UInt value )                                            { m_ducpbSizeScale = value;                                  }
  UInt    getDuCpbSizeScale( ) const                                                 { return m_ducpbSizeScale;                                   }

  Void    setInitialCpbRemovalDelayLengthMinus1( UInt value )                        { m_initialCpbRemovalDelayLengthMinus1 = value;              }
  UInt    getInitialCpbRemovalDelayLengthMinus1( ) const                             { return m_initialCpbRemovalDelayLengthMinus1;               }

  Void    setCpbRemovalDelayLengthMinus1( UInt value )                               { m_cpbRemovalDelayLengthMinus1 = value;                     }
  UInt    getCpbRemovalDelayLengthMinus1( ) const                                    { return m_cpbRemovalDelayLengthMinus1;                      }

  Void    setDpbOutputDelayLengthMinus1( UInt value )                                { m_dpbOutputDelayLengthMinus1 = value;                      }
  UInt    getDpbOutputDelayLengthMinus1( ) const                                     { return m_dpbOutputDelayLengthMinus1;                       }

  Void    setFixedPicRateFlag( Int layer, Bool flag )                                { m_HRD[layer].fixedPicRateFlag = flag;                      }
  Bool    getFixedPicRateFlag( Int layer ) const                                     { return m_HRD[layer].fixedPicRateFlag;                      }

  Void    setFixedPicRateWithinCvsFlag( Int layer, Bool flag )                       { m_HRD[layer].fixedPicRateWithinCvsFlag = flag;             }
  Bool    getFixedPicRateWithinCvsFlag( Int layer ) const                            { return m_HRD[layer].fixedPicRateWithinCvsFlag;             }

  Void    setPicDurationInTcMinus1( Int layer, UInt value )                          { m_HRD[layer].picDurationInTcMinus1 = value;                }
  UInt    getPicDurationInTcMinus1( Int layer ) const                                { return m_HRD[layer].picDurationInTcMinus1;                 }

  Void    setLowDelayHrdFlag( Int layer, Bool flag )                                 { m_HRD[layer].lowDelayHrdFlag = flag;                       }
  Bool    getLowDelayHrdFlag( Int layer ) const                                      { return m_HRD[layer].lowDelayHrdFlag;                       }

  Void    setCpbCntMinus1( Int layer, UInt value )                                   { m_HRD[layer].cpbCntMinus1 = value;                         }
  UInt    getCpbCntMinus1( Int layer ) const                                         { return m_HRD[layer].cpbCntMinus1;                          }

  Void    setBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value )   { m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl] = value; }
  UInt    getBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const         { return m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl];  }

  Void    setCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value )   { m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl] = value;       }
  UInt    getCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const         { return m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl];        }
  Void    setDuCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl] = value;     }
  UInt    getDuCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const       { return m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl];      }
  Void    setDuBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl] = value;     }
  UInt    getDuBitRateValueMinus1(Int layer, Int cpbcnt, Int nalOrVcl ) const        { return m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl];      }
  Void    setCbrFlag( Int layer, Int cpbcnt, Int nalOrVcl, Bool value )              { m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl] = value;            }
  Bool    getCbrFlag( Int layer, Int cpbcnt, Int nalOrVcl ) const                    { return m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl];             }

  Bool    getCpbDpbDelaysPresentFlag( ) const                      { return getNalHrdParametersPresentFlag() || getVclHrdParametersPresentFlag(); }
};

class TimingInfo
{
  Bool m_timingInfoPresentFlag;
  UInt m_numUnitsInTick;
  UInt m_timeScale;
  Bool m_pocProportionalToTimingFlag;
  Int  m_numTicksPocDiffOneMinus1;
public:
  TimingInfo()
  : m_timingInfoPresentFlag      (false)
  , m_numUnitsInTick             (1001)
  , m_timeScale                  (60000)
  , m_pocProportionalToTimingFlag(false)
  , m_numTicksPocDiffOneMinus1   (0)
  {}

  Void setTimingInfoPresentFlag( Bool flag )   { m_timingInfoPresentFlag = flag;       }
  Bool getTimingInfoPresentFlag( ) const       { return m_timingInfoPresentFlag;       }

  Void setNumUnitsInTick( UInt value )         { m_numUnitsInTick = value;             }
  UInt getNumUnitsInTick( ) const              { return m_numUnitsInTick;              }

  Void setTimeScale( UInt value )              { m_timeScale = value;                  }
  UInt getTimeScale( ) const                   { return m_timeScale;                   }

  Void setPocProportionalToTimingFlag(Bool x)  { m_pocProportionalToTimingFlag = x;    }
  Bool getPocProportionalToTimingFlag( ) const { return m_pocProportionalToTimingFlag; }

  Void setNumTicksPocDiffOneMinus1(Int x)      { m_numTicksPocDiffOneMinus1 = x;       }
  Int  getNumTicksPocDiffOneMinus1( ) const    { return m_numTicksPocDiffOneMinus1;    }
};

struct ChromaQpAdj
{
  union
  {
    struct {
      Int CbOffset;
      Int CrOffset;
    } comp;
    Int offset[2]; /* two chroma components */
  } u;
};

#if HEVC_VPS
class VPS
{
private:
  Int                   m_VPSId;
  UInt                  m_uiMaxTLayers;
  UInt                  m_uiMaxLayers;
  Bool                  m_bTemporalIdNestingFlag;

  UInt                  m_numReorderPics[MAX_TLAYER];
  UInt                  m_uiMaxDecPicBuffering[MAX_TLAYER];
  UInt                  m_uiMaxLatencyIncrease[MAX_TLAYER]; // Really max latency increase plus 1 (value 0 expresses no limit)

  UInt                  m_numHrdParameters;
  UInt                  m_maxNuhReservedZeroLayerId;
  std::vector<HRD>      m_hrdParameters;
  std::vector<UInt>     m_hrdOpSetIdx;
  std::vector<Bool>     m_cprmsPresentFlag;
  UInt                  m_numOpSets;
  Bool                  m_layerIdIncludedFlag[MAX_VPS_OP_SETS_PLUS1][MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1];

  PTL                   m_pcPTL;
  TimingInfo            m_timingInfo;

public:
                    VPS();

  virtual           ~VPS();

  Void              createHrdParamBuffer()
  {
    m_hrdParameters   .resize(getNumHrdParameters());
    m_hrdOpSetIdx     .resize(getNumHrdParameters());
    m_cprmsPresentFlag.resize(getNumHrdParameters());
  }

  HRD*              getHrdParameters( UInt i )                           { return &m_hrdParameters[ i ];                                    }
  const HRD*        getHrdParameters( UInt i ) const                     { return &m_hrdParameters[ i ];                                    }
  UInt              getHrdOpSetIdx( UInt i ) const                       { return m_hrdOpSetIdx[ i ];                                       }
  Void              setHrdOpSetIdx( UInt val, UInt i )                   { m_hrdOpSetIdx[ i ] = val;                                        }
  Bool              getCprmsPresentFlag( UInt i ) const                  { return m_cprmsPresentFlag[ i ];                                  }
  Void              setCprmsPresentFlag( Bool val, UInt i )              { m_cprmsPresentFlag[ i ] = val;                                   }

  Int               getVPSId() const                                     { return m_VPSId;                                                  }
  Void              setVPSId(Int i)                                      { m_VPSId = i;                                                     }

  UInt              getMaxTLayers() const                                { return m_uiMaxTLayers;                                           }
  Void              setMaxTLayers(UInt t)                                { m_uiMaxTLayers = t;                                              }

  UInt              getMaxLayers() const                                 { return m_uiMaxLayers;                                            }
  Void              setMaxLayers(UInt l)                                 { m_uiMaxLayers = l;                                               }

  Bool              getTemporalNestingFlag() const                       { return m_bTemporalIdNestingFlag;                                 }
  Void              setTemporalNestingFlag(Bool t)                       { m_bTemporalIdNestingFlag = t;                                    }

  Void              setNumReorderPics(UInt v, UInt tLayer)               { m_numReorderPics[tLayer] = v;                                    }
  UInt              getNumReorderPics(UInt tLayer) const                 { return m_numReorderPics[tLayer];                                 }

  Void              setMaxDecPicBuffering(UInt v, UInt tLayer)           { CHECK(tLayer >= MAX_TLAYER, "Invalid T-layer"); m_uiMaxDecPicBuffering[tLayer] = v; }
  UInt              getMaxDecPicBuffering(UInt tLayer) const             { return m_uiMaxDecPicBuffering[tLayer];                           }

  Void              setMaxLatencyIncrease(UInt v, UInt tLayer)           { m_uiMaxLatencyIncrease[tLayer] = v;                              }
  UInt              getMaxLatencyIncrease(UInt tLayer) const             { return m_uiMaxLatencyIncrease[tLayer];                           }

  UInt              getNumHrdParameters() const                          { return m_numHrdParameters;                                       }
  Void              setNumHrdParameters(UInt v)                          { m_numHrdParameters = v;                                          }

  UInt              getMaxNuhReservedZeroLayerId() const                 { return m_maxNuhReservedZeroLayerId;                              }
  Void              setMaxNuhReservedZeroLayerId(UInt v)                 { m_maxNuhReservedZeroLayerId = v;                                 }

  UInt              getMaxOpSets() const                                 { return m_numOpSets;                                              }
  Void              setMaxOpSets(UInt v)                                 { m_numOpSets = v;                                                 }
  Bool              getLayerIdIncludedFlag(UInt opsIdx, UInt id) const   { return m_layerIdIncludedFlag[opsIdx][id];                        }
  Void              setLayerIdIncludedFlag(Bool v, UInt opsIdx, UInt id) { m_layerIdIncludedFlag[opsIdx][id] = v;                           }

  PTL*              getPTL()                                             { return &m_pcPTL;                                                 }
  const PTL*        getPTL() const                                       { return &m_pcPTL;                                                 }
  TimingInfo*       getTimingInfo()                                      { return &m_timingInfo;                                            }
  const TimingInfo* getTimingInfo() const                                { return &m_timingInfo;                                            }
};
#endif

class Window
{
private:
  Bool m_enabledFlag;
  Int  m_winLeftOffset;
  Int  m_winRightOffset;
  Int  m_winTopOffset;
  Int  m_winBottomOffset;
public:
  Window()
  : m_enabledFlag    (false)
  , m_winLeftOffset  (0)
  , m_winRightOffset (0)
  , m_winTopOffset   (0)
  , m_winBottomOffset(0)
  { }

  Bool getWindowEnabledFlag() const   { return m_enabledFlag;                          }
  Int  getWindowLeftOffset() const    { return m_enabledFlag ? m_winLeftOffset : 0;    }
  Void setWindowLeftOffset(Int val)   { m_winLeftOffset = val; m_enabledFlag = true;   }
  Int  getWindowRightOffset() const   { return m_enabledFlag ? m_winRightOffset : 0;   }
  Void setWindowRightOffset(Int val)  { m_winRightOffset = val; m_enabledFlag = true;  }
  Int  getWindowTopOffset() const     { return m_enabledFlag ? m_winTopOffset : 0;     }
  Void setWindowTopOffset(Int val)    { m_winTopOffset = val; m_enabledFlag = true;    }
  Int  getWindowBottomOffset() const  { return m_enabledFlag ? m_winBottomOffset: 0;   }
  Void setWindowBottomOffset(Int val) { m_winBottomOffset = val; m_enabledFlag = true; }

  Void setWindow(Int offsetLeft, Int offsetLRight, Int offsetLTop, Int offsetLBottom)
  {
    m_enabledFlag     = true;
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetLRight;
    m_winTopOffset    = offsetLTop;
    m_winBottomOffset = offsetLBottom;
  }
};


class VUI
{
private:
  Bool       m_aspectRatioInfoPresentFlag;
  Int        m_aspectRatioIdc;
  Int        m_sarWidth;
  Int        m_sarHeight;
  Bool       m_overscanInfoPresentFlag;
  Bool       m_overscanAppropriateFlag;
  Bool       m_videoSignalTypePresentFlag;
  Int        m_videoFormat;
  Bool       m_videoFullRangeFlag;
  Bool       m_colourDescriptionPresentFlag;
  Int        m_colourPrimaries;
  Int        m_transferCharacteristics;
  Int        m_matrixCoefficients;
  Bool       m_chromaLocInfoPresentFlag;
  Int        m_chromaSampleLocTypeTopField;
  Int        m_chromaSampleLocTypeBottomField;
  Bool       m_neutralChromaIndicationFlag;
  Bool       m_fieldSeqFlag;
  Window     m_defaultDisplayWindow;
  Bool       m_frameFieldInfoPresentFlag;
  Bool       m_hrdParametersPresentFlag;
  Bool       m_bitstreamRestrictionFlag;
#if HEVC_TILES_WPP
  Bool       m_tilesFixedStructureFlag;
#endif
  Bool       m_motionVectorsOverPicBoundariesFlag;
  Bool       m_restrictedRefPicListsFlag;
  Int        m_minSpatialSegmentationIdc;
  Int        m_maxBytesPerPicDenom;
  Int        m_maxBitsPerMinCuDenom;
  Int        m_log2MaxMvLengthHorizontal;
  Int        m_log2MaxMvLengthVertical;
  HRD    m_hrdParameters;
  TimingInfo m_timingInfo;

public:
  VUI()
    : m_aspectRatioInfoPresentFlag        (false) //TODO: This initialiser list contains magic numbers
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_videoSignalTypePresentFlag        (false)
    , m_videoFormat                       (5)
    , m_videoFullRangeFlag                (false)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (0)
    , m_chromaSampleLocTypeBottomField    (0)
    , m_neutralChromaIndicationFlag       (false)
    , m_fieldSeqFlag                      (false)
    , m_frameFieldInfoPresentFlag         (false)
    , m_hrdParametersPresentFlag          (false)
    , m_bitstreamRestrictionFlag          (false)
#if HEVC_TILES_WPP
    , m_tilesFixedStructureFlag           (false)
#endif
    , m_motionVectorsOverPicBoundariesFlag(true)
    , m_restrictedRefPicListsFlag         (1)
    , m_minSpatialSegmentationIdc         (0)
    , m_maxBytesPerPicDenom               (2)
    , m_maxBitsPerMinCuDenom              (1)
    , m_log2MaxMvLengthHorizontal         (15)
    , m_log2MaxMvLengthVertical           (15)
  {}

  virtual           ~VUI() {}

  Bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  Void              setAspectRatioInfoPresentFlag(Bool i)                  { m_aspectRatioInfoPresentFlag = i;              }

  Int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  Void              setAspectRatioIdc(Int i)                               { m_aspectRatioIdc = i;                          }

  Int               getSarWidth() const                                    { return m_sarWidth;                             }
  Void              setSarWidth(Int i)                                     { m_sarWidth = i;                                }

  Int               getSarHeight() const                                   { return m_sarHeight;                            }
  Void              setSarHeight(Int i)                                    { m_sarHeight = i;                               }

  Bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  Void              setOverscanInfoPresentFlag(Bool i)                     { m_overscanInfoPresentFlag = i;                 }

  Bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  Void              setOverscanAppropriateFlag(Bool i)                     { m_overscanAppropriateFlag = i;                 }

  Bool              getVideoSignalTypePresentFlag() const                  { return m_videoSignalTypePresentFlag;           }
  Void              setVideoSignalTypePresentFlag(Bool i)                  { m_videoSignalTypePresentFlag = i;              }

  Int               getVideoFormat() const                                 { return m_videoFormat;                          }
  Void              setVideoFormat(Int i)                                  { m_videoFormat = i;                             }

  Bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  Void              setVideoFullRangeFlag(Bool i)                          { m_videoFullRangeFlag = i;                      }

  Bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  Void              setColourDescriptionPresentFlag(Bool i)                { m_colourDescriptionPresentFlag = i;            }

  Int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  Void              setColourPrimaries(Int i)                              { m_colourPrimaries = i;                         }

  Int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  Void              setTransferCharacteristics(Int i)                      { m_transferCharacteristics = i;                 }

  Int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  Void              setMatrixCoefficients(Int i)                           { m_matrixCoefficients = i;                      }

  Bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  Void              setChromaLocInfoPresentFlag(Bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  Int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  Void              setChromaSampleLocTypeTopField(Int i)                  { m_chromaSampleLocTypeTopField = i;             }

  Int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  Void              setChromaSampleLocTypeBottomField(Int i)               { m_chromaSampleLocTypeBottomField = i;          }

  Bool              getNeutralChromaIndicationFlag() const                 { return m_neutralChromaIndicationFlag;          }
  Void              setNeutralChromaIndicationFlag(Bool i)                 { m_neutralChromaIndicationFlag = i;             }

  Bool              getFieldSeqFlag() const                                { return m_fieldSeqFlag;                         }
  Void              setFieldSeqFlag(Bool i)                                { m_fieldSeqFlag = i;                            }

  Bool              getFrameFieldInfoPresentFlag() const                   { return m_frameFieldInfoPresentFlag;            }
  Void              setFrameFieldInfoPresentFlag(Bool i)                   { m_frameFieldInfoPresentFlag = i;               }

  Window&           getDefaultDisplayWindow()                              { return m_defaultDisplayWindow;                 }
  const Window&     getDefaultDisplayWindow() const                        { return m_defaultDisplayWindow;                 }
  Void              setDefaultDisplayWindow(Window& defaultDisplayWindow ) { m_defaultDisplayWindow = defaultDisplayWindow; }

  Bool              getHrdParametersPresentFlag() const                    { return m_hrdParametersPresentFlag;             }
  Void              setHrdParametersPresentFlag(Bool i)                    { m_hrdParametersPresentFlag = i;                }

  Bool              getBitstreamRestrictionFlag() const                    { return m_bitstreamRestrictionFlag;             }
  Void              setBitstreamRestrictionFlag(Bool i)                    { m_bitstreamRestrictionFlag = i;                }

#if HEVC_TILES_WPP
  Bool              getTilesFixedStructureFlag() const                     { return m_tilesFixedStructureFlag;              }
  Void              setTilesFixedStructureFlag(Bool i)                     { m_tilesFixedStructureFlag = i;                 }
#endif

  Bool              getMotionVectorsOverPicBoundariesFlag() const          { return m_motionVectorsOverPicBoundariesFlag;   }
  Void              setMotionVectorsOverPicBoundariesFlag(Bool i)          { m_motionVectorsOverPicBoundariesFlag = i;      }

  Bool              getRestrictedRefPicListsFlag() const                   { return m_restrictedRefPicListsFlag;            }
  Void              setRestrictedRefPicListsFlag(Bool b)                   { m_restrictedRefPicListsFlag = b;               }

  Int               getMinSpatialSegmentationIdc() const                   { return m_minSpatialSegmentationIdc;            }
  Void              setMinSpatialSegmentationIdc(Int i)                    { m_minSpatialSegmentationIdc = i;               }

  Int               getMaxBytesPerPicDenom() const                         { return m_maxBytesPerPicDenom;                  }
  Void              setMaxBytesPerPicDenom(Int i)                          { m_maxBytesPerPicDenom = i;                     }

  Int               getMaxBitsPerMinCuDenom() const                        { return m_maxBitsPerMinCuDenom;                 }
  Void              setMaxBitsPerMinCuDenom(Int i)                         { m_maxBitsPerMinCuDenom = i;                    }

  Int               getLog2MaxMvLengthHorizontal() const                   { return m_log2MaxMvLengthHorizontal;            }
  Void              setLog2MaxMvLengthHorizontal(Int i)                    { m_log2MaxMvLengthHorizontal = i;               }

  Int               getLog2MaxMvLengthVertical() const                     { return m_log2MaxMvLengthVertical;              }
  Void              setLog2MaxMvLengthVertical(Int i)                      { m_log2MaxMvLengthVertical = i;                 }

  HRD*              getHrdParameters()                                     { return &m_hrdParameters;                       }
  const HRD*        getHrdParameters()  const                              { return &m_hrdParameters;                       }

  TimingInfo*       getTimingInfo()                                        { return &m_timingInfo;                          }
  const TimingInfo* getTimingInfo() const                                  { return &m_timingInfo;                          }
};

/// SPS RExt class
class SPSRExt // Names aligned to text specification
{
private:
  Bool             m_transformSkipRotationEnabledFlag;
  Bool             m_transformSkipContextEnabledFlag;
  Bool             m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];
  Bool             m_extendedPrecisionProcessingFlag;
  Bool             m_intraSmoothingDisabledFlag;
  Bool             m_highPrecisionOffsetsEnabledFlag;
  Bool             m_persistentRiceAdaptationEnabledFlag;
  Bool             m_cabacBypassAlignmentEnabledFlag;

public:
  SPSRExt();

  Bool settingsDifferFromDefaults() const
  {
    return getTransformSkipRotationEnabledFlag()
        || getTransformSkipContextEnabledFlag()
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT)
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT)
        || getExtendedPrecisionProcessingFlag()
        || getIntraSmoothingDisabledFlag()
        || getHighPrecisionOffsetsEnabledFlag()
        || getPersistentRiceAdaptationEnabledFlag()
        || getCabacBypassAlignmentEnabledFlag();
  }


  Bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
  Void setTransformSkipRotationEnabledFlag(const Bool value)                           { m_transformSkipRotationEnabledFlag = value;    }

  Bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
  Void setTransformSkipContextEnabledFlag(const Bool value)                            { m_transformSkipContextEnabledFlag = value;     }

  Bool getRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode) const             { return m_rdpcmEnabledFlag[signallingMode];     }
  Void setRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode, const Bool value) { m_rdpcmEnabledFlag[signallingMode] = value;    }

  Bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
  Void setExtendedPrecisionProcessingFlag(Bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }

  Bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
  Void setIntraSmoothingDisabledFlag(Bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }

  Bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
  Void setHighPrecisionOffsetsEnabledFlag(Bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }

  Bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
  Void setPersistentRiceAdaptationEnabledFlag(const Bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }

  Bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
  Void setCabacBypassAlignmentEnabledFlag(const Bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
};


class SPS;
class SPSNext
{
private:
  SPS&  m_SPS;

  bool              m_NextEnabled;
#if JEM_TOOLS
  bool              m_GALFEnabled;
#endif
  //=====  tool enabling flags (4 bytes - NOTE: last flag must be used for new extensions) =====
  bool              m_QTBT;                       // 1
#if JEM_TOOLS
  bool              m_NSST;                       // 2
  bool              m_Intra4Tap;                  // 3
#if !INTRA67_3MPM
  bool              m_Intra65Ang;                 // 4
#endif
#endif
  bool              m_LargeCTU;                   // 5
#if JEM_TOOLS
  bool              m_IntraBoundaryFilter;        // 6
  bool              m_SubPuMvp;                   // 7
#endif
#if !JEM_TOOLS && JVET_K0346
  bool              m_SubPuMvp;
#endif
#if JEM_TOOLS
  bool              m_ModifiedCABACEngine;        // 8
#endif
#if JEM_TOOLS
  bool              m_IMV;                        // 9
#endif
#if JVET_K0072
#else
#if JEM_TOOLS
  bool              m_altResiComp;                // 10
#endif
#endif
#if JEM_TOOLS
  bool              m_highPrecMv;                 // 11
  bool              m_BIO;                        // 12
#endif
#if !JEM_TOOLS && (JVET_K0346 || JVET_K_AFFINE)
  bool              m_highPrecMv;
#endif
  bool              m_DisableMotionCompression;   // 13
#if JEM_TOOLS
  bool              m_LICEnabled;                 // 14
  bool              m_IntraPDPC;                  // 15
  bool              m_ALFEnabled;                 // 16
  bool              m_LMChroma;                   // 17
  bool              m_IntraEMT;                   // 18
  bool              m_InterEMT;                   // 19
#endif
#if JEM_TOOLS
  bool              m_OBMC;                       // 20
  bool              m_FRUC;                       // 21
  bool              m_Affine;                     // 22
#if JVET_K0337_AFFINE_6PARA
  bool              m_AffineType;
#endif
  bool              m_AClip;                      // 23
#endif
#if !JEM_TOOLS && JVET_K_AFFINE
  bool              m_Affine;
#if JVET_K0337_AFFINE_6PARA
  bool              m_AffineType;
#endif
#endif
#if JEM_TOOLS
  bool              m_CIPFEnabled;                // 24
#endif
#if JEM_TOOLS
  bool              m_BIF;                        // 25
#endif
#if JEM_TOOLS
  bool              m_DMVR;                       // 26
  bool              m_MDMS;                       // 27
#endif
  bool              m_MTTEnabled;                 //
#if ENABLE_WPP_PARALLELISM
  bool              m_NextDQP;
#endif

public:
  const static int  NumReservedFlags = 32 - 27; /* current number of tool enabling flags */

private:
  //=====  additional parameters  =====
  // qtbt
  unsigned    m_CTUSize;
  unsigned    m_minQT[3];   // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned    m_maxBTDepth[3];
  unsigned    m_maxBTSize[3];
  unsigned    m_dualITree;
#if JEM_TOOLS || JVET_K0346
  // sub-pu merging
  unsigned    m_subPuLog2Size;
  int         m_subPuMrgMode;
#endif
#if JEM_TOOLS
  // cabac engine
  unsigned    m_CABACEngineMode;
#endif
#if JEM_TOOLS
  //imv
  ImvMode     m_ImvMode;
#endif
#if JVET_K0072
#else
#if JEM_TOOLS
  // ARC
  unsigned    m_altResiCompId;
#endif
#endif
#if JEM_TOOLS
  // local illumination compensation
  unsigned    m_LICMode;
#endif
  // multi type tree (QTBT + triple split)
  unsigned    m_MTTMode;
#if JEM_TOOLS
  int         m_OBMCBlkSize;

  unsigned    m_FRUCRefineFilter;
  unsigned    m_FRUCRefineRange;
  unsigned    m_FRUCSmallBlkRefineDepth;

  // adaptive clipping
  unsigned    m_AClipQuant;
#endif

#if JEM_TOOLS
  int         m_ELMMode;
  int         m_IntraPDPCMode;
#endif
#if JEM_TOOLS
  unsigned    m_CIPFMode;
#endif
  // ADD_NEW_TOOL : (sps extension) add tool enabling flags and associated parameters here

public:
  SPSNext( SPS& sps );

  const SPS&  getSPS              () const                                          { return m_SPS; }
  SPS&        getSPS              ()                                                { return m_SPS; }
  bool        nextToolsEnabled    () const                                          { return m_NextEnabled; }
  void        setNextToolsEnabled ( bool next )                                     { m_NextEnabled = next; }

  //=====  tool enabling flags and extension bit  =====
  void      setUseQTBT            ( bool QTBT )                                     { m_QTBT = QTBT; }
  bool      getUseQTBT            ()                                      const     { return m_QTBT; }
#if JEM_TOOLS
  void      setUseNSST            ( bool b )                                        { m_NSST = b; }
  bool      getUseNSST            ()                                      const     { return m_NSST; }
  void      setUseIntra4Tap       ( bool b )                                        { m_Intra4Tap = b; }
  bool      getUseIntra4Tap       ()                                      const     { return m_Intra4Tap; }
#if !INTRA67_3MPM
  void      setUseIntra65Ang      ( bool b )                                        { m_Intra65Ang = b; }
  bool      getUseIntra65Ang      ()                                      const     { return m_Intra65Ang; }
#endif
#endif
  void      setUseLargeCTU        ( bool b )                                        { m_LargeCTU = b; }
  bool      getUseLargeCTU        ()                                      const     { return m_LargeCTU; }
#if JEM_TOOLS
  void      setUseIntraBoundaryFilter( bool b )                                     { m_IntraBoundaryFilter = b; }
  bool      getUseIntraBoundaryFilter()                                   const     { return m_IntraBoundaryFilter; }

  bool      getUseSubPuMvp        ()                                      const     { return m_SubPuMvp; }
  void      setSubPuMvpMode       ( int n )                                         { m_subPuMrgMode = n; m_SubPuMvp = n != 0; }
  bool      getUseATMVP           ()                                      const     { return ( m_subPuMrgMode & 1 ) == 1; }
  bool      getUseSTMVP           ()                                      const     { return ( m_subPuMrgMode & 2 ) == 2; }
#endif
#if !JEM_TOOLS && JVET_K0346
  bool      getUseSubPuMvp()                                   const { return m_SubPuMvp; }
  void      setSubPuMvpMode(int n)                             { m_subPuMrgMode = n; m_SubPuMvp = n != 0; }
  bool      getUseATMVP()                                      const { return (m_subPuMrgMode & 1) == 1; }
#endif
#if JEM_TOOLS
  bool      getModifiedCABACEngine()                                      const     { return m_ModifiedCABACEngine; }
#endif
#if JEM_TOOLS
  void      setUseIMV             ( bool b )                                        { m_IMV = b; }
  bool      getUseIMV             ()                                      const     { return m_IMV; }
  void      setUseAffine          ( bool b )                                        { m_Affine = b; }
  bool      getUseAffine          ()                                      const     { return m_Affine; }
#if JVET_K0337_AFFINE_6PARA
  void      setUseAffineType      ( bool b )                                        { m_AffineType = b; }
  bool      getUseAffineType      ()                                      const     { return m_AffineType; }
#endif
#endif
#if !JEM_TOOLS && JVET_K_AFFINE
  void      setUseAffine          ( bool b )                                        { m_Affine = b; }
  bool      getUseAffine          ()                                      const     { return m_Affine; }
#if JVET_K0337_AFFINE_6PARA
  void      setUseAffineType      ( bool b )                                        { m_AffineType = b; }
  bool      getUseAffineType      ()                                      const     { return m_AffineType; }
#endif
#endif
#if JVET_K0072
#else
#if JEM_TOOLS
  void      setUseAltResiComp     ( bool b )                                        { m_altResiComp = b; }
  bool      getUseAltResiComp     ()                                      const     { return m_altResiComp; }
#endif
#endif
#if JEM_TOOLS
  void      setUseHighPrecMv      ( bool b )                                        { m_highPrecMv = b; }
  bool      getUseHighPrecMv      ()                                      const     { return m_highPrecMv; }
  void      setUseBIO             ( bool b )                                        { m_BIO = b; }
  bool      getUseBIO             ()                                      const     { return m_BIO; }
#endif
#if !JEM_TOOLS && (JVET_K0346 || JVET_K_AFFINE)
  void      setUseHighPrecMv(bool b) { m_highPrecMv = b; }
  bool      getUseHighPrecMv()                                      const { return m_highPrecMv; }
#endif
  void      setDisableMotCompress ( bool b )                                        { m_DisableMotionCompression = b; }
  bool      getDisableMotCompress ()                                      const     { return m_DisableMotionCompression; }
#if JEM_TOOLS
  bool      getLICEnabled         ()                                      const     { return m_LICEnabled; }
#endif
  bool      getMTTEnabled         ()                                      const     { return m_MTTEnabled; }
#if ENABLE_WPP_PARALLELISM
  void      setUseNextDQP         ( bool b )                                        { m_NextDQP = b; }
  bool      getUseNextDQP         ()                                      const     { return m_NextDQP; }
#endif
#if JEM_TOOLS
  void      setUseIntraPDPC       ( bool b )                                        { m_IntraPDPC = b; }
  bool      getUseIntraPDPC       ()                                      const     { return m_IntraPDPC; }
  void      setALFEnabled         ( bool b )                                        { m_ALFEnabled = b; }
  bool      getALFEnabled         ()                                      const     { return m_ALFEnabled; }
  void      setGALFEnabled        ( bool b )                                        { m_GALFEnabled = b; }
  bool      getGALFEnabled        ()                                      const     { return m_GALFEnabled; }
#endif
#if JEM_TOOLS
  void      setUseLMChroma        ( bool b )                                        { m_LMChroma = b; }
  bool      getUseLMChroma        ()                                      const     { return m_LMChroma; }
  void      setUseIntraEMT        ( bool b )                                        { m_IntraEMT = b; }
  bool      getUseIntraEMT        ()                                      const     { return m_IntraEMT; }
  void      setUseInterEMT        ( bool b )                                        { m_InterEMT = b; }
  bool      getUseInterEMT        ()                                      const     { return m_InterEMT; }
#endif
#if JEM_TOOLS
  void      setUseOBMC            ( bool b )                                        { m_OBMC = b; }
  bool      getUseOBMC            ()                                      const     { return m_OBMC; }
  void      setOBMCBlkSize        ( unsigned n )                                    { m_OBMCBlkSize = n; }
  int       getOBMCBlkSize        ()                                      const     { return m_OBMCBlkSize; }
#endif
#if JEM_TOOLS
  void      setCIPFMode           ( unsigned n )                                    { m_CIPFMode = n; m_CIPFEnabled = (m_CIPFMode>0); }
  unsigned  getCIPFMode           ()                                      const     { return m_CIPFMode; }
#endif
#if JEM_TOOLS
  void      setUseBIF             ( bool b )                                        { m_BIF = b; }
  bool      getUseBIF             ()                                      const     { return m_BIF; }
  void      setUseAClip           ( bool b )                                        { m_AClip = b; }
  bool      getUseAClip           ()                                      const     { return m_AClip; }
#endif
#if JEM_TOOLS
  void      setUseDMVR            ( bool b )                                        { m_DMVR = b; }
  bool      getUseDMVR            ()                                      const     { return m_DMVR; }
#endif
#if JEM_TOOLS
  void      setUseMDMS            ( bool b )                                        { m_MDMS = b; }
  bool      getUseMDMS            ()                                      const     { return m_MDMS; }
#endif

  //=====  additional parameters  =====
  // qtbt
  void      setCTUSize            ( unsigned    ctuSize )                           { m_CTUSize = ctuSize; }
  unsigned  getCTUSize            ()                                      const     { return  m_CTUSize;   }
  void      setMinQTSizes         ( unsigned*   minQT )                             { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2]; }
  unsigned  getMinQTSize          ( SliceType   slicetype,
                                    ChannelType chType = CHANNEL_TYPE_LUMA )
                                                                          const     { return slicetype == I_SLICE ? ( chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2] ) : m_minQT[1]; }
  void      setMaxBTDepth         ( unsigned    maxBTDepth,
                                    unsigned    maxBTDepthI,
                                    unsigned    maxBTDepthIChroma )                 { m_maxBTDepth[1] = maxBTDepth; m_maxBTDepth[0] = maxBTDepthI; m_maxBTDepth[2] = maxBTDepthIChroma; }
  unsigned  getMaxBTDepth         ()                                      const     { return m_maxBTDepth[1]; }
  unsigned  getMaxBTDepthI        ()                                      const     { return m_maxBTDepth[0]; }
  unsigned  getMaxBTDepthIChroma  ()                                      const     { return m_maxBTDepth[2]; }
  void      setMaxBTSize          ( unsigned    maxBTSize,
                                    unsigned    maxBTSizeI,
                                    unsigned    maxBTSizeC )                        { m_maxBTSize[1] = maxBTSize; m_maxBTSize[0] = maxBTSizeI; m_maxBTSize[2] = maxBTSizeC; }
  unsigned  getMaxBTSize          ()                                      const     { return m_maxBTSize[1]; }
  unsigned  getMaxBTSizeI         ()                                      const     { return m_maxBTSize[0]; }
  unsigned  getMaxBTSizeIChroma   ()                                      const     { return m_maxBTSize[2]; }

  void      setUseDualITree       ( bool b )                                        { m_dualITree = b; }
  bool      getUseDualITree       ()                                      const     { return m_dualITree; }

#if JEM_TOOLS || JVET_K0346
  // sub pu tmvp
  void      setSubPuMvpLog2Size   ( unsigned    log2Size )                          { m_subPuLog2Size = log2Size; }
  unsigned  getSubPuMvpLog2Size   ()                                      const     { return m_subPuLog2Size; }
#endif
#if JEM_TOOLS
  // cabac engine
  unsigned  getCABACEngineMode    ()                                      const     { return m_CABACEngineMode; }
  void      setCABACEngineMode    ( unsigned    mode )                              { m_CABACEngineMode = mode; m_ModifiedCABACEngine = ( m_CABACEngineMode != 0 ); }
#endif
#if JEM_TOOLS
  void      setImvMode(ImvMode m) { m_ImvMode = m; m_IMV = m != 0;  }
  ImvMode   getImvMode            ()                                      const     { return m_ImvMode; }
#endif

#if JVET_K0072
#else
#if JEM_TOOLS
  // ARC
  void      setAltResiCompId      ( unsigned    mode )                              { m_altResiCompId = mode; }
  unsigned  getAltResiCompId      ()                                      const     { return m_altResiCompId; }
#endif
#endif

#if JEM_TOOLS
  // local illumination compensation
  unsigned  getLICMode            ()                                      const     { return m_LICMode; }
  void      setLICMode            ( unsigned    mode )                              { m_LICMode = mode; m_LICEnabled = ( m_LICMode != 0 ); }
#endif

  // multi type tree
  unsigned  getMTTMode            ()                                      const     { return m_MTTMode; }
  void      setMTTMode            ( unsigned    mode )                              { m_MTTMode = mode; m_MTTEnabled = ( m_MTTMode != 0 ); }

#if JEM_TOOLS
  void      setUseFRUCMrgMode     ( bool b )                                        { m_FRUC = b; }
  unsigned  getUseFRUCMrgMode     ()                                      const     { return m_FRUC; }
  void      setFRUCRefineFilter   ( unsigned n )                                    { m_FRUCRefineFilter = n; }
  unsigned  getFRUCRefineFilter   ()                                      const     { return m_FRUCRefineFilter; }
  void      setFRUCRefineRange    ( unsigned n )                                    { m_FRUCRefineRange = n; }
  unsigned  getFRUCRefineRange    ()                                      const     { return m_FRUCRefineRange; }
  void      setFRUCSmallBlkRefineDepth( unsigned n )                                { m_FRUCSmallBlkRefineDepth = n; }
  unsigned  getFRUCSmallBlkRefineDepth()                                  const     { return m_FRUCSmallBlkRefineDepth; }

  //adaptive clipping
  void      setAClipQuant         ( unsigned n )                                    { m_AClipQuant = n; }
  unsigned  getAClipQuant         ()                                      const     { return m_AClipQuant; }
#endif
#if JEM_TOOLS
  void      setELMMode            ( int m )                                         { m_ELMMode = m; }
  int       getELMMode()                                                  const     { return m_ELMMode; }
  bool      isELMModeMMLM()                                               const     { return 0 != (m_ELMMode & 1); }
  bool      isELMModeMFLM()                                               const     { return 0 != (m_ELMMode & 2); }
  int       getRealNumIntraMode()                                         const     { return 68 + (m_ELMMode & 1) + ((m_ELMMode & 2) << 1); }
#endif
#if JEM_TOOLS
  void      setIntraPDPCMode      ( int n )                                         { m_IntraPDPCMode = n; }
  int       getIntraPDPCMode      ()                                      const     { return m_IntraPDPCMode; }
  bool      isIntraPDPC           ()                                      const     { return 0 != (m_IntraPDPCMode&1); }
  bool      isPlanarPDPC          ()                                      const     { return 0 != (m_IntraPDPCMode&2); }
#endif
  // ADD_NEW_TOOL : (sps extension) add access functions for tool enabling flags and associated parameters here

};


/// SPS class
class SPS
{
private:
  Int               m_SPSId;
#if HEVC_VPS
  Int               m_VPSId;
#endif
  ChromaFormat      m_chromaFormatIdc;

  UInt              m_uiMaxTLayers;           // maximum number of temporal layers

  // Structure
  UInt              m_picWidthInLumaSamples;
  UInt              m_picHeightInLumaSamples;

  Int               m_log2MinCodingBlockSize;
  Int               m_log2DiffMaxMinCodingBlockSize;
  UInt              m_uiMaxCUWidth;
  UInt              m_uiMaxCUHeight;
  UInt              m_uiMaxCodingDepth; ///< Total CU depth, relative to the smallest possible transform block size.

  Window            m_conformanceWindow;

  RPSList           m_RPSList;
  Bool              m_bLongTermRefsPresent;
  Bool              m_SPSTemporalMVPEnabledFlag;
  Int               m_numReorderPics[MAX_TLAYER];

  // Tool list
  UInt              m_uiQuadtreeTULog2MaxSize;
  UInt              m_uiQuadtreeTULog2MinSize;
  UInt              m_uiQuadtreeTUMaxDepthInter;
  UInt              m_uiQuadtreeTUMaxDepthIntra;
  Bool              m_usePCM;
  UInt              m_pcmLog2MaxSize;
  UInt              m_uiPCMLog2MinSize;
  Bool              m_useAMP;

  // Parameter
  BitDepths         m_bitDepths;
  Int               m_qpBDOffset[MAX_NUM_CHANNEL_TYPE];
  Int               m_pcmBitDepths[MAX_NUM_CHANNEL_TYPE];
  Bool              m_bPCMFilterDisableFlag;

  UInt              m_uiBitsForPOC;
  UInt              m_numLongTermRefPicSPS;
  UInt              m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  Bool              m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  // Max physical transform size
  UInt              m_uiMaxTrSize;

  Bool              m_bUseSAO;

  Bool              m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

#if HEVC_USE_SCALING_LISTS
  Bool              m_scalingListEnabledFlag;
  Bool              m_scalingListPresentFlag;
  ScalingList       m_scalingList;
#endif
  UInt              m_uiMaxDecPicBuffering[MAX_TLAYER];
  UInt              m_uiMaxLatencyIncreasePlus1[MAX_TLAYER];

#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  Bool              m_useStrongIntraSmoothing;
#endif

  Bool              m_vuiParametersPresentFlag;
  VUI               m_vuiParameters;

  SPSRExt           m_spsRangeExtension;
  SPSNext           m_spsNextExtension;

  static const Int  m_winUnitX[NUM_CHROMA_FORMAT];
  static const Int  m_winUnitY[NUM_CHROMA_FORMAT];
  PTL               m_pcPTL;

public:

  SPS();
  virtual                 ~SPS();

#if HEVC_VPS
  Int                     getVPSId() const                                                                { return m_VPSId;                                                      }
  Void                    setVPSId(Int i)                                                                 { m_VPSId = i;                                                         }
#endif
  Int                     getSPSId() const                                                                { return m_SPSId;                                                      }
  Void                    setSPSId(Int i)                                                                 { m_SPSId = i;                                                         }
  ChromaFormat            getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  Void                    setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }

  static Int              getWinUnitX (Int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitX[chromaFormatIdc]; }
  static Int              getWinUnitY (Int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitY[chromaFormatIdc]; }

  // structure
  Void                    setPicWidthInLumaSamples( UInt u )                                              { m_picWidthInLumaSamples = u;                                         }
  UInt                    getPicWidthInLumaSamples() const                                                { return  m_picWidthInLumaSamples;                                     }
  Void                    setPicHeightInLumaSamples( UInt u )                                             { m_picHeightInLumaSamples = u;                                        }
  UInt                    getPicHeightInLumaSamples() const                                               { return  m_picHeightInLumaSamples;                                    }

  Window&                 getConformanceWindow()                                                          { return  m_conformanceWindow;                                         }
  const Window&           getConformanceWindow() const                                                    { return  m_conformanceWindow;                                         }
  Void                    setConformanceWindow(Window& conformanceWindow )                                { m_conformanceWindow = conformanceWindow;                             }

  UInt                    getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  Void                    setNumLongTermRefPicSPS(UInt val)                                               { m_numLongTermRefPicSPS = val;                                        }

  UInt                    getLtRefPicPocLsbSps(UInt index) const                                          { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_ltRefPicPocLsbSps[index]; }
  Void                    setLtRefPicPocLsbSps(UInt index, UInt val)                                      { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_ltRefPicPocLsbSps[index] = val;  }

  Bool                    getUsedByCurrPicLtSPSFlag(Int i) const                                          { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_usedByCurrPicLtSPSFlag[i];    }
  Void                    setUsedByCurrPicLtSPSFlag(Int i, Bool x)                                        { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  Int                     getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  Void                    setLog2MinCodingBlockSize(Int val)                                              { m_log2MinCodingBlockSize = val;                                      }
  Int                     getLog2DiffMaxMinCodingBlockSize() const                                        { return m_log2DiffMaxMinCodingBlockSize;                              }
  Void                    setLog2DiffMaxMinCodingBlockSize(Int val)                                       { m_log2DiffMaxMinCodingBlockSize = val;                               }

  Void                    setMaxCUWidth( UInt u )                                                         { m_uiMaxCUWidth = u;                                                  }
  UInt                    getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  Void                    setMaxCUHeight( UInt u )                                                        { m_uiMaxCUHeight = u;                                                 }
  UInt                    getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
  Void                    setMaxCodingDepth( UInt u )                                                     { m_uiMaxCodingDepth = u;                                              }
  UInt                    getMaxCodingDepth() const                                                       { return  m_uiMaxCodingDepth;                                          }
  Void                    setUsePCM( Bool b )                                                             { m_usePCM = b;                                                        }
  Bool                    getUsePCM() const                                                               { return m_usePCM;                                                     }
  Void                    setPCMLog2MaxSize( UInt u )                                                     { m_pcmLog2MaxSize = u;                                                }
  UInt                    getPCMLog2MaxSize() const                                                       { return  m_pcmLog2MaxSize;                                            }
  Void                    setPCMLog2MinSize( UInt u )                                                     { m_uiPCMLog2MinSize = u;                                              }
  UInt                    getPCMLog2MinSize() const                                                       { return  m_uiPCMLog2MinSize;                                          }
  Void                    setBitsForPOC( UInt u )                                                         { m_uiBitsForPOC = u;                                                  }
  UInt                    getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
  Bool                    getUseAMP() const                                                               { return m_useAMP;                                                     }
  Void                    setUseAMP( Bool b )                                                             { m_useAMP = b;                                                        }
  Void                    setQuadtreeTULog2MaxSize( UInt u )                                              { m_uiQuadtreeTULog2MaxSize = u;                                       }
  UInt                    getQuadtreeTULog2MaxSize() const                                                { return m_uiQuadtreeTULog2MaxSize;                                    }
  Void                    setQuadtreeTULog2MinSize( UInt u )                                              { m_uiQuadtreeTULog2MinSize = u;                                       }
  UInt                    getQuadtreeTULog2MinSize() const                                                { return m_uiQuadtreeTULog2MinSize;                                    }
  Void                    setQuadtreeTUMaxDepthInter( UInt u )                                            { m_uiQuadtreeTUMaxDepthInter = u;                                     }
  Void                    setQuadtreeTUMaxDepthIntra( UInt u )                                            { m_uiQuadtreeTUMaxDepthIntra = u;                                     }
  UInt                    getQuadtreeTUMaxDepthInter() const                                              { return m_uiQuadtreeTUMaxDepthInter;                                  }
  UInt                    getQuadtreeTUMaxDepthIntra() const                                              { return m_uiQuadtreeTUMaxDepthIntra;                                  }
  Void                    setNumReorderPics(Int i, UInt tlayer)                                           { m_numReorderPics[tlayer] = i;                                        }
  Int                     getNumReorderPics(UInt tlayer) const                                            { return m_numReorderPics[tlayer];                                     }
  Void                    createRPSList( Int numRPS );
  const RPSList*          getRPSList() const                                                              { return &m_RPSList;                                                   }
  RPSList*                getRPSList()                                                                    { return &m_RPSList;                                                   }
  Bool                    getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  Void                    setLongTermRefsPresent(Bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  Bool                    getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  Void                    setSPSTemporalMVPEnabledFlag(Bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
  // physical transform
  Void                    setMaxTrSize( UInt u )                                                          { m_uiMaxTrSize = u;                                                   }
  UInt                    getMaxTrSize() const                                                            { return  m_uiMaxTrSize;                                               }

  // Bit-depth
  Int                     getBitDepth(ChannelType type) const                                             { return m_bitDepths.recon[type];                                      }
  Void                    setBitDepth(ChannelType type, Int u )                                           { m_bitDepths.recon[type] = u;                                         }
  const BitDepths&        getBitDepths() const                                                            { return m_bitDepths;                                                  }
  Int                     getMaxLog2TrDynamicRange(ChannelType channelType) const                         { return getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ? std::max<Int>(15, Int(m_bitDepths.recon[channelType] + 6)) : 15; }

  Int                     getDifferentialLumaChromaBitDepth() const                                       { return Int(m_bitDepths.recon[CHANNEL_TYPE_LUMA]) - Int(m_bitDepths.recon[CHANNEL_TYPE_CHROMA]); }
  Int                     getQpBDOffset(ChannelType type) const                                           { return m_qpBDOffset[type];                                           }
  Void                    setQpBDOffset(ChannelType type, Int i)                                          { m_qpBDOffset[type] = i;                                              }

  Void                    setUseSAO(Bool bVal)                                                            { m_bUseSAO = bVal;                                                    }
  Bool                    getUseSAO() const                                                               { return m_bUseSAO;                                                    }

  UInt                    getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  Void                    setMaxTLayers( UInt uiMaxTLayers )                                              { CHECK( uiMaxTLayers > MAX_TLAYER, "Invalid number T-layers" ); m_uiMaxTLayers = uiMaxTLayers; }

  Bool                    getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  Void                    setTemporalIdNestingFlag( Bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }
  UInt                    getPCMBitDepth(ChannelType type) const                                          { return m_pcmBitDepths[type];                                         }
  Void                    setPCMBitDepth(ChannelType type, UInt u)                                        { m_pcmBitDepths[type] = u;                                            }
  Void                    setPCMFilterDisableFlag( Bool bValue )                                          { m_bPCMFilterDisableFlag = bValue;                                    }
  Bool                    getPCMFilterDisableFlag() const                                                 { return m_bPCMFilterDisableFlag;                                      }

#if HEVC_USE_SCALING_LISTS
  Bool                    getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  Void                    setScalingListFlag( Bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  Bool                    getScalingListPresentFlag() const                                               { return m_scalingListPresentFlag;                                     }
  Void                    setScalingListPresentFlag( Bool b )                                             { m_scalingListPresentFlag  = b;                                       }
  ScalingList&            getScalingList()                                                                { return m_scalingList; }
  const ScalingList&      getScalingList() const                                                          { return m_scalingList; }
#endif
  UInt                    getMaxDecPicBuffering(UInt tlayer) const                                        { return m_uiMaxDecPicBuffering[tlayer];                               }
  Void                    setMaxDecPicBuffering( UInt ui, UInt tlayer )                                   { CHECK(tlayer >= MAX_TLAYER, "Invalid T-layer"); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  UInt                    getMaxLatencyIncreasePlus1(UInt tlayer) const                                   { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  Void                    setMaxLatencyIncreasePlus1( UInt ui , UInt tlayer)                              { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }

#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  Void                    setUseStrongIntraSmoothing(Bool bVal)                                           { m_useStrongIntraSmoothing = bVal;                                    }
  Bool                    getUseStrongIntraSmoothing() const                                              { return m_useStrongIntraSmoothing;                                    }

#endif
  Bool                    getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  Void                    setVuiParametersPresentFlag(Bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
  VUI*                    getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const VUI*              getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const PTL*              getPTL() const                                                                  { return &m_pcPTL;                                                     }
  PTL*                    getPTL()                                                                        { return &m_pcPTL;                                                     }

  const SPSRExt&          getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  SPSRExt&                getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }

  const SPSNext&          getSpsNext() const                                                              { return m_spsNextExtension;                                           }
  SPSNext&                getSpsNext()                                                                    { return m_spsNextExtension;                                           }
};


/// Reference Picture Lists class

class RefPicListModification
{
private:
  Bool m_refPicListModificationFlagL0;
  Bool m_refPicListModificationFlagL1;
  UInt m_RefPicSetIdxL0[REF_PIC_LIST_NUM_IDX];
  UInt m_RefPicSetIdxL1[REF_PIC_LIST_NUM_IDX];

public:
          RefPicListModification();
  virtual ~RefPicListModification();

  Bool    getRefPicListModificationFlagL0() const        { return m_refPicListModificationFlagL0;                                  }
  Void    setRefPicListModificationFlagL0(Bool flag)     { m_refPicListModificationFlagL0 = flag;                                  }
  Bool    getRefPicListModificationFlagL1() const        { return m_refPicListModificationFlagL1;                                  }
  Void    setRefPicListModificationFlagL1(Bool flag)     { m_refPicListModificationFlagL1 = flag;                                  }
  UInt    getRefPicSetIdxL0(UInt idx) const              { CHECK(idx>=REF_PIC_LIST_NUM_IDX, "Invalid ref-pic-list index"); return m_RefPicSetIdxL0[idx];         }
  Void    setRefPicSetIdxL0(UInt idx, UInt refPicSetIdx) { CHECK(idx>=REF_PIC_LIST_NUM_IDX, "Invalid ref-pic-list index"); m_RefPicSetIdxL0[idx] = refPicSetIdx; }
  UInt    getRefPicSetIdxL1(UInt idx) const              { CHECK(idx>=REF_PIC_LIST_NUM_IDX, "Invalid ref-pic-list index"); return m_RefPicSetIdxL1[idx];         }
  Void    setRefPicSetIdxL1(UInt idx, UInt refPicSetIdx) { CHECK(idx>=REF_PIC_LIST_NUM_IDX, "Invalid ref-pic-list index"); m_RefPicSetIdxL1[idx] = refPicSetIdx; }
};



/// PPS RExt class
class PPSRExt // Names aligned to text specification
{
private:
  Int              m_log2MaxTransformSkipBlockSize;
  Bool             m_crossComponentPredictionEnabledFlag;

  // Chroma QP Adjustments
  Int              m_diffCuChromaQpOffsetDepth;
  Int              m_chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  UInt             m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];

public:
  PPSRExt();

  Bool settingsDifferFromDefaults(const bool bTransformSkipEnabledFlag) const
  {
    return (bTransformSkipEnabledFlag && (getLog2MaxTransformSkipBlockSize() !=2))
        || (getCrossComponentPredictionEnabledFlag() )
        || (getChromaQpOffsetListEnabledFlag() )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA) !=0 )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA) !=0 );
  }

  UInt                   getLog2MaxTransformSkipBlockSize() const                         { return m_log2MaxTransformSkipBlockSize;         }
  Void                   setLog2MaxTransformSkipBlockSize( UInt u )                       { m_log2MaxTransformSkipBlockSize  = u;           }

  Bool                   getCrossComponentPredictionEnabledFlag() const                   { return m_crossComponentPredictionEnabledFlag;   }
  Void                   setCrossComponentPredictionEnabledFlag(Bool value)               { m_crossComponentPredictionEnabledFlag = value;  }

  Void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  UInt                   getDiffCuChromaQpOffsetDepth () const                            { return m_diffCuChromaQpOffsetDepth;             }
  Void                   setDiffCuChromaQpOffsetDepth ( UInt u )                          { m_diffCuChromaQpOffsetDepth = u;                }

  Bool                   getChromaQpOffsetListEnabledFlag() const                         { return getChromaQpOffsetListLen()>0;            }
  Int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }

  const ChromaQpAdj&     getChromaQpOffsetListEntry( Int cuChromaQpOffsetIdxPlus1 ) const
  {
    CHECK(cuChromaQpOffsetIdxPlus1 >= m_chromaQpOffsetListLen+1, "Invalid chroma QP offset");
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  Void                   setChromaQpOffsetListEntry( Int cuChromaQpOffsetIdxPlus1, Int cbOffset, Int crOffset )
  {
    CHECK(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE, "Invalid chroma QP offset");
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    m_chromaQpOffsetListLen = std::max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }

  // Now: getPpsRangeExtension().getLog2SaoOffsetScale and getPpsRangeExtension().setLog2SaoOffsetScale
  UInt                   getLog2SaoOffsetScale(ChannelType type) const                    { return m_log2SaoOffsetScale[type];             }
  Void                   setLog2SaoOffsetScale(ChannelType type, UInt uiBitShift)         { m_log2SaoOffsetScale[type] = uiBitShift;       }

};


/// PPS class
class PPS
{
private:
  Int              m_PPSId;                    // pic_parameter_set_id
  Int              m_SPSId;                    // seq_parameter_set_id
  Int              m_picInitQPMinus26;
  Bool             m_useDQP;
  Bool             m_bConstrainedIntraPred;    // constrained_intra_pred_flag
  Bool             m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  // access channel
  UInt             m_uiMaxCuDQPDepth;

  Int              m_chromaCbQpOffset;
  Int              m_chromaCrQpOffset;

  UInt             m_numRefIdxL0DefaultActive;
  UInt             m_numRefIdxL1DefaultActive;

  Bool             m_bUseWeightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  Bool             m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  Bool             m_OutputFlagPresentFlag;             //!< Indicates the presence of output_flag in slice header
  Bool             m_TransquantBypassEnabledFlag;       //!< Indicates presence of cu_transquant_bypass_flag in CUs.
  Bool             m_useTransformSkip;
#if HEVC_DEPENDENT_SLICES
  Bool             m_dependentSliceSegmentsEnabledFlag; //!< Indicates the presence of dependent slices
#endif
#if HEVC_TILES_WPP
  Bool             m_tilesEnabledFlag;                  //!< Indicates the presence of tiles
  Bool             m_entropyCodingSyncEnabledFlag;      //!< Indicates the presence of wavefronts

  Bool             m_loopFilterAcrossTilesEnabledFlag;
  Bool             m_uniformSpacingFlag;
  Int              m_numTileColumnsMinus1;
  Int              m_numTileRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;
#endif

#if JVET_K0072
#else
#if HEVC_USE_SIGN_HIDING
  Bool             m_signDataHidingEnabledFlag;
#endif
#endif
  Bool             m_cabacInitPresentFlag;

  Bool             m_sliceHeaderExtensionPresentFlag;
  Bool             m_loopFilterAcrossSlicesEnabledFlag;
  Bool             m_deblockingFilterControlPresentFlag;
  Bool             m_deblockingFilterOverrideEnabledFlag;
  Bool             m_ppsDeblockingFilterDisabledFlag;
  Int              m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int              m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
#if HEVC_USE_SCALING_LISTS
  Bool             m_scalingListPresentFlag;
  ScalingList      m_scalingList;                       //!< ScalingList class
#endif
  Bool             m_listsModificationPresentFlag;
  UInt             m_log2ParallelMergeLevelMinus2;
  Int              m_numExtraSliceHeaderBits;

  PPSRExt          m_ppsRangeExtension;

public:
  PreCalcValues   *pcv;

public:
                         PPS();
  virtual                ~PPS();

  Int                    getPPSId() const                                                 { return m_PPSId;                               }
  Void                   setPPSId(Int i)                                                  { m_PPSId = i;                                  }
  Int                    getSPSId() const                                                 { return m_SPSId;                               }
  Void                   setSPSId(Int i)                                                  { m_SPSId = i;                                  }

  Int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  Void                   setPicInitQPMinus26( Int i )                                     { m_picInitQPMinus26 = i;                       }
  Bool                   getUseDQP() const                                                { return m_useDQP;                              }
  Void                   setUseDQP( Bool b )                                              { m_useDQP   = b;                               }
  Bool                   getConstrainedIntraPred() const                                  { return  m_bConstrainedIntraPred;              }
  Void                   setConstrainedIntraPred( Bool b )                                { m_bConstrainedIntraPred = b;                  }
  Bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  Void                   setSliceChromaQpFlag( Bool b )                                   { m_bSliceChromaQpFlag = b;                     }

  Void                   setMaxCuDQPDepth( UInt u )                                       { m_uiMaxCuDQPDepth = u;                        }
  UInt                   getMaxCuDQPDepth() const                                         { return m_uiMaxCuDQPDepth;                     }

  Void                   setQpOffset(ComponentID compID, Int i )
  {
    if      (compID==COMPONENT_Cb)
    {
      m_chromaCbQpOffset = i;
    }
    else if (compID==COMPONENT_Cr)
    {
      m_chromaCrQpOffset = i;
    }
    else
    {
      THROW( "Invalid chroma QP offset" );
    }
  }
  Int                    getQpOffset(ComponentID compID) const
  {
    return (compID==COMPONENT_Y) ? 0 : (compID==COMPONENT_Cb ? m_chromaCbQpOffset : m_chromaCrQpOffset );
  }

  Void                   setNumRefIdxL0DefaultActive(UInt ui)                             { m_numRefIdxL0DefaultActive=ui;                }
  UInt                   getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  Void                   setNumRefIdxL1DefaultActive(UInt ui)                             { m_numRefIdxL1DefaultActive=ui;                }
  UInt                   getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  Bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  Bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  Void                   setUseWP( Bool b )                                               { m_bUseWeightPred = b;                         }
  Void                   setWPBiPred( Bool b )                                            { m_useWeightedBiPred = b;                      }

  Void                   setOutputFlagPresentFlag( Bool b )                               { m_OutputFlagPresentFlag = b;                  }
  Bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  Void                   setTransquantBypassEnabledFlag( Bool b )                         { m_TransquantBypassEnabledFlag = b;            }
  Bool                   getTransquantBypassEnabledFlag() const                           { return m_TransquantBypassEnabledFlag;         }

  Bool                   getUseTransformSkip() const                                      { return m_useTransformSkip;                    }
  Void                   setUseTransformSkip( Bool b )                                    { m_useTransformSkip  = b;                      }

#if HEVC_TILES_WPP
  Void                   setLoopFilterAcrossTilesEnabledFlag(Bool b)                      { m_loopFilterAcrossTilesEnabledFlag = b;       }
  Bool                   getLoopFilterAcrossTilesEnabledFlag() const                      { return m_loopFilterAcrossTilesEnabledFlag;    }
#endif
#if HEVC_DEPENDENT_SLICES
  Bool                   getDependentSliceSegmentsEnabledFlag() const                     { return m_dependentSliceSegmentsEnabledFlag;   }
  Void                   setDependentSliceSegmentsEnabledFlag(Bool val)                   { m_dependentSliceSegmentsEnabledFlag = val;    }
#endif
#if HEVC_TILES_WPP
  Bool                   getEntropyCodingSyncEnabledFlag() const                          { return m_entropyCodingSyncEnabledFlag;        }
  Void                   setEntropyCodingSyncEnabledFlag(Bool val)                        { m_entropyCodingSyncEnabledFlag = val;         }

  Void                   setTilesEnabledFlag(Bool val)                                    { m_tilesEnabledFlag = val;                     }
  Bool                   getTilesEnabledFlag() const                                      { return m_tilesEnabledFlag;                    }
  Void                   setTileUniformSpacingFlag(Bool b)                                { m_uniformSpacingFlag = b;                     }
  Bool                   getTileUniformSpacingFlag() const                                { return m_uniformSpacingFlag;                  }
  Void                   setNumTileColumnsMinus1(Int i)                                   { m_numTileColumnsMinus1 = i;                   }
  Int                    getNumTileColumnsMinus1() const                                  { return m_numTileColumnsMinus1;                }
  Void                   setTileColumnWidth(const std::vector<Int>& columnWidth )         { m_tileColumnWidth = columnWidth;              }
  UInt                   getTileColumnWidth(UInt columnIdx) const                         { return  m_tileColumnWidth[columnIdx];         }
  Void                   setNumTileRowsMinus1(Int i)                                      { m_numTileRowsMinus1 = i;                      }
  Int                    getNumTileRowsMinus1() const                                     { return m_numTileRowsMinus1;                   }
  Void                   setTileRowHeight(const std::vector<Int>& rowHeight)              { m_tileRowHeight = rowHeight;                  }
  UInt                   getTileRowHeight(UInt rowIdx) const                              { return m_tileRowHeight[rowIdx];               }
#endif

#if JVET_K0072
#else
#if HEVC_USE_SIGN_HIDING
  Void                   setSignDataHidingEnabledFlag( Bool b )                           { m_signDataHidingEnabledFlag = b;              }
  Bool                   getSignDataHidingEnabledFlag() const                             { return m_signDataHidingEnabledFlag;           }
#endif
#endif
  Void                   setCabacInitPresentFlag( Bool flag )                             { m_cabacInitPresentFlag = flag;                }
  Bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
  Void                   setDeblockingFilterControlPresentFlag( Bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  Bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  Void                   setDeblockingFilterOverrideEnabledFlag( Bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  Bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  Void                   setPPSDeblockingFilterDisabledFlag(Bool val)                     { m_ppsDeblockingFilterDisabledFlag = val;      } //!< set offset for deblocking filter disabled
  Bool                   getPPSDeblockingFilterDisabledFlag() const                       { return m_ppsDeblockingFilterDisabledFlag;     } //!< get offset for deblocking filter disabled
  Void                   setDeblockingFilterBetaOffsetDiv2(Int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  Int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  Void                   setDeblockingFilterTcOffsetDiv2(Int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  Int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
#if HEVC_USE_SCALING_LISTS
  Bool                   getScalingListPresentFlag() const                                { return m_scalingListPresentFlag;              }
  Void                   setScalingListPresentFlag( Bool b )                              { m_scalingListPresentFlag  = b;                }
  ScalingList&           getScalingList()                                                 { return m_scalingList;                         }
  const ScalingList&     getScalingList() const                                           { return m_scalingList;                         }
#endif
  Bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  Void                   setListsModificationPresentFlag( Bool b )                        { m_listsModificationPresentFlag = b;           }
  UInt                   getLog2ParallelMergeLevelMinus2() const                          { return m_log2ParallelMergeLevelMinus2;        }
  Void                   setLog2ParallelMergeLevelMinus2(UInt mrgLevel)                   { m_log2ParallelMergeLevelMinus2 = mrgLevel;    }
  Int                    getNumExtraSliceHeaderBits() const                               { return m_numExtraSliceHeaderBits;             }
  Void                   setNumExtraSliceHeaderBits(Int i)                                { m_numExtraSliceHeaderBits = i;                }
  Void                   setLoopFilterAcrossSlicesEnabledFlag( Bool bValue )              { m_loopFilterAcrossSlicesEnabledFlag = bValue; }
  Bool                   getLoopFilterAcrossSlicesEnabledFlag() const                     { return m_loopFilterAcrossSlicesEnabledFlag;   }
  Bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  Void                   setSliceHeaderExtensionPresentFlag(Bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

  const PPSRExt&         getPpsRangeExtension() const                                     { return m_ppsRangeExtension;                   }
  PPSRExt&               getPpsRangeExtension()                                           { return m_ppsRangeExtension;                   }
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  Bool bPresentFlag;
  UInt uiLog2WeightDenom;
  Int  iWeight;
  Int  iOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  Int  w;
  Int  o;
  Int  offset;
  Int  shift;
  Int  round;

};
struct WPACDCParam
{
  Int64 iAC;
  Int64 iDC;
};



/// slice header class
class Slice
{

private:
  //  Bitstream writing
  Bool                       m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];
  Int                        m_iPPSId;               ///< picture parameter set ID
  Bool                       m_PicOutputFlag;        ///< pic_output_flag
  Int                        m_iPOC;
  Int                        m_iLastIDR;
  Int                        m_iAssociatedIRAP;
  NalUnitType                m_iAssociatedIRAPType;
  const ReferencePictureSet* m_pRPS;             //< pointer to RPS, either in the SPS or the local RPS in the same slice header
  ReferencePictureSet        m_localRPS;             //< RPS when present in slice header
  Int                        m_rpsIdx;               //< index of used RPS in the SPS or -1 for local RPS in the slice header
  RefPicListModification     m_RefPicListModification;
  NalUnitType                m_eNalUnitType;         ///< Nal unit type for the slice
  SliceType                  m_eSliceType;
  Int                        m_iSliceQp;
  Int                        m_iSliceQpBase;
#if HEVC_DEPENDENT_SLICES
  Bool                       m_dependentSliceSegmentFlag;
#endif
  Bool                       m_ChromaQpAdjEnabled;
  Bool                       m_deblockingFilterDisable;
  Bool                       m_deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  Int                        m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int                        m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  Int                        m_list1IdxToList0Idx[MAX_NUM_REF];
  Int                        m_aiNumRefIdx   [NUM_REF_PIC_LIST_01];    //  for multiple reference of current slice
  Bool                       m_pendingRasInit;

#if JEM_TOOLS
  Bool                       m_bioLDBPossible;
  bool                       m_UseLIC;
#endif
#if JVET_K0072 
  bool                       m_depQuantEnabledFlag;
#if HEVC_USE_SIGN_HIDING
  bool                       m_signDataHidingEnabledFlag;
#endif
#endif
  Bool                       m_bCheckLDC;

  //  Data
  Int                        m_iSliceQpDelta;
  Int                        m_iSliceChromaQpDelta[MAX_NUM_COMPONENT];
  Picture*                   m_apcRefPicList [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  Int                        m_aiRefPOCList  [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  Bool                       m_bIsUsedAsLongTerm[NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  Int                        m_iDepth;

#if JEM_TOOLS
  Int                        m_iFrucRefIdxPair[2][MAX_NUM_REF+1];
  Bool                       m_bFrucRefIdxPairValid;
  Int                        m_iScaleFactor[256][256];
  Bool                       m_bScaleFactorValid;
#endif

  // access channel
#if HEVC_VPS
  const VPS*                 m_pcVPS;
#endif
  const SPS*                 m_pcSPS;
  const PPS*                 m_pcPPS;
  Picture*                   m_pcPic;
  Bool                       m_colFromL0Flag;  // collocated picture from List0 flag

  Bool                       m_noOutputPriorPicsFlag;
  Bool                       m_noRaslOutputFlag;
  Bool                       m_handleCraAsBlaFlag;

  UInt                       m_colRefIdx;
  UInt                       m_maxNumMergeCand;

  Double                     m_lambdas[MAX_NUM_COMPONENT];

  Bool                       m_abEqualRef  [NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_REF];
  UInt                       m_uiTLayer;
  Bool                       m_bTLayerSwitchingFlag;

  SliceConstraint            m_sliceMode;
  UInt                       m_sliceArgument;
  UInt                       m_sliceCurStartCtuTsAddr;
  UInt                       m_sliceCurEndCtuTsAddr;
  UInt                       m_independentSliceIdx;
#if HEVC_DEPENDENT_SLICES
  UInt                       m_sliceSegmentIdx;
  SliceConstraint            m_sliceSegmentMode;
  UInt                       m_sliceSegmentArgument;
  UInt                       m_sliceSegmentCurStartCtuTsAddr;
  UInt                       m_sliceSegmentCurEndCtuTsAddr;
#endif
  Bool                       m_nextSlice;
#if HEVC_DEPENDENT_SLICES
  Bool                       m_nextSliceSegment;
#endif
  UInt                       m_sliceBits;
#if HEVC_DEPENDENT_SLICES
  UInt                       m_sliceSegmentBits;
#endif
  Bool                       m_bFinalized;

  Bool                       m_bTestWeightPred;
  Bool                       m_bTestWeightBiPred;
  WPScalingParam             m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT]; // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  WPACDCParam                m_weightACDCParam[MAX_NUM_COMPONENT];
  ClpRngs                    m_clpRngs;
  std::vector<UInt>          m_substreamSizes;

  Bool                       m_cabacInitFlag;
  int                        m_cabacWinUpdateMode;

  Bool                       m_bLMvdL1Zero;
  Bool                       m_temporalLayerNonReferenceFlag;
  Bool                       m_LFCrossSliceBoundaryFlag;

  Bool                       m_enableTMVPFlag;

#if JVET_K0346
  bool                       m_subPuMvpSubBlkSizeSliceEnable;
  int                        m_subPuMvpSubBlkLog2Size;
#endif

  SliceType                  m_encCABACTableIdx;           // Used to transmit table selection across slices.

  clock_t                    m_iProcessingStartTime;
  Double                     m_dProcessingTime;
  UInt                       m_uiMaxBTSize;


public:
                              Slice();
  virtual                     ~Slice();
  Void                        initSlice();
  Int                         getRefIdx4MVPair( RefPicList eCurRefPicList, Int nCurRefIdx );
#if JEM_TOOLS
  inline Int                  getScaleFactor( Int iTDB, Int iTDD )                   { return Slice::m_iScaleFactor[128+iTDB][128+iTDD];                    }
#endif
#if HEVC_VPS
  Void                        setVPS( VPS* pcVPS )                                   { m_pcVPS = pcVPS;                                              }
  const VPS*                  getVPS() const                                         { return m_pcVPS;                                               }
#endif
  Void                        setSPS( const SPS* pcSPS )                             { m_pcSPS = pcSPS;                                              }
  const SPS*                  getSPS() const                                         { return m_pcSPS;                                               }

  Void                        setPPS( const PPS* pcPPS )                             { m_pcPPS = pcPPS; m_iPPSId = (pcPPS) ? pcPPS->getPPSId() : -1; }
  const PPS*                  getPPS() const                                         { return m_pcPPS;                                               }

  Void                        setPPSId( Int PPSId )                                  { m_iPPSId = PPSId;                                             }
  Int                         getPPSId() const                                       { return m_iPPSId;                                              }
  Void                        setPicOutputFlag( Bool b   )                           { m_PicOutputFlag = b;                                          }
  Bool                        getPicOutputFlag() const                               { return m_PicOutputFlag;                                       }
  Void                        setSaoEnabledFlag(ChannelType chType, Bool s)          {m_saoEnabledFlag[chType] =s;                                   }
  Bool                        getSaoEnabledFlag(ChannelType chType) const            { return m_saoEnabledFlag[chType];                              }
#if JEM_TOOLS
  Void                        setBioLDBPossible( Bool b )                            { m_bioLDBPossible = b;                                         }
  Bool                        getBioLDBPossible() const                              { return m_bioLDBPossible;                                      }
#endif
  Void                        setRPS( const ReferencePictureSet *pcRPS )             { m_pRPS = pcRPS;                                               }
  const ReferencePictureSet*  getRPS()                                               { return m_pRPS;                                                }
  ReferencePictureSet*        getLocalRPS()                                          { return &m_localRPS;                                           }

  Void                        setRPSidx( Int rpsIdx )                                { m_rpsIdx = rpsIdx;                                            }
  Int                         getRPSidx() const                                      { return m_rpsIdx;                                              }
  RefPicListModification*     getRefPicListModification()                            { return &m_RefPicListModification;                             }
  Void                        setLastIDR(Int iIDRPOC)                                { m_iLastIDR = iIDRPOC;                                         }
  Int                         getLastIDR() const                                     { return m_iLastIDR;                                            }
  Void                        setAssociatedIRAPPOC(Int iAssociatedIRAPPOC)           { m_iAssociatedIRAP = iAssociatedIRAPPOC;                       }
  Int                         getAssociatedIRAPPOC() const                           { return m_iAssociatedIRAP;                                     }
  Void                        setAssociatedIRAPType(NalUnitType associatedIRAPType)  { m_iAssociatedIRAPType = associatedIRAPType;                   }
  NalUnitType                 getAssociatedIRAPType() const                          { return m_iAssociatedIRAPType;                                 }
  SliceType                   getSliceType() const                                   { return m_eSliceType;                                          }
  Int                         getPOC() const                                         { return m_iPOC;                                                }
  Int                         getSliceQp() const                                     { return m_iSliceQp;                                            }
  Bool                        getUseWeightedPrediction() const                       { return( (m_eSliceType==P_SLICE && testWeightPred()) || (m_eSliceType==B_SLICE && testWeightBiPred()) ); }
#if HEVC_DEPENDENT_SLICES
  Bool                        getDependentSliceSegmentFlag() const                   { return m_dependentSliceSegmentFlag;                           }
  Void                        setDependentSliceSegmentFlag(Bool val)                 { m_dependentSliceSegmentFlag = val;                            }
#endif
  Int                         getSliceQpDelta() const                                { return m_iSliceQpDelta;                                       }
  Int                         getSliceChromaQpDelta(ComponentID compID) const        { return isLuma(compID) ? 0 : m_iSliceChromaQpDelta[compID];    }
  Bool                        getUseChromaQpAdj() const                              { return m_ChromaQpAdjEnabled;                                  }
  Bool                        getDeblockingFilterDisable() const                     { return m_deblockingFilterDisable;                             }
  Bool                        getDeblockingFilterOverrideFlag() const                { return m_deblockingFilterOverrideFlag;                        }
  Int                         getDeblockingFilterBetaOffsetDiv2()const               { return m_deblockingFilterBetaOffsetDiv2;                      }
  Int                         getDeblockingFilterTcOffsetDiv2() const                { return m_deblockingFilterTcOffsetDiv2;                        }
  Bool                        getPendingRasInit() const                              { return m_pendingRasInit;                                      }
  Void                        setPendingRasInit( Bool val )                          { m_pendingRasInit = val;                                       }

  Int                         getNumRefIdx( RefPicList e ) const                     { return m_aiNumRefIdx[e];                                      }
  Picture*                    getPic()                                               { return m_pcPic;                                               }
  const Picture*              getPic() const                                         { return m_pcPic;                                               }
  const Picture*              getRefPic( RefPicList e, Int iRefIdx) const            { return m_apcRefPicList[e][iRefIdx];                           }
  Int                         getRefPOC( RefPicList e, Int iRefIdx) const            { return m_aiRefPOCList[e][iRefIdx];                            }
  Int                         getDepth() const                                       { return m_iDepth;                                              }
  Bool                        getColFromL0Flag() const                               { return m_colFromL0Flag;                                       }
  UInt                        getColRefIdx() const                                   { return m_colRefIdx;                                           }
  Void                        checkColRefIdx(UInt curSliceSegmentIdx, const Picture* pic);
  Bool                        getIsUsedAsLongTerm(Int i, Int j) const                { return m_bIsUsedAsLongTerm[i][j];                             }
  Void                        setIsUsedAsLongTerm(Int i, Int j, Bool value)          { m_bIsUsedAsLongTerm[i][j] = value;                            }
  Bool                        getCheckLDC() const                                    { return m_bCheckLDC;                                           }
  Bool                        getMvdL1ZeroFlag() const                               { return m_bLMvdL1Zero;                                         }
  Int                         getNumRpsCurrTempList() const;
  Int                         getList1IdxToList0Idx( Int list1Idx ) const            { return m_list1IdxToList0Idx[list1Idx];                        }
  Bool                        isReferenceNalu() const                                { return ((getNalUnitType() <= NAL_UNIT_RESERVED_VCL_R15) && (getNalUnitType()%2 != 0)) || ((getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) && (getNalUnitType() <= NAL_UNIT_RESERVED_IRAP_VCL23) ); }
  Void                        setPOC( Int i )                                        { m_iPOC              = i;                                      }
  Void                        setNalUnitType( NalUnitType e )                        { m_eNalUnitType      = e;                                      }
  NalUnitType                 getNalUnitType() const                                 { return m_eNalUnitType;                                        }
  Bool                        getRapPicFlag() const;
  Bool                        getIdrPicFlag() const                                  { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  Bool                        isIRAP() const                                         { return (getNalUnitType() >= 16) && (getNalUnitType() <= 23);  }
  Bool                        isIDRorBLA() const                                      { return (getNalUnitType() >= 16) && (getNalUnitType() <= 20);  }
  Void                        checkCRA(const ReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, PicList& rcListPic);
  Void                        decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled);
  Void                        setSliceType( SliceType e )                            { m_eSliceType        = e;                                      }
  Void                        setSliceQp( Int i )                                    { m_iSliceQp          = i;                                      }
  Void                        setSliceQpDelta( Int i )                               { m_iSliceQpDelta     = i;                                      }
  Void                        setSliceChromaQpDelta( ComponentID compID, Int i )     { m_iSliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i;       }
  Void                        setUseChromaQpAdj( Bool b )                            { m_ChromaQpAdjEnabled = b;                                     }
  Void                        setDeblockingFilterDisable( Bool b )                   { m_deblockingFilterDisable= b;                                 }
  Void                        setDeblockingFilterOverrideFlag( Bool b )              { m_deblockingFilterOverrideFlag = b;                           }
  Void                        setDeblockingFilterBetaOffsetDiv2( Int i )             { m_deblockingFilterBetaOffsetDiv2 = i;                         }
  Void                        setDeblockingFilterTcOffsetDiv2( Int i )               { m_deblockingFilterTcOffsetDiv2 = i;                           }

  Void                        setNumRefIdx( RefPicList e, Int i )                    { m_aiNumRefIdx[e]    = i;                                      }
  Void                        setPic( Picture* p )                                   { m_pcPic             = p;                                      }
  Void                        setDepth( Int iDepth )                                 { m_iDepth            = iDepth;                                 }

  Void                        setRefPicList( PicList& rcListPic, Bool checkNumPocTotalCurr = false, Bool bCopyL0toL1ErrorCase = false );
  Void                        setRefPOCList();

  Void                        setColFromL0Flag( Bool colFromL0 )                     { m_colFromL0Flag = colFromL0;                                  }
  Void                        setColRefIdx( UInt refIdx)                             { m_colRefIdx = refIdx;                                         }
  Void                        setCheckLDC( Bool b )                                  { m_bCheckLDC = b;                                              }
  Void                        setMvdL1ZeroFlag( Bool b)                              { m_bLMvdL1Zero = b;                                            }

  Bool                        isIntra() const                                        { return m_eSliceType == I_SLICE;                               }
  Bool                        isInterB() const                                       { return m_eSliceType == B_SLICE;                               }
  Bool                        isInterP() const                                       { return m_eSliceType == P_SLICE;                               }

  Void                        setLambdas( const Double lambdas[MAX_NUM_COMPONENT] )  { for (Int component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  const Double*               getLambdas() const                                     { return m_lambdas;                                             }

  Void                        setMaxBTSize(Int i)                                    { m_uiMaxBTSize = i; }
  UInt                        getMaxBTSize() const                                   { return m_uiMaxBTSize; }

#if JEM_TOOLS
  bool                        getUseLIC()                                   const    { return m_UseLIC; }
  void                        setUseLIC( bool b )                                    { m_UseLIC = b; }
  void                        setUseLICOnPicLevel( bool fastPicDecision );
#endif
#if JVET_K0072
  Void                        setDepQuantEnabledFlag( Bool b )                       { m_depQuantEnabledFlag = b; }
  Bool                        getDepQuantEnabledFlag() const                         { return m_depQuantEnabledFlag; }
#if HEVC_USE_SIGN_HIDING
  Void                        setSignDataHidingEnabledFlag( Bool b )                 { m_signDataHidingEnabledFlag = b;              }
  Bool                        getSignDataHidingEnabledFlag() const                   { return m_signDataHidingEnabledFlag;           }
#endif
#endif

  Void                        initEqualRef();
  Bool                        isEqualRef( RefPicList e, Int iRefIdx1, Int iRefIdx2 )
  {
    CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid reference picture list");
    if (iRefIdx1 < 0 || iRefIdx2 < 0)
    {
      return false;
    }
    else
    {
      return m_abEqualRef[e][iRefIdx1][iRefIdx2];
    }
  }

  Void                        setEqualRef( RefPicList e, Int iRefIdx1, Int iRefIdx2, Bool b)
  {
    CHECK( e >= NUM_REF_PIC_LIST_01, "Invalid reference picture list" );
    m_abEqualRef[e][iRefIdx1][iRefIdx2] = m_abEqualRef[e][iRefIdx2][iRefIdx1] = b;
  }

  static Void                 sortPicList( PicList& rcListPic );
  Void                        setList1IdxToList0Idx();

  UInt                        getTLayer() const                                      { return m_uiTLayer;                                            }
  Void                        setTLayer( UInt uiTLayer )                             { m_uiTLayer = uiTLayer;                                        }

  Void                        checkLeadingPictureRestrictions( PicList& rcListPic )                                         const;
  Void                        applyReferencePictureSet( PicList& rcListPic, const ReferencePictureSet *RPSList)             const;
  Bool                        isTemporalLayerSwitchingPoint( PicList& rcListPic )                                           const;
  Bool                        isStepwiseTemporalLayerSwitchingPointCandidate( PicList& rcListPic )                          const;
  Int                         checkThatAllRefPicsAreAvailable( PicList& rcListPic, const ReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess = 0, Bool bUseRecoveryPoint = false) const;
  Void                        createExplicitReferencePictureSetFromReference( PicList& rcListPic, const ReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint, const Bool bEfficientFieldIRAPEnabled);
  Void                        setMaxNumMergeCand(UInt val )                          { m_maxNumMergeCand = val;                                      }
  UInt                        getMaxNumMergeCand() const                             { return m_maxNumMergeCand;                                     }

  Void                        setNoOutputPriorPicsFlag( Bool val )                   { m_noOutputPriorPicsFlag = val;                                }
  Bool                        getNoOutputPriorPicsFlag() const                       { return m_noOutputPriorPicsFlag;                               }

  Void                        setNoRaslOutputFlag( Bool val )                        { m_noRaslOutputFlag = val;                                     }
  Bool                        getNoRaslOutputFlag() const                            { return m_noRaslOutputFlag;                                    }

  Void                        setHandleCraAsBlaFlag( Bool val )                      { m_handleCraAsBlaFlag = val;                                   }
  Bool                        getHandleCraAsBlaFlag() const                          { return m_handleCraAsBlaFlag;                                  }

  Void                        setSliceMode( SliceConstraint mode )                   { m_sliceMode = mode;                                           }
  SliceConstraint             getSliceMode() const                                   { return m_sliceMode;                                           }
  Void                        setSliceArgument( UInt uiArgument )                    { m_sliceArgument = uiArgument;                                 }
  UInt                        getSliceArgument() const                               { return m_sliceArgument;                                       }
  Void                        setSliceCurStartCtuTsAddr( UInt ctuTsAddr )            { m_sliceCurStartCtuTsAddr = ctuTsAddr;                         } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceCurStartCtuTsAddr() const                      { return m_sliceCurStartCtuTsAddr;                              } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setSliceCurEndCtuTsAddr( UInt ctuTsAddr )              { m_sliceCurEndCtuTsAddr = ctuTsAddr;                           } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceCurEndCtuTsAddr() const                        { return m_sliceCurEndCtuTsAddr;                                } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setIndependentSliceIdx( UInt i)                        { m_independentSliceIdx = i;                                    }
  UInt                        getIndependentSliceIdx() const                         { return  m_independentSliceIdx;                                }
#if HEVC_DEPENDENT_SLICES
  Void                        setSliceSegmentIdx( UInt i)                            { m_sliceSegmentIdx = i;                                        }
  UInt                        getSliceSegmentIdx() const                             { return  m_sliceSegmentIdx;                                    }
#endif
  Void                        copySliceInfo(Slice *pcSliceSrc, bool cpyAlmostAll = true);
#if HEVC_DEPENDENT_SLICES
  Void                        setSliceSegmentMode( SliceConstraint mode )            { m_sliceSegmentMode = mode;                                    }
  SliceConstraint             getSliceSegmentMode() const                            { return m_sliceSegmentMode;                                    }
  Void                        setSliceSegmentArgument( UInt uiArgument )             { m_sliceSegmentArgument = uiArgument;                          }
  UInt                        getSliceSegmentArgument() const                        { return m_sliceSegmentArgument;                                }
#if HEVC_TILES_WPP
  Void                        setSliceSegmentCurStartCtuTsAddr( UInt ctuTsAddr )     { m_sliceSegmentCurStartCtuTsAddr = ctuTsAddr;                  } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceSegmentCurStartCtuTsAddr() const               { return m_sliceSegmentCurStartCtuTsAddr;                       } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setSliceSegmentCurEndCtuTsAddr( UInt ctuTsAddr )       { m_sliceSegmentCurEndCtuTsAddr = ctuTsAddr;                    } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceSegmentCurEndCtuTsAddr() const                 { return m_sliceSegmentCurEndCtuTsAddr;                         } // CTU Tile-scan address (as opposed to raster-scan)
#endif
#endif
  Void                        setSliceBits( UInt uiVal )                             { m_sliceBits = uiVal;                                          }
  UInt                        getSliceBits() const                                   { return m_sliceBits;                                           }
#if HEVC_DEPENDENT_SLICES
  Void                        setSliceSegmentBits( UInt uiVal )                      { m_sliceSegmentBits = uiVal;                                   }
  UInt                        getSliceSegmentBits() const                            { return m_sliceSegmentBits;                                    }
#endif
  Void                        setFinalized( Bool uiVal )                             { m_bFinalized = uiVal;                                         }
  Bool                        getFinalized() const                                   { return m_bFinalized;                                          }
  Bool                        testWeightPred( ) const                                { return m_bTestWeightPred;                                     }
  Void                        setTestWeightPred( Bool bValue )                       { m_bTestWeightPred = bValue;                                   }
  Bool                        testWeightBiPred( ) const                              { return m_bTestWeightBiPred;                                   }
  Void                        setTestWeightBiPred( Bool bValue )                     { m_bTestWeightBiPred = bValue;                                 }
  Void                        setWpScaling( WPScalingParam  wp[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT] )
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam)*NUM_REF_PIC_LIST_01*MAX_NUM_REF*MAX_NUM_COMPONENT);
  }

  Void                        getWpScaling( RefPicList e, Int iRefIdx, WPScalingParam *&wp) const;

  Void                        resetWpScaling();
  Void                        initWpScaling(const SPS *sps);

  Void                        setWpAcDcParam( WPACDCParam wp[MAX_NUM_COMPONENT] )    { memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam)*MAX_NUM_COMPONENT); }

  Void                        getWpAcDcParam( const WPACDCParam *&wp ) const;
  Void                        initWpAcDcParam();

  Void                        clearSubstreamSizes( )                                 { return m_substreamSizes.clear();                              }
  UInt                        getNumberOfSubstreamSizes( )                           { return (UInt) m_substreamSizes.size();                        }
  Void                        addSubstreamSize( UInt size )                          { m_substreamSizes.push_back(size);                             }
  UInt                        getSubstreamSize( UInt idx )                           { CHECK(idx>=getNumberOfSubstreamSizes(),"Invalid index"); return m_substreamSizes[idx]; }

  Void                        setCabacInitFlag( Bool val )                           { m_cabacInitFlag = val;                                        } //!< set CABAC initial flag
  Bool                        getCabacInitFlag()                               const { return m_cabacInitFlag;                                       } //!< get CABAC initial flag
  Void                        setCabacWinUpdateMode( int mode )                      { m_cabacWinUpdateMode = mode;                                  }
  int                         getCabacWinUpdateMode()                          const { return m_cabacWinUpdateMode;                                  }
  Bool                        getTemporalLayerNonReferenceFlag()               const { return m_temporalLayerNonReferenceFlag;                       }
  Void                        setTemporalLayerNonReferenceFlag(Bool x)               { m_temporalLayerNonReferenceFlag = x;                          }
  Void                        setLFCrossSliceBoundaryFlag( Bool   val )              { m_LFCrossSliceBoundaryFlag = val;                             }
  Bool                        getLFCrossSliceBoundaryFlag()                    const { return m_LFCrossSliceBoundaryFlag;                            }

  Void                        setEnableTMVPFlag( Bool   b )                          { m_enableTMVPFlag = b;                                         }
  Bool                        getEnableTMVPFlag() const                              { return m_enableTMVPFlag;                                      }

  Void                        setEncCABACTableIdx( SliceType idx )                   { m_encCABACTableIdx = idx;                                     }
  SliceType                   getEncCABACTableIdx() const                            { return m_encCABACTableIdx;                                    }

#if JVET_K0346
  void                        setSubPuMvpSliceSubblkSizeEnable(bool b) { m_subPuMvpSubBlkSizeSliceEnable = b; }
  bool                        getSubPuMvpSliceSubblkSizeEnable()                  const { return m_subPuMvpSubBlkSizeSliceEnable; }
  void                        setSubPuMvpSubblkLog2Size(int n) { m_subPuMvpSubBlkLog2Size = n; }
  int                         getSubPuMvpSubblkLog2Size()                         const { return m_subPuMvpSubBlkLog2Size; }
#endif

  Void                        setSliceQpBase( Int i )                                { m_iSliceQpBase = i;                                           }
  Int                         getSliceQpBase()                                 const { return m_iSliceQpBase;                                        }

  Void                        setDefaultClpRng( const SPS& sps );
  const ClpRngs&              clpRngs()                                         const { return m_clpRngs;}
  const ClpRng&               clpRng( ComponentID id)                           const { return m_clpRngs.comp[id];}
  ClpRngs&                    getClpRngs()                                            { return m_clpRngs;}
  unsigned                    getMinPictureDistance()                           const ;
  void startProcessingTimer();
  void stopProcessingTimer();
  void resetProcessingTime()       { m_dProcessingTime = m_iProcessingStartTime = 0; }
  double getProcessingTime() const { return m_dProcessingTime; }

protected:
  Picture*              xGetRefPic        (PicList& rcListPic, Int poc);
  Picture*              xGetLongTermRefPic(PicList& rcListPic, Int poc, Bool pocHasMsb);
};// END CLASS DEFINITION Slice





Void calculateParameterSetChangedFlag(Bool &bChanged, const std::vector<UChar> *pOldData, const std::vector<UChar> *pNewData);

template <class T> class ParameterSetMap
{
public:
  template <class Tm>
  struct MapData
  {
    Bool                  bChanged;
    std::vector<UChar>   *pNaluData; // Can be null
    Tm*                   parameterSet;
  };

  ParameterSetMap(Int maxId)
  :m_maxId (maxId)
  ,m_activePsId(-1)
  ,m_lastActiveParameterSet(NULL)
  {}

  ~ParameterSetMap()
  {
    for (typename std::map<Int,MapData<T> >::iterator i = m_paramsetMap.begin(); i!= m_paramsetMap.end(); i++)
    {
      delete (*i).second.pNaluData;
      delete (*i).second.parameterSet;
    }
    delete m_lastActiveParameterSet; m_lastActiveParameterSet = NULL;
  }

  T *allocatePS(const Int psId)
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if ( m_paramsetMap.find(psId) == m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged = true;
      m_paramsetMap[psId].pNaluData=0;
      m_paramsetMap[psId].parameterSet = new T;
      setID(m_paramsetMap[psId].parameterSet, psId);
    }
    return m_paramsetMap[psId].parameterSet;
  }

  Void storePS(Int psId, T *ps, const std::vector<UChar> *pNaluData)
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      MapData<T> &mapData=m_paramsetMap[psId];

      // work out changed flag
      calculateParameterSetChangedFlag(mapData.bChanged, mapData.pNaluData, pNaluData);

      if( ! mapData.bChanged )
      {
        // just keep the old one
        delete ps;
        return;
      }

      if( m_activePsId == psId )
      {
        std::swap( m_paramsetMap[psId].parameterSet, m_lastActiveParameterSet );
      }
      delete m_paramsetMap[psId].pNaluData;
      delete m_paramsetMap[psId].parameterSet;

      m_paramsetMap[psId].parameterSet = ps;
    }
    else
    {
      m_paramsetMap[psId].parameterSet = ps;
      m_paramsetMap[psId].bChanged = false;
    }
    if (pNaluData != 0)
    {
      m_paramsetMap[psId].pNaluData=new std::vector<UChar>;
      *(m_paramsetMap[psId].pNaluData) = *pNaluData;
    }
    else
    {
      m_paramsetMap[psId].pNaluData=0;
    }
  }

  Void setChangedFlag(Int psId, Bool bChanged=true)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=bChanged;
    }
  }

  Void clearChangedFlag(Int psId)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=false;
    }
  }

  Bool getChangedFlag(Int psId) const
  {
    const typename std::map<Int,MapData<T> >::const_iterator constit=m_paramsetMap.find(psId);
    if ( constit != m_paramsetMap.end() )
    {
      return constit->second.bChanged;
    }
    return false;
  }

  T* getPS(Int psId)
  {
    typename std::map<Int,MapData<T> >::iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  const T* getPS(Int psId) const
  {
    typename std::map<Int,MapData<T> >::const_iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  T* getFirstPS()
  {
    return (m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second.parameterSet;
  }

  void setActive(Int psId ) { m_activePsId = psId;}

private:
  std::map<Int,MapData<T> > m_paramsetMap;
  Int                       m_maxId;
  Int                       m_activePsId;
  T*                        m_lastActiveParameterSet;
  static Void setID(T* parameterSet, const Int psId);
};


class ParameterSetManager
{
public:
                 ParameterSetManager();
  virtual        ~ParameterSetManager();

#if HEVC_VPS
  //! store sequence parameter set and take ownership of it
  Void           storeVPS(VPS *vps, const std::vector<UChar> &naluData)      { m_vpsMap.storePS( vps->getVPSId(), vps, &naluData); };
  //! get pointer to existing video parameter set
  VPS*           getVPS(Int vpsId)                                           { return m_vpsMap.getPS(vpsId); };
  Bool           getVPSChangedFlag(Int vpsId) const                          { return m_vpsMap.getChangedFlag(vpsId); }
  Void           clearVPSChangedFlag(Int vpsId)                              { m_vpsMap.clearChangedFlag(vpsId); }
  VPS*           getFirstVPS()                                               { return m_vpsMap.getFirstPS(); };
#endif

  //! store sequence parameter set and take ownership of it
  Void           storeSPS(SPS *sps, const std::vector<UChar> &naluData) { m_spsMap.storePS( sps->getSPSId(), sps, &naluData); };
  //! get pointer to existing sequence parameter set
  SPS*           getSPS(Int spsId)                                           { return m_spsMap.getPS(spsId); };
  Bool           getSPSChangedFlag(Int spsId) const                          { return m_spsMap.getChangedFlag(spsId); }
  Void           clearSPSChangedFlag(Int spsId)                              { m_spsMap.clearChangedFlag(spsId); }
  SPS*           getFirstSPS()                                               { return m_spsMap.getFirstPS(); };

  //! store picture parameter set and take ownership of it
  Void           storePPS(PPS *pps, const std::vector<UChar> &naluData) { m_ppsMap.storePS( pps->getPPSId(), pps, &naluData); };
  //! get pointer to existing picture parameter set
  PPS*           getPPS(Int ppsId)                                           { return m_ppsMap.getPS(ppsId); };
  Bool           getPPSChangedFlag(Int ppsId) const                          { return m_ppsMap.getChangedFlag(ppsId); }
  Void           clearPPSChangedFlag(Int ppsId)                              { m_ppsMap.clearChangedFlag(ppsId); }
  PPS*           getFirstPPS()                                               { return m_ppsMap.getFirstPS(); };

  //! activate a SPS from a active parameter sets SEI message
  //! \returns true, if activation is successful
  // Bool           activateSPSWithSEI(Int SPSId);

#if HEVC_VPS
  //! activate a PPS and depending on isIDR parameter also SPS and VPS
#else
  //! activate a PPS and depending on isIDR parameter also SPS
#endif
  //! \returns true, if activation is successful
  Bool           activatePPS(Int ppsId, Bool isIRAP);

#if HEVC_VPS
  const VPS*     getActiveVPS()const                                         { return m_vpsMap.getPS(m_activeVPSId); };
#endif
  const SPS*     getActiveSPS()const                                         { return m_spsMap.getPS(m_activeSPSId); };

protected:
#if HEVC_VPS
  ParameterSetMap<VPS> m_vpsMap;
#endif
  ParameterSetMap<SPS> m_spsMap;
  ParameterSetMap<PPS> m_ppsMap;

#if HEVC_VPS
  Int m_activeVPSId; // -1 for nothing active
#endif
  Int m_activeSPSId; // -1 for nothing active
};

class PreCalcValues
{
public:
  PreCalcValues( const SPS& sps, const PPS& pps, bool _isEncoder )
    : chrFormat           ( sps.getChromaFormatIdc() )
    , multiBlock422       ( chrFormat == CHROMA_422 && !sps.getSpsNext().getUseQTBT() )
#if JEM_TOOLS && !JVET_K0346
    , noMotComp           ( sps.getSpsNext().getDisableMotCompress() || sps.getSpsNext().getUseSubPuMvp() )
#else
    , noMotComp           ( sps.getSpsNext().getDisableMotCompress() )
#endif
    , maxCUWidth          ( sps.getMaxCUWidth() )
    , maxCUHeight         ( sps.getMaxCUHeight() )
    , maxCUWidthMask      ( maxCUWidth  - 1 )
    , maxCUHeightMask     ( maxCUHeight - 1 )
    , maxCUWidthLog2      ( g_aucLog2[ maxCUWidth  ] )
    , maxCUHeightLog2     ( g_aucLog2[ maxCUHeight ] )
    , minCUWidth          ( sps.getMaxCUWidth()  >> sps.getMaxCodingDepth() )
    , minCUHeight         ( sps.getMaxCUHeight() >> sps.getMaxCodingDepth() )
    , minCUWidthLog2      ( g_aucLog2[ minCUWidth  ] )
    , minCUHeightLog2     ( g_aucLog2[ minCUHeight ] )
    , partsInCtuWidth     ( 1 << sps.getMaxCodingDepth() )
    , partsInCtuHeight    ( 1 << sps.getMaxCodingDepth() )
    , partsInCtu          ( 1 << (sps.getMaxCodingDepth() << 1) )
    , widthInCtus         ( (sps.getPicWidthInLumaSamples () + sps.getMaxCUWidth () - 1) / sps.getMaxCUWidth () )
    , heightInCtus        ( (sps.getPicHeightInLumaSamples() + sps.getMaxCUHeight() - 1) / sps.getMaxCUHeight() )
    , sizeInCtus          ( widthInCtus * heightInCtus )
    , lumaWidth           ( sps.getPicWidthInLumaSamples() )
    , lumaHeight          ( sps.getPicHeightInLumaSamples() )
    , fastDeltaQPCuMaxSize( Clip3(sps.getMaxCUHeight() >> (sps.getLog2DiffMaxMinCodingBlockSize()), sps.getMaxCUHeight(), 32u) )
#if INTRA67_3MPM
    , numMPMs             (NUM_MOST_PROBABLE_MODES)
#else
#if JEM_TOOLS
    , numMPMs             ( sps.getSpsNext().getUseIntra65Ang() ? NUM_MOST_PROBABLE_MODES_67 : NUM_MOST_PROBABLE_MODES )
#else
    , numMPMs             ( NUM_MOST_PROBABLE_MODES )
#endif
#endif
    , noRQT               (  sps.getSpsNext().getUseQTBT() )
    , rectCUs             (  sps.getSpsNext().getUseQTBT() )
    , only2Nx2N           (  sps.getSpsNext().getUseQTBT() )
    , noChroma2x2         ( !sps.getSpsNext().getUseQTBT() )
    , isEncoder           ( _isEncoder )
    , ISingleTree         ( !sps.getSpsNext().getUseQTBT() || !sps.getSpsNext().getUseDualITree() )
    , maxBtDepth          { sps.getSpsNext().getMaxBTDepthI(), sps.getSpsNext().getMaxBTDepth(), sps.getSpsNext().getMaxBTDepthIChroma() }
    , minBtSize           { MIN_BT_SIZE, MIN_BT_SIZE_INTER, MIN_BT_SIZE_C }
    , maxBtSize           { sps.getSpsNext().getMaxBTSizeI(), sps.getSpsNext().getMaxBTSize(), sps.getSpsNext().getMaxBTSizeIChroma() }
    , minTtSize           { MIN_TT_SIZE, MIN_TT_SIZE_INTER, MIN_TT_SIZE_C }
    , maxTtSize           { MAX_TT_SIZE, MAX_TT_SIZE_INTER, MAX_TT_SIZE_C }
    , minQtSize           { sps.getSpsNext().getMinQTSize( I_SLICE, CHANNEL_TYPE_LUMA ), sps.getSpsNext().getMinQTSize( B_SLICE, CHANNEL_TYPE_LUMA ), sps.getSpsNext().getMinQTSize( I_SLICE, CHANNEL_TYPE_CHROMA ) }
  {}

  const ChromaFormat chrFormat;
  const bool         multiBlock422;
  const bool         noMotComp;
  const unsigned     maxCUWidth;
  const unsigned     maxCUHeight;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUWidthMask;
  const unsigned     maxCUHeightMask;
  const unsigned     maxCUWidthLog2;
  const unsigned     maxCUHeightLog2;
  const unsigned     minCUWidth;
  const unsigned     minCUHeight;
  const unsigned     minCUWidthLog2;
  const unsigned     minCUHeightLog2;
  const unsigned     partsInCtuWidth;
  const unsigned     partsInCtuHeight;
  const unsigned     partsInCtu;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     fastDeltaQPCuMaxSize;
  const unsigned     numMPMs;
  const bool         noRQT;
  const bool         rectCUs;
  const bool         only2Nx2N;
  const bool         noChroma2x2;
  const bool         isEncoder;
  const bool         ISingleTree;

private:
  const unsigned     maxBtDepth[3];
  const unsigned     minBtSize [3];
  const unsigned     maxBtSize [3];
  const unsigned     minTtSize [3];
  const unsigned     maxTtSize [3];
  const unsigned     minQtSize [3];

  unsigned getValIdx    ( const Slice &slice, const ChannelType chType ) const;

public:
  unsigned getMaxBtDepth( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinQtSize ( const Slice &slice, const ChannelType chType ) const;
};

#if ENABLE_TRACING
#if HEVC_VPS
void xTraceVPSHeader();
#endif
void xTraceSPSHeader();
void xTracePPSHeader();
void xTraceSliceHeader();
void xTraceAccessUnitDelimiter();
#endif

#endif // __SLICE__
