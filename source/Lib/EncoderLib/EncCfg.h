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

/** \file     EncCfg.h
    \brief    encoder configuration class (header)
*/

#ifndef __ENCCFG__
#define __ENCCFG__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"

#include "CommonLib/Unit.h"

struct GOPEntry
{
  Int m_POC;
  Int m_QPOffset;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  double m_QPOffsetModelOffset;
  double m_QPOffsetModelScale;
#endif
#if W0038_CQP_ADJ
  Int m_CbQPoffset;
  Int m_CrQPoffset;
#endif
  double m_QPFactor;
  Int m_tcOffsetDiv2;
  Int m_betaOffsetDiv2;
  Int m_temporalId;
  bool m_refPic;
  Int m_numRefPicsActive;
  SChar m_sliceType;
  Int m_numRefPics;
  Int m_referencePics[MAX_NUM_REF_PICS];
  Int m_usedByCurrPic[MAX_NUM_REF_PICS];
  Int m_interRPSPrediction;
  Int m_deltaRPS;
  Int m_numRefIdc;
  Int m_refIdc[MAX_NUM_REF_PICS+1];
  bool m_isEncoded;
  GOPEntry()
  : m_POC(-1)
  , m_QPOffset(0)
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  , m_QPOffsetModelOffset(0)
  , m_QPOffsetModelScale(0)
#endif
#if W0038_CQP_ADJ
  , m_CbQPoffset(0)
  , m_CrQPoffset(0)
#endif
  , m_QPFactor(0)
  , m_tcOffsetDiv2(0)
  , m_betaOffsetDiv2(0)
  , m_temporalId(0)
  , m_refPic(false)
  , m_numRefPicsActive(0)
  , m_sliceType('P')
  , m_numRefPics(0)
  , m_interRPSPrediction(false)
  , m_deltaRPS(0)
  , m_numRefIdc(0)
  , m_isEncoded(false)
  {
    ::memset( m_referencePics, 0, sizeof(m_referencePics) );
    ::memset( m_usedByCurrPic, 0, sizeof(m_usedByCurrPic) );
    ::memset( m_refIdc,        0, sizeof(m_refIdc) );
  }
};

std::istringstream &operator>>(std::istringstream &in, GOPEntry &entry);     //input
//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class EncCfg
{
protected:
  //==== File I/O ========
  Int       m_iFrameRate;
  Int       m_FrameSkip;
  UInt      m_temporalSubsampleRatio;
  Int       m_iSourceWidth;
  Int       m_iSourceHeight;
  Window    m_conformanceWindow;
  Int       m_framesToBeEncoded;
  double    m_adLambdaModifier[ MAX_TLAYER ];
  std::vector<double> m_adIntraLambdaModifier;
  double    m_dIntraQpFactor;                                 ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  bool      m_printMSEBasedSequencePSNR;
  bool      m_printHexPsnr;
  bool      m_printFrameMSE;
  bool      m_printSequenceMSE;
  bool      m_cabacZeroWordPaddingEnabled;


  /* profile & level */
  Profile::Name m_profile;
  Level::Tier   m_levelTier;
  Level::Name   m_level;
  bool m_progressiveSourceFlag;
  bool m_interlacedSourceFlag;
  bool m_nonPackedConstraintFlag;
  bool m_frameOnlyConstraintFlag;
  UInt              m_bitDepthConstraintValue;
  ChromaFormat      m_chromaFormatConstraintValue;
  bool              m_intraConstraintFlag;
  bool              m_onePictureOnlyConstraintFlag;
  bool              m_lowerBitRateConstraintFlag;

  //====== Coding Structure ========
  UInt      m_uiIntraPeriod;                    // TODO: make this an Int - it can be -1!
  UInt      m_uiDecodingRefreshType;            ///< the type of decoding refresh employed for the random access.
  Int       m_iGOPSize;
  GOPEntry  m_GOPList[MAX_GOP];
  Int       m_extraRPSs;
  Int       m_maxDecPicBuffering[MAX_TLAYER];
  Int       m_numReorderPics[MAX_TLAYER];

  Int       m_iQP;                              //  if (AdaptiveQP == OFF)
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  Int       m_intraQPOffset;                    ///< QP offset for intra slice (integer)
  Int       m_lambdaFromQPEnable;               ///< enable lambda derivation from QP
#endif
  Int       m_aiPad[2];

  bool      m_AccessUnitDelimiter;               ///< add Access Unit Delimiter NAL units

  Int       m_iMaxRefPicNum;                     ///< this is used to mimic the sliding mechanism used by the decoder
                                                 // TODO: We need to have a common sliding mechanism used by both the encoder and decoder

  int       m_maxTempLayer;                      ///< Max temporal layer
  bool      m_useAMP;
  bool      m_QTBT;
  unsigned  m_CTUSize;
  unsigned  m_uiMinQT[3]; //0: I slice; 1: P/B slice, 2: I slice chroma
  unsigned  m_uiMaxBTDepth;
  unsigned  m_uiMaxBTDepthI;
  unsigned  m_uiMaxBTDepthIChroma;
  bool      m_dualITree;
  unsigned  m_maxCUWidth;
  unsigned  m_maxCUHeight;
  unsigned  m_maxTotalCUDepth;
  unsigned  m_log2DiffMaxMinCodingBlockSize;

#if JEM_TOOLS
  bool      m_Intra4Tap;
#if !INTRA67_3MPM
  bool      m_Intra65Ang;
#endif
  bool      m_IntraBoundaryFilter;
#endif
#if JEM_TOOLS||JVET_K0190
  int       m_LMChroma;
#endif
#if JEM_TOOLS
  int       m_IntraPDPC;
#if !JVET_K0371_ALF
  int       m_ALF;
#endif
#endif
#if JEM_TOOLS
  bool      m_BIF;
#endif
#if JEM_TOOLS
  bool      m_MDMS;
#endif
#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  int       m_IntraEMT;
  int       m_InterEMT;
  int       m_FastIntraEMT;
  int       m_FastInterEMT;
#endif
#if JEM_TOOLS
  bool      m_NSST;
#endif
  bool      m_LargeCTU;
#if JEM_TOOLS || JVET_K0346
  int       m_SubPuMvpMode;
  unsigned  m_SubPuMvpLog2Size;
#endif
#if JEM_TOOLS
  unsigned  m_CABACEngineMode;
#if JVET_K0072
#else
  unsigned  m_altResiCompId;
#endif
#endif
#if JEM_TOOLS
  bool      m_highPrecMv;
  bool      m_Affine;
#if JVET_K0337_AFFINE_6PARA
  bool      m_AffineType;
#endif
  bool      m_BIO;
#endif
#if !JEM_TOOLS && JVET_K_AFFINE
  bool      m_Affine;
#if JVET_K0337_AFFINE_6PARA
  bool      m_AffineType;
#endif
#endif
#if !JEM_TOOLS && (JVET_K0346 || JVET_K_AFFINE)
  bool      m_highPrecMv;
#endif
  bool      m_DisableMotionCompression;
#if JEM_TOOLS
  unsigned  m_LICMode;
  bool      m_FastPicLevelLIC;
#endif
  unsigned  m_MTTMode;

#if ENABLE_WPP_PARALLELISM
  bool      m_AltDQPCoding;
#endif
#if JEM_TOOLS
  bool      m_OBMC;
  unsigned  m_uiObmcBlkSize;
  bool      m_FRUC;
  unsigned  m_FRUCRefineFilter;
  unsigned  m_FRUCRefineRange;
  unsigned  m_FRUCSmallBlkRefineDepth;
#endif
#if JEM_TOOLS
  unsigned  m_CIPF;
  bool      m_AClip;
  bool      m_AClipEnc;
#endif
#if JEM_TOOLS
  bool      m_DMVR;
#endif
  // ADD_NEW_TOOL : (encoder lib) add tool enabling flags and associated parameters here

  bool      m_useFastLCTU;
  bool      m_useFastMrg;
  bool      m_usePbIntraFast;
  bool      m_useAMaxBT;
#if !JVET_K0220_ENC_CTRL
  bool      m_useSaveLoadEncInfo;
  bool      m_useSaveLoadSplitDecision;
#endif
  bool      m_e0023FastEnc;
  bool      m_contentBasedFastQtbt;

  //======= Transform =============
  UInt      m_uiQuadtreeTULog2MaxSize;
  UInt      m_uiQuadtreeTULog2MinSize;
  UInt      m_uiQuadtreeTUMaxDepthInter;
  UInt      m_uiQuadtreeTUMaxDepthIntra;

  //====== Loop/Deblock Filter ========
  bool      m_bLoopFilterDisable;
  bool      m_loopFilterOffsetInPPS;
  Int       m_loopFilterBetaOffsetDiv2;
  Int       m_loopFilterTcOffsetDiv2;
#if W0038_DB_OPT
  Int       m_deblockingFilterMetric;
#else
  bool      m_DeblockingFilterMetric;
#endif
  bool      m_bUseSAO;
  bool      m_bTestSAODisableAtPictureLevel;
  double    m_saoEncodingRate;       // When non-0 SAO early picture termination is enabled for luma and chroma
  double    m_saoEncodingRateChroma; // The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  Int       m_maxNumOffsetsPerPic;
  bool      m_saoCtuBoundary;

#if K0238_SAO_GREEDY_MERGE_ENCODING
  bool      m_saoGreedyMergeEnc;
#endif
  //====== Motion search ========
  bool      m_bDisableIntraPUsInInterSlices;
  MESearchMethod m_motionEstimationSearchMethod;
  Int       m_iSearchRange;                     //  0:Full frame
  Int       m_bipredSearchRange;
  bool      m_bClipForBiPredMeEnabled;
  bool      m_bFastMEAssumingSmootherMVEnabled;
  Int       m_minSearchWindow;
  bool      m_bRestrictMESampling;

  //====== Quality control ========
  Int       m_iMaxDeltaQP;                      //  Max. absolute delta QP (1:default)
  Int       m_iMaxCuDQPDepth;                   //  Max. depth for a minimum CuDQP (0:default)
  Int       m_diffCuChromaQpOffsetDepth;        ///< If negative, then do not apply chroma qp offsets.

  Int       m_chromaCbQpOffset;                 //  Chroma Cb QP Offset (0:default)
  Int       m_chromaCrQpOffset;                 //  Chroma Cr Qp Offset (0:default)
  int       m_chromaCbQpOffsetDualTree;         //  Chroma Cb QP Offset for dual tree 
  int       m_chromaCrQpOffsetDualTree;         //  Chroma Cr Qp Offset for dual tree 
#if ER_CHROMA_QP_WCG_PPS
  WCGChromaQPControl m_wcgChromaQpControl;                    ///< Wide-colour-gamut chroma QP control.
#endif
#if W0038_CQP_ADJ
  UInt      m_sliceChromaQpOffsetPeriodicity;                 ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  Int       m_sliceChromaQpOffsetIntraOrPeriodic[2/*Cb,Cr*/]; ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
#endif

  ChromaFormat m_chromaFormatIDC;

  bool      m_extendedPrecisionProcessingFlag;
  bool      m_highPrecisionOffsetsEnabledFlag;
  bool      m_bUseAdaptiveQP;
  Int       m_iQPAdaptationRange;
#if ENABLE_QPA
  bool      m_bUsePerceptQPA;
  bool      m_bUseWPSNR;
#endif

  //====== Tool list ========
  Int       m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of input file
  Int       m_bitDepth[MAX_NUM_CHANNEL_TYPE];
  bool      m_bUseASR;
  bool      m_bUseHADME;
  bool      m_useRDOQ;
  bool      m_useRDOQTS;
#if T0196_SELECTIVE_RDOQ
  bool      m_useSelectiveRDOQ;
#endif
  UInt      m_rdPenalty;
  FastInterSearchMode m_fastInterSearchMode;
  bool      m_bUseEarlyCU;
  bool      m_useFastDecisionForMerge;
  bool      m_bUseCbfFastMode;
  bool      m_useEarlySkipDetection;
  bool      m_crossComponentPredictionEnabledFlag;
  bool      m_reconBasedCrossCPredictionEstimate;
  UInt      m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];
  bool      m_useTransformSkip;
  bool      m_useTransformSkipFast;
  UInt      m_log2MaxTransformSkipBlockSize;
  bool      m_transformSkipRotationEnabledFlag;
  bool      m_transformSkipContextEnabledFlag;
  bool      m_persistentRiceAdaptationEnabledFlag;
  bool      m_cabacBypassAlignmentEnabledFlag;
  bool      m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];
#if SHARP_LUMA_DELTA_QP
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping; ///< mapping from luma level to delta QP.
#endif
  Int*      m_aidQP;
  UInt      m_uiDeltaQpRD;
  bool      m_bFastDeltaQP;

  bool      m_bUseConstrainedIntraPred;
  bool      m_bFastUDIUseMPMEnabled;
  bool      m_bFastMEForGenBLowDelayEnabled;
  bool      m_bUseBLambdaForNonKeyLowDelayPictures;
  bool      m_usePCM;
  Int       m_PCMBitDepth[MAX_NUM_CHANNEL_TYPE];
  UInt      m_pcmLog2MaxSize;
  UInt      m_uiPCMLog2MinSize;
  //====== Slice ========
  SliceConstraint m_sliceMode;
  Int       m_sliceArgument;
  //====== Dependent Slice ========
  SliceConstraint m_sliceSegmentMode;
  Int       m_sliceSegmentArgument;
  bool      m_bLFCrossSliceBoundaryFlag;

  bool      m_bPCMInputBitDepthFlag;
  bool      m_bPCMFilterDisableFlag;
  bool      m_intraSmoothingDisabledFlag;
#if HEVC_TILES_WPP
  bool      m_loopFilterAcrossTilesEnabledFlag;
  bool      m_tileUniformSpacingFlag;
  Int       m_iNumColumnsMinus1;
  Int       m_iNumRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;

  bool      m_entropyCodingSyncEnabledFlag;
#endif

  HashType  m_decodedPictureHashSEIType;
  bool      m_bufferingPeriodSEIEnabled;
  bool      m_pictureTimingSEIEnabled;
  bool      m_recoveryPointSEIEnabled;
  bool      m_toneMappingInfoSEIEnabled;
  Int       m_toneMapId;
  bool      m_toneMapCancelFlag;
  bool      m_toneMapPersistenceFlag;
  Int       m_codedDataBitDepth;
  Int       m_targetBitDepth;
  Int       m_modelId;
  Int       m_minValue;
  Int       m_maxValue;
  Int       m_sigmoidMidpoint;
  Int       m_sigmoidWidth;
  Int       m_numPivots;
  Int       m_cameraIsoSpeedIdc;
  Int       m_cameraIsoSpeedValue;
  Int       m_exposureIndexIdc;
  Int       m_exposureIndexValue;
  bool      m_exposureCompensationValueSignFlag;
  Int       m_exposureCompensationValueNumerator;
  Int       m_exposureCompensationValueDenomIdc;
  Int       m_refScreenLuminanceWhite;
  Int       m_extendedRangeWhiteLevel;
  Int       m_nominalBlackLevelLumaCodeValue;
  Int       m_nominalWhiteLevelLumaCodeValue;
  Int       m_extendedWhiteLevelLumaCodeValue;
  Int*      m_startOfCodedInterval;
  Int*      m_codedPivotValue;
  Int*      m_targetPivotValue;
  bool      m_framePackingSEIEnabled;
  Int       m_framePackingSEIType;
  Int       m_framePackingSEIId;
  Int       m_framePackingSEIQuincunx;
  Int       m_framePackingSEIInterpretation;
  bool      m_segmentedRectFramePackingSEIEnabled;
  bool      m_segmentedRectFramePackingSEICancel;
  Int       m_segmentedRectFramePackingSEIType;
  bool      m_segmentedRectFramePackingSEIPersistence;
  Int       m_displayOrientationSEIAngle;
  bool      m_temporalLevel0IndexSEIEnabled;
  bool      m_gradualDecodingRefreshInfoEnabled;
  Int       m_noDisplaySEITLayer;
  bool      m_decodingUnitInfoSEIEnabled;
  bool      m_SOPDescriptionSEIEnabled;
  bool      m_scalableNestingSEIEnabled;
  bool      m_tmctsSEIEnabled;
  bool      m_timeCodeSEIEnabled;
  Int       m_timeCodeSEINumTs;
  SEITimeSet   m_timeSetArray[MAX_TIMECODE_SEI_SETS];
  bool      m_kneeSEIEnabled;
  Int       m_kneeSEIId;
  bool      m_kneeSEICancelFlag;
  bool      m_kneeSEIPersistenceFlag;
  Int       m_kneeSEIInputDrange;
  Int       m_kneeSEIInputDispLuminance;
  Int       m_kneeSEIOutputDrange;
  Int       m_kneeSEIOutputDispLuminance;
  Int       m_kneeSEINumKneePointsMinus1;
  Int*      m_kneeSEIInputKneePoint;
  Int*      m_kneeSEIOutputKneePoint;
  std::string m_colourRemapSEIFileRoot;          ///< SEI Colour Remapping File (initialized from external file)
  SEIMasteringDisplay m_masteringDisplay;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  bool      m_alternativeTransferCharacteristicsSEIEnabled;
  UChar     m_preferredTransferCharacteristics;
#endif
  bool      m_greenMetadataInfoSEIEnabled;
  UChar     m_greenMetadataType;
  UChar     m_xsdMetricType;
  //====== Weighted Prediction ========
  bool      m_useWeightedPred;       //< Use of Weighting Prediction (P_SLICE)
  bool      m_useWeightedBiPred;    //< Use of Bi-directional Weighting Prediction (B_SLICE)
  WeightedPredictionMethod m_weightedPredictionMethod;
  UInt      m_log2ParallelMergeLevelMinus2;       ///< Parallel merge estimation region
  UInt      m_maxNumMergeCand;                    ///< Maximum number of merge candidates
#if HEVC_USE_SCALING_LISTS
  ScalingListMode m_useScalingListId;             ///< Using quantization matrix i.e. 0=off, 1=default, 2=file.
  std::string m_scalingListFileName;              ///< quantization matrix file name
#endif
  Int       m_TMVPModeId;
#if JVET_K0072
  bool      m_DepQuantEnabledFlag;
#endif
  bool      m_SignDataHidingEnabledFlag;
  bool      m_RCEnableRateControl;
  Int       m_RCTargetBitrate;
  Int       m_RCKeepHierarchicalBit;
  bool      m_RCLCULevelRC;
  bool      m_RCUseLCUSeparateModel;
  Int       m_RCInitialQP;
  bool      m_RCForceIntraQP;
#if U0132_TARGET_BITS_SATURATION
  bool      m_RCCpbSaturationEnabled;
  UInt      m_RCCpbSize;
  double    m_RCInitialCpbFullness;
#endif
  bool      m_TransquantBypassEnabledFlag;                    ///< transquant_bypass_enabled_flag setting in PPS.
  bool      m_CUTransquantBypassFlagForce;                    ///< if transquant_bypass_enabled_flag, then, if true, all CU transquant bypass flags will be set to true.

  CostMode  m_costMode;                                       ///< The cost function to use, primarily when considering lossless coding.

#if HEVC_VPS
  VPS       m_cVPS;
#endif
  bool      m_recalculateQPAccordingToLambda;                 ///< recalculate QP value according to the lambda value
  Int       m_activeParameterSetsSEIEnabled;                  ///< enable active parameter set SEI message
  bool      m_vuiParametersPresentFlag;                       ///< enable generation of VUI parameters
  bool      m_aspectRatioInfoPresentFlag;                     ///< Signals whether aspect_ratio_idc is present
  bool      m_chromaResamplingFilterHintEnabled;              ///< Signals whether chroma sampling filter hint data is present
  Int       m_chromaResamplingHorFilterIdc;                   ///< Specifies the Index of filter to use
  Int       m_chromaResamplingVerFilterIdc;                   ///< Specifies the Index of filter to use
  Int       m_aspectRatioIdc;                                 ///< aspect_ratio_idc
  Int       m_sarWidth;                                       ///< horizontal size of the sample aspect ratio
  Int       m_sarHeight;                                      ///< vertical size of the sample aspect ratio
  bool      m_overscanInfoPresentFlag;                        ///< Signals whether overscan_appropriate_flag is present
  bool      m_overscanAppropriateFlag;                        ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  bool      m_videoSignalTypePresentFlag;                     ///< Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present
  Int       m_videoFormat;                                    ///< Indicates representation of pictures
  bool      m_videoFullRangeFlag;                             ///< Indicates the black level and range of luma and chroma signals
  bool      m_colourDescriptionPresentFlag;                   ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  Int       m_colourPrimaries;                                ///< Indicates chromaticity coordinates of the source primaries
  Int       m_transferCharacteristics;                        ///< Indicates the opto-electronic transfer characteristics of the source
  Int       m_matrixCoefficients;                             ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool      m_chromaLocInfoPresentFlag;                       ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  Int       m_chromaSampleLocTypeTopField;                    ///< Specifies the location of chroma samples for top field
  Int       m_chromaSampleLocTypeBottomField;                 ///< Specifies the location of chroma samples for bottom field
  bool      m_neutralChromaIndicationFlag;                    ///< Indicates that the value of all decoded chroma samples is equal to 1<<(BitDepthCr-1)
  Window    m_defaultDisplayWindow;                           ///< Represents the default display window parameters
  bool      m_frameFieldInfoPresentFlag;                      ///< Indicates that pic_struct and other field coding related values are present in picture timing SEI messages
  bool      m_pocProportionalToTimingFlag;                    ///< Indicates that the POC value is proportional to the output time w.r.t. first picture in CVS
  Int       m_numTicksPocDiffOneMinus1;                       ///< Number of ticks minus 1 that for a POC difference of one
  bool      m_bitstreamRestrictionFlag;                       ///< Signals whether bitstream restriction parameters are present
#if HEVC_TILES_WPP
  bool      m_tilesFixedStructureFlag;                        ///< Indicates that each active picture parameter set has the same values of the syntax elements related to tiles
#endif
  bool      m_motionVectorsOverPicBoundariesFlag;             ///< Indicates that no samples outside the picture boundaries are used for inter prediction
  Int       m_minSpatialSegmentationIdc;                      ///< Indicates the maximum size of the spatial segments in the pictures in the coded video sequence
  Int       m_maxBytesPerPicDenom;                            ///< Indicates a number of bytes not exceeded by the sum of the sizes of the VCL NAL units associated with any coded picture
  Int       m_maxBitsPerMinCuDenom;                           ///< Indicates an upper bound for the number of bits of coding_unit() data
  Int       m_log2MaxMvLengthHorizontal;                      ///< Indicate the maximum absolute value of a decoded horizontal MV component in quarter-pel luma units
  Int       m_log2MaxMvLengthVertical;                        ///< Indicate the maximum absolute value of a decoded vertical MV component in quarter-pel luma units

#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  bool      m_useStrongIntraSmoothing;                        ///< enable the use of strong intra smoothing (bi_linear interpolation) for 32x32 blocks when reference samples are flat.
#endif
  bool      m_bEfficientFieldIRAPEnabled;                     ///< enable to code fields in a specific, potentially more efficient, order.
  bool      m_bHarmonizeGopFirstFieldCoupleEnabled;

  std::string m_summaryOutFilename;                           ///< filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase;                       ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  UInt        m_summaryVerboseness;                           ///< Specifies the level of the verboseness of the text output.
#if JVET_K0357_AMVR
  int       m_ImvMode;
  int       m_Imv4PelFast;
  int       m_ImvMaxCand;
#endif
  std::string m_decodeBitstreams[2];                          ///< filename for decode bitstreams.
  bool        m_forceDecodeBitstream1;                        ///< guess what it means
  int         m_switchPOC;                                    ///< dbg poc.
  int         m_switchDQP;                                    ///< dqp applied to  switchPOC and subsequent pictures.
  int         m_fastForwardToPOC;                             ///<
  bool        m_stopAfterFFtoPOC;                             ///<
  bool        m_bs2ModPOCAndType;



#if ENABLE_SPLIT_PARALLELISM
  int         m_numSplitThreads;
  bool        m_forceSingleSplitThread;
#endif
#if ENABLE_WPP_PARALLELISM
  int         m_numWppThreads;
  int         m_numWppExtraLines;
  bool        m_ensureWppBitEqual;
#endif

#if JVET_K0371_ALF
  bool        m_alf;                                          ///< Adaptive Loop Filter
#endif

public:
  EncCfg()
 #if HEVC_TILES_WPP
  : m_tileColumnWidth()
  , m_tileRowHeight()
#endif
  {
    m_PCMBitDepth[CHANNEL_TYPE_LUMA]=8;
    m_PCMBitDepth[CHANNEL_TYPE_CHROMA]=8;
  }

  virtual ~EncCfg()
  {}

  void setProfile(Profile::Name profile) { m_profile = profile; }
  void setLevel(Level::Tier tier, Level::Name level) { m_levelTier = tier; m_level = level; }

  void      setFrameRate                    ( Int   i )      { m_iFrameRate = i; }
  void      setFrameSkip                    ( UInt  i )      { m_FrameSkip = i; }
  void      setTemporalSubsampleRatio       ( UInt  i )      { m_temporalSubsampleRatio = i; }
  void      setSourceWidth                  ( Int   i )      { m_iSourceWidth = i; }
  void      setSourceHeight                 ( Int   i )      { m_iSourceHeight = i; }

  Window   &getConformanceWindow()                           { return m_conformanceWindow; }
  void      setConformanceWindow (Int confLeft, Int confRight, Int confTop, Int confBottom ) { m_conformanceWindow.setWindow (confLeft, confRight, confTop, confBottom); }

  void      setFramesToBeEncoded            ( Int   i )      { m_framesToBeEncoded = i; }

  bool      getPrintMSEBasedSequencePSNR    ()         const { return m_printMSEBasedSequencePSNR;  }
  void      setPrintMSEBasedSequencePSNR    (bool value)     { m_printMSEBasedSequencePSNR = value; }

  bool getPrintHexPsnr() const { return m_printHexPsnr; }
  void setPrintHexPsnr(bool value) { m_printHexPsnr = value; }

  bool      getPrintFrameMSE                ()         const { return m_printFrameMSE;              }
  void      setPrintFrameMSE                (bool value)     { m_printFrameMSE = value;             }

  bool      getPrintSequenceMSE             ()         const { return m_printSequenceMSE;           }
  void      setPrintSequenceMSE             (bool value)     { m_printSequenceMSE = value;          }

  bool      getCabacZeroWordPaddingEnabled()           const { return m_cabacZeroWordPaddingEnabled;  }
  void      setCabacZeroWordPaddingEnabled(bool value)       { m_cabacZeroWordPaddingEnabled = value; }

  //====== Coding Structure ========
  void      setIntraPeriod                  ( Int   i )      { m_uiIntraPeriod = (UInt)i; }
  void      setDecodingRefreshType          ( Int   i )      { m_uiDecodingRefreshType = (UInt)i; }
  void      setGOPSize                      ( Int   i )      { m_iGOPSize = i; }
  void      setGopList                      ( const GOPEntry GOPList[MAX_GOP] ) {  for ( Int i = 0; i < MAX_GOP; i++ ) m_GOPList[i] = GOPList[i]; }
  void      setExtraRPSs                    ( Int   i )      { m_extraRPSs = i; }
  const GOPEntry &getGOPEntry               ( Int   i ) const { return m_GOPList[i]; }
  void      setEncodedFlag                  ( Int  i, bool value )  { m_GOPList[i].m_isEncoded = value; }
  void      setMaxDecPicBuffering           ( UInt u, UInt tlayer ) { m_maxDecPicBuffering[tlayer] = u;    }
  void      setNumReorderPics               ( Int  i, UInt tlayer ) { m_numReorderPics[tlayer] = i;    }

  void      setBaseQP                       ( Int   i )      { m_iQP = i; }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  void      setIntraQPOffset                ( Int   i )         { m_intraQPOffset = i; }
  void      setLambdaFromQPEnable           ( bool  b )         { m_lambdaFromQPEnable = b; }
#endif
  void      setPad                          ( Int*  iPad                   )      { for ( Int i = 0; i < 2; i++ ) m_aiPad[i] = iPad[i]; }

  Int       getMaxRefPicNum                 ()                              { return m_iMaxRefPicNum;           }
  void      setMaxRefPicNum                 ( Int iMaxRefPicNum )           { m_iMaxRefPicNum = iMaxRefPicNum;  }

  Int       getMaxTempLayer                 ()                              { return m_maxTempLayer;              }
  void      setMaxTempLayer                 ( Int maxTempLayer )            { m_maxTempLayer = maxTempLayer;      }

  void      setQTBT                         ( bool b )           { m_QTBT = b; }
  void      setCTUSize                      ( unsigned  u )      { m_CTUSize  = u; }
  void      setMinQTSizes                   ( unsigned* minQT)   { m_uiMinQT[0] = minQT[0]; m_uiMinQT[1] = minQT[1]; m_uiMinQT[2] = minQT[2]; }
  void      setMaxBTDepth                   ( unsigned uiMaxBTDepth, unsigned uiMaxBTDepthI, unsigned uiMaxBTDepthIChroma )
                                                             { m_uiMaxBTDepth = uiMaxBTDepth; m_uiMaxBTDepthI = uiMaxBTDepthI; m_uiMaxBTDepthIChroma = uiMaxBTDepthIChroma; }
  unsigned  getMaxBTDepth                   ()         const { return m_uiMaxBTDepth; }
  unsigned  getMaxBTDepthI                  ()         const { return m_uiMaxBTDepthI; }
  unsigned  getMaxBTDepthIChroma            ()         const { return m_uiMaxBTDepthIChroma; }
  bool      getQTBT                         ()         const { return m_QTBT; }
  int       getCTUSize                      ()         const { return m_CTUSize; }
  void      setDualITree                    ( bool b )       { m_dualITree = b; }
  bool      getDualITree                    ()         const { return m_dualITree; }

#if JEM_TOOLS
  void      setNSST                         ( bool b )       { m_NSST = b; }
  bool      getNSST                         ()         const { return m_NSST; }

  void      setIntra4Tap                    ( bool b )       { m_Intra4Tap = b; }
  bool      getIntra4Tap                    ()         const { return m_Intra4Tap; }

#if !INTRA67_3MPM
  void      setIntra65Ang                   ( bool b )       { m_Intra65Ang = b; }
  bool      getIntra65Ang                   ()         const { return m_Intra65Ang; }
#endif

#endif
  void      setLargeCTU                     ( bool b )       { m_LargeCTU = b; }
  bool      getLargeCTU                     ()         const { return m_LargeCTU; }

#if JEM_TOOLS
  void      setUseIntraBoundaryFilter       ( bool b )       { m_IntraBoundaryFilter = b; }
  bool      getUseIntraBoundaryFilter       ()         const { return m_IntraBoundaryFilter; }
#endif
#if JEM_TOOLS||JVET_K0190
  void      setUseLMChroma                  ( int n )        { m_LMChroma = n; }
  int       getUseLMChroma()                           const { return m_LMChroma; }
#endif
#if JEM_TOOLS
  void      setSubPuMvpMode                 ( int n )       { m_SubPuMvpMode = n; }
  bool      getSubPuMvpMode                 ()         const { return m_SubPuMvpMode; }
  void      setSubPuMvpLog2Size             ( unsigned n )   { m_SubPuMvpLog2Size = n; }
  unsigned  getSubPuMvpLog2Size             ()         const { return m_SubPuMvpLog2Size; }
#endif

#if !JEM_TOOLS && JVET_K0346
  void      setSubPuMvpMode(int n)          { m_SubPuMvpMode = n; }
  bool      getSubPuMvpMode()         const { return m_SubPuMvpMode; }
  void      setSubPuMvpLog2Size(unsigned n) { m_SubPuMvpLog2Size = n; }
  unsigned  getSubPuMvpLog2Size()      const { return m_SubPuMvpLog2Size; }
#endif

#if JEM_TOOLS
  void      setCABACEngineMode              ( UInt mode )    { m_CABACEngineMode = mode; }
  UInt      getCABACEngineMode              ()               { return m_CABACEngineMode; }

#if JVET_K0072
#else
  void      setAltResiCompId                ( unsigned n )   { m_altResiCompId = n; }
  unsigned  getAltResiCompId                ()               { return m_altResiCompId; }
#endif
#endif
#if JEM_TOOLS
  void      setHighPrecisionMv              ( bool b )       { m_highPrecMv = b; }
  bool      getHighPrecisionMv              ()               { return m_highPrecMv; }

  void      setBIO                          ( bool b )       { m_BIO = b; }
  bool      getBIO                          ()         const { return m_BIO; }

  void      setAffine                       ( bool b )       { m_Affine = b; }
  bool      getAffine                       ()         const { return m_Affine; }
#if JVET_K0337_AFFINE_6PARA
  void      setAffineType( bool b )                          { m_AffineType = b; }
  bool      getAffineType()                            const { return m_AffineType; }
#endif
#endif
#if !JEM_TOOLS && JVET_K_AFFINE
  void      setAffine                       ( bool b )       { m_Affine = b; }
  bool      getAffine                       ()         const { return m_Affine; }
#if JVET_K0337_AFFINE_6PARA
  void      setAffineType( bool b )                          { m_AffineType = b; }
  bool      getAffineType()                            const { return m_AffineType; }
#endif
#endif
#if !JEM_TOOLS && (JVET_K0346 || JVET_K_AFFINE)
  void      setHighPrecisionMv(bool b) { m_highPrecMv = b; }
  bool      getHighPrecisionMv()       { return m_highPrecMv; }
#endif
  void      setDisableMotionCompression     ( bool b )       { m_DisableMotionCompression = b; }
  bool      getDisableMotionCompression     ()         const { return m_DisableMotionCompression; }

#if JEM_TOOLS
  void      setLICMode                      ( unsigned u )   { m_LICMode = u; }
  unsigned  getLICMode                      ()         const { return m_LICMode; }
  void      setFastPicLevelLIC              ( bool b )       { m_FastPicLevelLIC = b; }
  bool      getFastPicLevelLIC              ()         const { return m_FastPicLevelLIC; }
#endif

  void      setMTTMode                      ( unsigned u )   { m_MTTMode = u; }
  unsigned  getMTTMode                      ()         const { return m_MTTMode; }
#if ENABLE_WPP_PARALLELISM
  void      setUseAltDQPCoding              ( bool b )       { m_AltDQPCoding = b; }
  bool      getUseAltDQPCoding              ()         const { return m_AltDQPCoding; }
#endif
#if JEM_TOOLS
  void      setIntraPDPC                    ( int n )        { m_IntraPDPC = n; }
  int       getIntraPDPC()                             const { return m_IntraPDPC; }

#if !JVET_K0371_ALF
  void      setALF                          ( int i )        { m_ALF = i; }
  int       getALF                          ()         const { return m_ALF; }
#endif
#endif

#if JEM_TOOLS || JVET_K1000_SIMPLIFIED_EMT
  void      setFastIntraEMT                 ( bool b )       { m_FastIntraEMT = b; }
  bool      getFastIntraEMT                 ()         const { return m_FastIntraEMT; }
  void      setFastInterEMT                 ( bool b )       { m_FastInterEMT = b; }
  bool      getFastInterEMT                 ()         const { return m_FastInterEMT; }
  void      setIntraEMT                     ( bool b )       { m_IntraEMT = b; }
  bool      getIntraEMT                     ()         const { return m_IntraEMT; }
  void      setInterEMT                     ( bool b )       { m_InterEMT = b; }
  bool      getInterEMT                     ()         const { return m_InterEMT; }
#endif

#if JEM_TOOLS
  void      setUseOBMC                      ( bool n )       { m_OBMC = n; }
  bool      getUseOBMC                      ()         const { return m_OBMC; }
  void      setOBMCBlkSize                  ( unsigned n )   { m_uiObmcBlkSize = n; }
  unsigned  getOBMCBlkSize                  ()         const { return m_uiObmcBlkSize; }

  void      setUseFRUCMrgMode               ( bool b )       { m_FRUC = b; }
  bool      getUseFRUCMrgMode               ()         const { return m_FRUC; }
  void      setFRUCRefineFilter             ( unsigned n )   { m_FRUCRefineFilter = n; }
  unsigned  getFRUCRefineFilter             ()         const { return m_FRUCRefineFilter; }
  void      setFRUCRefineRange              ( unsigned n )   { m_FRUCRefineRange = n; }
  unsigned  getFRUCRefineRange              ()         const { return m_FRUCRefineRange; }
  void      setFRUCSmallBlkRefineDepth      ( unsigned n )   { m_FRUCSmallBlkRefineDepth = n; }
  unsigned  getFRUCSmallBlkRefineDepth      ()         const { return m_FRUCSmallBlkRefineDepth; }
#endif

#if JEM_TOOLS
  void      setCIPF                         ( unsigned n )   { m_CIPF = n; }
  unsigned  getCIPF                         ()         const { return m_CIPF; }
#endif

#if JEM_TOOLS
  void      setUseBIF                       ( bool b )       { m_BIF = b; }
  bool      getUseBIF                       ()         const { return m_BIF; }
  void      setUseAClip                     ( bool b )       { m_AClip = b; }
  bool      getUseAClip                     ()         const { return m_AClip; }
  void      setUseAClipEnc                  ( bool b )       { m_AClipEnc = b; }
  bool      getUseAClipEnc                  ()         const { return m_AClipEnc; }
#endif

#if JEM_TOOLS
  void      setUseDMVR                      ( bool b )       { m_DMVR = b; }
  bool      getUseDMVR                      ()         const { return m_DMVR; }
#endif
#if JEM_TOOLS
  void      setMDMS                         ( bool b )       { m_MDMS = b; }
  bool      getMDMS                         ()         const { return m_MDMS; }
#endif
  // ADD_NEW_TOOL : (encoder lib) add access functions here

  void      setMaxCUWidth                   ( UInt  u )      { m_maxCUWidth  = u; }
  UInt      getMaxCUWidth                   () const         { return m_maxCUWidth; }
  void      setMaxCUHeight                  ( UInt  u )      { m_maxCUHeight = u; }
  UInt      getMaxCUHeight                  () const         { return m_maxCUHeight; }
  void      setMaxCodingDepth               ( UInt  u )      { m_maxTotalCUDepth = u; }
  UInt      getMaxCodingDepth               () const         { return m_maxTotalCUDepth; }
  void      setLog2DiffMaxMinCodingBlockSize( UInt  u )      { m_log2DiffMaxMinCodingBlockSize = u; }

  void      setUseFastLCTU                  ( bool  n )      { m_useFastLCTU = n; }
  bool      getUseFastLCTU                  () const         { return m_useFastLCTU; }
  void      setUseFastMerge                 ( bool  n )      { m_useFastMrg = n; }
  bool      getUseFastMerge                 () const         { return m_useFastMrg; }
  void      setUsePbIntraFast               ( bool  n )      { m_usePbIntraFast = n; }
  bool      getUsePbIntraFast               () const         { return m_usePbIntraFast; }
  void      setUseAMaxBT                    ( bool  n )      { m_useAMaxBT = n; }
  bool      getUseAMaxBT                    () const         { return m_useAMaxBT; }

#if !JVET_K0220_ENC_CTRL
  void      setUseSaveLoadEncInfo           ( bool  b )      { m_useSaveLoadEncInfo = b; }
  void      setUseSaveLoadSplitDecision     ( bool  b )      { m_useSaveLoadSplitDecision = b; }
  bool      getUseSaveLoadEncInfo           () const         { return m_useSaveLoadEncInfo; }
  bool      getUseSaveLoadSplitDecision     () const         { return m_useSaveLoadSplitDecision; }

#endif
  void      setUseE0023FastEnc              ( bool b )       { m_e0023FastEnc = b; }
  bool      getUseE0023FastEnc              () const         { return m_e0023FastEnc; }
  void      setUseContentBasedFastQtbt      ( bool b )       { m_contentBasedFastQtbt = b; }
  bool      getUseContentBasedFastQtbt      () const         { return m_contentBasedFastQtbt; }

  //======== Transform =============
  void      setQuadtreeTULog2MaxSize        ( UInt  u )      { m_uiQuadtreeTULog2MaxSize = u; }
  void      setQuadtreeTULog2MinSize        ( UInt  u )      { m_uiQuadtreeTULog2MinSize = u; }
  void      setQuadtreeTUMaxDepthInter      ( UInt  u )      { m_uiQuadtreeTUMaxDepthInter = u; }
  void      setQuadtreeTUMaxDepthIntra      ( UInt  u )      { m_uiQuadtreeTUMaxDepthIntra = u; }

  void setUseAMP( bool b ) { m_useAMP = b; }

  //====== Loop/Deblock Filter ========
  void      setLoopFilterDisable            ( bool  b )      { m_bLoopFilterDisable       = b; }
  void      setLoopFilterOffsetInPPS        ( bool  b )      { m_loopFilterOffsetInPPS      = b; }
  void      setLoopFilterBetaOffset         ( Int   i )      { m_loopFilterBetaOffsetDiv2  = i; }
  void      setLoopFilterTcOffset           ( Int   i )      { m_loopFilterTcOffsetDiv2    = i; }
#if W0038_DB_OPT
  void      setDeblockingFilterMetric       ( Int   i )      { m_deblockingFilterMetric = i; }
#else
  void      setDeblockingFilterMetric       ( bool  b )      { m_DeblockingFilterMetric = b; }
#endif
  //====== Motion search ========
  void      setDisableIntraPUsInInterSlices ( bool  b )      { m_bDisableIntraPUsInInterSlices = b; }
  void      setMotionEstimationSearchMethod ( MESearchMethod e ) { m_motionEstimationSearchMethod = e; }
  void      setSearchRange                  ( Int   i )      { m_iSearchRange = i; }
  void      setBipredSearchRange            ( Int   i )      { m_bipredSearchRange = i; }
  void      setClipForBiPredMeEnabled       ( bool  b )      { m_bClipForBiPredMeEnabled = b; }
  void      setFastMEAssumingSmootherMVEnabled ( bool b )    { m_bFastMEAssumingSmootherMVEnabled = b; }
  void      setMinSearchWindow              ( Int   i )      { m_minSearchWindow = i; }
  void      setRestrictMESampling           ( bool  b )      { m_bRestrictMESampling = b; }

  //====== Quality control ========
  void      setMaxDeltaQP                   ( Int   i )      { m_iMaxDeltaQP = i; }
  void      setMaxCuDQPDepth                ( Int   i )      { m_iMaxCuDQPDepth = i; }

  Int       getDiffCuChromaQpOffsetDepth    ()         const { return m_diffCuChromaQpOffsetDepth;  }
  void      setDiffCuChromaQpOffsetDepth    (Int value)      { m_diffCuChromaQpOffsetDepth = value; }

  void      setChromaCbQpOffset             ( Int   i )      { m_chromaCbQpOffset = i; }
  void      setChromaCrQpOffset             ( Int   i )      { m_chromaCrQpOffset = i; }
  void      setChromaCbQpOffsetDualTree     ( int   i )      { m_chromaCbQpOffsetDualTree = i; }
  void      setChromaCrQpOffsetDualTree     ( int   i )      { m_chromaCrQpOffsetDualTree = i; }
  int       getChromaCbQpOffsetDualTree     ()         const { return m_chromaCbQpOffsetDualTree; }
  int       getChromaCrQpOffsetDualTree     ()         const { return m_chromaCrQpOffsetDualTree; }
#if ER_CHROMA_QP_WCG_PPS
  void      setWCGChromaQpControl           ( const WCGChromaQPControl &ctrl )     { m_wcgChromaQpControl = ctrl; }
  const WCGChromaQPControl &getWCGChromaQPControl () const { return m_wcgChromaQpControl; }
#endif
#if W0038_CQP_ADJ
  void      setSliceChromaOffsetQpIntraOrPeriodic( UInt periodicity, Int sliceChromaQpOffsetIntraOrPeriodic[2]) { m_sliceChromaQpOffsetPeriodicity = periodicity; memcpy(m_sliceChromaQpOffsetIntraOrPeriodic, sliceChromaQpOffsetIntraOrPeriodic, sizeof(m_sliceChromaQpOffsetIntraOrPeriodic)); }
  Int       getSliceChromaOffsetQpIntraOrPeriodic( bool bIsCr) const                                            { return m_sliceChromaQpOffsetIntraOrPeriodic[bIsCr?1:0]; }
  UInt      getSliceChromaOffsetQpPeriodicity() const                                                           { return m_sliceChromaQpOffsetPeriodicity; }
#endif

  void      setChromaFormatIdc              ( ChromaFormat cf ) { m_chromaFormatIDC = cf; }
#if REUSE_CU_RESULTS
  ChromaFormat  getChromaFormatIdc          ( ) const        { return m_chromaFormatIDC; }
#else
  ChromaFormat  getChromaFormatIdc          ( )              { return m_chromaFormatIDC; }
#endif

#if SHARP_LUMA_DELTA_QP
  void      setLumaLevelToDeltaQPControls( const LumaLevelToDeltaQPMapping &lumaLevelToDeltaQPMapping ) { m_lumaLevelToDeltaQPMapping=lumaLevelToDeltaQPMapping; }
  const LumaLevelToDeltaQPMapping& getLumaLevelToDeltaQPMapping() const { return m_lumaLevelToDeltaQPMapping; }
#endif

  bool      getExtendedPrecisionProcessingFlag         ()         const { return m_extendedPrecisionProcessingFlag;  }
  void      setExtendedPrecisionProcessingFlag         (bool value)     { m_extendedPrecisionProcessingFlag = value; }

  bool      getHighPrecisionOffsetsEnabledFlag() const { return m_highPrecisionOffsetsEnabledFlag; }
  void      setHighPrecisionOffsetsEnabledFlag(bool value) { m_highPrecisionOffsetsEnabledFlag = value; }

  void      setUseAdaptiveQP                ( bool  b )      { m_bUseAdaptiveQP = b; }
  void      setQPAdaptationRange            ( Int   i )      { m_iQPAdaptationRange = i; }
#if ENABLE_QPA
  void      setUsePerceptQPA                ( const bool b ) { m_bUsePerceptQPA = b; }
  void      setUseWPSNR                     ( const bool b ) { m_bUseWPSNR = b; }
#endif

  //====== Sequence ========
  Int       getFrameRate                    () const     { return  m_iFrameRate; }
  UInt      getFrameSkip                    () const     { return  m_FrameSkip; }
  UInt      getTemporalSubsampleRatio       () const     { return  m_temporalSubsampleRatio; }
  Int       getSourceWidth                  () const     { return  m_iSourceWidth; }
  Int       getSourceHeight                 () const     { return  m_iSourceHeight; }
  Int       getFramesToBeEncoded            () const     { return  m_framesToBeEncoded; }

  //====== Lambda Modifiers ========
  void      setLambdaModifier               ( UInt uiIndex, double dValue ) { m_adLambdaModifier[ uiIndex ] = dValue; }
  double    getLambdaModifier               ( UInt uiIndex )          const { return m_adLambdaModifier[ uiIndex ]; }
  void      setIntraLambdaModifier          ( const std::vector<double> &dValue )               { m_adIntraLambdaModifier = dValue;       }
  const std::vector<double>& getIntraLambdaModifier()                        const { return m_adIntraLambdaModifier;         }
  void      setIntraQpFactor                ( double dValue )               { m_dIntraQpFactor = dValue;              }
  double    getIntraQpFactor                ()                        const { return m_dIntraQpFactor;                }

  //==== Coding Structure ========
  UInt      getIntraPeriod                  () const     { return  m_uiIntraPeriod; }
  UInt      getDecodingRefreshType          () const     { return  m_uiDecodingRefreshType; }
  Int       getGOPSize                      () const     { return  m_iGOPSize; }
  Int       getMaxDecPicBuffering           (UInt tlayer) { return m_maxDecPicBuffering[tlayer]; }
  Int       getNumReorderPics               (UInt tlayer) { return m_numReorderPics[tlayer]; }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  Int       getIntraQPOffset                () const    { return  m_intraQPOffset; }
  Int       getLambdaFromQPEnable           () const    { return  m_lambdaFromQPEnable; }
#if ENABLE_QPA
public:
#else
protected:
#endif
  Int       getBaseQP                       () const { return  m_iQP; } // public should use getQPForPicture.
public:
  Int       getQPForPicture                 (const UInt gopIndex, const Slice *pSlice) const; // Function actually defined in EncLib.cpp
#else
  Int       getBaseQP                       ()       { return  m_iQP; }
#endif
  Int       getPad                          ( Int i )      { CHECK(i >= 2, "Invalid index");                      return  m_aiPad[i]; }

  bool      getAccessUnitDelimiter() const  { return m_AccessUnitDelimiter; }
  void      setAccessUnitDelimiter(bool val){ m_AccessUnitDelimiter = val; }

  //======== Transform =============
  UInt      getQuadtreeTULog2MaxSize        ()      const { return m_uiQuadtreeTULog2MaxSize; }
  UInt      getQuadtreeTULog2MinSize        ()      const { return m_uiQuadtreeTULog2MinSize; }
  UInt      getQuadtreeTUMaxDepthInter      ()      const { return m_uiQuadtreeTUMaxDepthInter; }
  UInt      getQuadtreeTUMaxDepthIntra      ()      const { return m_uiQuadtreeTUMaxDepthIntra; }

  //==== Loop/Deblock Filter ========
  bool      getLoopFilterDisable            ()      { return  m_bLoopFilterDisable;       }
  bool      getLoopFilterOffsetInPPS        ()      { return m_loopFilterOffsetInPPS; }
  Int       getLoopFilterBetaOffset         ()      { return m_loopFilterBetaOffsetDiv2; }
  Int       getLoopFilterTcOffset           ()      { return m_loopFilterTcOffsetDiv2; }
#if W0038_DB_OPT
  Int       getDeblockingFilterMetric       ()      { return m_deblockingFilterMetric; }
#else
  bool      getDeblockingFilterMetric       ()      { return m_DeblockingFilterMetric; }
#endif

  //==== Motion search ========
  bool      getDisableIntraPUsInInterSlices    () const { return m_bDisableIntraPUsInInterSlices; }
  MESearchMethod getMotionEstimationSearchMethod ( ) const { return m_motionEstimationSearchMethod; }
  Int       getSearchRange                     () const { return m_iSearchRange; }
  bool      getClipForBiPredMeEnabled          () const { return m_bClipForBiPredMeEnabled; }
  bool      getFastMEAssumingSmootherMVEnabled () const { return m_bFastMEAssumingSmootherMVEnabled; }
  Int       getMinSearchWindow                 () const { return m_minSearchWindow; }
  bool      getRestrictMESampling              () const { return m_bRestrictMESampling; }

  //==== Quality control ========
  Int       getMaxDeltaQP                   () const { return m_iMaxDeltaQP; }
  Int       getMaxCuDQPDepth                () const { return m_iMaxCuDQPDepth; }
  bool      getUseAdaptiveQP                () const { return m_bUseAdaptiveQP; }
  Int       getQPAdaptationRange            () const { return m_iQPAdaptationRange; }
#if ENABLE_QPA
  bool      getUsePerceptQPA                () const { return m_bUsePerceptQPA; }
  bool      getUseWPSNR                     () const { return m_bUseWPSNR; }
#endif

  //==== Tool list ========
  void      setBitDepth( const ChannelType chType, Int internalBitDepthForChannel ) { m_bitDepth[chType] = internalBitDepthForChannel; }
  void      setInputBitDepth( const ChannelType chType, Int internalBitDepthForChannel ) { m_inputBitDepth[chType] = internalBitDepthForChannel; }
  void      setUseASR                       ( bool  b )     { m_bUseASR     = b; }
  void      setUseHADME                     ( bool  b )     { m_bUseHADME   = b; }
  void      setUseRDOQ                      ( bool  b )     { m_useRDOQ    = b; }
  void      setUseRDOQTS                    ( bool  b )     { m_useRDOQTS  = b; }
#if T0196_SELECTIVE_RDOQ
  void      setUseSelectiveRDOQ             ( bool b )      { m_useSelectiveRDOQ = b; }
#endif
  void      setRDpenalty                    ( UInt  u )     { m_rdPenalty  = u; }
  void      setFastInterSearchMode          ( FastInterSearchMode m ) { m_fastInterSearchMode = m; }
  void      setUseEarlyCU                   ( bool  b )     { m_bUseEarlyCU = b; }
  void      setUseFastDecisionForMerge      ( bool  b )     { m_useFastDecisionForMerge = b; }
  void      setUseCbfFastMode               ( bool  b )     { m_bUseCbfFastMode = b; }
  void      setUseEarlySkipDetection        ( bool  b )     { m_useEarlySkipDetection = b; }
  void      setUseConstrainedIntraPred      ( bool  b )     { m_bUseConstrainedIntraPred = b; }
  void      setFastUDIUseMPMEnabled         ( bool  b )     { m_bFastUDIUseMPMEnabled = b; }
  void      setFastMEForGenBLowDelayEnabled ( bool  b )     { m_bFastMEForGenBLowDelayEnabled = b; }
  void      setUseBLambdaForNonKeyLowDelayPictures ( bool b ) { m_bUseBLambdaForNonKeyLowDelayPictures = b; }

  void      setPCMInputBitDepthFlag         ( bool  b )     { m_bPCMInputBitDepthFlag = b; }
  void      setPCMFilterDisableFlag         ( bool  b )     {  m_bPCMFilterDisableFlag = b; }
  void      setUsePCM                       ( bool  b )     {  m_usePCM = b;               }
  void      setPCMBitDepth( const ChannelType chType, Int pcmBitDepthForChannel ) { m_PCMBitDepth[chType] = pcmBitDepthForChannel; }
  void      setPCMLog2MaxSize               ( UInt u )      { m_pcmLog2MaxSize = u;      }
  void      setPCMLog2MinSize               ( UInt u )     { m_uiPCMLog2MinSize = u;      }
  void      setdQPs                         ( Int*  p )     { m_aidQP       = p; }
  void      setDeltaQpRD                    ( UInt  u )     {m_uiDeltaQpRD  = u; }
  void      setFastDeltaQp                  ( bool  b )     {m_bFastDeltaQP = b; }
  Int       getBitDepth                     (const ChannelType chType) const { return m_bitDepth[chType]; }
  bool      getUseASR                       ()      { return m_bUseASR;     }
  bool      getUseHADME                     ()      { return m_bUseHADME;   }
  bool      getUseRDOQ                      ()      { return m_useRDOQ;    }
  bool      getUseRDOQTS                    ()      { return m_useRDOQTS;  }
#if T0196_SELECTIVE_RDOQ
  bool      getUseSelectiveRDOQ             ()      { return m_useSelectiveRDOQ; }
#endif
  Int       getRDpenalty                    ()      { return m_rdPenalty;  }
  FastInterSearchMode getFastInterSearchMode() const{ return m_fastInterSearchMode;  }
  bool      getUseEarlyCU                   () const{ return m_bUseEarlyCU; }
  bool      getUseFastDecisionForMerge      () const{ return m_useFastDecisionForMerge; }
  bool      getUseCbfFastMode               () const{ return m_bUseCbfFastMode; }
  bool      getUseEarlySkipDetection        () const{ return m_useEarlySkipDetection; }
  bool      getUseConstrainedIntraPred      ()      { return m_bUseConstrainedIntraPred; }
  bool      getFastUDIUseMPMEnabled         ()      { return m_bFastUDIUseMPMEnabled; }
  bool      getFastMEForGenBLowDelayEnabled ()      { return m_bFastMEForGenBLowDelayEnabled; }
  bool      getUseBLambdaForNonKeyLowDelayPictures () { return m_bUseBLambdaForNonKeyLowDelayPictures; }
  bool      getPCMInputBitDepthFlag         ()      { return m_bPCMInputBitDepthFlag;   }
  bool      getPCMFilterDisableFlag         ()      { return m_bPCMFilterDisableFlag;   }
  bool      getUsePCM                       ()      { return m_usePCM;                 }
  UInt      getPCMLog2MaxSize               ()      { return m_pcmLog2MaxSize;  }
  UInt      getPCMLog2MinSize               ()      { return  m_uiPCMLog2MinSize;  }

  bool      getCrossComponentPredictionEnabledFlag     ()                const { return m_crossComponentPredictionEnabledFlag;   }
  void      setCrossComponentPredictionEnabledFlag     (const bool value)      { m_crossComponentPredictionEnabledFlag = value;  }
  bool      getUseReconBasedCrossCPredictionEstimate ()                const { return m_reconBasedCrossCPredictionEstimate;  }
  void      setUseReconBasedCrossCPredictionEstimate (const bool value)      { m_reconBasedCrossCPredictionEstimate = value; }
  void      setLog2SaoOffsetScale(ChannelType type, UInt uiBitShift)         { m_log2SaoOffsetScale[type] = uiBitShift; }

  bool getUseTransformSkip                             ()      { return m_useTransformSkip;        }
  void setUseTransformSkip                             ( bool b ) { m_useTransformSkip  = b;       }
  bool getTransformSkipRotationEnabledFlag             ()            const { return m_transformSkipRotationEnabledFlag;  }
  void setTransformSkipRotationEnabledFlag             (const bool value)  { m_transformSkipRotationEnabledFlag = value; }
  bool getTransformSkipContextEnabledFlag              ()            const { return m_transformSkipContextEnabledFlag;  }
  void setTransformSkipContextEnabledFlag              (const bool value)  { m_transformSkipContextEnabledFlag = value; }
  bool getPersistentRiceAdaptationEnabledFlag          ()                 const { return m_persistentRiceAdaptationEnabledFlag;  }
  void setPersistentRiceAdaptationEnabledFlag          (const bool value)       { m_persistentRiceAdaptationEnabledFlag = value; }
  bool getCabacBypassAlignmentEnabledFlag              ()       const      { return m_cabacBypassAlignmentEnabledFlag;  }
  void setCabacBypassAlignmentEnabledFlag              (const bool value)  { m_cabacBypassAlignmentEnabledFlag = value; }
  bool getRdpcmEnabledFlag                             (const RDPCMSignallingMode signallingMode)        const      { return m_rdpcmEnabledFlag[signallingMode];  }
  void setRdpcmEnabledFlag                             (const RDPCMSignallingMode signallingMode, const bool value) { m_rdpcmEnabledFlag[signallingMode] = value; }
  bool getUseTransformSkipFast                         ()      { return m_useTransformSkipFast;    }
  void setUseTransformSkipFast                         ( bool b ) { m_useTransformSkipFast  = b;   }
  UInt getLog2MaxTransformSkipBlockSize                () const      { return m_log2MaxTransformSkipBlockSize;     }
  void setLog2MaxTransformSkipBlockSize                ( UInt u )    { m_log2MaxTransformSkipBlockSize  = u;       }
  bool getIntraSmoothingDisabledFlag               ()      const { return m_intraSmoothingDisabledFlag; }
  void setIntraSmoothingDisabledFlag               (bool bValue) { m_intraSmoothingDisabledFlag=bValue; }

  const Int* getdQPs                        () const { return m_aidQP;       }
  UInt      getDeltaQpRD                    () const { return m_uiDeltaQpRD; }
  bool      getFastDeltaQp                  () const { return m_bFastDeltaQP; }

  //====== Slice ========
  void  setSliceMode                   ( SliceConstraint  i )        { m_sliceMode = i;              }
  void  setSliceArgument               ( Int  i )                    { m_sliceArgument = i;          }
  SliceConstraint getSliceMode         () const                      { return m_sliceMode;           }
  Int   getSliceArgument               ()                            { return m_sliceArgument;       }
  //====== Dependent Slice ========
  void  setSliceSegmentMode            ( SliceConstraint  i )        { m_sliceSegmentMode = i;       }
  void  setSliceSegmentArgument        ( Int  i )                    { m_sliceSegmentArgument = i;   }
  SliceConstraint getSliceSegmentMode  () const                      { return m_sliceSegmentMode;    }
  Int   getSliceSegmentArgument        ()                            { return m_sliceSegmentArgument;}
  void      setLFCrossSliceBoundaryFlag     ( bool   bValue  )       { m_bLFCrossSliceBoundaryFlag = bValue; }
  bool      getLFCrossSliceBoundaryFlag     ()                       { return m_bLFCrossSliceBoundaryFlag;   }

  void      setUseSAO                  (bool bVal)                   { m_bUseSAO = bVal; }
  bool      getUseSAO                  ()                            { return m_bUseSAO; }
  void  setTestSAODisableAtPictureLevel (bool bVal)                  { m_bTestSAODisableAtPictureLevel = bVal; }
  bool  getTestSAODisableAtPictureLevel ( ) const                    { return m_bTestSAODisableAtPictureLevel; }

  void   setSaoEncodingRate(double v)                                { m_saoEncodingRate = v; }
  double getSaoEncodingRate() const                                  { return m_saoEncodingRate; }
  void   setSaoEncodingRateChroma(double v)                          { m_saoEncodingRateChroma = v; }
  double getSaoEncodingRateChroma() const                            { return m_saoEncodingRateChroma; }
  void  setMaxNumOffsetsPerPic                   (Int iVal)          { m_maxNumOffsetsPerPic = iVal; }
  Int   getMaxNumOffsetsPerPic                   ()                  { return m_maxNumOffsetsPerPic; }
  void  setSaoCtuBoundary              (bool val)                    { m_saoCtuBoundary = val; }
  bool  getSaoCtuBoundary              ()                            { return m_saoCtuBoundary; }

#if K0238_SAO_GREEDY_MERGE_ENCODING
  void  setSaoGreedyMergeEnc           (bool val)                    { m_saoGreedyMergeEnc = val; }
  bool  getSaoGreedyMergeEnc           ()                            { return m_saoGreedyMergeEnc; }
#endif
#if HEVC_TILES_WPP
  void  setLFCrossTileBoundaryFlag               ( bool   val  )     { m_loopFilterAcrossTilesEnabledFlag = val; }
  bool  getLFCrossTileBoundaryFlag               ()                  { return m_loopFilterAcrossTilesEnabledFlag;   }
  void  setTileUniformSpacingFlag      ( bool b )                    { m_tileUniformSpacingFlag = b; }
  bool  getTileUniformSpacingFlag      ()                            { return m_tileUniformSpacingFlag; }
  void  setNumColumnsMinus1            ( Int i )                     { m_iNumColumnsMinus1 = i; }
  Int   getNumColumnsMinus1            ()                            { return m_iNumColumnsMinus1; }
  void  setColumnWidth ( const std::vector<Int>& columnWidth )       { m_tileColumnWidth = columnWidth; }
  UInt  getColumnWidth                 ( UInt columnIdx )            { return m_tileColumnWidth[columnIdx]; }
  void  setNumRowsMinus1               ( Int i )                     { m_iNumRowsMinus1 = i; }
  Int   getNumRowsMinus1               ()                            { return m_iNumRowsMinus1; }
  void  setRowHeight ( const std::vector<Int>& rowHeight)            { m_tileRowHeight = rowHeight; }
  UInt  getRowHeight                   ( UInt rowIdx )               { return m_tileRowHeight[rowIdx]; }
#endif
  void  xCheckGSParameters();
#if HEVC_TILES_WPP
  void  setEntropyCodingSyncEnabledFlag(bool b)                      { m_entropyCodingSyncEnabledFlag = b; }
  bool  getEntropyCodingSyncEnabledFlag() const                      { return m_entropyCodingSyncEnabledFlag; }
#endif
  void  setDecodedPictureHashSEIType(HashType m)                     { m_decodedPictureHashSEIType = m; }
  HashType getDecodedPictureHashSEIType() const                      { return m_decodedPictureHashSEIType; }
  void  setBufferingPeriodSEIEnabled(bool b)                         { m_bufferingPeriodSEIEnabled = b; }
  bool  getBufferingPeriodSEIEnabled() const                         { return m_bufferingPeriodSEIEnabled; }
  void  setPictureTimingSEIEnabled(bool b)                           { m_pictureTimingSEIEnabled = b; }
  bool  getPictureTimingSEIEnabled() const                           { return m_pictureTimingSEIEnabled; }
  void  setRecoveryPointSEIEnabled(bool b)                           { m_recoveryPointSEIEnabled = b; }
  bool  getRecoveryPointSEIEnabled() const                           { return m_recoveryPointSEIEnabled; }
  void  setToneMappingInfoSEIEnabled(bool b)                         { m_toneMappingInfoSEIEnabled = b;  }
  bool  getToneMappingInfoSEIEnabled()                               { return m_toneMappingInfoSEIEnabled;  }
  void  setTMISEIToneMapId(Int b)                                    { m_toneMapId = b;  }
  Int   getTMISEIToneMapId()                                         { return m_toneMapId;  }
  void  setTMISEIToneMapCancelFlag(bool b)                           { m_toneMapCancelFlag=b;  }
  bool  getTMISEIToneMapCancelFlag()                                 { return m_toneMapCancelFlag;  }
  void  setTMISEIToneMapPersistenceFlag(bool b)                      { m_toneMapPersistenceFlag = b;  }
  bool   getTMISEIToneMapPersistenceFlag()                           { return m_toneMapPersistenceFlag;  }
  void  setTMISEICodedDataBitDepth(Int b)                            { m_codedDataBitDepth = b;  }
  Int   getTMISEICodedDataBitDepth()                                 { return m_codedDataBitDepth;  }
  void  setTMISEITargetBitDepth(Int b)                               { m_targetBitDepth = b;  }
  Int   getTMISEITargetBitDepth()                                    { return m_targetBitDepth;  }
  void  setTMISEIModelID(Int b)                                      { m_modelId = b;  }
  Int   getTMISEIModelID()                                           { return m_modelId;  }
  void  setTMISEIMinValue(Int b)                                     { m_minValue = b;  }
  Int   getTMISEIMinValue()                                          { return m_minValue;  }
  void  setTMISEIMaxValue(Int b)                                     { m_maxValue = b;  }
  Int   getTMISEIMaxValue()                                          { return m_maxValue;  }
  void  setTMISEISigmoidMidpoint(Int b)                              { m_sigmoidMidpoint = b;  }
  Int   getTMISEISigmoidMidpoint()                                   { return m_sigmoidMidpoint;  }
  void  setTMISEISigmoidWidth(Int b)                                 { m_sigmoidWidth = b;  }
  Int   getTMISEISigmoidWidth()                                      { return m_sigmoidWidth;  }
  void  setTMISEIStartOfCodedInterva( Int*  p )                      { m_startOfCodedInterval = p;  }
  Int*  getTMISEIStartOfCodedInterva()                               { return m_startOfCodedInterval;  }
  void  setTMISEINumPivots(Int b)                                    { m_numPivots = b;  }
  Int   getTMISEINumPivots()                                         { return m_numPivots;  }
  void  setTMISEICodedPivotValue( Int*  p )                          { m_codedPivotValue = p;  }
  Int*  getTMISEICodedPivotValue()                                   { return m_codedPivotValue;  }
  void  setTMISEITargetPivotValue( Int*  p )                         { m_targetPivotValue = p;  }
  Int*  getTMISEITargetPivotValue()                                  { return m_targetPivotValue;  }
  void  setTMISEICameraIsoSpeedIdc(Int b)                            { m_cameraIsoSpeedIdc = b;  }
  Int   getTMISEICameraIsoSpeedIdc()                                 { return m_cameraIsoSpeedIdc;  }
  void  setTMISEICameraIsoSpeedValue(Int b)                          { m_cameraIsoSpeedValue = b;  }
  Int   getTMISEICameraIsoSpeedValue()                               { return m_cameraIsoSpeedValue;  }
  void  setTMISEIExposureIndexIdc(Int b)                             { m_exposureIndexIdc = b;  }
  Int   getTMISEIExposurIndexIdc()                                   { return m_exposureIndexIdc;  }
  void  setTMISEIExposureIndexValue(Int b)                           { m_exposureIndexValue = b;  }
  Int   getTMISEIExposurIndexValue()                                 { return m_exposureIndexValue;  }
  void  setTMISEIExposureCompensationValueSignFlag(bool b)           { m_exposureCompensationValueSignFlag = b;  }
  bool  getTMISEIExposureCompensationValueSignFlag()                 { return m_exposureCompensationValueSignFlag;  }
  void  setTMISEIExposureCompensationValueNumerator(Int b)           { m_exposureCompensationValueNumerator = b;  }
  Int   getTMISEIExposureCompensationValueNumerator()                { return m_exposureCompensationValueNumerator;  }
  void  setTMISEIExposureCompensationValueDenomIdc(Int b)            { m_exposureCompensationValueDenomIdc =b;  }
  Int   getTMISEIExposureCompensationValueDenomIdc()                 { return m_exposureCompensationValueDenomIdc;  }
  void  setTMISEIRefScreenLuminanceWhite(Int b)                      { m_refScreenLuminanceWhite = b;  }
  Int   getTMISEIRefScreenLuminanceWhite()                           { return m_refScreenLuminanceWhite;  }
  void  setTMISEIExtendedRangeWhiteLevel(Int b)                      { m_extendedRangeWhiteLevel = b;  }
  Int   getTMISEIExtendedRangeWhiteLevel()                           { return m_extendedRangeWhiteLevel;  }
  void  setTMISEINominalBlackLevelLumaCodeValue(Int b)               { m_nominalBlackLevelLumaCodeValue = b;  }
  Int   getTMISEINominalBlackLevelLumaCodeValue()                    { return m_nominalBlackLevelLumaCodeValue;  }
  void  setTMISEINominalWhiteLevelLumaCodeValue(Int b)               { m_nominalWhiteLevelLumaCodeValue = b;  }
  Int   getTMISEINominalWhiteLevelLumaCodeValue()                    { return m_nominalWhiteLevelLumaCodeValue;  }
  void  setTMISEIExtendedWhiteLevelLumaCodeValue(Int b)              { m_extendedWhiteLevelLumaCodeValue =b;  }
  Int   getTMISEIExtendedWhiteLevelLumaCodeValue()                   { return m_extendedWhiteLevelLumaCodeValue;  }
  void  setFramePackingArrangementSEIEnabled(bool b)                 { m_framePackingSEIEnabled = b; }
  bool  getFramePackingArrangementSEIEnabled() const                 { return m_framePackingSEIEnabled; }
  void  setFramePackingArrangementSEIType(Int b)                     { m_framePackingSEIType = b; }
  Int   getFramePackingArrangementSEIType()                          { return m_framePackingSEIType; }
  void  setFramePackingArrangementSEIId(Int b)                       { m_framePackingSEIId = b; }
  Int   getFramePackingArrangementSEIId()                            { return m_framePackingSEIId; }
  void  setFramePackingArrangementSEIQuincunx(Int b)                 { m_framePackingSEIQuincunx = b; }
  Int   getFramePackingArrangementSEIQuincunx()                      { return m_framePackingSEIQuincunx; }
  void  setFramePackingArrangementSEIInterpretation(Int b)           { m_framePackingSEIInterpretation = b; }
  Int   getFramePackingArrangementSEIInterpretation()                { return m_framePackingSEIInterpretation; }
  void  setSegmentedRectFramePackingArrangementSEIEnabled(bool b)    { m_segmentedRectFramePackingSEIEnabled = b; }
  bool  getSegmentedRectFramePackingArrangementSEIEnabled() const    { return m_segmentedRectFramePackingSEIEnabled; }
  void  setSegmentedRectFramePackingArrangementSEICancel(Int b)      { m_segmentedRectFramePackingSEICancel = b; }
  Int   getSegmentedRectFramePackingArrangementSEICancel()           { return m_segmentedRectFramePackingSEICancel; }
  void  setSegmentedRectFramePackingArrangementSEIType(Int b)        { m_segmentedRectFramePackingSEIType = b; }
  Int   getSegmentedRectFramePackingArrangementSEIType()             { return m_segmentedRectFramePackingSEIType; }
  void  setSegmentedRectFramePackingArrangementSEIPersistence(Int b) { m_segmentedRectFramePackingSEIPersistence = b; }
  Int   getSegmentedRectFramePackingArrangementSEIPersistence()      { return m_segmentedRectFramePackingSEIPersistence; }
  void  setDisplayOrientationSEIAngle(Int b)                         { m_displayOrientationSEIAngle = b; }
  Int   getDisplayOrientationSEIAngle()                              { return m_displayOrientationSEIAngle; }
  void  setTemporalLevel0IndexSEIEnabled(bool b)                     { m_temporalLevel0IndexSEIEnabled = b; }
  bool  getTemporalLevel0IndexSEIEnabled() const                     { return m_temporalLevel0IndexSEIEnabled; }
  void  setGradualDecodingRefreshInfoEnabled(bool b)                 { m_gradualDecodingRefreshInfoEnabled = b;    }
  bool  getGradualDecodingRefreshInfoEnabled() const                 { return m_gradualDecodingRefreshInfoEnabled; }
  void  setNoDisplaySEITLayer(Int b)                                 { m_noDisplaySEITLayer = b;    }
  Int   getNoDisplaySEITLayer()                                      { return m_noDisplaySEITLayer; }
  void  setDecodingUnitInfoSEIEnabled(bool b)                        { m_decodingUnitInfoSEIEnabled = b;    }
  bool  getDecodingUnitInfoSEIEnabled() const                        { return m_decodingUnitInfoSEIEnabled; }
  void  setSOPDescriptionSEIEnabled(bool b)                          { m_SOPDescriptionSEIEnabled = b; }
  bool  getSOPDescriptionSEIEnabled() const                          { return m_SOPDescriptionSEIEnabled; }
  void  setScalableNestingSEIEnabled(bool b)                         { m_scalableNestingSEIEnabled = b; }
  bool  getScalableNestingSEIEnabled() const                         { return m_scalableNestingSEIEnabled; }
  void  setTMCTSSEIEnabled(bool b)                                   { m_tmctsSEIEnabled = b; }
  bool  getTMCTSSEIEnabled()                                         { return m_tmctsSEIEnabled; }
  void  setTimeCodeSEIEnabled(bool b)                                { m_timeCodeSEIEnabled = b; }
  bool  getTimeCodeSEIEnabled()                                      { return m_timeCodeSEIEnabled; }
  void  setNumberOfTimeSets(Int value)                               { m_timeCodeSEINumTs = value; }
  Int   getNumberOfTimesets()                                        { return m_timeCodeSEINumTs; }
  void  setTimeSet(SEITimeSet element, Int index)                    { m_timeSetArray[index] = element; }
  SEITimeSet &getTimeSet(Int index)                                  { return m_timeSetArray[index]; }
  const SEITimeSet &getTimeSet(Int index) const                      { return m_timeSetArray[index]; }
  void  setKneeSEIEnabled(Int b)                                     { m_kneeSEIEnabled = b; }
  bool  getKneeSEIEnabled()                                          { return m_kneeSEIEnabled; }
  void  setKneeSEIId(Int b)                                          { m_kneeSEIId = b; }
  Int   getKneeSEIId()                                               { return m_kneeSEIId; }
  void  setKneeSEICancelFlag(bool b)                                 { m_kneeSEICancelFlag=b; }
  bool  getKneeSEICancelFlag()                                       { return m_kneeSEICancelFlag; }
  void  setKneeSEIPersistenceFlag(bool b)                            { m_kneeSEIPersistenceFlag = b; }
  bool  getKneeSEIPersistenceFlag()                                  { return m_kneeSEIPersistenceFlag; }
  void  setKneeSEIInputDrange(Int b)                                 { m_kneeSEIInputDrange = b; }
  Int   getKneeSEIInputDrange()                                      { return m_kneeSEIInputDrange; }
  void  setKneeSEIInputDispLuminance(Int b)                          { m_kneeSEIInputDispLuminance = b; }
  Int   getKneeSEIInputDispLuminance()                               { return m_kneeSEIInputDispLuminance; }
  void  setKneeSEIOutputDrange(Int b)                                { m_kneeSEIOutputDrange = b; }
  Int   getKneeSEIOutputDrange()                                     { return m_kneeSEIOutputDrange; }
  void  setKneeSEIOutputDispLuminance(Int b)                         { m_kneeSEIOutputDispLuminance = b; }
  Int   getKneeSEIOutputDispLuminance()                              { return m_kneeSEIOutputDispLuminance; }
  void  setKneeSEINumKneePointsMinus1(Int b)                         { m_kneeSEINumKneePointsMinus1 = b; }
  Int   getKneeSEINumKneePointsMinus1()                              { return m_kneeSEINumKneePointsMinus1; }
  void  setKneeSEIInputKneePoint(Int *p)                             { m_kneeSEIInputKneePoint = p; }
  Int*  getKneeSEIInputKneePoint()                                   { return m_kneeSEIInputKneePoint; }
  void  setKneeSEIOutputKneePoint(Int *p)                            { m_kneeSEIOutputKneePoint = p; }
  Int*  getKneeSEIOutputKneePoint()                                  { return m_kneeSEIOutputKneePoint; }
  void  setColourRemapInfoSEIFileRoot( const std::string &s )        { m_colourRemapSEIFileRoot = s; }
  const std::string &getColourRemapInfoSEIFileRoot() const           { return m_colourRemapSEIFileRoot; }
  void  setMasteringDisplaySEI(const SEIMasteringDisplay &src)       { m_masteringDisplay = src; }
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void  setSEIAlternativeTransferCharacteristicsSEIEnable( bool b)   { m_alternativeTransferCharacteristicsSEIEnabled = b;    }
  bool  getSEIAlternativeTransferCharacteristicsSEIEnable( ) const   { return m_alternativeTransferCharacteristicsSEIEnabled; }
  void  setSEIPreferredTransferCharacteristics(UChar v)              { m_preferredTransferCharacteristics = v;    }
  UChar getSEIPreferredTransferCharacteristics() const               { return m_preferredTransferCharacteristics; }
#endif
  void  setSEIGreenMetadataInfoSEIEnable( bool b)                    { m_greenMetadataInfoSEIEnabled = b;    }
  bool  getSEIGreenMetadataInfoSEIEnable( ) const                    { return m_greenMetadataInfoSEIEnabled; }
  void  setSEIGreenMetadataType(UChar v)                             { m_greenMetadataType = v;    }
  UChar getSEIGreenMetadataType() const                              { return m_greenMetadataType; }
  void  setSEIXSDMetricType(UChar v)                                 { m_xsdMetricType = v;    }
  UChar getSEIXSDMetricType() const                                  { return m_xsdMetricType; }

  const SEIMasteringDisplay &getMasteringDisplaySEI() const          { return m_masteringDisplay; }
  void         setUseWP               ( bool b )                     { m_useWeightedPred   = b;    }
  void         setWPBiPred            ( bool b )                     { m_useWeightedBiPred = b;    }
  bool         getUseWP               ()                             { return m_useWeightedPred;   }
  bool         getWPBiPred            ()                             { return m_useWeightedBiPred; }
  void         setLog2ParallelMergeLevelMinus2   ( UInt u )          { m_log2ParallelMergeLevelMinus2       = u;    }
  UInt         getLog2ParallelMergeLevelMinus2   ()                  { return m_log2ParallelMergeLevelMinus2;       }
  void         setMaxNumMergeCand                ( UInt u )          { m_maxNumMergeCand = u;      }
  UInt         getMaxNumMergeCand                ()                  { return m_maxNumMergeCand;   }
#if HEVC_USE_SCALING_LISTS
  void         setUseScalingListId    ( ScalingListMode u )          { m_useScalingListId       = u;   }
  ScalingListMode getUseScalingListId    ()                          { return m_useScalingListId;      }
  void         setScalingListFileName       ( const std::string &s ) { m_scalingListFileName = s;      }
  const std::string& getScalingListFileName () const                 { return m_scalingListFileName;   }
#endif
  void         setTMVPModeId ( Int  u )                              { m_TMVPModeId = u;    }
  Int          getTMVPModeId ()                                      { return m_TMVPModeId; }
  WeightedPredictionMethod getWeightedPredictionMethod() const       { return m_weightedPredictionMethod; }
  void         setWeightedPredictionMethod( WeightedPredictionMethod m ) { m_weightedPredictionMethod = m; }
#if JVET_K0072
  void         setDepQuantEnabledFlag( bool b )                      { m_DepQuantEnabledFlag = b;    }
  bool         getDepQuantEnabledFlag()                              { return m_DepQuantEnabledFlag; }
#endif
#if HEVC_USE_SIGN_HIDING
  void         setSignDataHidingEnabledFlag( bool b )                { m_SignDataHidingEnabledFlag = b;    }
  bool         getSignDataHidingEnabledFlag()                        { return m_SignDataHidingEnabledFlag; }
#endif
  bool         getUseRateCtrl         () const                       { return m_RCEnableRateControl;   }
  void         setUseRateCtrl         ( bool b )                     { m_RCEnableRateControl = b;      }
  Int          getTargetBitrate       ()                             { return m_RCTargetBitrate;       }
  void         setTargetBitrate       ( Int bitrate )                { m_RCTargetBitrate  = bitrate;   }
  Int          getKeepHierBit         ()                             { return m_RCKeepHierarchicalBit; }
  void         setKeepHierBit         ( Int i )                      { m_RCKeepHierarchicalBit = i;    }
  bool         getLCULevelRC          ()                             { return m_RCLCULevelRC; }
  void         setLCULevelRC          ( bool b )                     { m_RCLCULevelRC = b; }
  bool         getUseLCUSeparateModel ()                             { return m_RCUseLCUSeparateModel; }
  void         setUseLCUSeparateModel ( bool b )                     { m_RCUseLCUSeparateModel = b;    }
  Int          getInitialQP           ()                             { return m_RCInitialQP;           }
  void         setInitialQP           ( Int QP )                     { m_RCInitialQP = QP;             }
  bool         getForceIntraQP        ()                             { return m_RCForceIntraQP;        }
  void         setForceIntraQP        ( bool b )                     { m_RCForceIntraQP = b;           }
#if U0132_TARGET_BITS_SATURATION
  bool         getCpbSaturationEnabled()                             { return m_RCCpbSaturationEnabled;}
  void         setCpbSaturationEnabled( bool b )                     { m_RCCpbSaturationEnabled = b;   }
  UInt         getCpbSize             ()                             { return m_RCCpbSize;}
  void         setCpbSize             ( UInt ui )                    { m_RCCpbSize = ui;   }
  double       getInitialCpbFullness  ()                             { return m_RCInitialCpbFullness;  }
  void         setInitialCpbFullness  (double f)                     { m_RCInitialCpbFullness = f;     }
#endif
  bool         getTransquantBypassEnabledFlag()                      { return m_TransquantBypassEnabledFlag; }
  void         setTransquantBypassEnabledFlag(bool flag)             { m_TransquantBypassEnabledFlag = flag; }
  bool         getCUTransquantBypassFlagForceValue() const           { return m_CUTransquantBypassFlagForce; }
  void         setCUTransquantBypassFlagForceValue(bool flag)        { m_CUTransquantBypassFlagForce = flag; }
  CostMode     getCostMode( ) const                                  { return m_costMode; }
  void         setCostMode(CostMode m )                              { m_costMode = m; }

#if HEVC_VPS
  void         setVPS(VPS *p)                                        { m_cVPS = *p; }
  VPS *        getVPS()                                              { return &m_cVPS; }
#endif
  void         setUseRecalculateQPAccordingToLambda (bool b)         { m_recalculateQPAccordingToLambda = b;    }
  bool         getUseRecalculateQPAccordingToLambda ()               { return m_recalculateQPAccordingToLambda; }

#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  void         setUseStrongIntraSmoothing ( bool b )                 { m_useStrongIntraSmoothing = b;    }
  bool         getUseStrongIntraSmoothing ()                         { return m_useStrongIntraSmoothing; }

#endif
  void         setEfficientFieldIRAPEnabled( bool b )                { m_bEfficientFieldIRAPEnabled = b; }
  bool         getEfficientFieldIRAPEnabled( ) const                 { return m_bEfficientFieldIRAPEnabled; }

  void         setHarmonizeGopFirstFieldCoupleEnabled( bool b )      { m_bHarmonizeGopFirstFieldCoupleEnabled = b; }
  bool         getHarmonizeGopFirstFieldCoupleEnabled( ) const       { return m_bHarmonizeGopFirstFieldCoupleEnabled; }

  void         setActiveParameterSetsSEIEnabled ( Int b )            { m_activeParameterSetsSEIEnabled = b; }
  Int          getActiveParameterSetsSEIEnabled ()                   { return m_activeParameterSetsSEIEnabled; }
  bool         getVuiParametersPresentFlag()                         { return m_vuiParametersPresentFlag; }
  void         setVuiParametersPresentFlag(bool i)                   { m_vuiParametersPresentFlag = i; }
  bool         getAspectRatioInfoPresentFlag()                       { return m_aspectRatioInfoPresentFlag; }
  void         setAspectRatioInfoPresentFlag(bool i)                 { m_aspectRatioInfoPresentFlag = i; }
  Int          getAspectRatioIdc()                                   { return m_aspectRatioIdc; }
  void         setAspectRatioIdc(Int i)                              { m_aspectRatioIdc = i; }
  Int          getSarWidth()                                         { return m_sarWidth; }
  void         setSarWidth(Int i)                                    { m_sarWidth = i; }
  Int          getSarHeight()                                        { return m_sarHeight; }
  void         setSarHeight(Int i)                                   { m_sarHeight = i; }
  bool         getOverscanInfoPresentFlag()                          { return m_overscanInfoPresentFlag; }
  void         setOverscanInfoPresentFlag(bool i)                    { m_overscanInfoPresentFlag = i; }
  bool         getOverscanAppropriateFlag()                          { return m_overscanAppropriateFlag; }
  void         setOverscanAppropriateFlag(bool i)                    { m_overscanAppropriateFlag = i; }
  bool         getVideoSignalTypePresentFlag()                       { return m_videoSignalTypePresentFlag; }
  void         setVideoSignalTypePresentFlag(bool i)                 { m_videoSignalTypePresentFlag = i; }
  Int          getVideoFormat()                                      { return m_videoFormat; }
  void         setVideoFormat(Int i)                                 { m_videoFormat = i; }
  bool         getVideoFullRangeFlag()                               { return m_videoFullRangeFlag; }
  void         setVideoFullRangeFlag(bool i)                         { m_videoFullRangeFlag = i; }
  bool         getColourDescriptionPresentFlag()                     { return m_colourDescriptionPresentFlag; }
  void         setColourDescriptionPresentFlag(bool i)               { m_colourDescriptionPresentFlag = i; }
  Int          getColourPrimaries()                                  { return m_colourPrimaries; }
  void         setColourPrimaries(Int i)                             { m_colourPrimaries = i; }
  Int          getTransferCharacteristics()                          { return m_transferCharacteristics; }
  void         setTransferCharacteristics(Int i)                     { m_transferCharacteristics = i; }
  Int          getMatrixCoefficients()                               { return m_matrixCoefficients; }
  void         setMatrixCoefficients(Int i)                          { m_matrixCoefficients = i; }
  bool         getChromaLocInfoPresentFlag()                         { return m_chromaLocInfoPresentFlag; }
  void         setChromaLocInfoPresentFlag(bool i)                   { m_chromaLocInfoPresentFlag = i; }
  Int          getChromaSampleLocTypeTopField()                      { return m_chromaSampleLocTypeTopField; }
  void         setChromaSampleLocTypeTopField(Int i)                 { m_chromaSampleLocTypeTopField = i; }
  Int          getChromaSampleLocTypeBottomField()                   { return m_chromaSampleLocTypeBottomField; }
  void         setChromaSampleLocTypeBottomField(Int i)              { m_chromaSampleLocTypeBottomField = i; }
  bool         getNeutralChromaIndicationFlag()                      { return m_neutralChromaIndicationFlag; }
  void         setNeutralChromaIndicationFlag(bool i)                { m_neutralChromaIndicationFlag = i; }
  Window      &getDefaultDisplayWindow()                             { return m_defaultDisplayWindow; }
  void         setDefaultDisplayWindow (Int offsetLeft, Int offsetRight, Int offsetTop, Int offsetBottom ) { m_defaultDisplayWindow.setWindow (offsetLeft, offsetRight, offsetTop, offsetBottom); }
  bool         getFrameFieldInfoPresentFlag()                        { return m_frameFieldInfoPresentFlag; }
  void         setFrameFieldInfoPresentFlag(bool i)                  { m_frameFieldInfoPresentFlag = i; }
  bool         getPocProportionalToTimingFlag()                      { return m_pocProportionalToTimingFlag; }
  void         setPocProportionalToTimingFlag(bool x)                { m_pocProportionalToTimingFlag = x;    }
  Int          getNumTicksPocDiffOneMinus1()                         { return m_numTicksPocDiffOneMinus1;    }
  void         setNumTicksPocDiffOneMinus1(Int x)                    { m_numTicksPocDiffOneMinus1 = x;       }
  bool         getBitstreamRestrictionFlag()                         { return m_bitstreamRestrictionFlag; }
  void         setBitstreamRestrictionFlag(bool i)                   { m_bitstreamRestrictionFlag = i; }
#if HEVC_TILES_WPP
  bool         getTilesFixedStructureFlag()                          { return m_tilesFixedStructureFlag; }
  void         setTilesFixedStructureFlag(bool i)                    { m_tilesFixedStructureFlag = i; }
#endif
  bool         getMotionVectorsOverPicBoundariesFlag()               { return m_motionVectorsOverPicBoundariesFlag; }
  void         setMotionVectorsOverPicBoundariesFlag(bool i)         { m_motionVectorsOverPicBoundariesFlag = i; }
  Int          getMinSpatialSegmentationIdc()                        { return m_minSpatialSegmentationIdc; }
  void         setMinSpatialSegmentationIdc(Int i)                   { m_minSpatialSegmentationIdc = i; }
  Int          getMaxBytesPerPicDenom()                              { return m_maxBytesPerPicDenom; }
  void         setMaxBytesPerPicDenom(Int i)                         { m_maxBytesPerPicDenom = i; }
  Int          getMaxBitsPerMinCuDenom()                             { return m_maxBitsPerMinCuDenom; }
  void         setMaxBitsPerMinCuDenom(Int i)                        { m_maxBitsPerMinCuDenom = i; }
  Int          getLog2MaxMvLengthHorizontal()                        { return m_log2MaxMvLengthHorizontal; }
  void         setLog2MaxMvLengthHorizontal(Int i)                   { m_log2MaxMvLengthHorizontal = i; }
  Int          getLog2MaxMvLengthVertical()                          { return m_log2MaxMvLengthVertical; }
  void         setLog2MaxMvLengthVertical(Int i)                     { m_log2MaxMvLengthVertical = i; }

  bool         getProgressiveSourceFlag() const                      { return m_progressiveSourceFlag; }
  void         setProgressiveSourceFlag(bool b)                      { m_progressiveSourceFlag = b; }

  bool         getInterlacedSourceFlag() const                       { return m_interlacedSourceFlag; }
  void         setInterlacedSourceFlag(bool b)                       { m_interlacedSourceFlag = b; }

  bool         getNonPackedConstraintFlag() const                    { return m_nonPackedConstraintFlag; }
  void         setNonPackedConstraintFlag(bool b)                    { m_nonPackedConstraintFlag = b; }

  bool         getFrameOnlyConstraintFlag() const                    { return m_frameOnlyConstraintFlag; }
  void         setFrameOnlyConstraintFlag(bool b)                    { m_frameOnlyConstraintFlag = b; }

  UInt         getBitDepthConstraintValue() const                    { return m_bitDepthConstraintValue; }
  void         setBitDepthConstraintValue(UInt v)                    { m_bitDepthConstraintValue=v; }

  ChromaFormat getChromaFormatConstraintValue() const                { return m_chromaFormatConstraintValue; }
  void         setChromaFormatConstraintValue(ChromaFormat v)        { m_chromaFormatConstraintValue=v; }

  bool         getIntraConstraintFlag() const                        { return m_intraConstraintFlag; }
  void         setIntraConstraintFlag(bool b)                        { m_intraConstraintFlag=b; }

  bool         getOnePictureOnlyConstraintFlag() const               { return m_onePictureOnlyConstraintFlag; }
  void         setOnePictureOnlyConstraintFlag(bool b)               { m_onePictureOnlyConstraintFlag=b; }

  bool         getLowerBitRateConstraintFlag() const                 { return m_lowerBitRateConstraintFlag; }
  void         setLowerBitRateConstraintFlag(bool b)                 { m_lowerBitRateConstraintFlag=b; }

  bool         getChromaResamplingFilterHintEnabled()                { return m_chromaResamplingFilterHintEnabled;}
  void         setChromaResamplingFilterHintEnabled(bool i)          { m_chromaResamplingFilterHintEnabled = i;}
  Int          getChromaResamplingHorFilterIdc()                     { return m_chromaResamplingHorFilterIdc;}
  void         setChromaResamplingHorFilterIdc(Int i)                { m_chromaResamplingHorFilterIdc = i;}
  Int          getChromaResamplingVerFilterIdc()                     { return m_chromaResamplingVerFilterIdc;}
  void         setChromaResamplingVerFilterIdc(Int i)                { m_chromaResamplingVerFilterIdc = i;}

  void         setSummaryOutFilename(const std::string &s)           { m_summaryOutFilename = s; }
  const std::string& getSummaryOutFilename() const                   { return m_summaryOutFilename; }
  void         setSummaryPicFilenameBase(const std::string &s)       { m_summaryPicFilenameBase = s; }
  const std::string& getSummaryPicFilenameBase() const               { return m_summaryPicFilenameBase; }

  void         setSummaryVerboseness(UInt v)                         { m_summaryVerboseness = v; }
  UInt         getSummaryVerboseness( ) const                        { return m_summaryVerboseness; }
#if JVET_K0357_AMVR
  void         setIMV(int n)                                         { m_ImvMode = n; }
  Int          getIMV() const                                        { return m_ImvMode; }
  void         setIMV4PelFast(int n)                                 { m_Imv4PelFast = n; }
  Int          getIMV4PelFast() const                                { return m_Imv4PelFast; }
  void         setIMVMaxCand(Int n)                                  { m_ImvMaxCand = n; }
  Int          getIMVMaxCand() const                                 { return m_ImvMaxCand; }
#endif
  void         setDecodeBitstream( int i, const std::string& s )     { m_decodeBitstreams[i] = s; }
  const std::string& getDecodeBitstream( int i )               const { return m_decodeBitstreams[i]; }
  bool         getForceDecodeBitstream1()                      const { return m_forceDecodeBitstream1; }
  void         setForceDecodeBitstream1( bool b )                    { m_forceDecodeBitstream1 = b; }
  void         setSwitchPOC( int i )                                 { m_switchPOC = i; }
  int          getSwitchPOC()                                  const { return m_switchPOC; }
  void         setSwitchDQP( int i )                                 { m_switchDQP = i; }
  int          getSwitchDQP()                                  const { return m_switchDQP; }
  void         setFastForwardToPOC( int i )                          { m_fastForwardToPOC = i; }
  int          getFastForwardToPOC()                           const { return m_fastForwardToPOC; }
  bool         useFastForwardToPOC()                           const { return m_fastForwardToPOC >= 0; }
  void         setStopAfterFFtoPOC( bool b )                         { m_stopAfterFFtoPOC = b; }
  bool         getStopAfterFFtoPOC()                           const { return m_stopAfterFFtoPOC; }
  void         setBs2ModPOCAndType( bool b )                         { m_bs2ModPOCAndType = b; }
  bool         getBs2ModPOCAndType()                           const { return m_bs2ModPOCAndType; }


#if ENABLE_SPLIT_PARALLELISM
  void         setNumSplitThreads( int n )                           { m_numSplitThreads = n; }
  int          getNumSplitThreads()                            const { return m_numSplitThreads; }
  void         setForceSingleSplitThread( bool b )                   { m_forceSingleSplitThread = b; }
  int          getForceSingleSplitThread()                     const { return m_forceSingleSplitThread; }
#endif
#if ENABLE_WPP_PARALLELISM
  void         setNumWppThreads( int n )                             { m_numWppThreads = n; }
  int          getNumWppThreads()                              const { return m_numWppThreads; }
  void         setNumWppExtraLines( int n )                          { m_numWppExtraLines = n; }
  int          getNumWppExtraLines()                           const { return m_numWppExtraLines; }
  void         setEnsureWppBitEqual( bool b)                         { m_ensureWppBitEqual = b; }
  bool         getEnsureWppBitEqual()                          const { return m_ensureWppBitEqual; }
#endif
#if JVET_K0371_ALF
  void        setUseALF( bool b ) { m_alf = b; }
  bool        getUseALF()                                      const { return m_alf; }
#endif
};

//! \}
  
#endif // !defined(AFX_TENCCFG_H__6B99B797_F4DA_4E46_8E78_7656339A6C41__INCLUDED_)
