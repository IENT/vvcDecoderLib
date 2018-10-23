
#include "VVCDecoderLib.h"


#include "Utilities/VideoIOYuv.h"
#include "Utilities/ColourRemapping.h"
#include "CommonLib/Picture.h"
#include "DecoderLib/DecLib.h"
#include "DecoderLib/NALread.h"
#include "CommonLib/UnitTools.h"

bool isNaluWithinTargetDecLayerIdSet(InputNALUnit* nalu) { return true; }

#define VVCDecoderWrapper_MAXBUFFERSIZE 1000
#define VTM_BMS_COMPAT 0
class VVCDecoderWrapper
{
public:
  VVCDecoderWrapper()
  {
    loopFiltered = false;
    maxTemporalLayer = -1; ///< maximum temporal layer to be decoded
    iPOCLastDisplay = -MAX_INT;
    iSkipFrame = 0;
    pcListPic = NULL;
    pcListPic_readIdx = 0;
    numPicsNotYetDisplayed = 0;
    dpbFullness = 0;
    lastNALTemporalID = 0;
    flushOutput = false;
    sheduleFlushing = false;

    // Initialize the decoder
    initROM();
    decoder.create();
    decoder.init();
    decoder.setDecodedPictureHashSEIEnabled(true);
    iPOCLastDisplay += iSkipFrame; // set the last displayed POC correctly for skip forward.

    internalsBlockDataValues = 0;
    pauseInternalsCUIdx = -1;
    parsedCUs.clear();
    lastStatsPOC = -1;
  }
  ~VVCDecoderWrapper()
  { 
    decoder.destroy(); 
  };

  bool loopFiltered;
  int  maxTemporalLayer; ///< maximum temporal layer to be decoded
  int iPOCLastDisplay;
  int iSkipFrame;
  PicList* pcListPic;
  int pcListPic_readIdx;
  int numPicsNotYetDisplayed;
  unsigned int numReorderPicsHighestTid;
  unsigned int maxDecPicBufferingHighestTid;
  int dpbFullness;
  int lastNALTemporalID;
  bool flushOutput;
  bool sheduleFlushing; // After the normal output function is finished, we will perform flushing.

  // The local memory for the global variable
  bool md5_mismatch;

  // Add the internals block value to the array. Check if the data array is full first.
  void addInternalsBlockData(libVVCDec_BlockValue val);
  bool internalsBlockDataFull() { return internalsBlockDataValues >= VVCDecoderWrapper_MAXBUFFERSIZE;  }
  void clearInternalsBlockData() { internalsBlockDataValues = 0; }

  DecLib decoder;

  // The array that is filled when internals are returned.
  // The array is defined, filled and cleared only in this library so that no chaos is created
  // between the heap of the shared library and the caller programm.
  libVVCDec_BlockValue internalsBlockData[VVCDecoderWrapper_MAXBUFFERSIZE];
  unsigned int internalsBlockDataValues;

  // If the array is full, we will save which CU we were processing when the array did overflow
  // so that we can continue the next time libVVCDEC_get_internal_info() is called.
  int pauseInternalsCUIdx;
  std::map<int, bool> parsedCUs;
  int lastStatsPOC;
};

void VVCDecoderWrapper::addInternalsBlockData(libVVCDec_BlockValue val)
{
  // Append the value
  internalsBlockData[internalsBlockDataValues] = val;
  internalsBlockDataValues++;
}

extern "C" {

  VVC_DEC_API const char *libVVCDec_get_version(void)
  {
    return NV_VERSION;
  }

  VVC_DEC_API libVVCDec_context* libVVCDec_new_decoder(void)
  {
    VVCDecoderWrapper *decCtx = new VVCDecoderWrapper();
    if (!decCtx)
      return NULL;

    return (libVVCDec_context*)decCtx;
  }

  VVC_DEC_API libVVCDec_error libVVCDec_free_decoder(libVVCDec_context* decCtx)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return libVVCDEC_ERROR;

    delete d;
    return libVVCDEC_OK;
  }

  VVC_DEC_API void libVVCDec_set_SEI_Check(libVVCDec_context* decCtx, bool check_hash)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->decoder.setDecodedPictureHashSEIEnabled(check_hash);
  }
  VVC_DEC_API void libVVCDec_set_max_temporal_layer(libVVCDec_context* decCtx, int max_layer)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->maxTemporalLayer = max_layer;
  }

  VVC_DEC_API libVVCDec_error libVVCDec_push_nal_unit(libVVCDec_context *decCtx, const void* data8, int length, bool eof, bool &bNewPicture, bool &checkOutputPictures)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return libVVCDEC_ERROR;

    if (length <= 0)
      return libVVCDEC_ERROR_READ_ERROR;

    // Check the NAL unit header
    uint8_t *data = (uint8_t*)data8;
    if (length < 4 && !eof)
      return libVVCDEC_ERROR_READ_ERROR;

    // Do not copy the start code (if present)
    int copyStart = 0;
    if (data[0] == 0 && data[1] == 1 && data[2] == 1)
      copyStart = 3;
    else if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1)
      copyStart = 4;

    // Create a new NAL unit and put the payload into the nal units buffer
    InputNALUnit nalu;
    InputBitstream &bitstream = nalu.getBitstream();
    vector<uint8_t>& nalUnitBuf = bitstream.getFifo();
    for (int i = 0; i < copyStart; i++)
      data++;
    for (int i = 0; i < length - copyStart; i++)
    {
      nalUnitBuf.push_back(*data);
      data++;
    }

    // Read the NAL unit
    read(nalu);

    if( (d->maxTemporalLayer >= 0 && nalu.m_temporalId > d->maxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
    {
      bNewPicture = false;
    }
    else
    {
      bNewPicture = d->decoder.decode(nalu, d->iSkipFrame, d->iPOCLastDisplay);
      if (bNewPicture)
      {
        // We encountered a new picture in this NAL unit. This means: we will filter the now complete former
        // picture. There might also be pictures to be output/read. After reading these pictures, this function
        // must be called again with the same NAL unit.
        // 
        // The original Decoder will rewind the bitstream for this. We don't have a bitstream like this.
        // This also means that eof is false for this call. Only in the next call it is set.
        eof = false;
      }
    }

    // Filter the picture if decoding is complete
    if (bNewPicture || eof || nalu.m_nalUnitType == NAL_UNIT_EOS && !d->decoder.getFirstSliceInSequence())
    {
      if (!d->loopFiltered || !eof)
      {
        int poc;
        d->decoder.executeLoopFilters();
        // TODO: finishPicture will actually clear all CUs/TUs/PUs from the CodingStructure, making them
        // unavailable for retrieval if internals are to be displayed
        // there is an alternative function called finishPictureLight which does not clear all this data, but it has
        // very strange side effects
        d->decoder.finishPictureLight(poc, d->pcListPic); 
        d->decoder.setFirstSliceInPicture(true);
        d->decoder.deleteTempBuffersAndCoeffs();
      }
      d->loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        d->decoder.setFirstSliceInSequence(true);
      }
    }
    else if ((bNewPicture || !eof || nalu.m_nalUnitType == NAL_UNIT_EOS) && d->decoder.getFirstSliceInSequence())
    {
      d->decoder.setFirstSliceInPicture(true);
    }

    // Check if we might be able to read pictures
    checkOutputPictures = false;
    d->flushOutput = false;
    bool fixCheckOutput;
    if ( bNewPicture &&
      (   nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP ) )
    {
      checkOutputPictures = true;
      d->flushOutput = true;
    }
    if (nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      checkOutputPictures = true;
      fixCheckOutput = true;
      d->decoder.setFirstSliceInPicture(false);
    }

    // TODO: what is this good for?
    // FIX_WRITING_OUTPUT
    fixCheckOutput = (!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31);

    // next, try to get frames from the deocder
    if((bNewPicture || fixCheckOutput) && d->pcListPic != NULL)
    {
      checkOutputPictures = true;

      d->lastNALTemporalID = nalu.m_temporalId;

      // This is what xWriteOutput does before iterating over the pictures
      const SPS* activeSPS = d->pcListPic->front()->cs->sps;
      unsigned int maxNrSublayers = activeSPS->getMaxTLayers();
      d->numPicsNotYetDisplayed = 0;
      d->dpbFullness = 0;

      if(d->maxTemporalLayer == -1 || d->maxTemporalLayer >= maxNrSublayers)
      {
        d->numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers-1);
        d->maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);
      }
      else
      {
        d->numReorderPicsHighestTid = activeSPS->getNumReorderPics(d->maxTemporalLayer);
        d->maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering(d->maxTemporalLayer);
      }

      PicList::iterator iterPic = d->pcListPic->begin();
      while (iterPic != d->pcListPic->end())
      {
        Picture* pcPic = *(iterPic);
        if (pcPic->neededForOutput && pcPic->getPOC() > d->iPOCLastDisplay)
        {
          d->numPicsNotYetDisplayed++;
          d->dpbFullness++;
        }
        else if(pcPic->referenced)
        {
          d->dpbFullness++;
        }
        iterPic++;
      }
    }

    if (eof)
    {
      // At the end of the file we have to use the normal output function once and then the flushing
      checkOutputPictures = true;
      d->sheduleFlushing = true;
    }

    if (checkOutputPictures)
      // Reset the iterator over the output images
      d->pcListPic_readIdx = 0;

    return libVVCDEC_OK;
  }

  VVC_DEC_API libVVCDec_error libVVCDec_get_nal_unit_info(libVVCDec_context *decCtx, const void* data8, int length, bool eof, int &poc, bool &isRAP, bool &isParameterSet, int &picWidthLumaSamples, int &picHeightLumaSamples, int &bitDepthLuma, int &bitDepthChroma, libVVCDec_ChromaFormat &chromaFormat)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return libVVCDEC_ERROR;

    if (length <= 0)
      return libVVCDEC_ERROR_READ_ERROR;

    // Check the NAL unit header
    uint8_t *data = (uint8_t*)data8;
    if (length < 4 && !eof)
      return libVVCDEC_ERROR_READ_ERROR;

    // Do not copy the start code (if present)
    int copyStart = 0;
    if (data[0] == 0 && data[1] == 1 && data[2] == 1)
      copyStart = 3;
    else if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1)

      copyStart = 4;
    // Create a new NAL unit and put the payload into the nal units buffer
    InputNALUnit nalu;
    InputBitstream &bitstream = nalu.getBitstream();
    vector<uint8_t>& nalUnitBuf = bitstream.getFifo();
    for (int i = 0; i < copyStart; i++)
      data++;
    for (int i = 0; i < length - copyStart; i++)
    {
      nalUnitBuf.push_back(*data);
      data++;
    }

    // Read the NAL unit and parse the NAL unit header
    read(nalu);

    // Set the default return values
    poc = -1;
    isRAP = false;
    isParameterSet = false;
    picWidthLumaSamples = -1;
    picHeightLumaSamples = -1;
    bitDepthLuma = -1;
    bitDepthChroma = -1;
    chromaFormat = libVVCDEC_CHROMA_UNKNOWN;

    if( (d->maxTemporalLayer >= 0 && nalu.m_temporalId > d->maxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
    {
      // We do not parse the NAL unit.
      return libVVCDEC_OK;
    }
    else
    {
      // We will parse the NAL unit. This NAL might be part of a new picture that generates an output picture.
      // Get the POC of this picture.
      ChromaFormat c = NUM_CHROMA_FORMAT;
      d->decoder.decode(nalu, d->iSkipFrame, d->iPOCLastDisplay);

      // Set the values
      isRAP = (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP   || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
               nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP   || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
               nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP   || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA        ||
               nalu.m_nalUnitType == NAL_UNIT_RESERVED_IRAP_VCL22    || nalu.m_nalUnitType == NAL_UNIT_RESERVED_IRAP_VCL23);
      isParameterSet = (nalu.m_nalUnitType == NAL_UNIT_SPS || nalu.m_nalUnitType == NAL_UNIT_PPS);
      
      if (nalu.m_nalUnitType == NAL_UNIT_SPS)
      {
        const SPS* firstSPS = d->decoder.getParameterSetManager()->getFirstSPS();
        picWidthLumaSamples = firstSPS->getPicWidthInLumaSamples() - firstSPS->getConformanceWindow().getWindowLeftOffset() - firstSPS->getConformanceWindow().getWindowRightOffset();
        picHeightLumaSamples = firstSPS->getPicHeightInLumaSamples() - firstSPS->getConformanceWindow().getWindowTopOffset() - firstSPS->getConformanceWindow().getWindowBottomOffset();
        bitDepthLuma = firstSPS->getBitDepth(CHANNEL_TYPE_LUMA);
        bitDepthChroma = firstSPS->getBitDepth(CHANNEL_TYPE_CHROMA);
        c = firstSPS->getChromaFormatIdc();
      }

      if (!isParameterSet)
      {
        poc = d->decoder.getSlicePilot()->getPOC();
      }
      
      if (c == CHROMA_400)
        chromaFormat = libVVCDEC_CHROMA_400;
      else if (c == CHROMA_420)
        chromaFormat = libVVCDEC_CHROMA_420;
      else if (c == CHROMA_422)
        chromaFormat = libVVCDEC_CHROMA_422;
      else if (c == CHROMA_444)
        chromaFormat = libVVCDEC_CHROMA_444;
      else
        chromaFormat = libVVCDEC_CHROMA_UNKNOWN;

      return libVVCDEC_OK;
    }
  }

  VVC_DEC_API libVVCDec_picture *libVVCDec_get_picture(libVVCDec_context* decCtx)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return NULL;

    if (d->pcListPic == NULL)
      return NULL;
    if (d->pcListPic->size() == 0)
      return NULL;
    if (d->pcListPic_readIdx < 0 || d->pcListPic_readIdx > d->pcListPic->size())
      return NULL;

    // Get the pcListPic_readIdx-th picture from the list
    PicList::iterator iterPic = d->pcListPic->begin();
    for (int i = 0; i < d->pcListPic_readIdx; i++)
      iterPic++;

    // Go on in the list until we run out of frames or find one that we can output
    while (iterPic != d->pcListPic->end())
    {
      Picture* pcPic = *(iterPic);

      if ((d->flushOutput && (pcPic->neededForOutput)) ||
        (pcPic->neededForOutput && pcPic->getPOC() > d->iPOCLastDisplay && (d->numPicsNotYetDisplayed > d->numReorderPicsHighestTid || d->dpbFullness > d->maxDecPicBufferingHighestTid)))
      {
        if (!d->flushOutput)
          // Output picture found
          d->numPicsNotYetDisplayed--;

        if(pcPic->referenced == false)
          d->dpbFullness--;

        // update POC of display order
        d->iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( !pcPic->referenced && pcPic->neededForOutput == true )
        {
#if !DYN_REF_FREE
          pcPic->neededForOutput = false;

          // mark it should be extended later
          pcPic->setBorderExtension( false );

#else
          pcPic->destroy();
          pcListPic->erase( iterPic );
          iterPic = pcListPic->begin(); // to the beginning, non-efficient way, have to be revised!
          continue;
#endif
        }
        pcPic->neededForOutput = false;

        // Return the picture
        return (libVVCDec_picture*)pcPic;
      }

      iterPic++;
      d->pcListPic_readIdx++;
    }

    // We reached the end of the list wothout finding an output picture
    if (d->flushOutput)
    {
      // Flushing over
      d->pcListPic->clear();
      d->iPOCLastDisplay = -MAX_INT;
      d->flushOutput = false;
    }
    if (d->sheduleFlushing)
    {
      // The normal output function is over. In the next call to this function, we will start flushing.
      d->flushOutput = true;
      d->sheduleFlushing = false;
      d->pcListPic_readIdx = 0;   // Iterate over all items again
    }

    return NULL;
  }

  VVC_DEC_API int libVVCDEC_get_POC(libVVCDec_picture *pic)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    return pcPic->getPOC();
  }

  VVC_DEC_API int libVVCDEC_get_picture_width(libVVCDec_picture *pic, libVVCDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    // Conformance window
    const Window &conf = pcPic->cs->slice->getSPS()->getConformanceWindow();
    
    if (c == libVVCDEC_LUMA)
    {
      int subtract = (conf.getWindowLeftOffset() >> getComponentScaleX(COMPONENT_Y, pcPic->chromaFormat)) + (conf.getWindowRightOffset() >> getComponentScaleX(COMPONENT_Y, pcPic->chromaFormat));
      return pcPic->getRecoBuf().get(COMPONENT_Y).width - subtract;
    }
    if (c == libVVCDEC_CHROMA_U)
    {
      int subtract = (conf.getWindowLeftOffset() >> getComponentScaleX(COMPONENT_Cb, pcPic->chromaFormat)) + (conf.getWindowRightOffset() >> getComponentScaleX(COMPONENT_Cb, pcPic->chromaFormat));
      return pcPic->getRecoBuf().get(COMPONENT_Cb).width - subtract;
    }
    if (c == libVVCDEC_CHROMA_V)
    {
      int subtract = (conf.getWindowLeftOffset() >> getComponentScaleX(COMPONENT_Cr, pcPic->chromaFormat)) + (conf.getWindowRightOffset() >> getComponentScaleX(COMPONENT_Cr, pcPic->chromaFormat));
      return pcPic->getRecoBuf().get(COMPONENT_Cr).width - subtract;
    }
    return -1;
  }

  VVC_DEC_API int libVVCDEC_get_picture_height(libVVCDec_picture *pic, libVVCDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    // Conformance window
    const Window &conf = pcPic->cs->slice->getSPS()->getConformanceWindow();

    if (c == libVVCDEC_LUMA)
    {
      int subtract = (conf.getWindowBottomOffset() >> getComponentScaleY(COMPONENT_Y, pcPic->chromaFormat)) + (conf.getWindowTopOffset() >> getComponentScaleY(COMPONENT_Y, pcPic->chromaFormat));
      return pcPic->getRecoBuf().get(COMPONENT_Y).height - subtract;
    }
    if (c == libVVCDEC_CHROMA_U)
    {
      int subtract = (conf.getWindowBottomOffset() >> getComponentScaleY(COMPONENT_Cb, pcPic->chromaFormat)) + (conf.getWindowTopOffset() >> getComponentScaleY(COMPONENT_Cb, pcPic->chromaFormat));
      return pcPic->getRecoBuf().get(COMPONENT_Cb).height - subtract;
    }
    if (c == libVVCDEC_CHROMA_V)
    {
      int subtract = (conf.getWindowBottomOffset() >> getComponentScaleY(COMPONENT_Cr, pcPic->chromaFormat)) + (conf.getWindowTopOffset() >> getComponentScaleY(COMPONENT_Cr, pcPic->chromaFormat));
      return pcPic->getRecoBuf().get(COMPONENT_Cr).height - subtract;
    }
    return -1;
  }

  VVC_DEC_API int libVVCDEC_get_picture_stride(libVVCDec_picture *pic, libVVCDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == libVVCDEC_LUMA)
      return pcPic->getRecoBuf().get(COMPONENT_Y).stride;
    if (c == libVVCDEC_CHROMA_U)
      return pcPic->getRecoBuf().get(COMPONENT_Cb).stride;
    if (c == libVVCDEC_CHROMA_V)
      return pcPic->getRecoBuf().get(COMPONENT_Cr).stride;
    return -1;
  }

  VVC_DEC_API short* libVVCDEC_get_image_plane(libVVCDec_picture *pic, libVVCDec_ColorComponent c)
  {
    if (pic == NULL)
      return NULL;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return NULL;

    // Conformance window
    const Window &conf = pcPic->cs->slice->getSPS()->getConformanceWindow();
    ComponentID compID = (c == libVVCDEC_LUMA) ? COMPONENT_Y : (c == libVVCDEC_CHROMA_U) ? COMPONENT_Cb : COMPONENT_Cr;
    const int csx = getComponentScaleX(compID,pcPic->chromaFormat);
    const int csy = getComponentScaleY(compID, pcPic->chromaFormat);
    const int planeOffset = (conf.getWindowLeftOffset() >> csx) + (conf.getWindowTopOffset() >> csy) * pcPic->getRecoBuf().get(compID).stride;
    
    if (c == libVVCDEC_LUMA)
      return pcPic->getRecoBuf().Y().buf + planeOffset;
    if (c == libVVCDEC_CHROMA_U)
      return pcPic->getRecoBuf().Cb().buf + planeOffset;
    if (c == libVVCDEC_CHROMA_V)
      return pcPic->getRecoBuf().Cr().buf + planeOffset;
    return NULL;
  }

  VVC_DEC_API libVVCDec_ChromaFormat libVVCDEC_get_chroma_format(libVVCDec_picture *pic)
  {
    if (pic == NULL)
      return libVVCDEC_CHROMA_UNKNOWN;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return libVVCDEC_CHROMA_UNKNOWN;

    ChromaFormat f = pcPic->chromaFormat;
    if (f == CHROMA_400)
      return libVVCDEC_CHROMA_400;
    if (f == CHROMA_420)
      return libVVCDEC_CHROMA_420;
    if (f == CHROMA_422)
      return libVVCDEC_CHROMA_422;
    if (f == CHROMA_444)
      return libVVCDEC_CHROMA_444;
    return libVVCDEC_CHROMA_UNKNOWN;
  }

  VVC_DEC_API int libVVCDEC_get_internal_bit_depth(libVVCDec_picture *pic, libVVCDec_ColorComponent c)
  {
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    const BitDepths &bitDepths = pcPic->cs->slice->getSPS()->getBitDepths();
    if (c == libVVCDEC_LUMA)
      return bitDepths.recon[CHANNEL_TYPE_LUMA];
    if (c == libVVCDEC_CHROMA_U || c == libVVCDEC_CHROMA_V)
      return bitDepths.recon[CHANNEL_TYPE_CHROMA];
    return -1;
  }
  
  // --------- internals --------

  typedef enum
  {
    // --------- CU-based --------
    libVVCDEC_CTU_SLICE_INDEX = 0,     ///< The slice index of the CTU
    libVVCDEC_CU_PREDICTION_MODE,      ///< Does the CU use inter (0) or intra(1) prediction?
    libVVCDEC_CU_TRQ_BYPASS,           ///< If transquant bypass is enabled, is the transquant bypass flag set?
    libVVCDEC_CU_SKIP_FLAG,            ///< Is the CU skip flag set?
    libVVCDEC_CU_INTRA_MODE_LUMA,      ///< If the CU uses intra prediction, get the intra mode for luma
    libVVCDEC_CU_INTRA_MODE_CHROMA,    ///< If the CU uses intra prediction, get the intra mode for chroma
    libVVCDEC_CU_ROOT_CBF,             ///< In the CU is inter, get the root coded block flag of the TU
    libVVCDEC_CU_QT_DEPTH,             ///< 
    libVVCDEC_CU_BT_DEPTH,             ///< 
    libVVCDEC_CU_MT_DEPTH,             ///< 
    libVVCDEC_CU_AFFINE_FLAG,          ///< 
    libVVCDEC_CU_AFFINE_MODE,          ///< 
    libVVCDEC_CU_PDPC_FLAG,            ///< 
    libVVCDEC_CU_IPCM_FLAG,            ///< 
    libVVCDEC_CU_QP,                   ///< 
    libVVCDEC_CU_EMT_MODE,             ///< 
    libVVCDEC_CU_NSST_MODE,            ///< 
    libVVCDEC_CU_LIC_FLAG,             ///< 
    libVVCDEC_CU_IMV_MODE,             ///< 
    libVVCDEC_CU_OBMC_FLAG,            ///< 
    libVVCDEC_CU_CPR_FLAG,             ///< 
    libVVCDEC_CU_GBI_MODE,             ///< 
    // --------- PU-based --------
    libVVCDEC_PU_MERGE_FLAG,           ///< If the PU is inter, is the merge flag set?
    libVVCDEC_PU_MERGE_INDEX,          ///< If the PU is merge, what is the merge index?
    libVVCDEC_PU_INTER_DIR,            ///< Does the PU use uni- (0) or biprediction (1)? Also called interDir.
    libVVCDEC_PU_MVP_IDX_L0,              ///< 
    libVVCDEC_PU_MVP_IDX_L1,              ///< 
    libVVCDEC_PU_REFERENCE_POC_L0,      ///< If the PU uses inter prediction, what is the reference POC of list 0?
    libVVCDEC_PU_MV_L0,                 ///< If the PU uses inter prediction, what is the motion vector of list 0?
    libVVCDEC_PU_REFERENCE_POC_L1,      ///< If the PU uses bi-directions inter prediction, what is the reference POC of list 1?
    libVVCDEC_PU_MV_L1,                 ///< If the PU uses bi-directions inter prediction, what is the motion vector of list 1?
    // --------- TU-based --------
    libVVCDEC_TU_CBF_Y,                ///< Get the coded block flag for luma
    libVVCDEC_TU_CBF_CB,               ///< Get the coded block flag for chroma U
    libVVCDEC_TU_CBF_CR,               ///< Get the coded block flag for chroma V
    libVVCDEC_TU_COEFF_TR_SKIP_Y,      ///< Get the transform skip flag for luma
    libVVCDEC_TU_COEFF_TR_SKIP_Cb,     ///< Get the transform skip flag for chroma U
    libVVCDEC_TU_COEFF_TR_SKIP_Cr,     ///< Get the transform skip flag for chroma V
    libVVCDEC_TU_EMT_MODE,             ///<
    libVVCDEC_NUM_TYPES
  } libVVCDec_info_types_idx;

  typedef enum
  {
    CUTypeStat=0,
    PUTypeStat,
    TUTypeStat,
    Other
  } libVVCDec_info_types_dataTye;

  // These types are supported here
  VVC_DEC_API unsigned int libVVCDEC_get_internal_type_number()
  {
    return libVVCDEC_NUM_TYPES;
  }

  VVC_DEC_API const char *libVVCDEC_get_internal_type_name(unsigned int idx)
  {
    switch (idx)
    {
        // --------- CU-based --------
        case libVVCDEC_CTU_SLICE_INDEX         :  return "CTU Slice Index ";
        case libVVCDEC_CU_PREDICTION_MODE      :  return "CU Prediction Mode";
        case libVVCDEC_CU_TRQ_BYPASS           :  return "CU TRQ Bypass";
        case libVVCDEC_CU_SKIP_FLAG            :  return "CU Skip Flag";
        case libVVCDEC_CU_INTRA_MODE_LUMA      :  return "CU Intra Luma Mode";
        case libVVCDEC_CU_INTRA_MODE_CHROMA    :  return "CU Intra Chroma Mode";
        case libVVCDEC_CU_ROOT_CBF             :  return "CU Root CBF";
        case libVVCDEC_CU_QT_DEPTH             :  return "CU QT Depth";
        case libVVCDEC_CU_BT_DEPTH             :  return "CU BT Depth";
        case libVVCDEC_CU_MT_DEPTH             :  return "CU MT Depth";
        case libVVCDEC_CU_AFFINE_FLAG          :  return "CU Affine Flag";
        case libVVCDEC_CU_AFFINE_MODE          :  return "CU Affine Mode";
        case libVVCDEC_CU_PDPC_FLAG            :  return "CU PDPC Flag";
        case libVVCDEC_CU_IPCM_FLAG            :  return "CU IPCM Flag";
        case libVVCDEC_CU_QP                   :  return "CU QP";
        case libVVCDEC_CU_EMT_MODE             :  return "CU EMT Mode";
        case libVVCDEC_CU_NSST_MODE            :  return "CU NSST Mode";
        case libVVCDEC_CU_LIC_FLAG             :  return "CU LIC Flag";
        case libVVCDEC_CU_IMV_MODE             :  return "CU IMV Mode";
        case libVVCDEC_CU_OBMC_FLAG            :  return "CU OBMC Flag";
        case libVVCDEC_CU_CPR_FLAG             :  return "CU CPR Flag";
        case libVVCDEC_CU_GBI_MODE             :  return "CU GBR Mode";
        // --------- PU-based --------
        case libVVCDEC_PU_MERGE_FLAG           :  return "PU Merge Flag";
        case libVVCDEC_PU_MERGE_INDEX          :  return "PU Merge Index";
        case libVVCDEC_PU_INTER_DIR            :  return "PU InterDir";
        case libVVCDEC_PU_MVP_IDX_L0           :  return "PU MVP Index L0";
        case libVVCDEC_PU_MVP_IDX_L1           :  return "PU MVP Index L1";
        case libVVCDEC_PU_REFERENCE_POC_L0     :  return "PU Reference POC L0";
        case libVVCDEC_PU_MV_L0                :  return "PU MV L0";
        case libVVCDEC_PU_REFERENCE_POC_L1     :  return "PU Reference POC L1";
        case libVVCDEC_PU_MV_L1                :  return "PU MV L1";
        // --------- TU-based --------
        case libVVCDEC_TU_CBF_Y                :  return "TU CBF Y";
        case libVVCDEC_TU_CBF_CB               :  return "TU CBF Cb";
        case libVVCDEC_TU_CBF_CR               :  return "TU CBF Cr";
        case libVVCDEC_TU_COEFF_TR_SKIP_Y      :  return "TU Transform Skip Y";
        case libVVCDEC_TU_COEFF_TR_SKIP_Cb     :  return "TU Transform Skip Cb";
        case libVVCDEC_TU_COEFF_TR_SKIP_Cr     :  return "TU Transform Skip Cr";
        case libVVCDEC_TU_EMT_MODE             :  return "TU EMT Mode";
    default: return "";
    }
  }

  VVC_DEC_API libVVCDec_InternalsType libVVCDEC_get_internal_type(unsigned int idx)
  {
    switch (idx)
    {
        // --------- CU-based --------
        case libVVCDEC_CTU_SLICE_INDEX         :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_PREDICTION_MODE      :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_TRQ_BYPASS           :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_SKIP_FLAG            :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_INTRA_MODE_LUMA      :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_INTRA_MODE_CHROMA    :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_ROOT_CBF             :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_QT_DEPTH             :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_BT_DEPTH             :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_MT_DEPTH             :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_AFFINE_FLAG          :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_AFFINE_MODE          :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_PDPC_FLAG            :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_IPCM_FLAG            :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_QP                   :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_EMT_MODE             :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_NSST_MODE            :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_LIC_FLAG             :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_IMV_MODE             :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_CU_OBMC_FLAG            :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_CPR_FLAG             :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_CU_GBI_MODE             :  return libVVCDEC_TYPE_RANGE;
        // --------- PU-based --------
        case libVVCDEC_PU_MERGE_FLAG           :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_PU_MERGE_INDEX          :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_PU_INTER_DIR            :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_PU_MVP_IDX_L0           :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_PU_MVP_IDX_L1           :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_PU_REFERENCE_POC_L0     :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_PU_MV_L0                :  return libVVCDEC_TYPE_VECTOR;
        case libVVCDEC_PU_REFERENCE_POC_L1     :  return libVVCDEC_TYPE_RANGE;
        case libVVCDEC_PU_MV_L1                :  return libVVCDEC_TYPE_VECTOR;
        // --------- TU-based --------
        case libVVCDEC_TU_CBF_Y                :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_TU_CBF_CB               :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_TU_CBF_CR               :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_TU_COEFF_TR_SKIP_Y      :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_TU_COEFF_TR_SKIP_Cb     :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_TU_COEFF_TR_SKIP_Cr     :  return libVVCDEC_TYPE_FLAG;
        case libVVCDEC_TU_EMT_MODE             :  return libVVCDEC_TYPE_RANGE;
    default: return libVVCDEC_TYPE_UNKNOWN;
    }
  }

  VVC_DEC_API unsigned int libVVCDEC_get_internal_type_max(unsigned int idx)
  {
    switch (idx)
    {
              // --------- CU-based --------
        case libVVCDEC_CTU_SLICE_INDEX         :  return 1000;
        case libVVCDEC_CU_INTRA_MODE_LUMA      :  return NUM_INTRA_MODE;
        case libVVCDEC_CU_INTRA_MODE_CHROMA    :  return NUM_INTRA_MODE;
        case libVVCDEC_CU_QT_DEPTH             :  return MAX_CU_DEPTH;
        case libVVCDEC_CU_BT_DEPTH             :  return MAX_BT_DEPTH;
        case libVVCDEC_CU_MT_DEPTH             :  return MAX_BT_DEPTH;
        case libVVCDEC_CU_AFFINE_MODE          :  return AFFINE_MODEL_NUM;
        case libVVCDEC_CU_QP                   :  return MAX_QP;
        case libVVCDEC_CU_EMT_MODE             :  return NUM_TRANS_TYPE;
#if VTM_BMS_COMPAT
        case libVVCDEC_CU_NSST_MODE            :  return 3;
        case libVVCDEC_CU_IMV_MODE             :  return NUM_IMV_MODES;
        case libVVCDEC_CU_GBI_MODE             :  return GBI_NUM;
#endif
        // --------- PU-based --------
        case libVVCDEC_PU_MERGE_INDEX          :  return MRG_MAX_NUM_CANDS;
        case libVVCDEC_PU_INTER_DIR            :  return 3;
        case libVVCDEC_PU_MVP_IDX_L0           :  return AMVP_MAX_NUM_CANDS;
        case libVVCDEC_PU_MVP_IDX_L1           :  return AMVP_MAX_NUM_CANDS;
        case libVVCDEC_PU_REFERENCE_POC_L0     :  return 100000;
        case libVVCDEC_PU_REFERENCE_POC_L1     :  return 100000;
        // --------- TU-based --------
        case libVVCDEC_TU_EMT_MODE             :  return NUM_TRANS_TYPE;

    default: return -1;
    }
  }

  VVC_DEC_API unsigned int libVVCDEC_get_internal_type_vector_scaling(unsigned int idx)
  {
    if (idx == libVVCDEC_PU_MV_L0 || idx == libVVCDEC_PU_MV_L1)
      return 4;
    return 1;
  }

  VVC_DEC_API const char *libVVCDEC_get_internal_type_description(unsigned int idx)
  {
    switch (idx)
    {
        // --------- CU-based --------
        case libVVCDEC_CTU_SLICE_INDEX         :  return "";
        case libVVCDEC_CU_PREDICTION_MODE      :  return "";
        case libVVCDEC_CU_TRQ_BYPASS           :  return "";
        case libVVCDEC_CU_SKIP_FLAG            :  return "";
        case libVVCDEC_CU_INTRA_MODE_LUMA      :  return "";
        case libVVCDEC_CU_INTRA_MODE_CHROMA    :  return "";
        case libVVCDEC_CU_ROOT_CBF             :  return "";
        case libVVCDEC_CU_QT_DEPTH             :  return "";
        case libVVCDEC_CU_BT_DEPTH             :  return "";
        case libVVCDEC_CU_MT_DEPTH             :  return "";
        case libVVCDEC_CU_AFFINE_FLAG          :  return "";
        case libVVCDEC_CU_AFFINE_MODE          :  return "";
        case libVVCDEC_CU_PDPC_FLAG            :  return "";
        case libVVCDEC_CU_IPCM_FLAG            :  return "";
        case libVVCDEC_CU_QP                   :  return "";
        case libVVCDEC_CU_EMT_MODE             :  return "";
        case libVVCDEC_CU_NSST_MODE            :  return "";
        case libVVCDEC_CU_LIC_FLAG             :  return "";
        case libVVCDEC_CU_IMV_MODE             :  return "";
        case libVVCDEC_CU_OBMC_FLAG            :  return "";
        case libVVCDEC_CU_CPR_FLAG             :  return "";
        case libVVCDEC_CU_GBI_MODE             :  return "";
        // --------- PU-based --------                  
        case libVVCDEC_PU_MERGE_FLAG           :  return "";
        case libVVCDEC_PU_MERGE_INDEX          :  return "";
        case libVVCDEC_PU_INTER_DIR            :  return "";
        case libVVCDEC_PU_MVP_IDX_L0           :  return "";
        case libVVCDEC_PU_MVP_IDX_L1           :  return "";
        case libVVCDEC_PU_REFERENCE_POC_L0     :  return "";
        case libVVCDEC_PU_MV_L0                :  return "";
        case libVVCDEC_PU_REFERENCE_POC_L1     :  return "";
        case libVVCDEC_PU_MV_L1                :  return "";
        // --------- TU-based --------                   "
        case libVVCDEC_TU_CBF_Y                :  return "";
        case libVVCDEC_TU_CBF_CB               :  return "";
        case libVVCDEC_TU_CBF_CR               :  return "";
        case libVVCDEC_TU_COEFF_TR_SKIP_Y      :  return "";
        case libVVCDEC_TU_COEFF_TR_SKIP_Cb     :  return "";
        case libVVCDEC_TU_COEFF_TR_SKIP_Cr     :  return "";
        case libVVCDEC_TU_EMT_MODE             :  return "";
        default: return "0";
    }
  }

  libVVCDec_info_types_dataTye getCUPUTUTypeForStat(unsigned int typeIdx)
  {
    switch (typeIdx)
    {
      // --------- CU-based --------
    case libVVCDEC_CTU_SLICE_INDEX:
    case libVVCDEC_CU_PREDICTION_MODE:
    case libVVCDEC_CU_TRQ_BYPASS:
    case libVVCDEC_CU_SKIP_FLAG:
    case libVVCDEC_CU_INTRA_MODE_LUMA:
    case libVVCDEC_CU_INTRA_MODE_CHROMA:
    case libVVCDEC_CU_ROOT_CBF:
    case libVVCDEC_CU_QT_DEPTH:
    case libVVCDEC_CU_BT_DEPTH:
    case libVVCDEC_CU_MT_DEPTH:
    case libVVCDEC_CU_AFFINE_FLAG:
    case libVVCDEC_CU_AFFINE_MODE:
    case libVVCDEC_CU_PDPC_FLAG:
    case libVVCDEC_CU_IPCM_FLAG:
    case libVVCDEC_CU_QP:
    case libVVCDEC_CU_EMT_MODE:
    case libVVCDEC_CU_NSST_MODE:
    case libVVCDEC_CU_LIC_FLAG:
    case libVVCDEC_CU_IMV_MODE:
    case libVVCDEC_CU_OBMC_FLAG:
    case libVVCDEC_CU_CPR_FLAG:
    case libVVCDEC_CU_GBI_MODE:
      return CUTypeStat;
    case libVVCDEC_PU_MERGE_FLAG:
    case libVVCDEC_PU_MERGE_INDEX:
    case libVVCDEC_PU_INTER_DIR:
    case libVVCDEC_PU_MVP_IDX_L0:
    case libVVCDEC_PU_MVP_IDX_L1:
    case libVVCDEC_PU_REFERENCE_POC_L0:
    case libVVCDEC_PU_MV_L0:
    case libVVCDEC_PU_REFERENCE_POC_L1:
    case libVVCDEC_PU_MV_L1:
      return PUTypeStat;
    case libVVCDEC_TU_CBF_Y:
    case libVVCDEC_TU_CBF_CB:
    case libVVCDEC_TU_CBF_CR:
    case libVVCDEC_TU_COEFF_TR_SKIP_Y:
    case libVVCDEC_TU_COEFF_TR_SKIP_Cb:
    case libVVCDEC_TU_COEFF_TR_SKIP_Cr:
    case libVVCDEC_TU_EMT_MODE:
      return TUTypeStat;
    default:
      return Other;
    }
  }

  bool addValuesForCU(VVCDecoderWrapper *d, CodingUnit* pcLCU, unsigned int typeIdx)
  {
    // Is there more space in the cache?
    if (d->internalsBlockDataFull())
      return false;

    libVVCDec_BlockValue b;
    b.x = pcLCU->lumaPos().x;
    b.y = pcLCU->lumaPos().y;
    b.w = pcLCU->lumaSize().width;
    b.h = pcLCU->lumaSize().height;
  
    if (getCUPUTUTypeForStat(typeIdx) == CUTypeStat)
    {
      if (typeIdx == libVVCDEC_CTU_SLICE_INDEX)
        b.value = pcLCU->slice->getIndependentSliceIdx();
      else if (typeIdx == libVVCDEC_CU_PREDICTION_MODE)
        b.value = pcLCU->predMode;
      else if (typeIdx == libVVCDEC_CU_TRQ_BYPASS)
        b.value = pcLCU->transQuantBypass;
      else if (typeIdx == libVVCDEC_CU_SKIP_FLAG)
        b.value = pcLCU->skip;
      else if (typeIdx == libVVCDEC_CU_INTRA_MODE_LUMA)
        b.value = pcLCU->firstPU->intraDir[CHANNEL_TYPE_LUMA];
      else if (typeIdx == libVVCDEC_CU_INTRA_MODE_CHROMA)
        b.value = pcLCU->firstPU->intraDir[CHANNEL_TYPE_CHROMA];
      else if (typeIdx == libVVCDEC_CU_ROOT_CBF)
        b.value = pcLCU->rootCbf;
      else if (typeIdx == libVVCDEC_CU_QT_DEPTH)
        b.value = pcLCU->qtDepth;
      else if (typeIdx == libVVCDEC_CU_BT_DEPTH)
        b.value = pcLCU->btDepth;
      else if (typeIdx == libVVCDEC_CU_MT_DEPTH)
        b.value = pcLCU->mtDepth;
      else if (typeIdx == libVVCDEC_CU_AFFINE_FLAG)
        b.value = pcLCU->affine;
      else if (typeIdx == libVVCDEC_CU_AFFINE_MODE)
        b.value = pcLCU->affineType;
#if VTM_BMS_COMPAT
      else if (typeIdx == libVVCDEC_CU_PDPC_FLAG)
        b.value = pcLCU->pdpc;
#endif
      else if (typeIdx == libVVCDEC_CU_IPCM_FLAG)
        b.value = pcLCU->ipcm;
      else if (typeIdx == libVVCDEC_CU_QP)
        b.value = pcLCU->qp;
      else if (typeIdx == libVVCDEC_CU_EMT_MODE)
        b.value = pcLCU->emtFlag;
#if VTM_BMS_COMPAT
      else if (typeIdx == libVVCDEC_CU_NSST_MODE)
        b.value = pcLCU->nsstIdx;
      else if (typeIdx == libVVCDEC_CU_LIC_FLAG)
        b.value = pcLCU->LICFlag;
      else if (typeIdx == libVVCDEC_CU_IMV_MODE)
        b.value = pcLCU->imv;
      else if (typeIdx == libVVCDEC_CU_OBMC_FLAG)
        b.value = pcLCU->obmcFlag;
      else if (typeIdx == libVVCDEC_CU_CPR_FLAG)
        b.value = pcLCU->ibc;
      else if (typeIdx == libVVCDEC_CU_GBI_MODE)
        b.value = pcLCU->GBiIdx;
#endif
      d->addInternalsBlockData(b);
      return true;
    }
    else if (getCUPUTUTypeForStat(typeIdx) == PUTypeStat)
    {
      for (auto& pu : CU::traversePUs(*pcLCU))
      {
        if (d->internalsBlockDataFull())
          return false;
        libVVCDec_BlockValue b;
        b.x = pu.lumaPos().x;
        b.y = pu.lumaPos().y;
        b.w = pu.lumaSize().width;
        b.h = pu.lumaSize().height;

        if (typeIdx == libVVCDEC_PU_MERGE_FLAG)
          b.value = pu.mergeFlag;
        else if (typeIdx == libVVCDEC_PU_MERGE_INDEX)
          b.value = pu.mergeIdx;
        else if (typeIdx == libVVCDEC_PU_INTER_DIR)
          b.value = pu.interDir;
        else if (typeIdx == libVVCDEC_PU_MVP_IDX_L0)
          b.value = pu.mvpIdx[REF_PIC_LIST_0];
        else if (typeIdx == libVVCDEC_PU_MVP_IDX_L1)
          b.value = pu.mvpIdx[REF_PIC_LIST_1];
        else if (typeIdx == libVVCDEC_PU_REFERENCE_POC_L0)
          b.value = pu.refIdx[REF_PIC_LIST_0];
        else if (typeIdx == libVVCDEC_PU_MV_L0)
        {
          b.value  = pu.mv[REF_PIC_LIST_0].getHor();
          b.value2 = pu.mv[REF_PIC_LIST_0].getVer();
        }
        else if (typeIdx == libVVCDEC_PU_REFERENCE_POC_L1)
          b.value = pu.refIdx[REF_PIC_LIST_1];
        else if (typeIdx == libVVCDEC_PU_MV_L1)
        {
          b.value  = pu.mv[REF_PIC_LIST_1].getHor();
          b.value2 = pu.mv[REF_PIC_LIST_1].getVer();
        }
        d->addInternalsBlockData(b);
      }
      return true;
    }
    else if (getCUPUTUTypeForStat(typeIdx) == TUTypeStat)
    {
      for (auto& tu : CU::traverseTUs(*pcLCU))
      {
        if (d->internalsBlockDataFull())
          return false;
        libVVCDec_BlockValue b;
        b.x = tu.lumaPos().x;
        b.y = tu.lumaPos().y;
        b.w = tu.lumaSize().width;
        b.h = tu.lumaSize().height;

        if (typeIdx == libVVCDEC_TU_CBF_Y)
          b.value = tu.cbf[COMPONENT_Y];
        else if (typeIdx == libVVCDEC_TU_CBF_CB)
          b.value = tu.cbf[COMPONENT_Cb];
        else if (typeIdx == libVVCDEC_TU_CBF_CR)
          b.value = tu.cbf[COMPONENT_Cr];
        else if (typeIdx == libVVCDEC_TU_COEFF_TR_SKIP_Y)
          b.value = tu.transformSkip[COMPONENT_Y];
        else if (typeIdx == libVVCDEC_TU_COEFF_TR_SKIP_Cb)
          b.value = tu.transformSkip[COMPONENT_Cb];
        else if (typeIdx == libVVCDEC_TU_COEFF_TR_SKIP_Cr)
          b.value = tu.transformSkip[COMPONENT_Cr];
        else if (typeIdx == libVVCDEC_TU_EMT_MODE)
          b.value = tu.emtIdx;
        d->addInternalsBlockData(b);
      }
      return true;
    }
    // never reach this
    return true;
  }

  VVC_DEC_API libVVCDec_BlockValue *libVVCDEC_get_internal_info(libVVCDec_context *decCtx, libVVCDec_picture *pic, unsigned int typeIdx, unsigned int &nrValues, bool &callAgain)
  {
    nrValues = 0;
    callAgain = false;

    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return NULL;

    // Clear the internals before adding new ones
    d->clearInternalsBlockData();

    if (pic == NULL)
      return NULL;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return NULL;
    CodingStructure* cs = pcPic->cs;
    if (cs == NULL)
      return NULL;

    if (d->pauseInternalsCUIdx != -1)
    {
      d->pauseInternalsCUIdx = -1;
    }
    
    if (pcPic->getPOC() != d->lastStatsPOC)
    {
      // we start collecting data for a new POC
      d->parsedCUs.clear();
      d->lastStatsPOC = pcPic->getPOC();
    }

    const int maxNumChannelType = cs->pcv->chrFormat != CHROMA_400 && CS::isDualITree(*cs) ? 1 : 1; // for now, only parse the Luma Tree

    for (int ch = 0; ch < maxNumChannelType; ch++)
    {
      const ChannelType chType = ChannelType(ch);

      for (auto &currCU : cs->traverseCUs(CS::getArea(*cs, cs->area, chType), chType))
      {
        // don't add those CUs again, that we already parsed
        if (d->parsedCUs.count(currCU.idx) > 0)
          continue;
        if (!addValuesForCU(d, &currCU, typeIdx))
        {
          // Cache is full. Remember the CU index so we can continue from here.
          d->pauseInternalsCUIdx = currCU.idx;
          d->parsedCUs.insert(std::make_pair(currCU.idx, true));
          nrValues = d->internalsBlockDataValues;
          callAgain = true;
          return d->internalsBlockData;
        }
      }
    }
    // Processing of all values is finished. The cache is not full.
    nrValues = d->internalsBlockDataValues;
    callAgain = false;
    return d->internalsBlockData;
  }

  VVC_DEC_API libVVCDec_error libVVCDEC_clear_internal_info(libVVCDec_context *decCtx)
  {
    VVCDecoderWrapper *d = (VVCDecoderWrapper*)decCtx;
    if (!d)
      return libVVCDEC_ERROR;

    // Clear the internals
    d->clearInternalsBlockData();

    return libVVCDEC_OK;
  }

} // extern "C"
