
#pragma once


#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"
#include "QuantRDOQ.h"



#if JVET_K0072


class DepQuant : public QuantRDOQ
{
public:
  DepQuant( const Quant* other, bool enc );
  virtual ~DepQuant();

  virtual void quant  ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );
  virtual void dequant( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP );

private:
  void* p;
};


#endif

