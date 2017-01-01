/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlmath.h 5629 2011-06-11 03:13:08Z mcaramello $ 
 * 
 *******************************************************************************/

#ifndef _ML_MATH_H_
#define	_ML_MATH_H_

#ifndef MLMATH
// This define makes Microsoft pickup things like M_PI
#define _USE_MATH_DEFINES
#include <math.h>
#endif // MLMATH

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef ABS
#define ABS(x) (((x)>=0)?(x):-(x))
#endif

#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
#define MAX(x,y) (((x)>(y))?(x):(y))
#endif

/*---------------------------*/
#endif /* !_ML_MATH_H_ */
