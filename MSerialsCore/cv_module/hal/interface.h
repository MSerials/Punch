#ifndef _MSERIALS_CORE_HAL_INTERFACE_
#define	_MSERIALS_CORE_HAL_INTERFACE_


#define IMAGE_HAL_ERROR_UNKNOWN -1

//ͨ���仯
#define IMAGE_CN_MAX     512
#define IMAGE_CN_SHIFT   3
#define IMAGE_DEPTH_MAX  (1 << IMAGE_CN_SHIFT)

#define IMAGE_8U		0
#define IMAGE_8S		1
#define IMAGE_16U		2
#define IMAGE_16S		3
#define IMAGE_32S		4
#define IMAGE_32F		5
#define IMAGE_64F		6
#define IMAGE_USRTYPE1	7

#define IMAGE_MAT_DEPTH_MASK		(IMAGE_DEPTH_MAX-1)
#define IMAGE_MAT_DEPTH(flags)		((flags) & IMAGE_MAT_DEPTH_MASK)

#define IMAGE_MAKETYPE(depth,cn)	(IMAGE_MAT_DEPTH(depth) + (((cn)-1)<<IMAGE_CN_SHIFT))
#define IMAGE_MAKE_TYPE IMAGE_MAKETYPE

#define IMAGE_8UC1 IMAGE_MAKETYPE(IMAGE_8U,1)
#define IMAGE_8UC2 IMAGE_MAKETYPE(IMAGE_8U,2)
#define IMAGE_8UC3 IMAGE_MAKETYPE(IMAGE_8U,3)
#define IMAGE_8UC4 IMAGE_MAKETYPE(IMAGE_8U,4)
#define IMAGE_8UC(n) IMAGE_MAKETYPE(IMAGE_8U,(n))

#define IMAGE_8SC1 IMAGE_MAKETYPE(IMAGE_8S,1)
#define IMAGE_8SC2 IMAGE_MAKETYPE(IMAGE_8S,2)
#define IMAGE_8SC3 IMAGE_MAKETYPE(IMAGE_8S,3)
#define IMAGE_8SC4 IMAGE_MAKETYPE(IMAGE_8S,4)
#define IMAGE_8SC(n) IMAGE_MAKETYPE(IMAGE_8S,(n))

#define IMAGE_16UC1 IMAGE_MAKETYPE(IMAGE_16U,1)
#define IMAGE_16UC2 IMAGE_MAKETYPE(IMAGE_16U,2)
#define IMAGE_16UC3 IMAGE_MAKETYPE(IMAGE_16U,3)
#define IMAGE_16UC4 IMAGE_MAKETYPE(IMAGE_16U,4)
#define IMAGE_16UC(n) IMAGE_MAKETYPE(IMAGE_16U,(n))

#define IMAGE_16SC1 IMAGE_MAKETYPE(IMAGE_16S,1)
#define IMAGE_16SC2 IMAGE_MAKETYPE(IMAGE_16S,2)
#define IMAGE_16SC3 IMAGE_MAKETYPE(IMAGE_16S,3)
#define IMAGE_16SC4 IMAGE_MAKETYPE(IMAGE_16S,4)
#define IMAGE_16SC(n) IMAGE_MAKETYPE(IMAGE_16S,(n))

#define IMAGE_32SC1 IMAGE_MAKETYPE(IMAGE_32S,1)
#define IMAGE_32SC2 IMAGE_MAKETYPE(IMAGE_32S,2)
#define IMAGE_32SC3 IMAGE_MAKETYPE(IMAGE_32S,3)
#define IMAGE_32SC4 IMAGE_MAKETYPE(IMAGE_32S,4)
#define IMAGE_32SC(n) IMAGE_MAKETYPE(IMAGE_32S,(n))

#define IMAGE_32FC1 IMAGE_MAKETYPE(IMAGE_32F,1)
#define IMAGE_32FC2 IMAGE_MAKETYPE(IMAGE_32F,2)
#define IMAGE_32FC3 IMAGE_MAKETYPE(IMAGE_32F,3)
#define IMAGE_32FC4 IMAGE_MAKETYPE(IMAGE_32F,4)
#define IMAGE_32FC(n) IMAGE_MAKETYPE(IMAGE_32F,(n))

#define IMAGE_64FC1 IMAGE_MAKETYPE(IMAGE_64F,1)
#define IMAGE_64FC2 IMAGE_MAKETYPE(IMAGE_64F,2)
#define IMAGE_64FC3 IMAGE_MAKETYPE(IMAGE_64F,3)
#define IMAGE_64FC4 IMAGE_MAKETYPE(IMAGE_64F,4)
#define IMAGE_64FC(n) IMAGE_MAKETYPE(IMAGE_64F,(n))

#endif