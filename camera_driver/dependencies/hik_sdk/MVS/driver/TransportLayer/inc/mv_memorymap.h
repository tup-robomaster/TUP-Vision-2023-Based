

#ifndef __MV_MEMORYMAP_H__
#define __MV_MEMORYMAP_H__

#include "mv_memory.h"  

static inline void* MV_MapGetKernelAddress( MV_MEMORYPAGELIST* aMap )
{
    return aMap->KernelAddress;
}

static inline void* MV_MapGetUserAddress( MV_MEMORYPAGELIST* aMap )
{
    return aMap->UserAddress;
}

static inline unsigned int MV_MapGetSize( MV_MEMORYPAGELIST* aMap )
{
    return aMap->Size;
}

 /*static inline*/ int MV_MapCreate( long* aContext, MV_MEMORYPAGELIST* aMap, void* aUserAddress, unsigned int aSize );
// static inline int MV_MapDelete( MV_MEMORYMAP* aMap );

// static inline int MV_MapCopyTo( MV_MEMORYMAP* aMap, unsigned int aMapOffset, void* aSourceAddress, unsigned int aSourceSize )
// {
// 	memcpy( ( ( unsigned char* ) aMap->KernelAddress ) + aMapOffset, aSourceAddress, aSourceSize );	
// 	return 0;
// }

 int MV_GetUserPages( long* aContext, MV_MEMORYPAGELIST* aMap, void* aUserAddress, unsigned int aSize );
 int MV_ReleaseUserPages(long* aContext, MV_MEMORYPAGELIST* aMap);
 void* MV_KMap_Atomic( long* aContext, void* pPage);
 void MV_KUnmap_Atomic(long* aContext, void* pkAddr);
 void* MV_VMap(long* aContext, void** pPageList, unsigned long nPageCount);
 void MV_VUnmap(long* aContext, void* pkAddr);
 void* MV_KMap( long* aContext, void* pPage);
 void MV_KUnmap(long* aContext, void* pkAddr);
 unsigned long MV_OffsetInPage(void* puAddr);
 unsigned long MV_PageShift(void);
 unsigned long MV_PageSize(void);
 int MV_MapUserBuffer(MV_MEMORYPAGELIST* aMap, void* vma);


#endif // __MV_MEMORYMAP_H__
