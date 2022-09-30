
#ifndef     __MEMORY_H__
#define     __MEMORY_H__

typedef struct _MV_MEMORYPAGELIST_
{
    unsigned long  PageCount;       // page的数量
    void**         PageList;        // page的列表指针
    unsigned long  OffsetPage;      // Buf在首个page中的偏移量
    void*          KernelAddress;   // 内核态中的逻辑地址
    void*          UserAddress;     // 用户态中的逻辑地址
    unsigned int   Size;            // Buf的总大小
    unsigned long  hMm;              // 所属进程内存空间
} MV_MEMORYPAGELIST;


#ifdef __cplusplus
extern "C"
{
#endif 

//#include "misc.h"

typedef struct __TEST__
{
    int a;
    int b;
}MV_TEST;

extern void* MV_KMalloc(int aSize);
extern void* MV_KMallocAtomic(int aSize);
extern void  MV_KFree( void* aAddress );
extern void* MV_VMalloc(int aSize);
extern void  MV_VFree(void* aAddress);
extern void  MV_Memset(void* aAddress, unsigned int aValue, int aSize);
extern void  MV_Memcpy(void* aDestination, const void* aSource, int aSize);
extern unsigned int  MV_MemcpyToUser( void* aTo, const void* aFrom, int aSize );
extern unsigned int  MV_MemcpyFromUser(void* aTo, const void* aFrom, int aSize );
extern unsigned int  test(MV_TEST myTest, int e);
extern int MV_MallocPages(MV_MEMORYPAGELIST* pPageList, unsigned int nSize);
extern int MV_FreePages(MV_MEMORYPAGELIST* pPageList);
extern unsigned int MV_GetVmaSize(void* vma);

#ifdef __cplusplus
}
#endif 


#endif		//	__MEMORY_H__

