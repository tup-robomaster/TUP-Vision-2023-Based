
#include "mv_memorymap.h"

#include <asm/current.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/gfp.h>


int MV_GetUserPages( long* aContext, MV_MEMORYPAGELIST* aMap, void* aUserAddress, unsigned int aSize )
{

    if (NULL == aMap || NULL == aUserAddress)
    {
        return -1;
    }
    
    int lPageCount              = 0;
    struct page** lPageList     = NULL;
    unsigned int i              = 0;
    unsigned long lFirstPage    = 0;
    unsigned long lLastPage     = 0;
    unsigned long nReqPageCount = 0;

    // 不做此处理
    // OS_UNREFERENCED_PARAMETER( aContext );

    aMap->UserAddress = NULL;

    //     if( unlikely( !aUserAddress 
    //         || !aSize
    //         || !access_ok( VERIFY_WRITE, aUserAddress, aSize ) ) )
    //     {
    //         return OS_RESULT_INVALID_ARGUMENT;
    //     }

    lFirstPage = ( ( unsigned long ) aUserAddress ) >> PAGE_SHIFT;
    lLastPage = ( ( ( unsigned long ) aUserAddress ) + ( unsigned long ) aSize ) >> PAGE_SHIFT;
    nReqPageCount = lLastPage - lFirstPage + 1; 

    lPageList = (struct page**)kmalloc( sizeof( struct page* ) * nReqPageCount, GFP_KERNEL );
    if( !lPageList )
    {
        return -1;
    }

    
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
    down_read( &current->mm->mmap_sem );
    lPageCount = get_user_pages( current, current->mm, ( ( unsigned long ) aUserAddress ) & PAGE_MASK,
        nReqPageCount, 1, 0, lPageList, NULL );
    up_read( &current->mm->mmap_sem );
#else
    lPageCount = get_user_pages_fast( ( ( unsigned long ) aUserAddress ) & PAGE_MASK, nReqPageCount,
        1, lPageList );
#endif 
    
    if( lPageCount != ( int ) nReqPageCount )
    {
        for( i = 0; i < ( unsigned int ) lPageCount; i++ )
        {
            put_page( lPageList[ i ] );
        }

        kfree( lPageList );
        return -1;
    }

    aMap->PageList = (void*)lPageList;
    aMap->PageCount = nReqPageCount;
    aMap->UserAddress = aUserAddress;
    aMap->Size = aSize;
    aMap->OffsetPage = offset_in_page( aUserAddress );
    return 0;
}


int MV_ReleaseUserPages(long* aContext, MV_MEMORYPAGELIST* aMap)
{
    unsigned int i          = 0;
    struct page* lPage      = NULL;
    struct page** lPageList = ( struct page** ) aMap->PageList;

    if (0 == aMap)
    {
        return -1;
    }

    if (0 == aMap->PageList)
    {
        return -1;
    }

    for( i = 0; i < aMap->PageCount; i++ )
    {
        lPage = lPageList[ i ];
        if( !PageReserved( lPage ) )
        {
            SetPageDirty( lPage );
        }  
        put_page( lPage ); 
    }        
    kfree( lPageList );

    return 0;
}

//同一时刻可以映射多个不会被覆盖，但是V2.6.0上需要自己配置km_type类型，应该设计User0，User1交替机制
inline void* MV_KMap_Atomic( long* aContext, void* pPage)
{
    if (!pPage)
    {
        return (void*)0;
    }

    return kmap_atomic((struct page*)pPage);
}

inline void MV_KUnmap_Atomic(long* aContext, void* pkAddr)
{
    if (!pkAddr)
    {
        return;
    }

    kunmap_atomic( ( void* ) ( ( ( unsigned long ) pkAddr ) & PAGE_MASK ));
}

inline void* MV_VMap(long* aContext, void** pPageList, unsigned long nPageCount)
{
    if (!pPageList)
    {
        return (void*)0;
    }

    return vmap(( struct page** ) pPageList, nPageCount, VM_MAP, PAGE_KERNEL);
}

inline void MV_VUnmap(long* aContext, void* pkAddr)
{
    if (!pkAddr)
    {
        return;
    }
    vunmap(( void* ) (((unsigned long)pkAddr) & PAGE_MASK ) );
}

inline void* MV_KMap( long* aContext, void* pPage)
{
    if (!pPage)
    {
        return (void*)0;
    }

    return kmap((struct page*)pPage);
}

inline void MV_KUnmap(long* aContext, void* pkAddr)
{
    if (!pkAddr)
    {
        return;
    }

    kunmap( ( void* ) ( ( ( unsigned long ) pkAddr ) & PAGE_MASK ));
}

inline unsigned long MV_OffsetInPage(void* puAddr)
{
    if (!puAddr)
    {
        return 0;
    }
    return offset_in_page( puAddr );
}

inline unsigned long MV_PageShift()
{
    return PAGE_SHIFT;
}

inline unsigned long MV_PageSize()
{
    return PAGE_SIZE;
}


int MV_MapCreate( long* aContext, MV_MEMORYPAGELIST* aMap, 
    void* aUserAddress, unsigned int  aSize )
{
    int lPageCount            = 0;
    struct page** lPageList   = NULL;
    void* lKernelPageAddress  = NULL;
    unsigned int i            = 0;
    unsigned long lFirstPage  = 0;
    unsigned long lLastPage   = 0;

    // 不做此处理
    // OS_UNREFERENCED_PARAMETER( aContext );

    aMap->UserAddress = NULL;
    aMap->KernelAddress = NULL; 

//     if( unlikely( !aUserAddress 
//         || !aSize
//         || !access_ok( VERIFY_WRITE, aUserAddress, aSize ) ) )
//     {
//         return OS_RESULT_INVALID_ARGUMENT;
//     }

    lFirstPage = ( ( unsigned long ) aUserAddress ) >> PAGE_SHIFT;
    lLastPage = ( ( ( unsigned long ) aUserAddress ) + ( unsigned long ) aSize ) >> PAGE_SHIFT;
    aMap->PageCount = lLastPage - lFirstPage + 1; 

    aMap->PageList = kmalloc( sizeof( struct page* ) * aMap->PageCount, GFP_KERNEL );
    if( !aMap->PageList )
    {
        return -1;
    }
    lPageList = ( struct page** ) aMap->PageList;


#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
    down_read( &current->mm->mmap_sem );
    lPageCount = get_user_pages( current, current->mm, ( ( unsigned long ) aUserAddress ) & PAGE_MASK,
        aMap->PageCount, 1, 0, lPageList, NULL );
    up_read( &current->mm->mmap_sem );
#else
    lPageCount = get_user_pages_fast( ( ( unsigned long ) aUserAddress ) & PAGE_MASK, aMap->PageCount,
        1, lPageList );
#endif 
    if( lPageCount != ( int ) aMap->PageCount )
    {
        for( i = 0; i < ( unsigned int ) lPageCount; i++ )
        {
            put_page( lPageList[ i ] );
        }
        kfree( lPageList );
        return -1;
    }
    
    lKernelPageAddress = vmap( ( struct page** ) aMap->PageList, aMap->PageCount, VM_MAP, PAGE_KERNEL );
    if( !lKernelPageAddress )
    {
        for( i = 0; i < aMap->PageCount; i++ )
        {
             put_page( lPageList[ i ] );
        }
        kfree( lPageList );
        return -1;
    }

    aMap->UserAddress = aUserAddress;
    aMap->KernelAddress = ( void* ) ( ( ( unsigned long ) lKernelPageAddress ) 
        + offset_in_page( aUserAddress ) );
    aMap->Size = aSize;

    return 0;
}

int MV_MapDelete( MV_MEMORYPAGELIST* aMap )
{
    unsigned int i          = 0;
    struct page* lPage      = NULL;
    struct page** lPageList = ( struct page** ) aMap->PageList;
    

    vunmap( ( void* ) ( ( ( unsigned long ) aMap->KernelAddress ) & PAGE_MASK ) );

    for( i = 0; i < aMap->PageCount; i++ )
    {
        lPage = lPageList[ i ];
        if( !PageReserved( lPage ) )
        {
            SetPageDirty( lPage );
        }  
        put_page( lPage ); 
    }        
    kfree( lPageList );
    return 0;
}

int MV_MapUserBuffer(MV_MEMORYPAGELIST* aMap, void* vma)
{
    struct vm_area_struct * pVma = (struct vm_area_struct *)vma;
    unsigned int nSize = PAGE_ALIGN(pVma->vm_end - pVma->vm_start);
    unsigned long pUserAddr = pVma->vm_start;
    int i = 0;

    if (aMap->PageCount < (nSize >> PAGE_SHIFT))
    {
        printk("GigEDriver: vma is too big\n");
        return -1;
    }

    for (i = 0; i < aMap->PageCount; i++)
    {
        int nRet = 0;

#if ( LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0))
        nRet = vm_insert_page(pVma, pUserAddr + i*PAGE_SIZE, aMap->PageList[i]);
#else
        nRet = remap_page_range(pVma, pUserAddr + i*PAGE_SIZE, (unsigned long)aMap->PageList[i], PAGE_SHARED);
#endif
        if (0 != nRet)
        {
            printk("GigEDriver: remap_page_range failed ,nRet[%d], i[%d]\n", nRet, i);
            return -1;
        }
        
    }

    aMap->UserAddress = pVma->vm_start;
    aMap->Size = pVma->vm_end - pVma->vm_start;
    aMap->OffsetPage = offset_in_page( pVma->vm_start );
    aMap->hMm         = (void*)pVma->vm_mm;
    pVma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;


    return 0;

}

