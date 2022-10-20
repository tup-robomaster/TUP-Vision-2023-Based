
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/gfp.h>

#include "mv_memory.h"
#include "MvErrorDefine.h"

/*******************************************************************************
@   函数原型：void* MV_KMalloc(int aSize)
@   函数功能：kmalloc函数封装
@   参数aSize： 需要kmalloc的大小
@   返回值：成功分配内存首地址或失败NULL
*******************************************************************************/
void* MV_KMalloc(int aSize)
{
    return kmalloc(aSize, GFP_KERNEL);
}

/*******************************************************************************
@   函数原型：void* MV_KMallocAtomic(int aSize)
@   函数功能：kmalloc函数封装
@   参数aSize： 需要kmalloc的大小
@   返回值：成功分配内存首地址或失败NULL
*******************************************************************************/
void* MV_KMallocAtomic(int aSize)
{
    return kmalloc(aSize, GFP_ATOMIC);
}


/*******************************************************************************
@   函数原型：void MV_KFree(void* aAddress)
@   函数功能：kfree函数封装
@   参数aAddress： 需要kfree的内存首地址
@   返回值：无
*******************************************************************************/
void MV_KFree(void* aAddress)
{
    if(NULL == aAddress)
    {
        return;
    }
    kfree(aAddress);
}

/*******************************************************************************
@   函数原型：void* MV_VMalloc(int aSize)
@   函数功能：vmalloc函数封装
@   参数aSize： 需要vmalloc的大小
@   返回值：成功分配内存首地址或失败NULL
*******************************************************************************/
void* MV_VMalloc(int aSize)
{

    return vmalloc(aSize);
}

/*******************************************************************************
@   函数原型：void MV_VFree(void* aAddress)
@   函数功能：vfree函数封装
@   参数aAddress： 需要vfree的内存首地址
@   返回值：无
*******************************************************************************/
void MV_VFree(void* aAddress)
{
    if(NULL == aAddress)
    {
        return;
    }
    vfree(aAddress);
}

/*******************************************************************************
@   函数原型：void MV_Memset(void* aAddress, unsigned int aValue, int aSize)
@   函数功能：memset函数封装
@   参数aAddress: 需要memset的内存首地址
@   参数aValue: 需要memset的值
@   参数aSize: 需要memset的大小
@   返回值：无
*******************************************************************************/
void MV_Memset(void* aAddress, unsigned int aValue, int aSize)
{
    if(NULL == aAddress)
    {
        return;
    }
    memset(aAddress, aValue, aSize);
}

/*******************************************************************************
@   函数原型：void MV_Memcpy(void* aDestination, const void* aSource, int aSize)
@   函数功能：memcpy函数封装
@   参数aDestination: 拷贝目的地址
@   参数aSource: 拷贝原地址
@   参数aSize: 拷贝字节数
@   返回值：无
*******************************************************************************/
void MV_Memcpy(void* aDestination, const void* aSource, int aSize)
{
    if(NULL == aDestination || NULL == aSource)
    {
        return;
    }
    memcpy(aDestination, aSource, aSize);
}

/*******************************************************************************
@   函数原型：unsigned int MV_MemcpyToUser(void* aTo, const void* aFrom, int aSize)
@   函数功能：copy_to_user函数封装
@   参数aTo: 拷贝到用户层的目的地址
@   参数aFrom: 拷贝原地址
@   参数aSize: 拷贝字节数
@   返回值：成功返回0 失败返回MV_E_PARAMETER
*******************************************************************************/
unsigned int MV_MemcpyToUser(void* aTo, const void* aFrom, int aSize)
{
    if(NULL == aTo || NULL == aFrom)
    {
        return MV_E_PARAMETER;
    }
    return copy_to_user(aTo, aFrom, (unsigned long)aSize);
}

/*******************************************************************************
@   函数原型：unsigned int MV_MemcpyFromUser(void* aTo, const void* aFrom, int aSize )
@   函数功能：copy_from_user函数封装
@   参数aTo: 用户层拷贝到内核的目的地址
@   参数aFrom: 用户层拷贝原地址
@   参数aSize: 拷贝字节数
@   返回值：成功返回0 失败返回MV_E_PARAMETER
*******************************************************************************/
unsigned int MV_MemcpyFromUser(void* aTo, const void* aFrom, int aSize )
{
    aFrom = aFrom;
    aSize = aSize;
    //printk("aTo[%p] aFrom[%p] size[%d]\n", aTo, aFrom, aSize);
    if(NULL == aTo || NULL == aFrom)
    {
        return MV_E_PARAMETER;
    }

    return copy_from_user(aTo, aFrom, ( unsigned long )aSize);
}

unsigned int test(MV_TEST myTest, int e)
{
//     a = a;
//     b = b;
//     int *p;
//     printk("p:%d\n", sizeof(p));
//     printk("add2 a:%p b:%p\n", &a, &b);
//     printk("a[%d], b[%d]\n", a, *(&b+8));
    printk("myTest[%d][%d]\n", myTest.b/*, myTest.b*/, e);
    return 0;
}


int MV_MallocPages(MV_MEMORYPAGELIST* pPageList, unsigned int nSize)
{
    unsigned int nPageCount = PAGE_ALIGN(nSize) >> PAGE_SHIFT;
    struct page** lPageList     = NULL;
    int i = 0;
    const gfp_t alloc_mask = GFP_KERNEL | __GFP_NOWARN;
    const gfp_t highmem_mask = __GFP_HIGHMEM;
    struct page* pPage = NULL;

    lPageList = (struct page**)kmalloc( sizeof( struct page* ) * nPageCount, GFP_KERNEL );
    if( !lPageList )
    {
        printk("GigEDriver: kmalloc PageList failed\n");
        return -1;
    }

    for (i = 0; i < nPageCount; i++)
    {
        pPage = alloc_page(alloc_mask | highmem_mask);
        if (NULL == pPage)
        {
            printk("GigEDriver: alloc_page failed, i = [%d]\n", i);
            break;
        }
        lPageList[i] = pPage;
    }

    if (i != nPageCount)
    {
        for (; i >=0; i--)
        {
            __free_page(lPageList[i]);
            lPageList[i] = NULL;
        }

        kfree(lPageList);

        return -1;
    }

    pPageList->PageList = (void**)lPageList;
    pPageList->PageCount = nPageCount;

    return 0;
}

unsigned int MV_GetVmaSize(void* vma)
{

    if (NULL == vma)
    {
        return 0;
    }

    struct vm_area_struct * pVma = (struct vm_area_struct *)vma;

    return pVma->vm_end - pVma->vm_start;
}

int MV_FreePages(MV_MEMORYPAGELIST* pPageList)
{
    int i = 0;
    if (NULL == pPageList)
    {
        return -1;
    }

    if (NULL == pPageList->PageList)
    {
        return -1;
    }

    for (i = 0; i < pPageList->PageCount; i++)
    {
        if (pPageList->PageList[i])
        {
            __free_page(pPageList->PageList[i]);
            pPageList->PageList[i] = NULL;
        }
    }

    kfree(pPageList->PageList);
    pPageList->PageList = NULL;
    pPageList->PageCount = 0;

    return 0;
}
