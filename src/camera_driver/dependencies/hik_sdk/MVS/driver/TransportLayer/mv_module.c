
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/ioctl.h>
#include <linux/mm.h>

#include <linux/uaccess.h>

#include <linux/blkdev.h>

#include <linux/netfilter.h>  
#include <linux/netfilter_ipv4.h>
#include <linux/skbuff.h>
#include <linux/udp.h>  
#include <linux/ip.h>
#include <linux/slab.h>
#include <linux/types.h>  
#include <linux/string.h>
//#include <asm/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/delay.h>

#include <linux/kthread.h>

#include <linux/utsname.h>
#include <linux/version.h>

//#include <generated/autoconf.h> 

#include "mv_module.h"
#include "mv_misc.h"
#include "mv_lock.h"
#include "mv_memory.h"
#include "mv_netpacket.h"
#include "mv_nettypes.h"
#include "mv_device.h"

#include "MvErrorDefine.h"

/*udp hook函数的初始化变量*/
static struct nf_hook_ops g_sNetfilterHook = { 0 };   //net filter hook option struct

/*读写模块的全局变量*/
extern struct file_operations s_fileOperations;
struct cdev *s_devChar   = NULL;
dev_t s_devMajor;
static struct device *s_devNode = NULL;
static int s_nParamMajor = 0;
static struct class *s_classForDev  = NULL; //设备类，用于设备文件的创建

int MV_ModuleOpen(struct inode * node, struct file * file)
{
    //file->private_data = 0;
    //MV_Printk("Enter MV_ModuleOpen\n");

    return MV_OK;
}

static int MV_ModuleRelease( struct inode* aNode, struct file* aFile )
{  
    int nRet = MV_OK;

    //MV_Printk("Enter MV_ModuleRelease\n");
    ModuleRelease( (void*)aFile );

    aFile->private_data = 0;


    return nRet;
}

/*******************************************************************************
@   函数原型：static ssize_t simple_write(struct file *filp, const char *buffer, size_t count, loff_t *ppos)
@   函数功能：用户层向底层写数据(系统函数，格式固定)
@   参数filp： 未使用
@   参数buffer： 上层向底层写入的数据
@   参数count： 上层向底层写入的数据字节数
@   参数ppos： 未使用
@   返回值：写入字节数或错误码
*******************************************************************************/
static ssize_t MV_ModuleWrite(struct file *filp, const char *buffer, size_t count, loff_t *ppos)
{
    if(NULL == buffer)
    {
        printk("write buf is NULL\n");
        return MV_E_PARAMETER;
    }

    return ModuleWrite((void*)filp, buffer, count);
}

/*******************************************************************************
@   函数原型：long  simple_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long        arg)
@   函数功能：用户层向底层写数据(系统函数，格式固定)
@   参数inode： 未使用
@   参数filp： 未使用
@   参数cmd： 上层向底层写入的数据字节数
@   参数arg： 上层向底层写入的数据
@   返回值：写入字节数或错误码
*******************************************************************************/
#if HAVE_UNLOCKED_IOCTL
static long MV_ModuleUnlockedIoctl( struct file* filp, unsigned int cmd, unsigned long arg )
#else
static long  MV_ModuleIoctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
 
     //MV_USER_CONFIG stInputConfig = {0};
     //int nRet = copy_from_user(&stInputConfig, (MV_USER_CONFIG*)arg, sizeof(MV_USER_CONFIG)); 
     //MV_Printk("nret [%d]\n", nRet);
 
     //MV_Printk("MV_ModuleIoctl buf: flag[%x] srcip[%x] dstip[%x] paksize[%d] MemMapSize[%d]\n", 
     //    stInputConfig.flag, stInputConfig.nFilterSrcIp, stInputConfig.nFilterDstPort, stInputConfig.nGvspPacketSize, stInputConfig.nMemoryMapSize);
     //return 0;

     //MV_USER_CONFIG *pstArg = (MV_USER_CONFIG*)arg;
     //MV_Printk("test: flag[%x] srcip[%x] dstip[%x] paksize[%d] MemMapSize[%d]\n", 
     //    pstArg->flag, pstArg->nFilterSrcIp, pstArg->nFilterDstPort, pstArg->nGvspPacketSize, pstArg->nMemoryMapSize);

    
    return ModuleIoctl(( void* ) filp, cmd, (void *)arg);
}

int MV_ModuleMmap (struct file * filp, struct vm_area_struct * vma)
{
    return ModuleMmap((void*) filp, (void*)vma);
}


/*******************************************************************************
@   函数原型：static ssize_t simple_read(struct file *filp, unsigned char *buffer, size_t count, loff_t *ppos)
@   函数功能：用户层从底层读取数据(系统函数，格式固定)
@   参数filp： 未使用
@   参数aBuffer： 上层从底层读取的数据
@   参数aCount： 上层从底层读取的数据字节数
@   参数ppos： 未使用
@   返回值：读取字节数或错误码
*******************************************************************************/
static ssize_t MV_ModuleRead(struct file *filp, /*unsigned*/ char *buffer, size_t count, loff_t *ppos)
{
//     if(NULL == buffer)
//     {
//         printk("read buf is NULL\n");
//         return MV_E_PARAMETER;
//     }
    return ModuleRead(buffer, count);
}

__attribute__( ( unused ) )
static unsigned int LFT_ModuleHookV1( unsigned int aHookNumber, struct sk_buff** skb, const struct net_device* aIn, 
                                     const struct net_device* aOut, int (*okfn)(struct sk_buff*) )
{
    return ModuleHookPoint(*skb);
}

__attribute__( ( unused ) )
static unsigned int LFT_ModuleHookV2( unsigned int aHookNumber, struct sk_buff* skb, const struct net_device* aIn, 
                                     const struct net_device* aOut, int (*okfn)(struct sk_buff*) )
{
    return ModuleHookPoint(skb);
}

__attribute__( ( unused ) )
static unsigned int LFT_ModuleHookV3( const struct nf_hook_ops* aOperations, struct sk_buff* skb, 
                                     const struct net_device* aIn, const struct net_device* aOut, int (*okfn)(struct sk_buff*) )
{
    return ModuleHookPoint(skb);
}

// ubuntu16.04使用
__attribute__( ( unused ) )
static unsigned int LFT_ModuleHookV4(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
    return ModuleHookPoint(skb);
}

/*read write结构体赋值(固定格式)*/
struct file_operations s_fileOperations=
{
    .owner = THIS_MODULE,
    .open = MV_ModuleOpen,
    .read  = MV_ModuleRead,
    .write = MV_ModuleWrite,
#if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl  = MV_ModuleUnlockedIoctl, 
    .compat_ioctl    = MV_ModuleUnlockedIoctl,
#else
    .ioctl           = MV_ModuleIoctl,
#endif
    .release        = MV_ModuleRelease,
    .mmap           = MV_ModuleMmap,
};

/*******************************************************************************
@   函数原型：static int InitSkeleton(void)
@   函数功能：设备初始化
@   参数： 无
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
static int InitSkeleton(void)
{
    int nRet       = MV_OK;
    MV_DEV_T stDev = { 0 };

    printk(KERN_DEBUG "Module skeleton init\n");
    // 注册主设备和从设备
    if(0 != s_nParamMajor)
    {
        s_devMajor = MKDEV(s_nParamMajor, 0);           // 获得设备号 
        nRet = register_chrdev_region(s_devMajor, 1, MV_MODULE_NAME); // 静态注册设备号 
    }
    else
    {
        // 动态分配设备
        nRet = alloc_chrdev_region(&s_devMajor, 0, 1, MV_MODULE_NAME);
        s_nParamMajor = MAJOR(s_devMajor);
    }
    if(nRet < 0)    // 判断设备号注册是否成功
    {
        printk(KERN_ERR "cannot get major %d \n", s_nParamMajor);
        return MV_E_PARAMETER;
    }
    /*register character device driver*/
    s_devChar = cdev_alloc();
    if(NULL != s_devChar)
    {
        cdev_init(s_devChar, &s_fileOperations);
        s_devChar->ops = &s_fileOperations;
        s_devChar->owner = THIS_MODULE;
        if(cdev_add(s_devChar, s_devMajor, 1) != 0)
        {
            printk(KERN_NOTICE "something wrong when adding s_devChar!\n");
        }
        else
        {
            printk("success adding s_devChar!\n");
        }
    }
    else
    {
        printk(KERN_ERR "register s_devMajor error!\n");
        return MV_E_PARAMETER;
    }

    stDev.devt = s_devMajor;

    printk("begin InitModuleDevice\n");
    nRet = InitModuleDevice((void*)(&s_classForDev), (void*)(&s_devNode), (void*)(&(stDev)));
    printk("end InitModuleDevice, nRet: %d\n", nRet);

    return nRet;
}

/*******************************************************************************
@   函数原型：static void DestroySkeleton(void)
@   函数功能：销毁设备
@   参数： 无
@   返回值：无
*******************************************************************************/
static void DestroySkeleton(void)
{
    printk(KERN_DEBUG "Module skeleton exit\n");
    device_destroy(s_classForDev, s_devMajor);
    class_destroy(s_classForDev);

    cdev_del(s_devChar);
    unregister_chrdev_region(s_devMajor, 1);
    printk(KERN_DEBUG "Module skeleton exit ok\n");
}


/*******************************************************************************
@   函数原型：int InitHook(void)
@   函数功能：hook函数初始化
@   参数： 无
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
int InitHook(void)
{
    printk("linux version %x\n", LINUX_VERSION_CODE);
#if defined(OS_ALLOW_GPL)
    printk("Defined ALLOW_GPL");
#else
    printk("not Defined ALLOW_GPL\n");
#endif
    //printk("kernel version %x\n", KERNEL_VERSION( 2, 6, 24 )); 
#if LINUX_VERSION_CODE < KERNEL_VERSION( 2, 6, 24 )
    printk("using LFT_ModuleHookV1\n");
    g_sNetfilterHook.hook = LFT_ModuleHookV1;
#elif LINUX_VERSION_CODE < KERNEL_VERSION( 3, 13, 0 )
    g_sNetfilterHook.hook = LFT_ModuleHookV2;
    printk("using LFT_ModuleHookV2\n");
#elif LINUX_VERSION_CODE < KERNEL_VERSION( 4, 4, 0 )
    g_sNetfilterHook.hook = LFT_ModuleHookV3;
    printk("using LFT_ModuleHookV3\n");
#else
    g_sNetfilterHook.hook = LFT_ModuleHookV4;
    printk("using LFT_ModuleHookV4\n");
#endif

    g_sNetfilterHook.hooknum = NF_INET_PRE_ROUTING;  
    g_sNetfilterHook.pf = PF_INET;  
    g_sNetfilterHook.priority = NF_IP_PRI_FIRST;    

#if LINUX_VERSION_CODE < KERNEL_VERSION( 4, 13, 0 )
    if(nf_register_hook(&g_sNetfilterHook))
#else
    if(nf_register_net_hook(&init_net, &g_sNetfilterHook))
#endif
    {
        return MV_E_PARAMETER;
    }

    return MV_OK;
}




/*******************************************************************************
@   函数原型：int MV_InitModule()    
@   函数功能：模块初始化
@   参数： 无
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
static int __init MV_InitModule(void)
{
    int nRet = MV_OK;

    nRet = InitMemory();
    if(nRet)
    {
        printk(KERN_ERR "InitMemory failed[%d]", nRet);
        return MV_E_PARAMETER;
    }

    printk("into InitSkeleton\n");
    if(InitSkeleton())
    {
        printk(KERN_ERR "InitSkeletonn failed\n");
        return MV_E_PARAMETER;
    }

    printk("into InitHook\n");
    if(InitHook())
    {
        printk(KERN_ERR "InitHook failed");
        return MV_E_PARAMETER;
    }

    return MV_OK;
}


/*******************************************************************************
@   函数原型：void MV_ExitModule()  
@   函数功能：模块卸载
@   参数： 无
@   返回值：无
*******************************************************************************/     
static void __exit MV_ExitModule(void)  
{    
    CleanMemory();
    DestroySkeleton();

#if LINUX_VERSION_CODE < KERNEL_VERSION( 4, 13, 0 )
    nf_unregister_hook(&g_sNetfilterHook);
#else
    nf_unregister_net_hook(&init_net, &g_sNetfilterHook);
#endif
}

module_init( MV_InitModule );
module_exit( MV_ExitModule );

MODULE_AUTHOR( MV_COMPANY );
MODULE_LICENSE( MV_COPYRIGHT );
MODULE_DESCRIPTION( MV_MODULE_LONG_NAME );
MODULE_VERSION( MV_VERSION );




