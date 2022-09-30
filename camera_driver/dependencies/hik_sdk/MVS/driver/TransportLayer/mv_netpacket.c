
#include <linux/skbuff.h>
#include <linux/ip.h>


#include "mv_netpacket.h"
#include "MvErrorDefine.h"
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>

/*******************************************************************************
@   函数原型：int NET_PacketDestroy(void* aPacket)
@   函数功能：销毁数据包
@   参数aPacket： 数据包buf
@   返回值：成功MV_OK或失败MV_E_PARAMETER
*******************************************************************************/
int NET_PacketDestroy(void* aPacket)
{
    struct sk_buff* lSKBuffer = NULL;

    if(NULL == aPacket)
    {
        return MV_E_PARAMETER;
    }
    lSKBuffer = (struct sk_buff *)aPacket;

    kfree_skb(lSKBuffer);
    return MV_OK;
}

/*******************************************************************************
@   函数原型：void* NET_PacketCopy(void* aPacket)
@   函数功能：拷贝数据包
@   参数aPacket： 数据包buf
@   返回值：成功:拷贝的数据包buf 失败NULL
*******************************************************************************/
void* NET_PacketCopy(void* aPacket)
{
    struct sk_buff* lSKBuffer = NULL;

    if(NULL == aPacket)
    {
        return NULL;
    }
    lSKBuffer = (struct sk_buff *)aPacket;

    return skb_copy(lSKBuffer, GFP_ATOMIC);
}

int NET_PacketCopyToUser(void* aPacket, unsigned int nOffset, void* pDst, unsigned int nLen)
{
    if (NULL == aPacket
        || NULL == pDst)
    {
        return -EINVAL;
    }

    return skb_copy_bits((struct sk_buff *)aPacket, nOffset, pDst, nLen);
}

/*******************************************************************************
@   函数原型：NET_HEADER_IPV4* NET_ExtractIphdr(void* aPacket)
@   函数功能：提取ip头
@   参数aPacket： 数据包buf
@   返回值：成功:ip头 失败NULL
*******************************************************************************/
NET_HEADER_IPV4* NET_ExtractIphdr(void* aPacket)
{
    struct sk_buff* lSKBuffer = NULL;
    if(NULL == aPacket)
    {
        return NULL;
    }
    lSKBuffer = (struct sk_buff *)aPacket;

    return (NET_HEADER_IPV4 *)skb_network_header(lSKBuffer);
}

/*******************************************************************************
@   函数原型：int	NET_ExtractIpLen(NET_HEADER_IPV4* aIpheader)
@   函数功能：提取ip头长度
@   参数aIpheader： ip头
@   返回值：成功:ip长度 失败:MV_E_PARAMETER
*******************************************************************************/
int NET_ExtractIpLen(NET_HEADER_IPV4* aIpheader)
{
    if(NULL == aIpheader)
    {
        return MV_E_PARAMETER;
    }
    return ((struct iphdr*)aIpheader)->ihl * 4;
}

/*******************************************************************************
@   函数原型：unsigned char* NET_ExtractSkbufData(void *aPacket)
@   函数功能：提取buf的数据部分
@   参数aPacket： 数据包buf
@   返回值：成功:buf数据部分 失败NULL
*******************************************************************************/
unsigned char* NET_ExtractSkbufData(void *aPacket)
{
    struct sk_buff* lSKBuffer = NULL;
    if(NULL == aPacket)
    {
        return NULL;
    }
    lSKBuffer = (struct sk_buff *)aPacket;
    return lSKBuffer->data;
}

/*******************************************************************************
@   函数原型：unsigned short NET_Htons(unsigned short aAddr)
@   函数功能：主机序转网络序(二字节)
@   参数aAddr： 地址
@   返回值：网络字节序地址
*******************************************************************************/
unsigned short NET_Htons(unsigned short aAddr)
{
    return htons(aAddr);	
}

/*******************************************************************************
@   函数原型：unsigned int NET_Htonl(unsigned int aAddr)
@   函数功能：主机序转网络序(二字节)
@   参数aAddr： 地址
@   返回值：网络字节序地址
*******************************************************************************/
unsigned int NET_Htonl(unsigned int aAddr)
{
    return htonl(aAddr);	
}

/*******************************************************************************
@   函数原型：unsigned short NET_Htons(unsigned short aAddr)
@   函数功能：主机序转网络序(二字节)
@   参数aAddr： 地址
@   返回值：网络字节序地址
*******************************************************************************/
unsigned int NET_Ntohl(unsigned int aAddr)
{
    return ntohl(aAddr);	
}

/*******************************************************************************
@   函数原型：unsigned int NET_Htonl(unsigned int aAddr)
@   函数功能：主机序转网络序(二字节)
@   参数aAddr： 地址
@   返回值：网络字节序地址
*******************************************************************************/
unsigned short NET_Ntohs(unsigned short aAddr)
{
    return ntohs(aAddr);	
}

