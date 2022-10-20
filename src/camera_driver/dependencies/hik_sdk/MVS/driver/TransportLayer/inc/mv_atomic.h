
#ifndef     __MV_ATOMIC_H__
#define     __MV_ATOMIC_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus


typedef struct
{
    void* pAtomic;
}MV_ATOMIC;


extern int MV_AtomicInit(MV_ATOMIC* aAtomic);
extern int MV_AtomicFree(MV_ATOMIC* aAtomic);
extern int MV_AtomicSet(MV_ATOMIC* aAtomic, int i);
extern int MV_AtomicGet(MV_ATOMIC* aAtomic, int* i);
extern int MV_AtomicAdd(MV_ATOMIC* aAtomic, int i);
extern int MV_AtomicSub(MV_ATOMIC* aAtomic, int i);
extern int MV_AtomicInc(MV_ATOMIC* aAtomic);
extern int MV_AtomicDec(MV_ATOMIC* aAtomic);


#ifdef __cplusplus
}
#endif // __cplusplus


#endif        //    __MV_ATOMIC_H__

