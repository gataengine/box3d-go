// Generates testdata/math_golden.txt from the pinned upstream C sources (Go-only tooling).
// cc -O2 -ffp-contract=off -std=c17 -I include -I src testdata/math_golden_gen.c src/math_functions.c src/core.c
// run inside third_party/box3d, then ./a.out > libs/box3d-go/testdata/math_golden.txt
#include "box3d/math_functions.h"
#include "math_internal.h"
#include <stdio.h>
#include <string.h>
static uint32_t seed = 12345;
static float rnd(void){ seed ^= seed<<13; seed ^= seed>>17; seed ^= seed<<5; return ((float)(seed % 20001) / 10000.0f - 1.0f) * 3.0f; }
static b3Vec3 rv(void){ b3Vec3 v = { rnd(), rnd(), rnd() }; return v; }
static b3Quat rq(void){ b3Quat q = { rv(), rnd() }; return b3NormalizeQuat(q); }
static uint32_t B(float f){ uint32_t u; memcpy(&u,&f,4); return u; }
static void pf(float f){ printf(" %08x", B(f)); }
static void pv(b3Vec3 v){ pf(v.x); pf(v.y); pf(v.z); }
static void pq(b3Quat q){ pv(q.v); pf(q.s); }
static void pm(b3Matrix3 m){ pv(m.cx); pv(m.cy); pv(m.cz); }
int main(void){
  for(int i=0;i<300;i++){
    float y=rnd(), x=rnd(); printf("atan2"); pf(y); pf(x); printf(" |"); pf(b3Atan2(y,x)); printf("\n");
    float a=rnd()*10.0f; b3CosSin cs=b3ComputeCosSin(a); printf("cossin"); pf(a); printf(" |"); pf(cs.cosine); pf(cs.sine); printf("\n");
    b3Quat q1=rq(), q2=rq(); b3Vec3 v=rv();
    printf("quat"); pq(q1); pq(q2); pv(v); printf(" |");
    pq(b3MulQuat(q1,q2)); pq(b3InvMulQuat(q1,q2)); pv(b3RotateVector(q1,v)); pv(b3InvRotateVector(q1,v));
    pq(b3NLerp(q1,q2,0.3f)); pm(b3MakeMatrixFromQuat(q1)); pf(b3GetTwistAngle(q1)); pf(b3GetSwingAngle(q1));
    b3Matrix3 m=b3MakeMatrixFromQuat(q2); pq(b3MakeQuatFromMatrix(&m)); pv(b3DeltaQuatToRotation(q1,q2)); pq(b3IntegrateRotation(q1,v));
    printf("\n");
    b3Vec3 p1=rv(),q1v=rv(),p2=rv(),q2v=rv();
    printf("seg"); pv(p1); pv(q1v); pv(p2); pv(q2v); printf(" |");
    b3SegmentDistanceResult r=b3SegmentDistance(p1,q1v,p2,q2v); pv(r.point1); pf(r.fraction1); pv(r.point2); pf(r.fraction2);
    r=b3LineDistance(p1,q1v,p2,q2v); pv(r.point1); pf(r.fraction1); pv(r.point2); pf(r.fraction2);
    pv(b3PointToSegmentDistance(p1,q1v,p2));
    b3TrianglePoint tp=b3ClosestPointOnTriangle(p1,q1v,p2,q2v); pv(tp.point); printf(" %08x",(unsigned)tp.feature);
    pv(b3Normalize(p1)); pv(b3Cross(p1,p2)); pf(b3Dot(p1,p2)); pv(b3ArbitraryPerp(b3Normalize(p1)));
    b3Quat qb=b3ComputeQuatBetweenUnitVectors(b3Normalize(p1),b3Normalize(p2)); pq(qb);
    printf("\n");
    b3Matrix3 mm={rv(),rv(),rv()}; b3Vec3 w=rv(); float mass=rnd()+4.0f;
    printf("mat"); pm(mm); pv(w); pf(mass); printf(" |");
    pm(b3InvertMatrix(mm)); pv(b3Solve3(mm,w)); pm(b3InvertT(mm)); pf(b3Det(mm)); pm(b3Steiner(mass,w)); pm(b3MulMM(mm,mm)); pv(b3MulMV(mm,w));
    pm(b3RotateInertia(q1,mm)); pf(b3ScalarTripleProduct(mm.cx,mm.cy,mm.cz)); pv(b3Blend3(mass,mm.cx,w.x,mm.cy,w.y,mm.cz));
    printf("\n");
  }
}
