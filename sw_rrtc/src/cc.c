#include "cc.h"
#include "panda.h"

const mvt_t *cc_mvt;
const points_t *cc_brute;
uint32_t cc_t_fk;

int cc_env_sphere(float x, float y, float z, float r)
{
    st.env_queries++;
#if TIMING
    uint32_t t0 = tnow();
#endif
    int hit = cc_brute ? brute_sphere_collides(cc_brute, x, y, z, r) : mvt_sphere_collides(cc_mvt, x, y, z, r);
#if TIMING
    st.cyc_env += (uint32_t)(tnow() - t0);
    st.timer_reads += 2;
#endif
    st.env_hits += (uint32_t)hit;
    return hit;
}

int cc_validate_pose(const float *q)
{
    st.poses++;
    ENVSTAT(envstat_pose(q));
#if TIMING
    cc_t_fk = tnow();
#endif
    int ok = panda_fkcc(q);
#if TIMING
    st.cyc_fkcc += (uint32_t)(tnow() - cc_t_fk);
    st.timer_reads += 2;
#endif
    return ok;
}
