
#include "AxisConv.h"

void Vect3AxisConv(tVect3State* vs, AXIS org, AXIS cnv){
    switch(org){
        case AXIS_UVW:
            switch(cnv){
                case AXIS_AB:
                    uvw2ab(vs->uvw[0], vs->uvw[1], vs->uvw[2], &(vs->ab[0]), &(vs->ab[1]));
                    break;
                case AXIS_DQ:
                    uvw2ab(vs->uvw[0], vs->uvw[1], vs->uvw[2], &(vs->ab[0]), &(vs->ab[1]));
                    ab2dq(vs->ab[0], vs->ab[1], vs->phase, &(vs->dq[0]), &(vs->dq[1]));
                    break;
                default:
                    break;
            }
            break;
        case AXIS_AB:
            switch(cnv){
                case AXIS_UVW:
                    ab2uvw(vs->ab[0], vs->ab[1], &(vs->uvw[0]), &(vs->uvw[1]), &(vs->uvw[2]));
                    break;
                case AXIS_DQ:
                    ab2dq(vs->ab[0], vs->ab[1], vs->phase, &(vs->dq[0]), &(vs->dq[1]));
                    break;
                default:
                    break;
            }
            break;
        case AXIS_DQ:
            switch(cnv){
                case AXIS_UVW:
                    dq2ab(vs->dq[0], vs->dq[1], vs->phase, &(vs->ab[0]), &(vs->ab[1]));
                    ab2uvw(vs->ab[0], vs->ab[1], &(vs->uvw[0]), &(vs->uvw[1]), &(vs->uvw[2]));
                    break;
                case AXIS_AB:
                    dq2ab(vs->dq[0], vs->dq[1], vs->phase, &(vs->ab[0]), &(vs->ab[1]));
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    return;
}

void Vect2AxisConv(tVect2State* vs, AXIS org, AXIS cnv){
    switch(org){
        case AXIS_AB:
            switch(cnv){
                case AXIS_DQ:
                    ab2dq(vs->ab[0], vs->ab[1], vs->phase, &(vs->dq[0]), &(vs->dq[1]));
                    break;
                default:
                    break;
            }
            break;
        case AXIS_DQ:
            switch(cnv){
                case AXIS_AB:
                    dq2ab(vs->dq[0], vs->dq[1], vs->phase, &(vs->ab[0]), &(vs->ab[1]));
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    return;
}
