#include <cmath>
#include <assert.h>

#include "Box.h"

bool Box::intersectLocal( const ray& r, isect& i ) const
{
   
    vec3f p = r.getPosition();
    vec3f d = r.getDirection();

    double tmin = -1.0e308;
    double tmax =  1.0e308;
    int minAxis = -1, maxAxis = -1;
    double minNormalSign = 0.0, maxNormalSign = 0.0;

    for( int axis = 0; axis < 3; ++axis ) {
        if( d[axis] == 0.0 ) {
            if( p[axis] < -0.5 || p[axis] > 0.5 )
                return false;
            continue;
        }

        double t1 = (-0.5 - p[axis]) / d[axis];   // face at -0.5, normal -1
        double t2 = ( 0.5 - p[axis]) / d[axis];   // face at +0.5, normal +1

        double tnear, tfar, nearNormal, farNormal;
        if( t1 < t2 ) {
            tnear = t1; tfar = t2;
            nearNormal = -1.0; farNormal = +1.0;
        } else {
            tnear = t2; tfar = t1;
            nearNormal = +1.0; farNormal = -1.0;
        }

        if( tnear > tmin ) { tmin = tnear; minAxis = axis; minNormalSign = nearNormal; }
        if( tfar  < tmax ) { tmax = tfar;  maxAxis = axis; maxNormalSign = farNormal;  }

        if( tmin > tmax ) return false;
    }

    if( tmax < RAY_EPSILON ) return false;

    if( tmin > RAY_EPSILON ) {
        i.t   = tmin;
        i.obj = this;
        i.N   = vec3f( 0.0, 0.0, 0.0 );
        i.N[minAxis] = minNormalSign;
        return true;
    } else if( tmax > RAY_EPSILON ) {
        i.t   = tmax;
        i.obj = this;
        i.N   = vec3f( 0.0, 0.0, 0.0 );
        i.N[maxAxis] = maxNormalSign;
        return true;
    }

    return false;
}