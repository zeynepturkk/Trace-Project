#include <cmath>
#include <cstdlib>

#include "light.h"

#define MAX_SHADOW_SEGMENTS 32

double DirectionalLight::distanceAttenuation( const vec3f& P ) const
{
	return 1.0;
}

vec3f DirectionalLight::shadowAttenuation( const vec3f& P ) const
{
	vec3f dir = (-orientation).normalize();
	vec3f attenuation( 1.0, 1.0, 1.0 );
	ray shadowRay( P, dir );

	for( int seg = 0; seg < MAX_SHADOW_SEGMENTS; ++seg ) {
		isect hit;
		if( !scene->intersect( shadowRay, hit ) )
			break;

		const Material& mat = hit.getMaterial();
		attenuation[0] *= mat.kt[0];
		attenuation[1] *= mat.kt[1];
		attenuation[2] *= mat.kt[2];

		if( attenuation[0] < RAY_EPSILON &&
		    attenuation[1] < RAY_EPSILON &&
		    attenuation[2] < RAY_EPSILON )
			break;

		shadowRay = ray( shadowRay.at( hit.t + RAY_EPSILON ), dir );
	}

	return attenuation;
}

vec3f DirectionalLight::getColor( const vec3f& P ) const
{
	return color;
}

vec3f DirectionalLight::getDirection( const vec3f& P ) const
{
	return -orientation;
}

double PointLight::distanceAttenuation( const vec3f& P ) const
{
	double dist = (position - P).length();
	double denom = constantAtt + linearAtt * dist + quadraticAtt * dist * dist;
	if( denom <= 0.0 )
		return 1.0;
	return 1.0 / denom;
}

vec3f PointLight::getColor( const vec3f& P ) const
{
	return color;
}

vec3f PointLight::getDirection( const vec3f& P ) const
{
	return (position - P).normalize();
}

// Cast one shadow ray from P toward a specific light position.
// Returns accumulated transmissive attenuation along the path.
static vec3f castShadowRay( Scene *scene, const vec3f& P, const vec3f& lightPos )
{
	vec3f toLight = lightPos - P;
	double distToLight = toLight.length();
	if( distToLight < RAY_EPSILON )
		return vec3f( 1.0, 1.0, 1.0 );

	vec3f dir = toLight / distToLight;
	vec3f attenuation( 1.0, 1.0, 1.0 );
	ray shadowRay( P, dir );
	double traveled = 0.0;

	for( int seg = 0; seg < MAX_SHADOW_SEGMENTS; ++seg ) {
		isect hit;
		if( !scene->intersect( shadowRay, hit ) ) break;
		if( traveled + hit.t > distToLight ) break;

		const Material& mat = hit.getMaterial();
		attenuation[0] *= mat.kt[0];
		attenuation[1] *= mat.kt[1];
		attenuation[2] *= mat.kt[2];

		if( attenuation[0] < RAY_EPSILON &&
		    attenuation[1] < RAY_EPSILON &&
		    attenuation[2] < RAY_EPSILON ) break;

		traveled += hit.t;
		shadowRay = ray( shadowRay.at( hit.t + RAY_EPSILON ), dir );
	}
	return attenuation;
}

// Random point uniformly distributed inside a sphere of radius R.
static vec3f randomInSphere( double R )
{
	vec3f p;
	do {
		p = vec3f(
			2.0 * rand() / (double)RAND_MAX - 1.0,
			2.0 * rand() / (double)RAND_MAX - 1.0,
			2.0 * rand() / (double)RAND_MAX - 1.0 );
	} while( p.length_squared() > 1.0 );
	return p * R;
}

vec3f PointLight::shadowAttenuation( const vec3f& P ) const
{
	// Hard shadow (single ray)
	if( size <= 0.0 || samples <= 1 ) {
		return castShadowRay( scene, P, position );
	}

	// Soft shadow (distributed)
	vec3f totalAtten( 0.0, 0.0, 0.0 );
	for( int s = 0; s < samples; ++s ) {
		vec3f offset = randomInSphere( size );
		totalAtten += castShadowRay( scene, P, position + offset );
	}
	return totalAtten / (double)samples;
}
vec3f SpotLight::getColor( const vec3f& P ) const
{
	vec3f toPoint = (P - position).normalize();
	double cosAngle = spotDirection.dot( toPoint );
	if( cosAngle < cosOuterAngle )
		return vec3f( 0.0, 0.0, 0.0 );     // outside cone → no light
	if( cosAngle >= 1.0 )
		return color;                      // straight down the axis → full color

	// Smooth falloff from outer edge (0) to axis (1)
	double falloff = (cosAngle - cosOuterAngle) / (1.0 - cosOuterAngle);
	if( spotExponent > 0.0 )
		falloff = pow( falloff, spotExponent );
	return color * falloff;
}