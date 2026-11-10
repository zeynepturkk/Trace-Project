#ifndef __LIGHT_H__
#define __LIGHT_H__

#include "scene.h"

#ifndef PI
#define PI 3.14159265359
#endif

class Light
	: public SceneElement
{
public:
	virtual vec3f shadowAttenuation(const vec3f& P) const = 0;
	virtual double distanceAttenuation( const vec3f& P ) const = 0;
	virtual vec3f getColor( const vec3f& P ) const = 0;
	virtual vec3f getDirection( const vec3f& P ) const = 0;

protected:
	Light( Scene *scene, const vec3f& col )
		: SceneElement( scene ), color( col ) {}

	vec3f 		color;
};

class DirectionalLight
	: public Light
{
public:
	DirectionalLight( Scene *scene, const vec3f& orien, const vec3f& color )
		: Light( scene, color ), orientation( orien ) {}
	virtual vec3f shadowAttenuation(const vec3f& P) const;
	virtual double distanceAttenuation( const vec3f& P ) const;
	virtual vec3f getColor( const vec3f& P ) const;
	virtual vec3f getDirection( const vec3f& P ) const;

protected:
	vec3f 		orientation;
};

class PointLight
	: public Light
{
public:
	// Defaults match "no attenuation": kc=1, kl=0, kq=0.
	PointLight( Scene *scene, const vec3f& pos, const vec3f& color,
	            double kc = 1.0, double kl = 0.0, double kq = 0.0,
	            double sz = 0.0, int sm = 1 )
		: Light( scene, color ), position( pos ),
		  constantAtt( kc ), linearAtt( kl ), quadraticAtt( kq ),
		  size( sz ), samples( sm ) {}
	virtual vec3f shadowAttenuation(const vec3f& P) const;
	virtual double distanceAttenuation( const vec3f& P ) const;
	virtual vec3f getColor( const vec3f& P ) const;
	virtual vec3f getDirection( const vec3f& P ) const;

protected:
	vec3f position;
	double constantAtt;
	double linearAtt;
	double quadraticAtt;
	double size;
	int samples;
};

class SpotLight
	: public PointLight
{
public:
	// angleDeg is the half-angle of the cone (from axis to edge)
	// exponent controls softness: 0 = hard cutoff, higher = softer edges inside the cone
	SpotLight( Scene *scene, const vec3f& pos, const vec3f& dir,
	           const vec3f& color, double angleDeg, double exponent,
	           double kc = 1.0, double kl = 0.0, double kq = 0.0,
	           double sz = 0.0, int sm = 1 )
		: PointLight( scene, pos, color, kc, kl, kq, sz, sm )
		, spotDirection( dir.normalize() )
		, cosOuterAngle( std::cos( angleDeg * PI / 180.0 ) )
		, spotExponent( exponent ) {}

	virtual vec3f getColor( const vec3f& P ) const;

protected:
	vec3f spotDirection;
	double cosOuterAngle;
	double spotExponent;
};

#endif // __LIGHT_H__
