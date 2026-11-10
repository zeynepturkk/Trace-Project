#include <cmath>

#include "ray.h"
#include "material.h"
#include "light.h"

vec3f Material::shade( Scene *scene, const ray& r, const isect& i ) const
{
	vec3f P = r.at( i.t );
	vec3f N = i.N.normalize();
	vec3f V = (-r.getDirection()).normalize();

	// Emissive + material ambient.
	// The scene's ambient_light (if declared) adds extra ambient on top.
	vec3f result = ke + prod( ka, scene->ambientLight );

	Scene::cliter lit;
	for( lit = scene->beginLights(); lit != scene->endLights(); ++lit ) {
		Light *light = *lit;

		vec3f lightColor = light->getColor( P );
		if( lightColor[0] <= 0.0 && lightColor[1] <= 0.0 && lightColor[2] <= 0.0 )
			continue;

		vec3f L = light->getDirection( P ).normalize();
		double NdotL = N.dot( L );
		if( NdotL <= 0.0 )
			continue;

		double distAtten = light->distanceAttenuation( P );
		if( distAtten <= 0.0 )
			continue;

		vec3f shadowAtten = light->shadowAttenuation( P + RAY_EPSILON * N );
		if( shadowAtten[0] < RAY_EPSILON &&
		    shadowAtten[1] < RAY_EPSILON &&
		    shadowAtten[2] < RAY_EPSILON )
			continue;

		vec3f diffuseTerm = kd * NdotL;

		vec3f specularTerm( 0.0, 0.0, 0.0 );
		vec3f R = (2.0 * NdotL * N - L).normalize();
		double RdotV = R.dot( V );
		if( RdotV > 0.0 ) {
			double exponent = 128.0 * shininess;
			if( exponent < 1.0 ) exponent = 1.0;
			specularTerm = ks * pow( RdotV, exponent );
		}


		vec3f contrib = (diffuseTerm + specularTerm) * distAtten;
		contrib = prod( contrib, shadowAtten );
		contrib = prod( contrib, lightColor );
		result += contrib;
	}

	return result;
}
