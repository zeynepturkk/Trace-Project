// The main ray tracer.

#include <Fl/fl_ask.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "fileio/bitmap.h"

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"
#include "fileio/read.h"
#include "fileio/parse.h"
#include "ui/TraceUI.h"

// Globals defined in main.cpp and scene.cpp — used for max recursion depth.
// In GUI mode, traceUI is non-NULL and we read the slider.
// In text mode, traceUI is NULL and we fall back to the -r command-line arg.
extern TraceUI* traceUI;
extern int recursion_depth;
extern double adaptive_threshold;
extern int supersample_level;
extern bool jitter_enabled;

#ifndef PI
#define PI 3.14159265359
#endif

// Generate a random direction inside a cone of half-angle halfAngleRad around dir.
static vec3f randomDirectionInCone( const vec3f& dir, double halfAngleRad )
{
    // Orthonormal basis around dir
    vec3f u = (fabs(dir[0]) < 0.9 ? vec3f(1,0,0) : vec3f(0,1,0)).cross(dir).normalize();
    vec3f v = dir.cross(u);

    // Uniform sampling inside cone of half-angle θ:
    // cos(α) uniform in [cos θ, 1]
    double cosTheta = cos( halfAngleRad );
    double cosAlpha = cosTheta + (1.0 - cosTheta) * (rand() / (double)RAND_MAX);
    double sinAlpha = sqrt( 1.0 - cosAlpha * cosAlpha );
    double phi = 2.0 * PI * (rand() / (double)RAND_MAX);

    return (cosAlpha * dir + sinAlpha * (cos(phi) * u + sin(phi) * v)).normalize();
}

// Trace a top-level ray through normalized window coordinates (x,y)
vec3f RayTracer::trace( Scene *scene, double x, double y )
{
	ray r( vec3f(0,0,0), vec3f(0,0,0) );
	scene->getCamera()->rayThrough( x,y,r );
	return traceRay( scene, r, vec3f(1.0,1.0,1.0), 0 ).clamp();
}

// Do recursive ray tracing: shade + reflect + refract, bounded by max depth.
vec3f RayTracer::traceRay( Scene *scene, const ray& r, 
	const vec3f& thresh, int depth )
{
	isect i;

	if( !scene->intersect( r, i ) ) {
		
		if( hasBackground && backgroundBuffer ) {
			vec3f d = r.getDirection().normalize();
			
			double u = 0.5 + atan2( d[0], -d[2] ) / (2.0 * PI);
			double v = 0.5 - asin( d[1] ) / PI;
			
			int px = (int)(u * bgWidth) % bgWidth;
			int py = (int)(v * bgHeight) % bgHeight;
			if( px < 0 ) px += bgWidth;
			if( py < 0 ) py += bgHeight;
			
			int idx = (px + py * bgWidth) * 3;
			return vec3f( backgroundBuffer[idx] / 255.0, 
			              backgroundBuffer[idx+1] / 255.0, 
			              backgroundBuffer[idx+2] / 255.0 );
		}
		return vec3f( 0.0, 0.0, 0.0 );
	}

	// Maximum recursion depth: prefer the GUI slider, fall back to -r arg.
	int maxDepth = traceUI ? traceUI->getDepth() : recursion_depth;

	const Material& m = i.getMaterial();
	vec3f shadeColor = m.shade( scene, r, i );

	// Reached depth limit — return just the local shading.
	if( depth >= maxDepth ) {
		return shadeColor;
	}

	
	double cutoff = traceUI ? traceUI->getThreshold() : adaptive_threshold;
	if( cutoff > 0.0 ) {
		double maxThresh = thresh[0];
		if( thresh[1] > maxThresh ) maxThresh = thresh[1];
		if( thresh[2] > maxThresh ) maxThresh = thresh[2];
		if( maxThresh < cutoff ) {
			return shadeColor;
		}
	}

	vec3f result = shadeColor;
	vec3f P = r.at( i.t );
	vec3f N = i.N.normalize();
	vec3f d = r.getDirection();
	double dotDN = d.dot( N );

	// ---- Reflection (glossy-aware) ----
	if( m.kr[0] > 0.0 || m.kr[1] > 0.0 || m.kr[2] > 0.0 ) {
		vec3f idealReflDir = (d - 2.0 * dotDN * N).normalize();
		vec3f reflOrigin = P + RAY_EPSILON * (dotDN < 0.0 ? N : -N);

		int rSamples = (m.glossiness > 0.0) ? m.glossySamples : 1;
		if( rSamples < 1 ) rSamples = 1;

		vec3f reflColor( 0.0, 0.0, 0.0 );
		double halfAngle = m.glossiness * PI / 180.0;

		for( int s = 0; s < rSamples; ++s ) {
			vec3f dir = idealReflDir;
			if( halfAngle > 0.0 && rSamples > 1 ) {
				dir = randomDirectionInCone( idealReflDir, halfAngle );
			}
			ray reflRay( reflOrigin, dir );
			reflColor += traceRay( scene, reflRay, prod( thresh, m.kr ), depth + 1 );
		}
		reflColor /= (double)rSamples;

		result += prod( m.kr, reflColor );
	}

	// ---- Refraction ----
	if( m.kt[0] > 0.0 || m.kt[1] > 0.0 || m.kt[2] > 0.0 ) {
		// Determine if we're entering (outside -> inside) or exiting
		bool entering = dotDN < 0.0;
		double eta_i = entering ? 1.0 : m.index;   // medium we're coming from
		double eta_t = entering ? m.index : 1.0;   // medium we're going into
		double eta = eta_i / eta_t;

		// Normal oriented toward the incoming ray
		vec3f Nf = entering ? N : -N;
		double cosI = -d.dot( Nf );
		double sin2T = eta * eta * (1.0 - cosI * cosI);

		if( sin2T > 1.0 ) {
			// Total internal reflection — reflect instead of refract
			vec3f reflDir = (d - 2.0 * dotDN * N).normalize();
			vec3f reflOrigin = P + RAY_EPSILON * (dotDN < 0.0 ? N : -N);
			ray reflRay( reflOrigin, reflDir );
			vec3f reflColor = traceRay( scene, reflRay, prod( thresh, m.kt ), depth + 1 );
			result += prod( m.kt, reflColor );
		} else {
			double cosT = sqrt( 1.0 - sin2T );
			vec3f refrDir = (eta * d + (eta * cosI - cosT) * Nf).normalize();
			// Origin goes *into* the new medium
			vec3f refrOrigin = P + (entering ? -RAY_EPSILON : RAY_EPSILON) * N;
			ray refrRay( refrOrigin, refrDir );
			vec3f refrColor = traceRay( scene, refrRay, prod( thresh, m.kt ), depth + 1 );
			result += prod( m.kt, refrColor );
		}
	}

	return result;
}

RayTracer::RayTracer()
{
	buffer = NULL;
	buffer_width = buffer_height = 256;
	scene = NULL;
	backgroundBuffer = NULL;
	bgWidth = bgHeight = 0;
	hasBackground = false;

	m_bSceneLoaded = false;
}

RayTracer::~RayTracer()
{
	delete [] buffer;
	if( backgroundBuffer ) delete [] backgroundBuffer;
	delete scene;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return scene ? scene->getCamera()->getAspectRatio() : 1;
}

bool RayTracer::sceneLoaded()
{
	return m_bSceneLoaded;
}

bool RayTracer::loadScene( char* fn )
{
	try
	{
		scene = readScene( fn );
	}
	catch( ParseError pe )
	{
		fl_alert( "ParseError: %s\n", pe.getMsg().c_str() );
		return false;
	}

	if( !scene )
		return false;
	
	buffer_width = 256;
	buffer_height = (int)(buffer_width / scene->getCamera()->getAspectRatio() + 0.5);

	bufferSize = buffer_width * buffer_height * 3;
	buffer = new unsigned char[ bufferSize ];
	
	scene->initScene();

	m_bSceneLoaded = true;

	return true;
}

void RayTracer::traceSetup( int w, int h )
{
	if( buffer_width != w || buffer_height != h )
	{
		buffer_width = w;
		buffer_height = h;

		bufferSize = buffer_width * buffer_height * 3;
		delete [] buffer;
		buffer = new unsigned char[ bufferSize ];
	}
	memset( buffer, 0, w*h*3 );
}

void RayTracer::traceLines( int start, int stop )
{
	vec3f col;
	if( !scene )
		return;

	if( stop > buffer_height )
		stop = buffer_height;

	for( int j = start; j < stop; ++j )
		for( int i = 0; i < buffer_width; ++i )
			tracePixel(i,j);
}

void RayTracer::tracePixel( int i, int j )
{
	if( !scene )
		return;

	// Determine supersample level and jitter flag.
	int N = traceUI ? traceUI->getSupersample() : supersample_level;
	bool jitter = traceUI ? traceUI->getJitter() : jitter_enabled;
	if( N < 1 ) N = 1;
	if( N > 5 ) N = 5;

	vec3f col( 0.0, 0.0, 0.0 );
	int samples = 0;

	for( int sy = 0; sy < N; ++sy ) {
		for( int sx = 0; sx < N; ++sx ) {
			double jx = 0.5, jy = 0.5;
			if( jitter ) {
				jx = (double)rand() / (double)RAND_MAX;
				jy = (double)rand() / (double)RAND_MAX;
			}
			double x = (double(i) + (sx + jx) / N) / double(buffer_width);
			double y = (double(j) + (sy + jy) / N) / double(buffer_height);
			col += trace( scene, x, y );
			++samples;
		}
	}

	if( samples > 0 )
		col /= (double)samples;

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
}
void RayTracer::loadBackground( const char* fn )
{
	if( backgroundBuffer ) {
		delete [] backgroundBuffer;
		backgroundBuffer = NULL;
	}
	hasBackground = false;
	
	if( !fn ) return;
	
	backgroundBuffer = readBMP( (char*)fn, bgWidth, bgHeight );
	if( backgroundBuffer ) {
		hasBackground = true;
	}
}