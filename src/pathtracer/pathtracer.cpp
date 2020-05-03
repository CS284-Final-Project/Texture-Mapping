#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
  if (!isect.primitive)
	  return Spectrum(0.0);

  Spectrum spct(0.0);
  //double pdf = 1.0 / (2.0 * PI);
  for (int i = 0; i < num_samples; i++) {
	  Vector3D w_in;
	  float pdf;
	  Spectrum spe = isect.bsdf->sample_f(w_out, &w_in, &pdf);
	  Ray newRay(hit_p, o2w*w_in);
	  newRay.d.normalize();
	  newRay.min_t = EPS_F;
	  if (bvh->has_intersection(newRay)) {
		  Intersection itsc = Intersection();
		  bvh->intersect(newRay, &itsc);
		  Spectrum spe = itsc.bsdf->get_emission();
		  if (spe[0] < EPS_D && spe[1] < EPS_D && spe[2] < EPS_D)
			  continue; // don't intersect with light source
		  // double cosTheta = abs(w_in.z);
		  spct += isect.bsdf->f(w_out, w_in) * spe * abs(w_in.z) / pdf;
	  }
  }

  return spct / (1.0 * num_samples);
}

Spectrum
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.
  if (!isect.primitive)
	  return Spectrum(0.0);
  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;

  size_t N = 0;
  for (int j = 0; j < scene->lights.size(); j++) {
	  N = (scene->lights.at(j)->is_delta_light()) ? 1 : ns_area_light;
	  Spectrum spc_light;
	  for (int i = 0; i < N; i++) {
		  Vector3D wi;
		  float distToLight;
		  float pdf;
		  Spectrum L = scene->lights.at(j)->sample_L(hit_p, &wi, &distToLight, &pdf); // hit_p should be world coord or object coord?
		  Ray shadowRay(hit_p, wi);
		  //shadowRay.min_t = EPS_F;
		  shadowRay.max_t = distToLight;
		  if (bvh->has_intersection(shadowRay))
			  continue;
		  Intersection it;
		  //bvh->intersect(shadowRay, &it);
		  Vector3D wi_obj = w2o * wi;
		  wi_obj.normalize();
		  spc_light += isect.bsdf->f(w_out, wi_obj) * L * (abs(wi_obj.z) / pdf);
	  }
	  L_out += spc_light / N;
  }

  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
	if (isect.bsdf)
		return isect.bsdf->get_emission();
  return Spectrum(0.0);
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

	if (direct_hemisphere_sample)
		return estimate_direct_lighting_hemisphere(r, isect);
	else
		return estimate_direct_lighting_importance(r, isect);
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * (isect.t);
  Vector3D w_out = w2o * (-r.d);
  Spectrum L_out(0, 0, 0);

  if (!isect.bsdf->is_delta())
	  L_out += one_bounce_radiance(r, isect);

  float pdf;
  Vector3D wi;
  isect.bsdf->sample_f(w_out, &wi, &pdf);
  Ray nextRay(hit_p, o2w*wi);
  nextRay.min_t = EPS_D;
  nextRay.d.normalize();
  nextRay.depth = r.depth + 1;

  // volumetric scattering
  float absorption_coef = 0.5;
  float ext_coef = 1;
  Spectrum in_scattering(0, 0, 0);
  if (!bvh->has_intersection(nextRay))
	  return Spectrum(0, 0, 0);

	  float stride = 0.5;
	  float g = 0.5;
	  Vector3D sample_point = r.o + r.d * (isect.t - fmod(isect.t, stride) * random_uniform());
	  int sample_num = isect.t / stride;
	  int sample_rate = 2;
	  for (int i = 0; i < sample_num; i++) {
		  // uniform sphere sampling
		  Spectrum tmp(0, 0, 0);
		  for (int j = 0; j < sample_rate; j++) {
			  double theta = acos(random_uniform());
			  double phi = 2.0 * PI * random_uniform();
			  Vector3D dir(sinf(theta) * cosf(phi),
				  sinf(theta) * sinf(phi),
				  coin_flip(0.5) ? cosf(theta) : -1 * cosf(theta));
			  double cosine = dot(dir, r.d); // not sure !!!
			  double phase = (1 - g * g) / (4 * PI*pow(1 + g * g - 2 * g*cosine, 1.5));
			  Ray in_sca_ray(sample_point, dir);
			  in_sca_ray.min_t = EPS_F;
			  if (!bvh->has_intersection(in_sca_ray))
				  continue;
			  Intersection in_sca_ite;
			  bvh->intersect(in_sca_ray, &in_sca_ite);
			  in_sca_ite.t -= EPS_F;
			  tmp += phase * (one_bounce_radiance(in_sca_ray, in_sca_ite) +
				  zero_bounce_radiance(in_sca_ray, in_sca_ite)) * 4 * PI / sample_rate;
		  }
		  in_scattering += tmp * exp(-absorption_coef * (r.o - sample_point).norm()) * isect.t / sample_num;
		  sample_point -= r.d * stride;
	  }
 
  //int ext_coef = 0;
  //Spectrum in_scattering;

  if (!coin_flip(0.65) || nextRay.depth >= max_ray_depth) 
	  // russian roulette,
	  // reach maxmum ray
	  return ext_coef * in_scattering + L_out * exp(-absorption_coef * isect.t);
  
  Intersection next_isect;
  bvh->intersect(nextRay, &next_isect);
  next_isect.t -= EPS_F;
  Spectrum tmp = at_least_one_bounce_radiance(nextRay, next_isect);
  if (isect.bsdf->is_delta())
	  tmp += zero_bounce_radiance(nextRay, next_isect);
  L_out += tmp * isect.bsdf->f(w_out, wi) * abs(wi.z) / pdf / 0.65;

  return ext_coef * in_scattering + L_out * exp(-absorption_coef * isect.t);
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect))
    return L_out;
  isect.t -= EPS_F;
  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  // direct_hemisphere_sample = true;
  L_out = (isect.t == INF_D) ? debug_shading(r.d) : at_least_one_bounce_radiance(r, isect);
  
  //if (isect.bsdf->is_delta())
	  L_out += zero_bounce_radiance(r, isect);
  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

   // volumetric scattering
	  return L_out;
}

Spectrum medium_scattering(const Ray& r, const Intersection& isect) {
	
	if (!isect.primitive)
		return Spectrum(0.0);

	// sample the ray
	
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

  // TODO (Part 1.1):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Spectrum.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Spectrum spct(0.0, 0.0, 0.0);
  double s2 = 0;
  int i = 1;
  for (; i <= num_samples; i++) {
	  
	  Vector2D sample = PathTracer::gridSampler->get_sample();
	  sample.x += origin.x;
	  sample.y += origin.y;
	  Ray ray = PathTracer::camera->generate_ray(sample.x/sampleBuffer.w, sample.y/sampleBuffer.h);
	  Spectrum tmp = est_radiance_global_illumination(ray);
	  spct += tmp;
	  s2 += pow(tmp.illum(), 2);
	  if (i % samplesPerBatch == 0) {
		  double s1 = spct.illum();
		  if (s1*maxTolerance >= 1.96*sqrt((i*s2 - s1 * s1) / (i - 1)))
			  break;
	  }
  }
  spct /= i;
  sampleBuffer.update_pixel(spct, x, y);	// Spectrum(0.2, 1.0, 0.8)
  sampleCountBuffer[x + y * sampleBuffer.w] = i;
}

} // namespace CGL
