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
  const Vector3D &hit_p = r.o + r.d * isect.t; //world space
  const Vector3D &w_out = w2o * (-r.d);// o space

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
    
    for (int i = 0; i< num_samples; i++){
        Vector3D wj = hemisphereSampler->get_sample();
        Ray temp_r = Ray(hit_p + (EPS_D*o2w * wj ), o2w * wj);
        Intersection* inte = new Intersection();
        if (bvh->intersect(temp_r, inte)){
            Spectrum fr = isect.bsdf->f(w_out,  wj);
            Spectrum li = inte->bsdf->get_emission();
            double costheta = abs(dot(wj,Vector3D(0.0,0.0,1.0)));
            L_out += (fr *li)*costheta *2.0*PI/num_samples;
        }
    }
    return L_out;
    

}

Spectrum
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

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
    
    for (SceneLight* light : scene->lights){
        if (light->is_delta_light()){
            Vector3D wi;
            float dist2light;
            float pdf;
            Vector3D li = light->sample_L(hit_p , &wi, &dist2light, &pdf);
            Ray ray = Ray(hit_p + EPS_D , wi);
            ray.max_t = dist2light - EPS_F;
            ray.min_t = EPS_F;
            Intersection i ;
            if(! bvh->intersect(ray, &i)){
                
                    Spectrum fr = isect.bsdf->f(w_out,  w2o*wi);
                    double costheta = abs(dot(w2o*wi,Vector3D(0.0,0.0,1.0)));
                    L_out += fr*li*costheta/pdf;
                
            }
        }
        else{
            Spectrum l;
            for(int i = 0; i < ns_area_light; i++){
              Vector3D wi;
              float dist2light;
              float pdf;
              Vector3D li = light->sample_L(hit_p, &wi, &dist2light, &pdf);
              if((w2o * wi).z >= 0){
                Ray ray = Ray(hit_p + EPS_D, wi);
                ray.max_t = dist2light - EPS_F;
                ray.min_t = EPS_F;
                Intersection i;
                if(!bvh->intersect(ray, &i)){
                  Spectrum fr = isect.bsdf->f(w_out, w2o * wi);
                  double costheta = abs(dot(w2o*wi,Vector3D(0.0,0.0,1.0)));
                  l+= li * fr *costheta / pdf;
                  
                }
              }
            }
            
            L_out += (l / (ns_area_light*1.0));
        }
        
    }
  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
    return isect.bsdf->get_emission();
  
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if(direct_hemisphere_sample){
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  return estimate_direct_lighting_importance(r, isect);
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);
    
    float p = 0.65;

  Spectrum L_out(0, 0, 0);
    if (r.depth < max_ray_depth){
        L_out = one_bounce_radiance(r, isect);
        Vector3D wi;
        float pdf;
        if (coin_flip(p)){
            Spectrum li = isect.bsdf->sample_f(w_out, &wi, &pdf);
            Ray ray=Ray(hit_p + EPS_D*o2w * wi, o2w * wi);
            ray.depth = r.depth + 1;
            Intersection i ;
            if(bvh->intersect(ray, &i)){
                double costheta = abs(dot(wi,Vector3D(0.0,0.0,1.0)));
                L_out += li * at_least_one_bounce_radiance(ray, i) * costheta / (pdf*p);
            }
        }
    }

  return L_out;
     

}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect))
    return L_out;

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
    //L_out = (isect.t == INF_D) ? debug_shading(r.d) : one_bounce_radiance(r, isect);
    //L_out = zero_bounce_radiance(r, isect);
   // L_out = one_bounce_radiance(r, isect);
  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  //return zero_bounce_radiance(r, isect)+one_bounce_radiance(r, isect);
    L_out = zero_bounce_radiance(r, isect);
    if(max_ray_depth > 0){
      L_out += at_least_one_bounce_radiance(r, isect);
    }
  
  // volumetric scattering
	  int stride = 2;
	  float absorption_coef = 0.5;
	  float ext_coef = 0.5;
	  float g = 0.1;
	  Vector3D sample_point = r.o + r.d * (isect.t - fmod(isect.t, stride) * random_uniform());
	  int sample_num = isect.t / stride;
	  Spectrum in_scattering(0, 0, 0);
	  int sample_rate = 32;
	  for (int i = 0; i < sample_num; i++) {
		  // uniform sphere sampling
		  Spectrum tmp(0, 0, 0);
		  for (int j = 0; j < sample_rate; j++) {
			  double theta = acos(random_uniform());
			  double phi = 2.0 * PI * random_uniform();
			  Vector3D dir = (sinf(theta) * cosf(phi),
				  sinf(theta) * sinf(phi),
				  coin_flip(0.5) ? cosf(theta) : -1 * cosf(theta));
			  double cosine = dot(dir, r.d); // not sure !!!
			  double phase = (1 - g * g) / (4 * PI*pow(1 + g * g - 2 * g*cosine, 1.5));
			  Ray in_sca_ray(sample_point, dir);
			  Intersection in_sca_ite;
			  bvh->intersect(in_sca_ray, &in_sca_ite);
			  tmp += phase * (one_bounce_radiance(in_sca_ray, in_sca_ite) + 
							  zero_bounce_radiance(in_sca_ray, in_sca_ite)) * 4 * PI / sample_rate;
		  }
		  in_scattering += tmp * exp(-absorption_coef * (r.o - sample_point).norm()) * isect.t / sample_num;
		  sample_point -= r.d * stride;
	  }

	  return ext_coef * in_scattering + L_out * exp(-absorption_coef * isect.t);
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
    Spectrum avg = Spectrum(0.0, 0.0, 0.0);
//    for (int i = 0; i<num_samples; i++){
//        Vector2D sample = gridSampler->get_sample();
//        Ray r = camera->generate_ray((sample.x + x*1.0)/(1.0*sampleBuffer.w),
//                                     (sample.y + y*1.0)/(1.0*sampleBuffer.h));
//        avg += est_radiance_global_illumination(r);
//    }
//    avg = avg/(1.0*num_samples);
    int i = 0;
    double s1 = 0;
    double s2 = 0;
    while(i<num_samples){
        i+=1;
        Vector2D sample = gridSampler->get_sample();
        Ray r = camera->generate_ray((sample.x + x*1.0)/(1.0*sampleBuffer.w),
                                     (sample.y + y*1.0)/(1.0*sampleBuffer.h));
        Spectrum illu = est_radiance_global_illumination(r);
        avg += illu;
        s1 += illu.illum();
        s2 += (illu.illum())*(illu.illum());
        if ( i % samplesPerBatch == 0){
            double mu = s1 / (1.0*i);
            double sigma = sqrt((s2-s1*s1/(1.0*i))/( i*1.0 -1.0));
            double convergence = 1.96*sigma/sqrt(1.0*i);
            if (convergence <= maxTolerance * mu){
                break;
            }
        }
        
    }
    avg = avg/(1.0*i);
  //sampleBuffer.update_pixel(Spectrum(0.2, 1.0, 0.8), x, y);
    sampleBuffer.update_pixel(avg, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = i;
}

} // namespace CGL
