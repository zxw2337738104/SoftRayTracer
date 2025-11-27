//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    //this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection shade_point_inter = Scene::intersect(ray);
    Vector3f L_dir(0), L_indir(0), result_color = this->backgroundColor;
    if (shade_point_inter.happened) {
        Vector3f p = shade_point_inter.coords;
        Vector3f wo = -ray.direction;
        Vector3f N = shade_point_inter.normal.normalized();

        switch(shade_point_inter.m->getType()) {
        case MIRROR:
        {
            float ksi = get_random_float();
            if (ksi <= RussianRoulette) {
                //wi = sample(wo, N)
                Vector3f wi = normalize(shade_point_inter.m->sample(wo, N));
                //Trace a ray r(p, wi)
                Ray p_along_wi(p, wi);
                Intersection bounce_point_inter(intersect(p_along_wi));

                if (bounce_point_inter.happened) {
                    //L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)/ pdf(wo, wi, N) / RussianRoulette
                    float pdf_wi = shade_point_inter.m->pdf(wo, wi, N);
                    if (pdf_wi > EPSILON) {
                        L_indir = castRay(p_along_wi) * shade_point_inter.m->eval(wo, wi, N) *
                            dotProduct(wi, N) / (pdf_wi * RussianRoulette);
                    }
                }
            }
            break;
        }
        default:
            //SampleLight(inter,pdf_light)
            Intersection light_point_inter;
            float pdf_light;
            sampleLight(light_point_inter, pdf_light);
            //Get x,ws,NN,emit from inter
            Vector3f x = light_point_inter.coords;
            Vector3f ws = normalize(x - p);
            Vector3f NN = light_point_inter.normal.normalized();
            Vector3f emit = light_point_inter.emit;
            float distance = (p - x).norm();
            float distance_square = pow(distance, 2);

            //Shoot a ray from p to x
            Ray test_block_ray1(p, ws);
            Intersection light_to_shade = Scene::intersect(test_block_ray1);
            //If the ray is not blocked in the middle
            //L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,NN) / |x - p | ^ 2 / pdf_light
            if (light_to_shade.happened && distance - light_to_shade.distance <  0.0001 && pdf_light > EPSILON) {
                L_dir = emit * shade_point_inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) /
                    (distance_square * pdf_light);
            }

            //Test Russian Roulette with probability RussianRoulette
            float ksi = get_random_float();
            if (ksi <= RussianRoulette) {
                //wi = sample(wo, N)
                Vector3f wi = normalize(shade_point_inter.m->sample(wo, N));
                //Trace a ray r(p, wi)
                Ray p_along_wi(p, wi);
                Intersection bounce_point_inter(intersect(p_along_wi));

                if (bounce_point_inter.happened && !bounce_point_inter.m->hasEmission()) {
                    //L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)/ pdf(wo, wi, N) / RussianRoulette
                    float pdf_wi = shade_point_inter.m->pdf(wo, wi, N);
                    if (pdf_wi > EPSILON) {
                        L_indir = castRay(p_along_wi) * shade_point_inter.m->eval(wo, wi, N, false) *
                            dotProduct(wi, N) / (pdf_wi * RussianRoulette);
                    }
                }
            }
            break;
        }
        result_color = shade_point_inter.m->getEmission() + L_dir + L_indir;
        result_color.x = clamp(0, 1, result_color.x);
        result_color.y = clamp(0, 1, result_color.y);
        result_color.z = clamp(0, 1, result_color.z);
    }
    return result_color;
}