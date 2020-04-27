#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

    BBox bbox;
    int leaf_size = 0;
    for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
        leaf_size+=1;
    }
    if (leaf_size < int(max_leaf_size)){
        BVHNode *node = new BVHNode(bbox);
        node->start = start;
        node->end = end;
        return node;
    }
    //create internal node, split to half
    else{
        BVHNode *node = new BVHNode(bbox);
        int max_axis = 0;
        double max_distance = 0.0;
        for (int i = 0; i<3; i++){
            if (bbox.extent[i]>max_distance){
                max_distance = bbox.extent[i];
                max_axis = i;
            }
        }
        if (max_axis == 0){
             sort(start, end, []( Primitive*  lhs,Primitive*  rhs){
                 return (lhs)->get_bbox().centroid()[0] <(rhs)->get_bbox().centroid()[0];
             });
        }
        else if (max_axis == 1){
            sort(start, end, []( Primitive*  lhs,Primitive*  rhs){
                return (lhs)->get_bbox().centroid()[1] <(rhs)->get_bbox().centroid()[1];
            });
        }
        else{
            sort(start, end, []( Primitive*  lhs,Primitive*  rhs){
                return (lhs)->get_bbox().centroid()[2] <(rhs)->get_bbox().centroid()[2];
            });
        }
       
        int count = 0;
        std::vector<Primitive*>::iterator split_p;
        for (split_p = start; count!= int(leaf_size/2); split_p++){
            count += 1;
        }
        node->start = start;
        node->end = end;
        node->bb = bbox;
        node->l = construct_bvh(start, split_p, max_leaf_size);
        node->r =construct_bvh(split_p, end, max_leaf_size);
        return node;
    }
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
//
//  for (auto p : primitives) {
//    total_isects++;
//    if (p->has_intersection(ray))
//      return true;
//  }
//  return false;
    double t0;
    double t1;
    return node->bb.intersect(ray, t0, t1);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    double t0;
    double t1;
        //return node->bb.intersect(ray, t0, t1);
    bool output = false;
    if (!node->bb.intersect(ray, t0, t1)){
        return false;
    }
    if (node->isLeaf()){
        
        for (auto p = node->start; p!=node->end; p++){
            if ( (*p)->intersect(ray, i)){
                output = true;
            }

        }
        //return intersect;
    }
    //Intersection* i_1 = new Intersection();
    else{
        bool hit1 = intersect(ray, i, node->l);
        bool hit2 = intersect(ray, i, node->r);
        output = hit1||hit2;
    }
    return output;

//  bool hit = false;
//  for (auto p : primitives) {
//    total_isects++;
//    hit = p->intersect(ray, i) || hit;
//  }
//  return hit;
}

} // namespace SceneObjects
} // namespace CGL
