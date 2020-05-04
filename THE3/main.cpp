#include <iostream>

#include <math.h>

#include "parser.h"

#include "ppm.h"

using namespace std;

float dot_product(parser::Vec3f f, parser::Vec3f s) {
  return (f.x * s.x) + (f.y * s.y) + (f.z * s.z);
}

float square_difference(parser::Vec3f a, parser::Vec3f b)
{
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

parser::Vec3f cross_product(parser::Vec3f f, parser::Vec3f s) {
   parser::Vec3f res_vector ;
   res_vector.x=(f.y * s.z) - (f.z*s.y);
   res_vector.y=(-1)*((f.x * s.z) - (f.z*s.x));
   res_vector.z=(f.x * s.y) - (f.y*s.x);
   return res_vector;
}

parser::Vec3f mult(float f, parser::Vec3f vec) {
    parser::Vec3f res_vector ;
    res_vector.x= f * vec.x;
    res_vector.y = f * vec.y;
    res_vector.z = f * vec.z;
    return res_vector;
}

parser::Vec3f adding(parser::Vec3f f,parser::Vec3f s) {
   parser::Vec3f res_vector;
   res_vector.x= f.x + s.x;
   res_vector.y = f.y + s.y;
   res_vector.z = f.z + s.z;
   return res_vector;
}

parser::Vec3f subs(parser::Vec3f f,parser::Vec3f s) {
   parser::Vec3f res_vector;
   res_vector.x= f.x - s.x;
   res_vector.y = f.y - s.y;
   res_vector.z = f.z - s.z;
   return res_vector;
}

parser::Vec3f division(parser::Vec3f f,float s) {
   parser::Vec3f res_vector;
   res_vector.x= f.x / s;
   res_vector.y = f.y / s;
   res_vector.z = f.z / s;
   return res_vector;
}

parser::Vec3f ray_direction(int i, int j, parser::Vec3f u, parser::Vec3f gaz, parser::Vec3f camera, parser::Vec4f lrbt, float near_distance, int width, int height)
{
	parser::Vec3f a;
  parser::Vec3f m = adding(camera, mult(near_distance, gaz));
  parser::Vec3f v = cross_product(u, mult(-1,gaz));
	parser::Vec3f q = adding(adding(m, mult(lrbt.x, v)), mult(lrbt.w, u));
  float su = ((lrbt.y - lrbt.x)*(i+0.5))/width;
  float sv = ((lrbt.w - lrbt.z)*(j+0.5))/height;

  parser::Vec3f s = adding(adding(mult(su, v), mult(sv*(-1), u)), q);

	return subs(s, camera);
}

float sphere_intersect(parser::Vec3f c, parser::Vec3f d, parser::Vec3f o, float r)
{
  parser::Vec3f oc = subs(o, c);
  float d_oc = dot_product(d, oc);
  float oc_oc = dot_product(oc, oc);
  float d_d = dot_product(d, d);
  float x = sqrt(d_oc*d_oc - d_d*(oc_oc - r*r));
  float artili = ((-1)*(d_oc) + x)/d_d;
  float eksili = ((-1)*(d_oc) - x)/d_d;

  if (artili>0 && eksili>0)
  {
    if (artili > eksili)
    {
      return eksili;
    }
    return artili;
  }

  else if(artili > 0)
  {
    return artili;
  }

  else if (eksili > 0)
  {
    return eksili;
  }
  return 0;
}

float deter(parser::Vec3f a, parser::Vec3f b, parser::Vec3f c)
{
  return a.x*(b.y*c.z - c.y*b.z) + a.y*(c.x*b.z - b.x*c.z) + a.z*(b.x*c.y - b.y*c.x);
}

float tri_intersect(float max_, float min_, parser::Vec3f a, parser::Vec3f b, parser::Vec3f c, parser::Vec3f d, parser::Vec3f o)
{
  float detA = deter(subs(a,b), subs(a,c), d);
  float t = deter(subs(a,b), subs(a,c), subs(a,o) ) / detA;
  float beta = deter(subs(a,o), subs(a,c), d )/detA;
  float g = deter(subs(a,b), subs(a,o), d ) /detA;

  if(t < (-1)*min_ || t > max_)
    return 0;
  if(g < min_ || g>1)
    return 0;
  if(beta <min_ || ( beta > (1 - g) ) )
    return 0;

  return t;
}


parser::Vec3f normalize(parser::Vec3f a)
{
  parser::Vec3f r = a;
  float x = sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
  r.x /= x;
  r.y /= x;
  r.z /= x;
  return r;
}

parser::Vec3f find_RGB(float max_, float min_, parser::Scene& scene, parser::Material& m, parser::Vec3f ambient, vector<parser::PointLight> p, parser::Vec3f normal, parser::Vec3f ray,  parser::Vec3f camera, int r_depth)
{
  parser::Vec3f res;
  float eben = 0;
  int l1 = 0;
  int l2 = 0;
  int r1 = 0;
  int r2 = 0;
  int qq1 = 0;
  int qq2 = 0;
  float eben21 = 0;
  float eben22 = 0;
  float eben23 = 0;
  float dif1 = 0;
  parser::Vec3f dir;
  parser::Vec3f dir2;
  bool blacked =false;
  int i;
  //float dirt;
  res.x = (ambient.x)*m.ambient.x;
  res.y = (ambient.y)*m.ambient.y;
  res.z = (ambient.z)*m.ambient.z;

  parser::Vec3f temp1 = ray;

  parser::Vec3f default1;
  default1.x = 0;
  default1.y = 0;
  default1.z = 0;

  if (r_depth > 0 && (m.mirror.x != 0 || m.mirror.y !=0 || m.mirror.z !=0) )
  {
    qq1 = 0;
    qq2 = 0;
    eben21 = 0;
    eben22 = 0;
    eben23 = 0;
    parser::Vec3f tri_a;
    parser::Vec3f tri_b;
    parser::Vec3f tri_c;
    parser::Vec3f cross;
    int which_one = 0;
    int which_r;
    parser::Vec3f fark2 = subs( camera, ray );
    parser::Vec3f wo = division(fark2 , sqrt(fark2.x*fark2.x + fark2.y*fark2.y + fark2.z*fark2.z));
    parser::Vec3f wr = subs(mult(2*dot_product(normal, wo), normal), wo);
    parser::Vec3f temp2;
    wr = normalize(wr);
    temp2.x = ray.x + wr.x*scene.shadow_ray_epsilon;
    temp2.y = ray.y + wr.y*scene.shadow_ray_epsilon;
    temp2.z = ray.z + wr.z*scene.shadow_ray_epsilon;
    parser::Vec3f normal2;
    parser::Vec3f pp;

    for (r1 = 0; r1 < scene.spheres.size(); r1++)
    {
      eben21 = sphere_intersect(scene.vertex_data[scene.spheres[r1].center_vertex_id - 1], wr, temp2, scene.spheres[r1].radius);
      if (eben21 > 0)
      {
        max_ = eben21;
        float radius = scene.spheres[r1].radius;
        pp = adding(mult(eben21, wr), ray);
        parser::Vec3f pay =  subs( pp, scene.vertex_data[scene.spheres[r1].center_vertex_id-1]) ;

        normal2.x = pay.x / radius;
        normal2.y = pay.y / radius;
        normal2.z = pay.z / radius;

        which_one = 1;
        which_r = r1;
        qq1 = 1;
      }
    }
    for (r1 = 0; r1 < scene.triangles.size(); r1++)
    {
      eben22 = tri_intersect(INFINITY,min_, scene.vertex_data[scene.triangles[r1].indices.v0_id - 1], scene.vertex_data[scene.triangles[r1].indices.v1_id - 1], scene.vertex_data[scene.triangles[r1].indices.v2_id - 1], wr, temp2);
      if (eben22 > 0 && (qq1 == 0 || eben22 < eben21))
      {
        max_ = eben22;
        pp = adding(mult(eben22, wr), ray);

        tri_a= scene.vertex_data[scene.triangles[r1].indices.v0_id-1];
        tri_b= scene.vertex_data[scene.triangles[r1].indices.v1_id-1];
        tri_c= scene.vertex_data[scene.triangles[r1].indices.v2_id-1];

        cross = cross_product( subs(tri_b, tri_a), subs(tri_c, tri_a));
        normal2 = division(cross , sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z) );
        which_one = 2;
        which_r = r1;
        //default1 = find_RGB(max_, scene, scene.materials[scene.triangles[r1].material_id - 1], scene.ambient_light, scene.point_lights, normal2, pp, ray, r_depth-1);
        qq2 = 1;
      }
    }
    for (r2 = 0; r2 < scene.meshes.size(); r2++)
    {
      for (r1 = 0; r1 < scene.meshes[r2].faces.size(); r1++)
      {
        eben23 = tri_intersect(INFINITY,min_, scene.vertex_data[scene.meshes[r2].faces[r1].v0_id - 1], scene.vertex_data[scene.meshes[r2].faces[r1].v1_id - 1], scene.vertex_data[scene.meshes[r2].faces[r1].v2_id - 1], wr, temp2);
        if (eben23 > 0 && ((qq1 == 0 || eben23 < eben21) && (qq2 == 0 || eben23 < eben22)))
        {
          max_ = eben23;
          pp = adding(mult(eben23, wr), ray);

          tri_a= scene.vertex_data[scene.meshes[r2].faces[r1].v0_id-1];
          tri_b= scene.vertex_data[scene.meshes[r2].faces[r1].v1_id-1];
          tri_c= scene.vertex_data[scene.meshes[r2].faces[r1].v2_id-1];

          cross = cross_product(subs(tri_b, tri_a), subs(tri_c, tri_a));
          normal2 = division(cross , sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z));

          which_one = 3;
          which_r = r2;
        }
      }
    }

    if (which_one != 0)
    {
      if (which_one == 1)
      {
        default1 = find_RGB(max_,min_, scene, scene.materials[scene.spheres[which_r].material_id - 1], scene.ambient_light, scene.point_lights, normal2, pp, ray, r_depth-1);
        default1.x = m.mirror.x*default1.x;
        default1.y = m.mirror.y*default1.y;
        default1.z = m.mirror.z*default1.z;

      }
      else if (which_one == 2)
      {
        default1 = find_RGB(max_,min_, scene, scene.materials[scene.triangles[which_r].material_id - 1], scene.ambient_light, scene.point_lights, normal2, pp, ray, r_depth-1);
        default1.x = m.mirror.x*default1.x;
        default1.y = m.mirror.y*default1.y;
        default1.z = m.mirror.z*default1.z;
      }
      else if (which_one == 3)
      {
        default1=find_RGB(max_,min_, scene, scene.materials[scene.meshes[which_r].material_id - 1], scene.ambient_light, scene.point_lights, normal2, pp, ray, r_depth-1);
        default1.x = m.mirror.x*default1.x;
        default1.y = m.mirror.y*default1.y;
        default1.z = m.mirror.z*default1.z;

      }
    }
  }


  for(i = 0; i < p.size() ; i++)
  {
    dir.x = (p[i].position.x - ray.x) / square_difference(ray, p[i].position);
    dir.y = (p[i].position.y - ray.y) / square_difference(ray, p[i].position);
    dir.z = (p[i].position.z - ray.z) / square_difference(ray, p[i].position);

    temp1.x = ray.x + dir.x*scene.shadow_ray_epsilon;
    temp1.y = ray.y + dir.y*scene.shadow_ray_epsilon;
    temp1.z = ray.z + dir.z*scene.shadow_ray_epsilon;

    dir2.x = p[i].position.x - ray.x;
    dir2.y = p[i].position.y - ray.y;
    dir2.z = p[i].position.z - ray.z;

    for (l1 = 0; l1<scene.spheres.size() && blacked == false; l1++)
    {
      eben = sphere_intersect(scene.vertex_data[scene.spheres[l1].center_vertex_id-1], dir2, temp1, scene.spheres[l1].radius);
      if (eben > 0 && eben < 1)
      {
        blacked = true;
        break;
      }
    }

    for (l1 = 0; l1<scene.triangles.size() && blacked == false; l1++)
    {
      eben = tri_intersect(max_,min_, scene.vertex_data[scene.triangles[l1].indices.v0_id - 1], scene.vertex_data[scene.triangles[l1].indices.v1_id - 1], scene.vertex_data[scene.triangles[l1].indices.v2_id - 1], dir2, temp1);
      if (eben > 0 && eben < 1)
      {
        blacked = true;
        break;
      }
    }

    for (l2 = 0; l2<scene.meshes.size() && blacked == false; l2++)
    {
      for (l1 = 0; l1<scene.meshes[l2].faces.size() && blacked == false; l1++)
      {
        eben = tri_intersect(max_,min_, scene.vertex_data[scene.meshes[l2].faces[l1].v0_id - 1], scene.vertex_data[scene.meshes[l2].faces[l1].v1_id - 1], scene.vertex_data[scene.meshes[l2].faces[l1].v2_id - 1], dir2, temp1);
        if (eben > 0 && eben < 1)
        {
          blacked = true;
          break;
        }
      }
    }

    if (blacked == true)
		{
			blacked = false;
		}

		else
		{
      //cout<<"here"<<endl;
      float d_square=(ray.x - p[i].position.x)*(ray.x - p[i].position.x) + (ray.y - p[i].position.y)*(ray.y - p[i].position.y) + (ray.z - p[i].position.z)*(ray.z - p[i].position.z);

      if(d_square==0)
        d_square=1;

      parser::Vec3f fark = subs( p[i].position , ray );
      parser::Vec3f wi = division( fark, sqrt(fark.x*fark.x + fark.y*fark.y + fark.z*fark.z) );
      float cosdif= dot_product( normal , wi );

      if(cosdif < 0)
        cosdif = 0;

      parser::Vec3f fark2 = subs( camera, ray );
      parser::Vec3f wo = division(fark2 , sqrt(fark2.x*fark2.x + fark2.y*fark2.y + fark2.z*fark2.z));
      parser::Vec3f pay = adding(wo, wi);
      parser::Vec3f h = division(pay ,sqrt(pay.x*pay.x + pay.y*pay.y + pay.z*pay.z) );
      float cosspe= pow(dot_product( normal , h ), m.phong_exponent);

      if(cosspe < 0)
        cosspe = 0;

      float px = (p[i].intensity.x);
      float py = (p[i].intensity.y);
      float pz = (p[i].intensity.z);

      res.x += ( cosdif * m.diffuse.x * px ) / d_square + ( cosspe * m.specular.x * px ) /d_square;
      res.y += ( cosdif * m.diffuse.y * py ) / d_square + ( cosspe * m.specular.y * py ) /d_square;
      res.z += ( cosdif * m.diffuse.z * pz ) / d_square + ( cosspe * m.specular.z * pz ) /d_square ;
    }
    blacked = false;
  }

  res.x += default1.x;
  res.y += default1.y;
  res.z += default1.z;

  return res;
}

int main(int argc, char* argv[])
{

    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    parser::Vec3f v;
    parser::Vec3f rgb_value;
    parser::Vec3f normal;
    parser::Vec3f ray;
    parser::Vec3f pay;
    parser::Vec3f tri_c;
    parser::Vec3f tri_b;
    parser::Vec3f tri_a;
    parser::Vec3f cross;

    for (int z = 0; z < scene.cameras.size(); z++)
    {
        float b, b2, b3;
        int i = 0, q = 0, q2 = 0, q3 = 0;
        int j = 0, j2 = 0;
        int width = scene.cameras[z].image_width;
        int height = scene.cameras[z].image_height;
        unsigned char* image = new unsigned char [width * height * 3];

        i = 0;

        float max_= INFINITY;

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                max_ = INFINITY;
                v = ray_direction(x, y, scene.cameras[z].up, scene.cameras[z].gaze, scene.cameras[z].position, scene.cameras[z].near_plane, scene.cameras[z].near_distance, width, height);

              //  v = adding(v , mult(scene.shadow_ray_epsilon, v));
                q = 0;
                q2 = 0;
                q3 = 0;
                b = 0;
                b2 = 0;
                b3 = 0;
                normal.x = 0;
                normal.y = 0;
                normal.z = 0;
                rgb_value.x = scene.background_color.x;
                rgb_value.y = scene.background_color.y;
                rgb_value.z = scene.background_color.z;

                for(j=0;j<scene.spheres.size();j++)
                {
                  b = sphere_intersect(scene.vertex_data[scene.spheres[j].center_vertex_id-1], v, scene.cameras[z].position, scene.spheres[j].radius);
                  if(b > 0)
                  {
                    max_ = b;
                    float radius = scene.spheres[j].radius;
                    ray = adding(mult(b, v), scene.cameras[z].position);
                    pay =  subs( ray, scene.vertex_data[scene.spheres[j].center_vertex_id-1]) ;
                    normal.x = pay.x / radius;
                    normal.y = pay.y / radius;
                    normal.z = pay.z / radius;

                    rgb_value = find_RGB(max_,(-1)*scene.shadow_ray_epsilon, scene,scene.materials[scene.spheres[j].material_id-1], scene.ambient_light, scene.point_lights, normal, ray, scene.cameras[z].position, scene.max_recursion_depth);

                    q = 1;
                  }
                }

                for (j=0; j<scene.triangles.size(); j++)
                {
                  b2 = tri_intersect(max_,(-1)*scene.shadow_ray_epsilon, scene.vertex_data[scene.triangles[j].indices.v0_id-1], scene.vertex_data[scene.triangles[j].indices.v1_id-1], scene.vertex_data[scene.triangles[j].indices.v2_id-1], v, scene.cameras[z].position);
                  if (b2 > 0 && (q == 0 || b2 < b))
                  {
                    max_ = b2;
                    ray = adding(mult(b2, v), scene.cameras[z].position);
                    tri_a= scene.vertex_data[scene.triangles[j].indices.v0_id-1];
                    tri_b= scene.vertex_data[scene.triangles[j].indices.v1_id-1];
                    tri_c= scene.vertex_data[scene.triangles[j].indices.v2_id-1];

                    cross = cross_product( subs(tri_b, tri_a), subs(tri_c, tri_a));

                    normal = division(cross , sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z) );

                    rgb_value = find_RGB(max_,(-1)*scene.shadow_ray_epsilon, scene,scene.materials[scene.triangles[j].material_id-1], scene.ambient_light, scene.point_lights, normal, ray, scene.cameras[z].position, scene.max_recursion_depth);

                    q2 = 1;
                  }
                }
                for(j=0; j<scene.meshes.size(); j++)
                {
                  for (j2=0; j2<scene.meshes[j].faces.size(); j2++)
                  {
                    b3 = tri_intersect(max_,(-1)*scene.shadow_ray_epsilon, scene.vertex_data[scene.meshes[j].faces[j2].v0_id-1], scene.vertex_data[scene.meshes[j].faces[j2].v1_id-1], scene.vertex_data[scene.meshes[j].faces[j2].v2_id-1], v, scene.cameras[z].position);
                    if (b3 > 0 && ((q == 0 || b3 < b) && (q2 == 0 || b3 < b2)))
                    {
                      max_ = b3;
                      ray = adding(mult(b3, v), scene.cameras[z].position);
                      tri_a= scene.vertex_data[scene.meshes[j].faces[j2].v0_id-1];
                      tri_b= scene.vertex_data[scene.meshes[j].faces[j2].v1_id-1];
                      tri_c= scene.vertex_data[scene.meshes[j].faces[j2].v2_id-1];
                      cross = cross_product( subs(tri_b, tri_a), subs(tri_c, tri_a) );
                      normal = division(cross , sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z) );
                      rgb_value = find_RGB(max_,(-1)*scene.shadow_ray_epsilon, scene, scene.materials[scene.meshes[j].material_id-1], scene.ambient_light, scene.point_lights, normal ,ray, scene.cameras[z].position, scene.max_recursion_depth);
                      q3 = 1;
                    }
                  }
                }
                if(rgb_value.x > 255)
                  rgb_value.x = 255;
                if(rgb_value.y > 255)
                  rgb_value.y = 255;
                if(rgb_value.z > 255)
                  rgb_value.z = 255;
                image[i++] = rgb_value.x;
                image[i++] = rgb_value.y;
                image[i++] = rgb_value.z;
            }
        }
        write_ppm(scene.cameras[z].image_name.c_str(), image, width, height);
    }
}
