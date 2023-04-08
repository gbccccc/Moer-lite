#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray& ray, int* primID, float* u, float* v) const
{
	//* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
	//* 1.光线变换到局部空间
	//* 2.联立方程求解
	//* 3.检验交点是否在圆柱范围内
	//* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
	//* Write your code here.
	Ray localRay = transform.inverseRay(ray);
	float equaA = powf(localRay.direction[0], 2) + powf(localRay.direction[1], 2),
		equaB = 2 * (localRay.origin[0] * localRay.direction[0] + localRay.origin[1] * localRay.direction[1]),
		equaC = powf(localRay.origin[0], 2) + powf(localRay.origin[1], 2) - powf(radius, 2);
	float tInter[2];
	if (!Quadratic(equaA, equaB, equaC, &tInter[0], &tInter[1]))
		return false;

	bool valid[2];
	Point3f inter[2];
	float phiInter[2];
	for (int i = 0; i < 2; i++)
	{
		valid[i] = true;
		valid[i] &= tInter[i] >= localRay.tNear && tInter[i] <= localRay.tFar;

		inter[i] = localRay.origin + localRay.direction * tInter[i];
		valid[i] &= inter[i][2] >= 0 && inter[i][2] <= height;

		phiInter[i] = atan2f(inter[i][1], inter[i][0]);
		if (phiInter[i] < 0)
			phiInter[i] += M_PI * 2;
		valid[i] &= phiInter[i] <= phiMax;
	}


	int index;
	if (!valid[0] && !valid[1])
		return false;
	else if (!valid[0] || !valid[1])
		index = valid[0] ? 0 : 1;
	else
		index = tInter[0] < tInter[1] ? 0 : 1;

	ray.tFar = tInter[index];
	*primID = 0;
	*u = phiInter[index] / phiMax;
	*v = inter[index][2] / height;
	return true;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection* intersection) const
{
	/// ----------------------------------------------------
	//* todo 填充圆柱相交信息中的法线以及相交位置信息
	//* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
	//* 2.位置信息可以根据uv计算出，同样需要变换
	//* Write your code here.
	/// ----------------------------------------------------
	float phiInter = u * phiMax;
	Point3f inter(
		radius * cosf(phiInter),
		radius * sinf(phiInter),
		v * height);

	Vector3f normal(inter[0], inter[1], 0.0f);
	normal /= normal.length();

	intersection->position = transform.toWorld(inter);
	intersection->normal = transform.toWorld(normal);

	intersection->shape = this;
	intersection->distance = distance;
	intersection->texCoord = Vector2f{ u, v };
	Vector3f tangent{ 1.f, 0.f, .0f };
	Vector3f bitangent;
	if (std::abs(dot(tangent, intersection->normal)) > .9f)
	{
		tangent = Vector3f(.0f, 1.f, .0f);
	}
	bitangent = normalize(cross(tangent, intersection->normal));
	tangent = normalize(cross(intersection->normal, bitangent));
	intersection->tangent = tangent;
	intersection->bitangent = bitangent;
}

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection* result, float* pdf) const
{

}

Cylinder::Cylinder(const Json& json) : Shape(json)
{
	radius = fetchOptional(json, "radius", 1.f);
	height = fetchOptional(json, "height", 1.f);
	phiMax = fetchOptional(json, "phi_max", 2 * PI);
	AABB localAABB = AABB(Point3f(-radius, -radius, 0), Point3f(radius, radius, height));
	boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder, "cylinder")
