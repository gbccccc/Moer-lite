#include "Disk.h"
#include "ResourceLayer/Factory.h"
bool Disk::rayIntersectShape(Ray& ray, int* primID, float* u, float* v) const
{
	//* todo 完成光线与圆环的相交 填充primId,u,v.如果相交，更新光线的tFar
	//* 1.光线变换到局部空间
	//* 2.判断局部光线的方向在z轴分量是否为0
	//* 3.计算光线和平面交点
	//* 4.检验交点是否在圆环内
	//* 5.更新ray的tFar,减少光线和其他物体的相交计算次数
	//* Write your code here.
	Ray localRay = transform.inverseRay(ray);
	if (localRay.direction[2] == 0.0f)
		return false;

	float tInter = (0.0f - localRay.origin[2]) / localRay.direction[2];
	if (tInter < localRay.tNear || tInter > localRay.tFar)
		return false;

	float xInter = localRay.origin[0] + tInter * localRay.direction[0],
		yInter = localRay.origin[1] + tInter * localRay.direction[1];
	float rInter = sqrt(xInter * xInter + yInter * yInter);
	//assert(rInter * rInter - xInter * xInter - yInter * yInter < 0.01f);
	if (rInter < innerRadius || rInter > radius)
		return false;

	float phiInter;
	phiInter = atan2f(yInter, xInter);
	if (phiInter < 0.0f)
		phiInter += M_PI * 2;
	if (phiInter > phiMax)
		return false;
	//assert(phiInter <= phiMax && phiInter >= 0);

	ray.tFar = tInter;
	*primID = 0;
	*u = phiInter / phiMax;
	*v = (rInter - innerRadius) / (radius - innerRadius);
	return true;
}

void Disk::fillIntersection(float distance, int primID, float u, float v, Intersection* intersection) const
{
	/// ----------------------------------------------------
	//* todo 填充圆环相交信息中的法线以及相交位置信息
	//* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
	//* 2.位置信息可以根据uv计算出，同样需要变换
	//* Write your code here.
	/// ----------------------------------------------------
	Vector3f normal(0.0f, 0.0f, 1.0f);
	intersection->normal = transform.toWorld(normal);

	float phiInter = u * phiMax;
	float rInter = innerRadius + v * (radius - innerRadius);
	Point3f localInter(rInter * cos(phiInter), rInter * sin(phiInter), 0.0f);
	intersection->position = transform.toWorld(localInter);

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

Disk::Disk(const Json& json) : Shape(json)
{
	//    normal = transform.toWorld(Vector3f(0,0,1));
	//    origin = transform.toWorld(Point3f(0,0,0));
	//    auto
	//    //radius认为是三个方向的上的scale平均
	//    vecmat::vec4f v(1,1,1,0);
	//    auto radiusVec = transform.scale * v;
	//    radiusVec/=radiusVec[3];
	//    radius = (radiusVec[0]+radiusVec[1]+radiusVec[2])/3;
	radius = fetchOptional(json, "radius", 1.f);
	innerRadius = fetchOptional(json, "inner_radius", 0.f);
	phiMax = fetchOptional(json, "phi_max", 2 * PI);
	AABB local(Point3f(-radius, -radius, 0), Point3f(radius, radius, 0));
	boundingBox = transform.toWorld(local);
}

void Disk::uniformSampleOnSurface(Vector2f sample, Intersection* result, float* pdf) const
{
	//采样光源 暂时不用实现
}
REGISTER_CLASS(Disk, "disk")

