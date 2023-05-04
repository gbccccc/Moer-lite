#pragma once
#include "BSDF.h"
#include "Warp.h"

class PhongReflection : public BSDF
{
public:
	PhongReflection(const Vector3f& _normal, const Vector3f& _tangent,
		const Vector3f& _bitangent, Spectrum _albedo, float _kd,
		float _ks, float _p)
		: BSDF(_normal, _tangent, _bitangent), albedo(_albedo), kd(_kd), ks(_ks),
		p(_p), specularReflectance(1.f)
	{
	}

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		// TODO
		// 1. 转换坐标系到局部坐标
		// 2. 根据公式计算 K_d, K_s
		// 3. return K_d + K_s
		// tips:
		// Phong模型brdf实现不包括环境光项；其I/r^2项包含在光源采样步骤中，因此brdf中不包含I/r^2。
		Spectrum diffuse(kd * dot(normal, wi));
		Vector3f wr = 2 * dot(normal, wi) * normal - wi;
		Spectrum specular(ks * powf(dot(wr, wo), p));
		return diffuse + specular;
	}

	float pdf(const Vector3f& wo, const Vector3f& wi) const
	{
		Vector3f woLocal = toLocal(wo), wiLocal = toLocal(wi);
		return ks + kd * squareToCosineHemispherePdf(wiLocal);
	}

	virtual BSDFSampleResult sample(const Vector3f& wo,
		const Vector2f& sample) const override
	{
		Vector3f wiLocal = squareToCosineHemisphere(sample);
		auto wi = toWorld(wiLocal);
		auto bsdf_f = f(wo, wi);
		auto bsdf_pdf = pdf(wo, wi);
		return { bsdf_f / bsdf_pdf, wi, bsdf_pdf, BSDFType::Diffuse };
	}

private:
	Spectrum albedo;
	Spectrum specularReflectance; // 假设Phong模型高光仅是白光
	float kd;                     // 漫反射系数
	float ks;                     // 高光（镜面反射）系数
	float p;                      // 高光衰减系数
};