#pragma once
#include "BSDF.h"
#include "Warp.h"
#include <algorithm>

class OrenNayarBSDF : public BSDF
{
public:
	OrenNayarBSDF(const Vector3f& _normal, const Vector3f& _tangent,
		const Vector3f& _bitangent, Spectrum _albedo, float _sigma)
		: BSDF(_normal, _tangent, _bitangent), albedo(_albedo), sigma(_sigma)
	{
	}

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		// TODO
		// 1. 转换坐标系到局部坐标
		// 2. 计算 A, B, \alpha, \beta（可以直接求\sin\alpha,\tan\beta）,
		// \cos(\phi_i-\phi_o)
		// 3. return Oren-Nayar brdf
		//Vector3f wiLocal = wi, woLocal = wo;
		Vector3f wiLocal = toLocal(wi), woLocal = toLocal(wo);
		float squareSigma = sigma * sigma;
		float a = 1 - squareSigma / (2 * (squareSigma + 0.33)),
			b = 0.45 * squareSigma / (squareSigma + 0.09);

		float cosAlpha = std::min(wiLocal[1], woLocal[1]),
			cosBeta = std::max(wiLocal[1], woLocal[1]);
		float sinAlpha = sqrtf(1 - powf(cosAlpha, 2));
		float tanBeta = sqrtf(1 - powf(cosBeta, 2)) / cosBeta;

		float ri = sqrtf(1 - powf(wiLocal[1], 2)),
			ro = sqrtf(1 - powf(woLocal[1], 2));
		float cosPhiDiffer = (wiLocal[0] * woLocal[0]) / (ri * ro) +
			(wiLocal[2] * woLocal[2]) / (ri * ro);

		return albedo * wiLocal[1] * INV_PI * (a + b * std::max(0.f, cosPhiDiffer) * sinAlpha * tanBeta);
	}

	virtual BSDFSampleResult sample(const Vector3f& wo,
		const Vector2f& sample) const override
	{
		Vector3f wi = squareToCosineHemisphere(sample);
		float pdf = squareToCosineHemispherePdf(wi);
		return { albedo, toWorld(wi), pdf, BSDFType::Diffuse };
	}

private:
	Spectrum albedo;
	float sigma;
};