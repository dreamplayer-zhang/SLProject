vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float distributionGGX(vec3 N, vec3 H, float roughness)
{
    float a      = roughness*roughness;
    float a2     = a*a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;

    float nom   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return nom / denom;
}

float geometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float nom   = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / denom;
}

float geometrySmith(vec3 N, vec3 E, vec3 L, float roughness)
{
    float NdotV = max(dot(N, E), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2  = geometrySchlickGGX(NdotV, roughness);
    float ggx1  = geometrySchlickGGX(NdotL, roughness);
    return ggx1 * ggx2;
}

void pointLightCookTorrance(in    vec3  N,        // Normalized normal at v_P_VS
                            in    vec3  E,        // Normalized vector from v_P to the eye
                            in    vec3  L,        // Vector from v_P to the light
                            in    vec3  lightDiff,// diffuse light intensity
                            in    vec3  matDiff,  // diffuse material reflection
                            in    float matMetal, // diffuse material reflection
                            in    float matRough, // diffuse material reflection
                            inout vec3  Lo)       // reflected intensity
{
    float distance = length(L);// distance to light
    L /= distance;// normalize light vector
    vec3 H = normalize(E + L);// Normalized halfvector between eye and light vector
    float att = 1.0 / (distance*distance);// quadratic light attenuation
    vec3 radiance = lightDiff * att;// per light radiance

    // Init Fresnel reflection at 90 deg. (0 to N)
    vec3 F0 = vec3(0.04);
    F0 = mix(F0, matDiff, matMetal);

    // cook-torrance brdf
    float NDF = distributionGGX(N, H, matRough);
    float G   = geometrySmith(N, E, L, matRough);
    vec3  F   = fresnelSchlick(max(dot(H, E), 0.0), F0);

    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;
    kD *= 1.0 - matMetal;

    vec3  nominator   = NDF * G * F;
    float denominator = 4.0 * max(dot(N, E), 0.0) * max(dot(N, L), 0.0) + 0.001;
    vec3  specular    = nominator / denominator;

    // add to outgoing radiance Lo
    float NdotL = max(dot(N, L), 0.0);

    Lo += (kD*matDiff.rgb/PI + specular) * radiance * NdotL;
}
