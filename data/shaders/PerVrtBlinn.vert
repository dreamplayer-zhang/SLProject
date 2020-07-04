//#############################################################################
//  File:      PerVrtBlinn.vert
//  Purpose:   GLSL vertex program for per vertex Blinn-Phong lighting
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

/*
The preprocessor constant #define NUM_LIGHTS will be added at the shader
compilation time. It must be constant to be used in the for loop in main().
Therefore this number it can not be passed as a uniform variable.
*/

//-----------------------------------------------------------------------------
layout (location = 0) in vec4 a_position; // Vertex position attribute
layout (location = 1) in vec3 a_normal;   // Vertex normal attribute

uniform mat4   u_mvMatrix;          // modelview matrix 
uniform mat3   u_nMatrix;           // normal matrix=transpose(inverse(mv))
uniform mat4   u_mvpMatrix;         // = projection * modelView

uniform bool   u_lightIsOn[NUM_LIGHTS];       // flag if light is on
uniform vec4   u_lightPosVS[NUM_LIGHTS];      // position of light in view space
uniform vec4   u_lightAmbient[NUM_LIGHTS];    // ambient light intensity (Ia)
uniform vec4   u_lightDiffuse[NUM_LIGHTS];    // diffuse light intensity (Id)
uniform vec4   u_lightSpecular[NUM_LIGHTS];   // specular light intensity (Is)
uniform vec3   u_lightSpotDirVS[NUM_LIGHTS];  // spot direction in view space
uniform float  u_lightSpotCutoff[NUM_LIGHTS]; // spot cutoff angle 1-180 degrees
uniform float  u_lightSpotCosCut[NUM_LIGHTS]; // cosine of spot cutoff angle
uniform float  u_lightSpotExp[NUM_LIGHTS];    // spot exponent
uniform vec3   u_lightAtt[NUM_LIGHTS];        // attenuation (const,linear,quadr.)
uniform bool   u_lightDoAtt[NUM_LIGHTS];      // flag if att. must be calc.
uniform vec4   u_globalAmbient;               // Global ambient scene color
uniform float  u_oneOverGamma;                // 1.0f / Gamma correction value

uniform vec4   u_matAmbient;        // ambient color reflection coefficient (ka)
uniform vec4   u_matDiffuse;        // diffuse color reflection coefficient (kd)
uniform vec4   u_matSpecular;       // specular color reflection coefficient (ks)
uniform vec4   u_matEmissive;       // emissive color for self-shining materials
uniform float  u_matShininess;      // shininess exponent

out     vec4   v_color;             // The resulting color per vertex
out     vec3   v_P_VS;              // Point of illumination in view space (VS)
//-----------------------------------------------------------------------------
void directLightBlinnPhong(in    int  i,    // Light number between 0 and NUM_LIGHTS
in    vec3 N,    // Normalized normal at v_P
in    vec3 E,    // Normalized direction at v_P to the eye
in    vec3 S,    // Normalized light spot direction
inout vec4 Ia,   // Ambient light intensity
inout vec4 Id,   // Diffuse light intensity
inout vec4 Is)   // Specular light intensity
{
    // Calculate diffuse & specular factors
    float diffFactor = max(dot(N, S), 0.0);
    float specFactor = 0.0;

    if (diffFactor!=0.0)
    {
        vec3 H = normalize(S + E);// Half vector H between S and E
        specFactor = pow(max(dot(N, H), 0.0), u_matShininess);
    }

    // accumulate directional light intesities w/o attenuation
    Ia += u_lightAmbient[i];
    Id += u_lightDiffuse[i] * diffFactor;
    Is += u_lightSpecular[i] * specFactor;
}
//-----------------------------------------------------------------------------
void pointLightBlinnPhong( in    int  i,    // Light number between 0 and NUM_LIGHTS
in    vec3 N,    // Normalized normal at v_P
in    vec3 E,    // Normalized direction at v_P to the eye
in    vec3 S,    // Normalized light spot direction
in    vec3 L,    // Unnormalized direction at v_P to the light
inout vec4 Ia,   // Ambient light intensity
inout vec4 Id,   // Diffuse light intensity
inout vec4 Is)   // Specular light intensity
{
    // Normalized halfvector between the eye and the light vector
    vec3 H = normalize(E + L);

    // Calculate attenuation over distance & normalize L
    float att = 1.0;
    if (u_lightDoAtt[i])
    {
        vec3 att_dist;
        att_dist.x = 1.0;
        att_dist.z = dot(L, L);// = distance * distance
        att_dist.y = sqrt(att_dist.z);// = distance
        att = 1.0 / dot(att_dist, u_lightAtt[i]);
        L /= att_dist.y;// = normalize(L)
    } else L = normalize(L);

    // Calculate diffuse & specular factors
    float diffFactor = max(dot(N, L), 0.0);
    float specFactor = 0.0;
    if (diffFactor!=0.0)
    specFactor = pow(max(dot(N, H), 0.0), u_matShininess);

    // Calculate spot attenuation
    if (u_lightSpotCutoff[i] < 180.0)
    {
        float spotDot;// Cosine of angle between L and spotdir
        float spotAtt;// Spot attenuation
        spotDot = dot(-L, S);
        if (spotDot < u_lightSpotCosCut[i]) spotAtt = 0.0;
        else spotAtt = max(pow(spotDot, u_lightSpotExp[i]), 0.0);
        att *= spotAtt;
    }

    // Accumulate light intesities
    Ia += att * u_lightAmbient[i];
    Id += att * u_lightDiffuse[i] * diffFactor;
    Is += att * u_lightSpecular[i] * specFactor;
}
//-----------------------------------------------------------------------------
void main()
{
    vec4 Ia = vec4(0.0); // Accumulated ambient light intensity at v_P_VS
    vec4 Id = vec4(0.0); // Accumulated diffuse light intensity at v_P_VS
    vec4 Is = vec4(0.0); // Accumulated specular light intensity at v_P_VS

    v_P_VS = vec3(u_mvMatrix * a_position);
    vec3 N = normalize(u_nMatrix * a_normal);
    vec3 E = normalize(-v_P_VS);

    for (int i = 0; i < NUM_LIGHTS; ++i)
    {
        if (u_lightIsOn[i])
        {
            if (u_lightPosVS[i].w == 0.0)
            {
                // We use the spot light direction as the light direction vector
                vec3 S = normalize(-u_lightSpotDirVS[i].xyz);
                directLightBlinnPhong(i, N, E, S, Ia, Id, Is);
            }
            else
            {
                vec3 S = u_lightSpotDirVS[i]; // normalized spot direction in VS
                vec3 L = u_lightPosVS[i].xyz - v_P_VS; // Vector from v_P to light in VS
                pointLightBlinnPhong(i, N, E, S, L, Ia, Id, Is);
            }
        }
    }

    // Sum up all the reflected color components
    v_color =  u_matEmissive +
               u_globalAmbient +
               Ia * u_matAmbient +
               Id * u_matDiffuse +
               Is * u_matSpecular;

    // For correct alpha blending overwrite alpha component
    v_color.a = u_matDiffuse.a;

    // Apply gamma correction
    v_color.rgb = pow(v_color.rgb, vec3(u_oneOverGamma));

    // Set the transformes vertex position           
    gl_Position = u_mvpMatrix * a_position;
}
//-----------------------------------------------------------------------------
