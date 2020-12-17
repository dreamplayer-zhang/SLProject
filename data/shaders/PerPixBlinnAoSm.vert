//#############################################################################
//  File:      PerPixBlinnAoSm.vert
//  Purpose:   GLSL vertex shader for per pixel Blinn-Phong lighting with 
//             shadow mapping and ambient occlusion.
//  Author:    Marcus Hudritsch
//  Date:      July 2019
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

precision highp float;

//-----------------------------------------------------------------------------
layout (location = 0) in vec4  a_position;  // Vertex position attribute
layout (location = 1) in vec3  a_normal;    // Vertex normal attribute
layout (location = 3) in vec2  a_uv2;       // Vertex tex.coord. 2 for AO

uniform mat4  u_mvMatrix;   // modelview matrix
uniform mat3  u_nMatrix;    // normal matrix=transpose(inverse(mv))
uniform mat4  u_mvpMatrix;  // = projection * modelView
uniform mat4  u_mMatrix;    // model matrix

out     vec3  v_P_VS;       // Point of illumination in view space (VS)
out     vec3  v_P_WS;       // Point of illumination in world space (WS)
out     vec3  v_N_VS;       // Normal at P_VS in view space
out     vec2  v_uv2;        // Texture coordiante 1 output for AO
//-----------------------------------------------------------------------------
void main(void)
{
    v_uv2 = a_uv2;  // pass ambient occlusion tex.coord. 2 for interpolation

    v_P_VS = vec3(u_mvMatrix *  a_position); // vertex position in view space
    v_P_WS = vec3(u_mMatrix * a_position);   // vertex position in world space
    v_N_VS = vec3(u_nMatrix * a_normal);     // vertex normal in view space

    gl_Position = u_mvpMatrix * a_position;
}
//-----------------------------------------------------------------------------