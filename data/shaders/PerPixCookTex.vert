//#############################################################################
//  File:      PerPixCookTex.vert
//  Purpose:   GLSL vertex shader for Cook-Torrance physical based rendering.
//             Based on the physically based rendering (PBR) tutorial with GLSL
//             from Joey de Vries on https://learnopengl.com/#!PBR/Theory
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

//-----------------------------------------------------------------------------
layout (location = 0) in vec4  a_position; // Vertex position attribute
layout (location = 1) in vec3  a_normal;   // Vertex normal attribute
layout (location = 2) in vec2  a_texCoord; // Vertex texture coordiante attribute

uniform mat4  u_mvMatrix;    // modelview matrix
uniform mat3  u_nMatrix;     // normal matrix=transpose(inverse(mv))
uniform mat4  u_mvpMatrix;   // = projection * modelView

out     vec3  v_P_VS;        // Point of illumination in view space (VS)
out     vec3  v_N_VS;        // Normal at P_VS in view space
out     vec2  v_texCoord;    // Texture coordiante output
//-----------------------------------------------------------------------------
void main(void)
{  
    v_P_VS = vec3(u_mvMatrix * a_position);
    v_N_VS = vec3(u_nMatrix * a_normal);  
    v_texCoord = a_texCoord;
    gl_Position = u_mvpMatrix * a_position;
}
//-----------------------------------------------------------------------------